#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from sensor_msgs.msg import JointState, Imu
from nav_msgs.msg import Odometry
from geometry_msgs.msg import Twist
import numpy as np
import matplotlib.pyplot as plt
from matplotlib.animation import FuncAnimation
import threading
import time
import os

# For Clearpath A200, these values should be close:
WHEEL_RADIUS = 0.165  # meters (estimated for Husky/A200)
WHEEL_TRACK = 0.555    # meters (distance between left and right wheels)
WHEEL_BASE = 0.4572    # meters (distance between front and rear wheels)

class WheelSlipMonitor(Node):
    def __init__(self):
        super().__init__('wheel_slip_monitor')
        
        # Create subscribers
        self.joint_states_sub = self.create_subscription(
            JointState,
            '/a200_0000/platform/joint_states',
            self.joint_states_callback,
            10)
        
        self.imu_sub = self.create_subscription(
            Imu,
            '/a200_0000/sensors/imu_0/data',
            self.imu_callback,
            10)
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/a200_0000/platform/odom',
            self.odom_callback,
            10)
        
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/a200_0000/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        # Data storage - Increased to store more history
        self.max_data_points = 1000  # Increased from 500 to keep more history
        self.wheel_velocities = {
            'front_left': [],
            'front_right': [],
            'rear_left': [],
            'rear_right': []
        }
        self.timestamps = []
        self.imu_angular_vel = []
        self.odom_linear_vel = []
        self.odom_angular_vel = []
        self.cmd_linear_vel = []
        self.cmd_angular_vel = []
        
        # Store the last received command to use for continuous updates
        self.last_cmd_lin_vel = 0.0
        self.last_cmd_ang_vel = 0.0
        self.last_cmd_time = 0.0
        
        # Thread control
        self.lock = threading.Lock()
        self.plotting = False
        self.start_time = time.time()
        
        # Timer for regular data updates (including command velocity)
        # Updated to run more frequently for real-time updates
        self.data_update_timer = self.create_timer(0.05, self.update_data_timer)
        
        # Status report timer
        self.status_timer = self.create_timer(1.0, self.print_status)
        
        # Initialize plotting in a separate thread
        self.create_plotting_thread()
        
        self.get_logger().info("Wheel slip monitor initialized. Close the plot window to exit.")

    def create_plotting_thread(self):
        """Create and start the plotting thread"""
        if not self.plotting:
            self.plotting = True
            self.plot_thread = threading.Thread(target=self.run_plot)
            self.plot_thread.daemon = True
            self.plot_thread.start()
            
    def wheel_angular_to_linear(self, angular_velocity):
        """Convert wheel angular velocity to linear velocity"""
        return angular_velocity * WHEEL_RADIUS
    
    def update_data_timer(self):
        """Timer callback to regularly update all data including command velocity"""
        current_time = time.time() - self.start_time
        
        with self.lock:
            # Only add data point if we already have some data
            if self.timestamps:
                # Add a data point for the current time with the last known command
                self.timestamps.append(current_time)
                self.cmd_linear_vel.append(self.last_cmd_lin_vel)
                self.cmd_angular_vel.append(self.last_cmd_ang_vel)
                
                # Also maintain other arrays by duplicating their last values
                # This ensures all arrays stay in sync
                for wheel in self.wheel_velocities.keys():
                    if self.wheel_velocities[wheel]:
                        self.wheel_velocities[wheel].append(self.wheel_velocities[wheel][-1])
                    else:
                        self.wheel_velocities[wheel].append(0.0)
                
                if self.odom_linear_vel:
                    self.odom_linear_vel.append(self.odom_linear_vel[-1])
                else:
                    self.odom_linear_vel.append(0.0)
                
                if self.odom_angular_vel:
                    self.odom_angular_vel.append(self.odom_angular_vel[-1])
                else:
                    self.odom_angular_vel.append(0.0)
                
                if self.imu_angular_vel:
                    self.imu_angular_vel.append(self.imu_angular_vel[-1])
                else:
                    self.imu_angular_vel.append(0.0)
                
                # Trim data if it gets too large
                self._trim_data()

    def joint_states_callback(self, msg):
        if not msg.name:
            return
            
        current_time = time.time() - self.start_time
        noise_threshold = 0.005  # 5mm/s threshold for noise filtering
        
        with self.lock:
            # Extract wheel velocities
            wheel_values_updated = False
            for i, name in enumerate(msg.name):
                # Convert angular to linear velocity
                linear_vel = self.wheel_angular_to_linear(msg.velocity[i])
                
                # Store values but filter out tiny values (sensor noise)
                if 'front_left_wheel' in name:
                    self.wheel_velocities['front_left'].append(linear_vel)
                    wheel_values_updated = True
                elif 'front_right_wheel' in name:
                    self.wheel_velocities['front_right'].append(linear_vel)
                    wheel_values_updated = True
                elif 'rear_left_wheel' in name:
                    self.wheel_velocities['rear_left'].append(linear_vel)
                    wheel_values_updated = True
                elif 'rear_right_wheel' in name:
                    self.wheel_velocities['rear_right'].append(linear_vel)
                    wheel_values_updated = True
            
            # Only append timestamp if we actually received wheel data
            if wheel_values_updated:
                self.timestamps.append(current_time)
                
                # Add corresponding entries for command velocity
                self.cmd_linear_vel.append(self.last_cmd_lin_vel)
                self.cmd_angular_vel.append(self.last_cmd_ang_vel)
                
                # Add corresponding entries for other data types too
                if self.odom_linear_vel:
                    self.odom_linear_vel.append(self.odom_linear_vel[-1])
                else:
                    self.odom_linear_vel.append(0.0)
                
                if self.odom_angular_vel:
                    self.odom_angular_vel.append(self.odom_angular_vel[-1])
                else:
                    self.odom_angular_vel.append(0.0)
                
                if self.imu_angular_vel:
                    self.imu_angular_vel.append(self.imu_angular_vel[-1])
                else:
                    self.imu_angular_vel.append(0.0)
                
                # If all wheels have very small values but different signs, log a warning
                latest_values = []
                for wheel in self.wheel_velocities.keys():
                    if self.wheel_velocities[wheel]:
                        latest_values.append(self.wheel_velocities[wheel][-1])
                
                if len(latest_values) == 4:
                    all_small = all(abs(val) < 0.01 for val in latest_values)
                    different_signs = (sum(1 for val in latest_values if val > 0) > 0 and 
                                      sum(1 for val in latest_values if val < 0) > 0)
                    
                    if all_small and different_signs:
                        # This indicates drift in the encoder readings
                        left_vals = [self.wheel_velocities['front_left'][-1], self.wheel_velocities['rear_left'][-1]]
                        right_vals = [self.wheel_velocities['front_right'][-1], self.wheel_velocities['rear_right'][-1]]
                        
                        left_avg = sum(left_vals)/2
                        right_avg = sum(right_vals)/2
                        
                        if left_avg > 0 and right_avg > 0:
                            self.get_logger().debug("Small forward drift in wheel velocities detected")
                        elif left_avg < 0 and right_avg < 0:
                            self.get_logger().debug("Small backward drift in wheel velocities detected")
                        elif left_avg > 0 and right_avg < 0:
                            self.get_logger().debug("Small counter-clockwise rotation drift detected")
                        elif left_avg < 0 and right_avg > 0:
                            self.get_logger().debug("Small clockwise rotation drift detected")
            
            # Trim data if it gets too large
            self._trim_data()

    def imu_callback(self, msg):
        with self.lock:
            # Extract angular velocity about Z-axis
            ang_vel_z = msg.angular_velocity.z
            
            # Add a timestamp and the IMU data
            current_time = time.time() - self.start_time
            self.timestamps.append(current_time)
            self.imu_angular_vel.append(ang_vel_z)
            
            # Add corresponding command velocity entries
            self.cmd_linear_vel.append(self.last_cmd_lin_vel)
            self.cmd_angular_vel.append(self.last_cmd_ang_vel)
            
            # Add corresponding entries for other data types
            for wheel in self.wheel_velocities.keys():
                if self.wheel_velocities[wheel]:
                    self.wheel_velocities[wheel].append(self.wheel_velocities[wheel][-1])
                else:
                    self.wheel_velocities[wheel].append(0.0)
            
            if self.odom_linear_vel:
                self.odom_linear_vel.append(self.odom_linear_vel[-1])
            else:
                self.odom_linear_vel.append(0.0)
                
            if self.odom_angular_vel:
                self.odom_angular_vel.append(self.odom_angular_vel[-1])
            else:
                self.odom_angular_vel.append(0.0)
            
            # Trim data if it gets too large
            self._trim_data()

    def odom_callback(self, msg):
        with self.lock:
            # Extract linear and angular velocity
            lin_vel_x = msg.twist.twist.linear.x
            ang_vel_z = msg.twist.twist.angular.z
            
            # Add timestamp and odom data
            current_time = time.time() - self.start_time
            self.timestamps.append(current_time)
            self.odom_linear_vel.append(lin_vel_x)
            self.odom_angular_vel.append(ang_vel_z)
            
            # Add corresponding command velocity entries
            self.cmd_linear_vel.append(self.last_cmd_lin_vel)
            self.cmd_angular_vel.append(self.last_cmd_ang_vel)
            
            # Add corresponding entries for other data types
            for wheel in self.wheel_velocities.keys():
                if self.wheel_velocities[wheel]:
                    self.wheel_velocities[wheel].append(self.wheel_velocities[wheel][-1])
                else:
                    self.wheel_velocities[wheel].append(0.0)
            
            if self.imu_angular_vel:
                self.imu_angular_vel.append(self.imu_angular_vel[-1])
            else:
                self.imu_angular_vel.append(0.0)
            
            # Trim data if it gets too large
            self._trim_data()

    def cmd_vel_callback(self, msg):
        with self.lock:
            # Extract commanded velocities
            lin_vel_x = msg.linear.x
            ang_vel_z = msg.angular.z
            
            # Store current time and update last command values
            current_time = time.time() - self.start_time
            self.last_cmd_lin_vel = lin_vel_x
            self.last_cmd_ang_vel = ang_vel_z
            self.last_cmd_time = current_time
            
            # Handle the special case where we receive a zero command
            # This likely means the command was meant to stop the robot
            if lin_vel_x == 0.0 and ang_vel_z == 0.0 and self.timestamps:
                # If we have previous commands that weren't zero, add a transition point
                if (self.cmd_linear_vel and self.cmd_linear_vel[-1] != 0.0) or \
                   (self.cmd_angular_vel and self.cmd_angular_vel[-1] != 0.0):
                    # Add a data point that shows the command just before it goes to zero
                    # This creates a visual step in the graph instead of connecting points with a line
                    self.timestamps.append(current_time - 0.01)  # Slightly before current time
                    
                    if self.cmd_linear_vel:
                        self.cmd_linear_vel.append(self.cmd_linear_vel[-1])
                    else:
                        self.cmd_linear_vel.append(0.0)
                        
                    if self.cmd_angular_vel:
                        self.cmd_angular_vel.append(self.cmd_angular_vel[-1])
                    else:
                        self.cmd_angular_vel.append(0.0)
                        
                    # Make sure we add counterparts for other data structures
                    for wheel in self.wheel_velocities.keys():
                        if self.wheel_velocities[wheel]:
                            self.wheel_velocities[wheel].append(self.wheel_velocities[wheel][-1])
                        else:
                            self.wheel_velocities[wheel].append(0.0)
                            
                    if self.odom_linear_vel:
                        self.odom_linear_vel.append(self.odom_linear_vel[-1])
                    else:
                        self.odom_linear_vel.append(0.0)
                        
                    if self.odom_angular_vel:
                        self.odom_angular_vel.append(self.odom_angular_vel[-1])
                    else:
                        self.odom_angular_vel.append(0.0)
                        
                    if self.imu_angular_vel:
                        self.imu_angular_vel.append(self.imu_angular_vel[-1])
                    else:
                        self.imu_angular_vel.append(0.0)
            
            # Add the current command values with the current timestamp
            self.timestamps.append(current_time)
            self.cmd_linear_vel.append(lin_vel_x)
            self.cmd_angular_vel.append(ang_vel_z)
            
            # Add matching data points for other values to keep arrays in sync
            for wheel in self.wheel_velocities.keys():
                if self.wheel_velocities[wheel]:
                    self.wheel_velocities[wheel].append(self.wheel_velocities[wheel][-1])
                else:
                    self.wheel_velocities[wheel].append(0.0)
                    
            if self.odom_linear_vel:
                self.odom_linear_vel.append(self.odom_linear_vel[-1])
            else:
                self.odom_linear_vel.append(0.0)
                
            if self.odom_angular_vel:
                self.odom_angular_vel.append(self.odom_angular_vel[-1])
            else:
                self.odom_angular_vel.append(0.0)
                
            if self.imu_angular_vel:
                self.imu_angular_vel.append(self.imu_angular_vel[-1])
            else:
                self.imu_angular_vel.append(0.0)
            
            # Trim data if it gets too large
            self._trim_data()
    
    def _trim_data(self):
        """Helper method to trim all data arrays to max_data_points"""
        if len(self.timestamps) > self.max_data_points:
            self.timestamps = self.timestamps[-self.max_data_points:]
            
            # Trim all data arrays
            self.cmd_linear_vel = self.cmd_linear_vel[-self.max_data_points:]
            self.cmd_angular_vel = self.cmd_angular_vel[-self.max_data_points:]
            
            for wheel in self.wheel_velocities.keys():
                if len(self.wheel_velocities[wheel]) > self.max_data_points:
                    self.wheel_velocities[wheel] = self.wheel_velocities[wheel][-self.max_data_points:]
                    
            if len(self.odom_linear_vel) > self.max_data_points:
                self.odom_linear_vel = self.odom_linear_vel[-self.max_data_points:]
                
            if len(self.odom_angular_vel) > self.max_data_points:
                self.odom_angular_vel = self.odom_angular_vel[-self.max_data_points:]
                
            if len(self.imu_angular_vel) > self.max_data_points:
                self.imu_angular_vel = self.imu_angular_vel[-self.max_data_points:]

    def detect_wheel_slip(self):
        """Detect wheel slip by comparing wheel velocities with body velocity"""
        with self.lock:
            if (not self.wheel_velocities['front_left'] or 
                not self.odom_linear_vel or 
                not self.odom_angular_vel):
                return False, {}
            
            # Calculate expected wheel speeds based on odometry
            robot_lin_vel = self.odom_linear_vel[-1]
            robot_ang_vel = self.odom_angular_vel[-1]
            
            # If the robot is barely moving, don't check for slip
            if abs(robot_lin_vel) < 0.05 and abs(robot_ang_vel) < 0.05:
                return False, {}
            
            # Calculate expected wheel velocities for a skid-steer robot
            expected_velocities = {}
            expected_velocities['front_left'] = robot_lin_vel - (robot_ang_vel * WHEEL_TRACK / 2)
            expected_velocities['front_right'] = robot_lin_vel + (robot_ang_vel * WHEEL_TRACK / 2)
            expected_velocities['rear_left'] = robot_lin_vel - (robot_ang_vel * WHEEL_TRACK / 2)
            expected_velocities['rear_right'] = robot_lin_vel + (robot_ang_vel * WHEEL_TRACK / 2)
            
            # Compare with actual wheel velocities
            slipping_wheels = {}
            slip_threshold = 0.1  # m/s difference - lowered for detection
            
            for wheel in self.wheel_velocities.keys():
                if self.wheel_velocities[wheel]:
                    actual_vel = self.wheel_velocities[wheel][-1]
                    expected_vel = expected_velocities[wheel]
                    
                    diff = abs(actual_vel - expected_vel)
                    if diff > slip_threshold:
                        slipping_wheels[wheel] = diff
            
            if slipping_wheels:
                return True, slipping_wheels
            
            return False, {}

    def print_status(self):
        """Print information about wheel velocities and slip detection"""
        try:
            is_slipping, slipping_wheels = self.detect_wheel_slip()
            
            with self.lock:
                if not self.wheel_velocities['front_left']:
                    self.get_logger().info("Waiting for wheel data...")
                    return
                
                # Get the latest wheel velocities and filter out very small values
                # Any value below 0.005 m/s is considered zero (noise threshold)
                noise_threshold = 0.005
                
                fl = self.wheel_velocities['front_left'][-1]
                fr = self.wheel_velocities['front_right'][-1]
                rl = self.wheel_velocities['rear_left'][-1]
                rr = self.wheel_velocities['rear_right'][-1]
                
                # Zero out very small values (noise)
                fl = 0.0 if abs(fl) < noise_threshold else fl
                fr = 0.0 if abs(fr) < noise_threshold else fr
                rl = 0.0 if abs(rl) < noise_threshold else rl
                rr = 0.0 if abs(rr) < noise_threshold else rr
                
                # Get the latest odometry velocity
                odom_vel = self.odom_linear_vel[-1] if self.odom_linear_vel else 0.0
                odom_ang = self.odom_angular_vel[-1] if self.odom_angular_vel else 0.0
                
                # Filter odometry values too
                odom_vel = 0.0 if abs(odom_vel) < noise_threshold else odom_vel
                odom_ang = 0.0 if abs(odom_ang) < noise_threshold else odom_ang
                
                cmd_vel = self.cmd_linear_vel[-1] if self.cmd_linear_vel else 0.0
                cmd_ang = self.cmd_angular_vel[-1] if self.cmd_angular_vel else 0.0
                
                # Filter commanded values
                cmd_vel = 0.0 if abs(cmd_vel) < noise_threshold else cmd_vel
                cmd_ang = 0.0 if abs(cmd_ang) < noise_threshold else cmd_ang
            
            # Print status
            self.get_logger().info(f"Wheel velocities [m/s]: FL={fl:.2f}, FR={fr:.2f}, RL={rl:.2f}, RR={rr:.2f}")
            self.get_logger().info(f"Robot velocity: Linear={odom_vel:.2f} m/s, Angular={odom_ang:.2f} rad/s")
            self.get_logger().info(f"Command velocity: Linear={cmd_vel:.2f} m/s, Angular={cmd_ang:.2f} rad/s")
            
            if is_slipping:
                slip_text = ", ".join([f"{wheel}: {diff:.2f}" for wheel, diff in slipping_wheels.items()])
                self.get_logger().warn(f"WHEEL SLIP DETECTED: {slip_text}")
        except Exception as e:
            self.get_logger().error(f"Error in status printing: {str(e)}")

    def run_plot(self):
        """Run the real-time plotting"""
        try:
            # Create figure
            plt.ion()  # Interactive mode
            fig, axs = plt.subplots(3, 1, figsize=(10, 12))
            fig.suptitle('Wheel Slip and Speed Monitoring')
            
            # Initialize lines for wheel velocities
            wheel_lines = {}
            for wheel in self.wheel_velocities.keys():
                wheel_lines[wheel], = axs[0].plot([], [], label=wheel)
            
            # Initialize lines for linear velocity
            cmd_vel_line, = axs[1].plot([], [], label='Commanded')
            odom_vel_line, = axs[1].plot([], [], label='Odometry')
            
            # Initialize lines for angular velocity
            cmd_ang_vel_line, = axs[2].plot([], [], label='Commanded')
            odom_ang_vel_line, = axs[2].plot([], [], label='Odometry')
            imu_ang_vel_line, = axs[2].plot([], [], label='IMU')
            
            # Set labels and titles
            axs[0].set_title('Wheel Velocities')
            axs[0].set_ylabel('Velocity (m/s)')
            axs[0].set_xlabel('Time (s)')
            axs[0].grid(True)
            axs[0].legend()
            
            axs[1].set_title('Linear Velocity Comparison')
            axs[1].set_ylabel('Velocity (m/s)')
            axs[1].set_xlabel('Time (s)')
            axs[1].grid(True)
            axs[1].legend()
            
            axs[2].set_title('Angular Velocity Comparison')
            axs[2].set_ylabel('Velocity (rad/s)')
            axs[2].set_xlabel('Time (s)')
            axs[2].grid(True)
            axs[2].legend()
            
            plt.tight_layout()
            
            # Update in real-time (as fast as possible while keeping CPU reasonable)
            update_interval = 0.05  # Update every 50ms for real-time appearance
            last_update_time = time.time()
            
            # Update plot function
            while plt.fignum_exists(fig.number):
                try:
                    current_time = time.time()
                    
                    # Update more frequently for real-time feel
                    if current_time - last_update_time < update_interval:
                        time.sleep(0.01)  # Shorter sleep for more responsive updates
                        continue
                    
                    last_update_time = current_time
                    
                    with self.lock:
                        if not self.timestamps:
                            time.sleep(0.1)
                            continue
                        
                        # Make copies to avoid threading issues
                        timestamps = self.timestamps.copy()
                        
                        wheel_data = {}
                        for wheel in self.wheel_velocities.keys():
                            wheel_data[wheel] = self.wheel_velocities[wheel].copy()
                        
                        cmd_linear = self.cmd_linear_vel.copy()
                        odom_linear = self.odom_linear_vel.copy()
                        
                        cmd_angular = self.cmd_angular_vel.copy()
                        odom_angular = self.odom_angular_vel.copy()
                        imu_angular = self.imu_angular_vel.copy()
                    
                    # Update wheel velocity lines
                    for wheel, line in wheel_lines.items():
                        if wheel_data[wheel]:
                            line.set_data(timestamps, wheel_data[wheel])
                    
                    # Update linear velocity lines
                    if cmd_linear and len(cmd_linear) > 0:
                        if len(cmd_linear) < len(timestamps):
                            padded_cmd = [0] * (len(timestamps) - len(cmd_linear)) + cmd_linear
                        else:
                            padded_cmd = cmd_linear[-len(timestamps):]
                        cmd_vel_line.set_data(timestamps, padded_cmd)
                    
                    if odom_linear and len(odom_linear) > 0:
                        if len(odom_linear) < len(timestamps):
                            padded_odom = [0] * (len(timestamps) - len(odom_linear)) + odom_linear
                        else:
                            padded_odom = odom_linear[-len(timestamps):]
                        odom_vel_line.set_data(timestamps, padded_odom)
                    
                    # Update angular velocity lines
                    if cmd_angular and len(cmd_angular) > 0:
                        if len(cmd_angular) < len(timestamps):
                            padded_cmd_ang = [0] * (len(timestamps) - len(cmd_angular)) + cmd_angular
                        else:
                            padded_cmd_ang = cmd_angular[-len(timestamps):]
                        cmd_ang_vel_line.set_data(timestamps, padded_cmd_ang)
                    
                    if odom_angular and len(odom_angular) > 0:
                        if len(odom_angular) < len(timestamps):
                            padded_odom_ang = [0] * (len(timestamps) - len(odom_angular)) + odom_angular
                        else:
                            padded_odom_ang = odom_angular[-len(timestamps):]
                        odom_ang_vel_line.set_data(timestamps, padded_odom_ang)
                    
                    if imu_angular and len(imu_angular) > 0:
                        if len(imu_angular) < len(timestamps):
                            padded_imu_ang = [0] * (len(timestamps) - len(imu_angular)) + imu_angular
                        else:
                            padded_imu_ang = imu_angular[-len(timestamps):]
                        imu_ang_vel_line.set_data(timestamps, padded_imu_ang)
                    
                    # Update axis limits to show more data history
                    # Show all data if less than 300 seconds, otherwise show last 300 seconds
                    if len(timestamps) > 2:
                        t_max = timestamps[-1]
                        t_min = max(timestamps[0], t_max - 300)  # Show at most 300 seconds of history
                        
                        for ax in axs:
                            ax.set_xlim(t_min, t_max)
                    
                    # Set fixed y-axis limits for all plots
                    for ax in axs:
                        ax.set_ylim(-1.25, 1.25)  # Fixed range as requested
                    
                    # Update the figure
                    fig.canvas.draw_idle()
                    fig.canvas.flush_events()
                    
                    # Check for wheel slip
                    is_slipping, slipping_wheels = self.detect_wheel_slip()
                    if is_slipping:
                        slip_text = ", ".join([f"{wheel}: {diff:.2f}" for wheel, diff in slipping_wheels.items()])
                        axs[0].set_title(f'Wheel Velocities - SLIP DETECTED: {slip_text}')
                    else:
                        axs[0].set_title('Wheel Velocities')
                    
                except Exception as e:
                    self.get_logger().error(f"Error updating plot: {str(e)}")
                    time.sleep(0.5)
            
            self.plotting = False
            self.get_logger().info("Plot window closed. Exiting program...")
            rclpy.shutdown()
            
        except Exception as e:
            self.get_logger().error(f"Error in plotting thread: {str(e)}")
            self.plotting = False

def main(args=None):
    rclpy.init(args=args)
    
    wheel_slip_monitor = WheelSlipMonitor()
    
    # Use a multi-threaded executor for the ROS node
    executor = MultiThreadedExecutor()
    executor.add_node(wheel_slip_monitor)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        wheel_slip_monitor.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()