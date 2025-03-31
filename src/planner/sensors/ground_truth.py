#!/usr/bin/env python3

import rclpy
from rclpy.node import Node
from rclpy.executors import MultiThreadedExecutor
from geometry_msgs.msg import Twist
from tf2_msgs.msg import TFMessage
from nav_msgs.msg import Odometry
import numpy as np
import matplotlib.pyplot as plt
import threading
import time
import math

class SlipDetector(Node):
    def __init__(self):
        super().__init__('slip_detector')
        
        # Create subscribers
        self.cmd_vel_sub = self.create_subscription(
            Twist,
            '/a200_0000/cmd_vel',
            self.cmd_vel_callback,
            10)
        
        self.odom_sub = self.create_subscription(
            Odometry,
            '/a200_0000/platform/odom/filtered',
            self.odom_callback,
            10)
        
        # For the robot pose, subscribe to the /tf topic
        self.tf_sub = self.create_subscription(
            TFMessage,
            '/tf',
            self.tf_callback,
            10)
        
        # Data storage
        self.max_data_points = 1000
        self.timestamps = []
        self.cmd_vel_data = {'linear_x': [], 'angular_z': []}
        self.odom_data = {'linear_x': [], 'angular_z': []}
        self.ground_truth_data = {'x': [], 'y': [], 'theta': [], 'timestamps': []}
        self.last_ground_truth_pos = {'x': 0.0, 'y': 0.0, 'theta': 0.0, 'time': 0.0}
        self.derived_velocity = {'linear': [], 'angular': []}
        self.slip_metric = {'value': []}
        
        # Last values for continuous updates
        self.last_cmd_linear = 0.0
        self.last_cmd_angular = 0.0
        self.last_odom_linear = 0.0
        self.last_odom_angular = 0.0
        self.last_derived_linear = 0.0
        self.last_derived_angular = 0.0
        self.last_slip_value = 0.0
        
        # Parameters
        self.slip_threshold = 0.2
        self.start_time = time.time()
        self.use_odom_for_slip = True  # Use odometry when ground truth isn't reliable
        
        # Message counters for debugging
        self.cmd_vel_count = 0
        self.odom_count = 0
        self.tf_count = 0
        self.robot_tf_count = 0
        
        # Thread control
        self.lock = threading.Lock()
        self.plotting = False
        
        # Data update timer
        self.data_update_timer = self.create_timer(0.05, self.update_data_timer)
        
        # Status report timer
        self.status_timer = self.create_timer(1.0, self.print_status)
        
        # Initialize plotting in a separate thread
        self.create_plotting_thread()
        
        self.get_logger().info("Slip detector initialized. Close the plot window to exit.")
    
    def create_plotting_thread(self):
        """Create and start the plotting thread"""
        if not self.plotting:
            self.plotting = True
            self.plot_thread = threading.Thread(target=self.run_plot)
            self.plot_thread.daemon = True
            self.plot_thread.start()
    
    def cmd_vel_callback(self, msg):
        """Callback for commanded velocity"""
        current_time = time.time() - self.start_time
        self.cmd_vel_count += 1
        
        with self.lock:
            # Extract linear and angular velocity
            self.last_cmd_linear = msg.linear.x
            self.last_cmd_angular = msg.angular.z
            
            # Store data with timestamp
            self.timestamps.append(current_time)
            self.cmd_vel_data['linear_x'].append(msg.linear.x)
            self.cmd_vel_data['angular_z'].append(msg.angular.z)
            
            # Add corresponding entries for other data types to maintain sync
            self.odom_data['linear_x'].append(self.last_odom_linear)
            self.odom_data['angular_z'].append(self.last_odom_angular)
            self.derived_velocity['linear'].append(self.last_derived_linear)
            self.derived_velocity['angular'].append(self.last_derived_angular)
            
            # Calculate slip metric using odometry
            if abs(self.last_cmd_linear) > 0.05:  # Only calculate slip when commanded velocity is significant
                slip = abs(self.last_cmd_linear - self.last_odom_linear) / max(0.1, abs(self.last_cmd_linear))
                self.last_slip_value = slip
                
                # Log slip detection
                if slip > self.slip_threshold:
                    self.get_logger().warn(f"Slip detected! Metric: {slip:.2f}")
            
            self.slip_metric['value'].append(self.last_slip_value)
            
            # Trim data if needed
            self._trim_data()
    
    def odom_callback(self, msg):
        """Callback for odometry data"""
        current_time = time.time() - self.start_time
        self.odom_count += 1
        
        with self.lock:
            # Extract linear and angular velocity
            self.last_odom_linear = msg.twist.twist.linear.x
            self.last_odom_angular = msg.twist.twist.angular.z
            
            # Store data with timestamp
            self.timestamps.append(current_time)
            self.odom_data['linear_x'].append(msg.twist.twist.linear.x)
            self.odom_data['angular_z'].append(msg.twist.twist.angular.z)
            
            # Add corresponding entries for other data types
            self.cmd_vel_data['linear_x'].append(self.last_cmd_linear)
            self.cmd_vel_data['angular_z'].append(self.last_cmd_angular)
            self.derived_velocity['linear'].append(self.last_derived_linear)
            self.derived_velocity['angular'].append(self.last_derived_angular)
            
            # Calculate slip metric using odometry
            if abs(self.last_cmd_linear) > 0.05:  # Only calculate slip when commanded velocity is significant
                slip = abs(self.last_cmd_linear - self.last_odom_linear) / max(0.1, abs(self.last_cmd_linear))
                self.last_slip_value = slip
                
                # Log slip detection
                if slip > self.slip_threshold:
                    self.get_logger().warn(f"Slip detected! Metric: {slip:.2f}")
            
            self.slip_metric['value'].append(self.last_slip_value)
            
            # Trim data if needed
            self._trim_data()
    
    def tf_callback(self, msg):
        """Callback for TF messages to find our robot pose"""
        self.tf_count += 1
        
        for transform in msg.transforms:
            # Look for transforms related to our robot
            if 'spring_world' in transform.header.frame_id and 'a200_0000/robot' in transform.child_frame_id:
                # This is the transform from world to robot - our ground truth position
                self.robot_tf_count += 1
                
                # Process the transform to extract position data
                x = transform.transform.translation.x
                y = transform.transform.translation.y
                qx = transform.transform.rotation.x
                qy = transform.transform.rotation.y
                qz = transform.transform.rotation.z
                qw = transform.transform.rotation.w
                
                # Log the transform periodically
                if self.robot_tf_count % 100 == 1:
                    self.get_logger().info(f"Robot transform: ({x:.2f}, {y:.2f})")
                
                self._update_ground_truth(x, y, qx, qy, qz, qw)
                return  # Found what we need, exit the loop
    
    def _update_ground_truth(self, x, y, qx, qy, qz, qw):
        """Update ground truth position and calculate derived velocities"""
        current_time = time.time() - self.start_time
        
        with self.lock:
            # Convert quaternion to Euler angles
            _, _, theta = self.euler_from_quaternion(qx, qy, qz, qw)
            
            # Check for significant position change to log
            dx = x - self.last_ground_truth_pos['x']
            dy = y - self.last_ground_truth_pos['y']
            dist_change = math.sqrt(dx*dx + dy*dy)
            
            if dist_change > 0.1:  # Log position changes greater than 10cm
                self.get_logger().info(f"Robot moved: ({self.last_ground_truth_pos['x']:.2f}, "
                                    f"{self.last_ground_truth_pos['y']:.2f}) -> ({x:.2f}, {y:.2f})")
            
            # Store previous position and time for velocity calculation
            prev_pos = self.last_ground_truth_pos.copy()
            
            # Update current position
            self.last_ground_truth_pos['x'] = x
            self.last_ground_truth_pos['y'] = y
            self.last_ground_truth_pos['theta'] = theta
            self.last_ground_truth_pos['time'] = current_time
            
            # Store ground truth data for plotting
            self.ground_truth_data['x'].append(x)
            self.ground_truth_data['y'].append(y)
            self.ground_truth_data['theta'].append(theta)
            self.ground_truth_data['timestamps'].append(current_time)
            
            # Calculate derived velocity if we have a previous position
            dt = current_time - prev_pos['time']
            if dt > 0.01:  # Only calculate if enough time has passed
                dx = x - prev_pos['x']
                dy = y - prev_pos['y']
                dtheta = theta - prev_pos['theta']
                
                # Normalize dtheta to [-pi, pi]
                if dtheta > math.pi:
                    dtheta -= 2 * math.pi
                elif dtheta < -math.pi:
                    dtheta += 2 * math.pi
                
                # Calculate linear velocity (magnitude of position change)
                linear_vel = math.sqrt(dx*dx + dy*dy) / dt
                angular_vel = dtheta / dt
                
                # Filter out unreasonably high velocities (likely from discontinuities)
                if linear_vel < 10.0:  # Max reasonable velocity
                    # Log significant velocity changes for debugging
                    if abs(linear_vel) > 0.1 and abs(linear_vel - self.last_derived_linear) > 0.05:
                        self.get_logger().info(f"Ground truth velocity: {linear_vel:.3f} m/s ({dx:.3f}, {dy:.3f}) / {dt:.3f}s")
                    
                    self.last_derived_linear = linear_vel
                    self.last_derived_angular = angular_vel
            
            # Add data to time series
            self.timestamps.append(current_time)
            self.derived_velocity['linear'].append(self.last_derived_linear)
            self.derived_velocity['angular'].append(self.last_derived_angular)
            
            # Add corresponding entries for other data types
            self.cmd_vel_data['linear_x'].append(self.last_cmd_linear)
            self.cmd_vel_data['angular_z'].append(self.last_cmd_angular)
            self.odom_data['linear_x'].append(self.last_odom_linear)
            self.odom_data['angular_z'].append(self.last_odom_angular)
            
            # Calculate slip metric using odometry since ground truth is not reliable
            if abs(self.last_cmd_linear) > 0.05:  # Only calculate slip when commanded velocity is significant
                slip = abs(self.last_cmd_linear - self.last_odom_linear) / max(0.1, abs(self.last_cmd_linear))
                self.last_slip_value = slip
                
                # Log slip detection
                if slip > self.slip_threshold:
                    self.get_logger().warn(f"Slip detected! Metric: {slip:.2f}")
            
            self.slip_metric['value'].append(self.last_slip_value)
            
            # Trim data if needed
            self._trim_data()
    
    def update_data_timer(self):
        """Timer callback to regularly update all data"""
        current_time = time.time() - self.start_time
        
        with self.lock:
            # Only add data if we have some existing data
            if self.timestamps:
                self.timestamps.append(current_time)
                
                # Add all current values
                self.cmd_vel_data['linear_x'].append(self.last_cmd_linear)
                self.cmd_vel_data['angular_z'].append(self.last_cmd_angular)
                self.odom_data['linear_x'].append(self.last_odom_linear)
                self.odom_data['angular_z'].append(self.last_odom_angular)
                self.derived_velocity['linear'].append(self.last_derived_linear)
                self.derived_velocity['angular'].append(self.last_derived_angular)
                self.slip_metric['value'].append(self.last_slip_value)
                
                # Trim data if needed
                self._trim_data()
    
    def euler_from_quaternion(self, x, y, z, w):
        """Convert quaternion to Euler angles (roll, pitch, yaw)"""
        t0 = +2.0 * (w * x + y * z)
        t1 = +1.0 - 2.0 * (x * x + y * y)
        roll_x = math.atan2(t0, t1)
        
        t2 = +2.0 * (w * y - z * x)
        t2 = +1.0 if t2 > +1.0 else t2
        t2 = -1.0 if t2 < -1.0 else t2
        pitch_y = math.asin(t2)
        
        t3 = +2.0 * (w * z + x * y)
        t4 = +1.0 - 2.0 * (y * y + z * z)
        yaw_z = math.atan2(t3, t4)
        
        return roll_x, pitch_y, yaw_z
    
    def _trim_data(self):
        """Helper method to trim all data arrays to max_data_points"""
        if len(self.timestamps) > self.max_data_points:
            self.timestamps = self.timestamps[-self.max_data_points:]
            
            # Trim all data arrays
            self.cmd_vel_data['linear_x'] = self.cmd_vel_data['linear_x'][-self.max_data_points:]
            self.cmd_vel_data['angular_z'] = self.cmd_vel_data['angular_z'][-self.max_data_points:]
            self.odom_data['linear_x'] = self.odom_data['linear_x'][-self.max_data_points:]
            self.odom_data['angular_z'] = self.odom_data['angular_z'][-self.max_data_points:]
            self.derived_velocity['linear'] = self.derived_velocity['linear'][-self.max_data_points:]
            self.derived_velocity['angular'] = self.derived_velocity['angular'][-self.max_data_points:]
            self.slip_metric['value'] = self.slip_metric['value'][-self.max_data_points:]
        
        # Separately trim ground truth data
        if len(self.ground_truth_data['timestamps']) > self.max_data_points:
            self.ground_truth_data['timestamps'] = self.ground_truth_data['timestamps'][-self.max_data_points:]
            self.ground_truth_data['x'] = self.ground_truth_data['x'][-self.max_data_points:]
            self.ground_truth_data['y'] = self.ground_truth_data['y'][-self.max_data_points:]
            self.ground_truth_data['theta'] = self.ground_truth_data['theta'][-self.max_data_points:]
    
    def print_status(self):
        """Print status information"""
        with self.lock:
            self.get_logger().info(f"Messages: cmd_vel={self.cmd_vel_count}, odom={self.odom_count}, "
                                 f"tf={self.tf_count}, robot_tf={self.robot_tf_count}")
            
            if not self.timestamps:
                self.get_logger().info("Waiting for data...")
                return
            
            cmd_lin = self.last_cmd_linear
            actual_lin = self.last_derived_linear
            odom_lin = self.last_odom_linear
            slip = self.last_slip_value
            
            x = self.last_ground_truth_pos['x']
            y = self.last_ground_truth_pos['y']
            
            self.get_logger().info(f"Robot position: ({x:.2f}, {y:.2f})")
            self.get_logger().info(f"Velocities [m/s]: Cmd={cmd_lin:.2f}, Actual={actual_lin:.2f}, Odom={odom_lin:.2f}")
            self.get_logger().info(f"Slip metric: {slip:.2f} (threshold: {self.slip_threshold})")
            
            if slip > self.slip_threshold:
                self.get_logger().warn("SLIP DETECTED!")
    
    def run_plot(self):
        """Run the real-time plotting"""
        try:
            # Create figure
            plt.ion()  # Interactive mode
            fig, axs = plt.subplots(3, 1, figsize=(10, 10))
            fig.suptitle('Robot Slip Detection')
            
            # Initialize lines for velocity plot
            cmd_vel_line, = axs[0].plot([], [], 'b-', label='Commanded Velocity')
            actual_vel_line, = axs[0].plot([], [], 'g-', label='Actual Velocity (Ground Truth)')
            odom_vel_line, = axs[0].plot([], [], 'r--', label='Odometry Velocity')
            
            # Initialize lines for position plot
            gt_line, = axs[1].plot([], [], 'b-', linewidth=2, label='Robot Path')
            current_pos, = axs[1].plot([], [], 'ro', markersize=8, label='Current Position')
            
            # Initialize lines for slip metric plot
            slip_line, = axs[2].plot([], [], 'r-', label='Slip Metric')
            threshold_line, = axs[2].plot([], [], 'k--', label=f'Threshold ({self.slip_threshold})')
            
            # Set labels and titles
            axs[0].set_title('Velocity Comparison')
            axs[0].set_ylabel('Linear Velocity (m/s)')
            axs[0].set_xlabel('Time (s)')
            axs[0].grid(True)
            axs[0].legend()
            
            axs[1].set_title('Robot Position (X-Y)')
            axs[1].set_ylabel('Y Position (m)')
            axs[1].set_xlabel('X Position (m)')
            axs[1].grid(True)
            axs[1].legend()
            axs[1].set_aspect('equal')  # Equal aspect ratio for position plot
            
            axs[2].set_title('Slip Metric')
            axs[2].set_ylabel('Slip Metric')
            axs[2].set_xlabel('Time (s)')
            axs[2].grid(True)
            axs[2].legend()
            
            plt.tight_layout()
            
            # Update in real-time
            update_interval = 0.05  # Update every 50ms
            last_update_time = time.time()
            
            # Update plot function
            while plt.fignum_exists(fig.number):
                try:
                    current_time = time.time()
                    
                    # Control update rate
                    if current_time - last_update_time < update_interval:
                        time.sleep(0.01)
                        continue
                    
                    last_update_time = current_time
                    
                    with self.lock:
                        if not self.timestamps:
                            time.sleep(0.1)
                            continue
                        
                        # Make copies to avoid threading issues
                        timestamps = self.timestamps.copy()
                        cmd_linear = self.cmd_vel_data['linear_x'].copy()
                        odom_linear = self.odom_data['linear_x'].copy()
                        actual_linear = self.derived_velocity['linear'].copy()
                        slip_values = self.slip_metric['value'].copy()
                        
                        # Ground truth position data
                        gt_x = self.ground_truth_data['x'].copy()
                        gt_y = self.ground_truth_data['y'].copy()
                        
                        # Current position
                        current_x = self.last_ground_truth_pos['x']
                        current_y = self.last_ground_truth_pos['y']
                    
                    # Update velocity lines
                    cmd_vel_line.set_data(timestamps, cmd_linear)
                    odom_vel_line.set_data(timestamps, odom_linear)
                    actual_vel_line.set_data(timestamps, actual_linear)
                    
                    # Update position lines
                    gt_line.set_data(gt_x, gt_y)
                    current_pos.set_data([current_x], [current_y])
                    
                    # Update slip metric line
                    slip_line.set_data(timestamps, slip_values)
                    
                    # Update threshold line
                    if len(timestamps) > 0:
                        threshold_line.set_data([min(timestamps), max(timestamps)], 
                                               [self.slip_threshold, self.slip_threshold])
                    
                    # Update axis limits for time-based plots
                    if len(timestamps) > 2:
                        t_max = timestamps[-1]
                        t_min = max(timestamps[0], t_max - 30)  # Show last 30 seconds
                        
                        axs[0].set_xlim(t_min, t_max)
                        axs[2].set_xlim(t_min, t_max)
                    
                    # Update axis limits for position plot
                    if len(gt_x) >= 2 and len(gt_y) >= 2:
                        # Calculate the range of position data
                        x_center = sum(gt_x) / len(gt_x)
                        y_center = sum(gt_y) / len(gt_y)
                        
                        # Make sure we have a sufficient viewing area
                        view_range = 5.0
                        
                        # Center on average position with fixed range
                        axs[1].set_xlim(x_center - view_range/2, x_center + view_range/2)
                        axs[1].set_ylim(y_center - view_range/2, y_center + view_range/2)
                    
                    # Dynamic y-axis for velocity plot
                    if cmd_linear and odom_linear:
                        all_values = cmd_linear + odom_linear + actual_linear
                        if all_values:
                            max_val = max(max(abs(val) for val in cmd_linear), 
                                         max(abs(val) for val in odom_linear))
                            y_max = max(1.0, max_val * 1.2)
                            axs[0].set_ylim(-y_max * 0.2, y_max)
                    
                    # Fixed y-axis for slip metric
                    axs[2].set_ylim(0, max(1.0, self.slip_threshold * 3))
                    
                    # Update the figure
                    fig.canvas.draw_idle()
                    fig.canvas.flush_events()
                    
                    # Check for slip and update plot title
                    if slip_values and any(val > self.slip_threshold for val in slip_values[-5:]):
                        # Get the most recent slip value
                        recent_slip = slip_values[-1] if slip_values else 0
                        axs[2].set_title(f'Slip Metric - SLIP DETECTED! ({recent_slip:.2f})')
                    else:
                        axs[2].set_title('Slip Metric')
                    
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
    
    slip_detector = SlipDetector()
    
    # Use a multi-threaded executor for the ROS node
    executor = MultiThreadedExecutor()
    executor.add_node(slip_detector)
    
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        slip_detector.destroy_node()
        rclpy.shutdown()

if __name__ == '__main__':
    main()