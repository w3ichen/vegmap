#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/plugin/Register.hh>
#include <iostream>

namespace gazebo
{
  class FluidResistancePlugin : public WorldPlugin
  {
    // Private variables
    private: physics::WorldPtr world;
    private: event::ConnectionPtr updateConnection;
    private: double resistanceCoef;
    private: double min_x, max_x;
    private: double min_y, max_y;
    private: double min_z, max_z;
    private: common::Time lastDebugTime;
    
    public: void Load(physics::WorldPtr _world, sdf::ElementPtr _sdf)
    {
      std::cout << "\033[1;32m[FluidResistancePlugin] LOADING PLUGIN\033[0m" << std::endl;
      
      // Store the world pointer
      this->world = _world;
      
      // Default values
      this->resistanceCoef = 0.5;
      this->min_x = -5.0;
      this->max_x = 5.0;
      this->min_y = -5.0;
      this->max_y = 5.0;
      this->min_z = 0.0;
      this->max_z = 2.0;
      
      // Get resistance coefficient if available
      if (_sdf->HasElement("resistance_coefficient"))
        this->resistanceCoef = _sdf->GetElement("resistance_coefficient")->Get<double>();
      
      // Parse region dimensions if specified
      if (_sdf->HasElement("region"))
      {
        auto region = _sdf->GetElement("region");
        
        if (region->HasElement("min_x"))
          this->min_x = region->GetElement("min_x")->Get<double>();
        if (region->HasElement("max_x"))
          this->max_x = region->GetElement("max_x")->Get<double>();
        if (region->HasElement("min_y"))
          this->min_y = region->GetElement("min_y")->Get<double>();
        if (region->HasElement("max_y"))
          this->max_y = region->GetElement("max_y")->Get<double>();
        if (region->HasElement("min_z"))
          this->min_z = region->GetElement("min_z")->Get<double>();
        if (region->HasElement("max_z"))
          this->max_z = region->GetElement("max_z")->Get<double>();
      }
      
      // Initialize debug timer
      this->lastDebugTime = this->world->SimTime();
      
      // Setup update callback
      this->updateConnection = event::Events::ConnectWorldUpdateBegin(
          std::bind(&FluidResistancePlugin::OnUpdate, this));
      
      std::cout << "\033[1;32m[FluidResistancePlugin] Fluid Resistance Plugin loaded for world: " 
                << this->world->Name() << "\033[0m" << std::endl;
      std::cout << "\033[1;32m[FluidResistancePlugin] Region: ["
                << this->min_x << "," << this->max_x << "] x ["
                << this->min_y << "," << this->max_y << "] x ["
                << this->min_z << "," << this->max_z << "]\033[0m" << std::endl;
      std::cout << "\033[1;32m[FluidResistancePlugin] Resistance coefficient: " 
                << this->resistanceCoef << "\033[0m" << std::endl;
    }

    // Check if a point is inside the resistance zone
    private: bool IsInZone(const ignition::math::Vector3d& pos)
    {
      return (pos.X() >= this->min_x && pos.X() <= this->max_x &&
              pos.Y() >= this->min_y && pos.Y() <= this->max_y &&
              pos.Z() >= this->min_z && pos.Z() <= this->max_z);
    }

    public: void OnUpdate()
    {
      // Get all models
      auto models = this->world->Models();
      bool foundAnyRobot = false;
      int modelsInZone = 0;
      
      // Current simulation time
      common::Time currentTime = this->world->SimTime();
      
      // Debug output every second
      bool debugOutput = (currentTime - this->lastDebugTime).Double() >= 1.0;
      if (debugOutput) {
        this->lastDebugTime = currentTime;
      }
      
      for (const auto& model : models)
      {
        // Skip static models and the zone visualization
        if (model->IsStatic() || model->GetName() == "resistance_zone_visual" || 
            model->GetName() == "ground_plane")
          continue;
        
        bool modelInZone = false;
        std::string robotName = model->GetName();
        
        // Get all links in the model
        auto links = model->GetLinks();
        
        if (debugOutput) {
          std::cout << "\033[1;34m[FluidResistancePlugin] Checking model: " << robotName 
                    << " with " << links.size() << " links\033[0m" << std::endl;
        }
        
        for (const auto& link : links)
        {
          // Get link position
          auto pos = link->WorldPose().Pos();
          
          // Check if link is in the resistance zone
          if (IsInZone(pos))
          {
            modelInZone = true;
            foundAnyRobot = true;
            
            // Get the link's velocity
            auto vel = link->WorldLinearVel();
            
            // Skip if velocity is very small
            if (vel.Length() < 0.01)
              continue;
            
            // Calculate resistance force (opposite to velocity direction)
            ignition::math::Vector3d force = -this->resistanceCoef * vel;
            
            // Apply the force to the link in world frame
            link->AddForce(force);
            
            // Debug output
            if (debugOutput) {
              std::cout << "\033[1;31m[FluidResistancePlugin] Model '" << model->GetName() 
                        << "' link '" << link->GetName() << "' in resistance zone\033[0m" << std::endl;
              std::cout << "\033[1;31m[FluidResistancePlugin] Link position: " << pos 
                        << "\033[0m" << std::endl;
              std::cout << "\033[1;31m[FluidResistancePlugin] Velocity: " << vel 
                        << " m/s\033[0m" << std::endl;
              std::cout << "\033[1;31m[FluidResistancePlugin] Applied force: " << force 
                        << " N\033[0m" << std::endl;
            }
          }
        }
        
        if (modelInZone) {
          modelsInZone++;
        }
      }
      
      // Print status message once per second
      if (debugOutput) {
        if (!foundAnyRobot) {
          std::cout << "\033[1;33m[FluidResistancePlugin] No robots found in resistance zone\033[0m" << std::endl;
        } else {
          std::cout << "\033[1;32m[FluidResistancePlugin] " << modelsInZone 
                    << " robots in resistance zone\033[0m" << std::endl;
        }
      }
    }
  };

  // Register this plugin with the simulator
  GZ_REGISTER_WORLD_PLUGIN(FluidResistancePlugin)
}

// Register this plugin with Ignition
IGNITION_ADD_PLUGIN(
  gazebo::FluidResistancePlugin,
  gazebo::WorldPlugin
)

// Register this plugin with Ignition
IGNITION_ADD_PLUGIN(
  gazebo::FluidResistancePlugin,
  gazebo::WorldPlugin
)