#include "panda_anti_gravity_plugin.hpp"

#include <gz/math/Vector3.hh>
#include <ignition/gazebo/Link.hh>
#include <ignition/gazebo/Model.hh>

#include <ignition/plugin/Register.hh>

namespace franka_gazebo {

void PandaAntiGravityPlugin::Configure(const ignition::gazebo::Entity& _entity,
                                       const std::shared_ptr<const sdf::Element>& _sdf,
                                       ignition::gazebo::EntityComponentManager& _ecm,
                                       ignition::gazebo::EventManager& /*_eventMgr*/) {
  // Get robot's model to later acess it's joints
  model_ = ignition::gazebo::Model(_entity);
  if (!model_.Valid(_ecm)) {
    ignerr << "[PandaAntiGravityPlugin] Please make sure that "
           << "the plugin is attached to a valid model!" << std::endl;
    return;
  }

  // Parse gravity parameters
  double gravity_x = 0.0;
  double gravity_y = 0.0;
  double gravity_z = 9.8;
  if (_sdf->HasElement("gravity_x")) {
    gravity_x = _sdf->Get<double>("gravity_x");
  }
  if (_sdf->HasElement("gravity_y")) {
    gravity_y = _sdf->Get<double>("gravity_y");
  }
  if (_sdf->HasElement("gravity_z")) {
    gravity_z = _sdf->Get<double>("gravity_z");
  }

  gravity_vect_ = gz::math::Vector3d(gravity_x, gravity_y, gravity_z);

  // Enable velocity check for each joint
  // This is a walkaraound to enable force removal
  // See: https://robotics.stackexchange.com/a/103562
  for (const auto& link_entity : model_.Links(_ecm)) {
    gz::sim::Link link(link_entity);
    if (!link.Valid(_ecm)) {
      ignerr << "[PandaAntiGravityPlugin] Invalid link entity: " << link_entity << std::endl;
      return;
    } else {
      link.EnableVelocityChecks(_ecm);
    }
  }
  ignmsg << "[PandaAntiGravityPlugin] Configured correctly. "
         << "Removing gravity of: " << gravity_vect_ << std::endl;
}

void PandaAntiGravityPlugin::PreUpdate(const gz::sim::UpdateInfo& _info,
                                       gz::sim::EntityComponentManager& _ecm) {
  if (_info.paused) {
    return;
  }

  // Apply the force on every simulation step
  for (const auto& link_entity : model_.Links(_ecm)) {
    gz::sim::Link link(link_entity);
    if (!link.Valid(_ecm)) {
      ignerr << "[PandaAntiGravityPlugin] Invalid link entity: " << link_entity << std::endl;
    } else {
      link.AddWorldForce(_ecm, gravity_vect_);
    }
  }
}
};  // namespace franka_gazebo

// Register the plugin
IGNITION_ADD_PLUGIN(franka_gazebo::PandaAntiGravityPlugin,
                    ignition::gazebo::System,
                    franka_gazebo::PandaAntiGravityPlugin::ISystemConfigure,
                    ignition::gazebo::ISystemPreUpdate)