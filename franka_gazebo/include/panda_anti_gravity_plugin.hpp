#ifndef PANDA_ANTI_GRAVITY_HPP_
#define PANDA_ANTI_GRAVITY_HPP_

#include <ignition/gazebo/System.hh>

namespace franka_gazebo {

class PandaAntiGravityPlugin : public ignition::gazebo::System,
                               public ignition::gazebo::ISystemConfigure {
 public:
  void Configure(const ignition::gazebo::Entity& _id,
                 const std::shared_ptr<const sdf::Element>& _sdf,
                 ignition::gazebo::EntityComponentManager& _ecm,
                 ignition::gazebo::EventManager& /*_eventMgr*/) final;
};
}  // namespace franka_gazebo

#endif  // PANDA_ANTI_GRAVITY_HPP_