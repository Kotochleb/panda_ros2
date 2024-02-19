#ifndef PANDA_ANTI_GRAVITY_HPP_
#define PANDA_ANTI_GRAVITY_HPP_

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/System.hh>

#include <ignition/gazebo/World.hh>

#include <gz/math/Vector3.hh>

namespace franka_gazebo {

class PandaAntiGravityPlugin : public ignition::gazebo::System,
                               public ignition::gazebo::ISystemConfigure,
                               public ignition::gazebo::ISystemPreUpdate {
 public:
  void Configure(const ignition::gazebo::Entity& _id,
                 const std::shared_ptr<const sdf::Element>& _sdf,
                 ignition::gazebo::EntityComponentManager& _ecm,
                 ignition::gazebo::EventManager& /*_eventMgr*/) final;

  void PreUpdate(const gz::sim::UpdateInfo& _info, gz::sim::EntityComponentManager& _ecm) final;

 private:
  ignition::gazebo::Model model_;
  gz::math::Vector3d gravity_vect_;
};
}  // namespace franka_gazebo

#endif  // PANDA_ANTI_GRAVITY_HPP_