#include "panda_anti_gravity_plugin.hpp"

#include <ignition/gazebo/Model.hh>
#include <ignition/gazebo/components/Gravity.hh>
#include <ignition/plugin/Register.hh>

namespace franka_gazebo {

void PandaAntiGravityPlugin::Configure(const ignition::gazebo::Entity& _entity,
                                       const std::shared_ptr<const sdf::Element>& _sdf,
                                       ignition::gazebo::EntityComponentManager& _ecm,
                                       ignition::gazebo::EventManager& /*_eventMgr*/) {
  ignmsg << "[PandaAntiGravityPlugin] plugin attached to entity: " << _entity << std::endl;

  ignition::gazebo::Model model(_entity);
  if (!model.Valid(_ecm)) {
    ignerr << "[PandaAntiGravityPlugin] Please make sure that "
           << "the plugin is attached to a valid model!" << std::endl;
    return;
  }
  ignerr << "[PandaAntiGravityPlugin] link number: " << model.LinkCount(_ecm) << std::endl;
  for (const auto& link_entity : model.Links(_ecm)) {
    ignerr << "[PandaAntiGravityPlugin] link name: " << link_entity << std::endl;
  }

  _ecm.CreateComponent<ignition::gazebo::components::Gravity>(
      _entity, ignition::gazebo::components::Gravity());

  // const components::Gravity *gravity = _ecm.Component<components::Gravity>(this->dataPtr->world);
  const auto* gravity = _ecm.Component<ignition::gazebo::components::Gravity>(_entity);

  ignmsg << "[PandaAntiGravityPlugin]" << std::endl
         << std::endl
         << std::endl
         << std::endl
         << std::endl
         << std::endl
         << std::endl
         << std::endl
         << std::endl;

  if (nullptr == gravity) {
    ignmsg << "gravity is a null pointer" << std::endl;
  } else {
    ignmsg << "[PandaAntiGravityPlugin]" << gravity->Data().X() << gravity->Data().Y()
           << gravity->Data().Z() << std::endl;
  }
}
};  // namespace franka_gazebo

IGNITION_ADD_PLUGIN(franka_gazebo::PandaAntiGravityPlugin,
                    ignition::gazebo::System,
                    franka_gazebo::PandaAntiGravityPlugin::ISystemConfigure)