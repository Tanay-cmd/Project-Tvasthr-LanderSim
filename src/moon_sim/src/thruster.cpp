// ConstantForcePlugin.cpp
#include <gz/plugin/Register.hh>
#include <gz/sim/System.hh>
#include <gz/sim/components/ExternalWorldWrenchCmd.hh>
#include <gz/sim/components/World.hh>
#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Pose.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/msgs/wrench.pb.h>

namespace sim = gz::sim;

class ConstantForcePlugin : public sim::System,
                            public sim::ISystemPreUpdate
{
public:
  void Configure(const sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &,
                 sim::EntityComponentManager &_ecm,
                 sim::EventManager &) override
  {
    model = sim::Model(_entity);
    linkEntity = model.LinkByName(_ecm, "lander_link");  // <- Your link name
  }

  void PreUpdate(const sim::UpdateInfo &,
                 sim::EntityComponentManager &_ecm) override
  {
    if (linkEntity == sim::kNullEntity)
      return;

    const ignition::math::Vector3d force(0, 0, 20);  // Example force
    const ignition::math::Vector3d applicationPoint(0, 0, 0);  // World coords

    gz::msgs::Wrench wrenchMsg;
    wrenchMsg.mutable_force()->set_x(force.X());
    wrenchMsg.mutable_force()->set_y(force.Y());
    wrenchMsg.mutable_force()->set_z(force.Z());

    // Set torque = r × F if needed
    _ecm.SetComponentData<sim::components::ExternalWorldWrenchCmd>(
        linkEntity,
        sim::components::ExternalWorldWrenchCmd(wrenchMsg));
  }

private:
  sim::Model model;
  sim::Entity linkEntity;
};

GZ_ADD_PLUGIN(ConstantForcePlugin,
              sim::System,
              sim::ISystemPreUpdate)
