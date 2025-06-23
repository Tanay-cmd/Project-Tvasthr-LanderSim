#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/Link.hh>
#include <gz/sim/components/WorldPose.hh>
#include <gz/plugin/Register.hh>
#include <ignition/math/Vector3.hh>

namespace moon_sim
{
  class SimpleThrusterPlugin : public gz::sim::System,
                               public gz::sim::ISystemConfigure,
                               public gz::sim::ISystemPreUpdate
  {
    public: void Configure(const gz::sim::Entity &_entity,
                           const std::shared_ptr<const sdf::Element> &_sdf,
                           gz::sim::EntityComponentManager &_ecm,
                           gz::sim::EventManager &) override
    {
      this->model = gz::sim::Model(_entity);
      std::string linkName = "lander_link";
      if (_sdf->HasElement("link_name"))
        linkName = _sdf->Get<std::string>("link_name");

      this->link = this->model.LinkByName(_ecm, linkName);
      if (!this->link)
        std::cerr << "❌ Could not find link: " << linkName << std::endl;

      if (_sdf->HasElement("force"))
        this->thrust = _sdf->Get<double>("force");

      // Clamp the force
      if (this->thrust > 15500)
        this->thrust = 15500;
      if (this->thrust < 0)
        this->thrust = 0;
    }

    public: void PreUpdate(const gz::sim::UpdateInfo &,
                           gz::sim::EntityComponentManager &_ecm) override
    {
      if (this->link)
      {
        ignition::math::Vector3d forceVec(0, 0, this->thrust);
        gz::sim::Link(this->link).AddForce(_ecm, forceVec);
      }
    }

    private: double thrust = 0;
    private: gz::sim::Model model{gz::sim::kNullEntity};
    private: gz::sim::Entity link{gz::sim::kNullEntity};
  };
}

GZ_ADD_PLUGIN(moon_sim::SimpleThrusterPlugin,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)
