#include <string>
#include <atomic>

#include <gz/msgs/boolean.pb.h>
#include <gz/transport/Node.hh>
#include <gz/sim/System.hh>
#include <gz/sim/Model.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/components/Joint.hh>
#include <gz/sim/components/Name.hh>

namespace payload
{
class PayloadDropSystem:
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &_ecm,
                 gz::sim::EventManager &) override
  {
    model_ = gz::sim::Model(_entity);

    if (_sdf && _sdf->HasElement("topic"))
      topic_ = _sdf->Get<std::string>("topic");
    if (_sdf && _sdf->HasElement("joint_name"))
      jointName_ = _sdf->Get<std::string>("joint_name");

    node_.Subscribe(topic_, &PayloadDropSystem::OnDrop, this);

    jointEntity_ = FindJointEntity(_ecm, jointName_);
  }

  void PreUpdate(const gz::sim::UpdateInfo &,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    if (dropped_) return;

    if (jointEntity_ == gz::sim::kNullEntity)
      jointEntity_ = FindJointEntity(_ecm, jointName_);

    if (dropRequested_ && jointEntity_ != gz::sim::kNullEntity)
    {
      _ecm.RemoveComponent<gz::sim::components::Joint>(jointEntity_);
      dropped_ = true;
    }
  }

private:
  void OnDrop(const gz::msgs::Boolean &_msg)
  {
    if (_msg.data())
      dropRequested_ = true;
  }

  gz::sim::Entity FindJointEntity(gz::sim::EntityComponentManager &_ecm,
                                  const std::string &name)
  {
    gz::sim::Entity found = gz::sim::kNullEntity;
    _ecm.Each<gz::sim::components::Name, gz::sim::components::Joint>(
      [&](const gz::sim::Entity &e,
          const gz::sim::components::Name *n,
          const gz::sim::components::Joint *) -> bool
      {
        if (n && n->Data() == name)
        {
          found = e;
          return false;
        }
        return true;
      });
    return found;
  }

  gz::transport::Node node_;
  gz::sim::Model model_{gz::sim::kNullEntity};

  std::string topic_{"/payload_drop"};
  std::string jointName_{"payload_fixed_joint"};

  std::atomic<bool> dropRequested_{false};
  bool dropped_{false};
  gz::sim::Entity jointEntity_{gz::sim::kNullEntity};
};
} // namespace payload

GZ_ADD_PLUGIN(payload::PayloadDropSystem,
              gz::sim::System,
              payload::PayloadDropSystem::ISystemConfigure,
              payload::PayloadDropSystem::ISystemPreUpdate)

GZ_ADD_PLUGIN_ALIAS(payload::PayloadDropSystem, "payload::PayloadDropSystem")
