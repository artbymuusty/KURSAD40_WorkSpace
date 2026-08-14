#include <atomic>
#include <mutex>
#include <string>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>

#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>

#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/DetachableJoint.hh>

namespace hook_attach
{

// Same runtime-joint mechanism as gz-sim's own DetachableJoint system (an
// entity carrying only a components::DetachableJoint is materialized into a
// real fixed joint by the physics system -- confirmed against the gz-sim8
// DetachableJoint.cc source). The one deliberate behavioral difference: the
// stock system's `attachRequested` flag defaults to true and is never reset
// by Configure() or by detaching, so it auto-attaches the instant its
// configured child model becomes resolvable -- fine for a child that is
// spawned already touching its parent, wrong for a payload that is dropped
// long before the drone is meant to pick it up. Here attach only ever
// becomes true in response to an explicit attach message (sent by
// payload.py only after hook_sensor_service.py reports a genuine contact),
// and the target child model name comes from that message instead of being
// fixed in SDF, since the dropped payload's spawned name isn't known until
// PayloadDropSystem actually drops it.
class HookAttachSystem :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &_entity,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &,
                 gz::sim::EventManager &) override
  {
    ownModelEntity_ = _entity;

    if (_sdf && _sdf->HasElement("parent_link"))
      parentLinkName_ = _sdf->Get<std::string>("parent_link");
    if (_sdf && _sdf->HasElement("child_link"))
      childLinkName_ = _sdf->Get<std::string>("child_link");
    if (_sdf && _sdf->HasElement("attach_topic"))
      attachTopic_ = _sdf->Get<std::string>("attach_topic");
    if (_sdf && _sdf->HasElement("detach_topic"))
      detachTopic_ = _sdf->Get<std::string>("detach_topic");
    if (_sdf && _sdf->HasElement("output_topic"))
      outputTopic_ = _sdf->Get<std::string>("output_topic");

    const bool subAttachOk = node_.Subscribe(attachTopic_, &HookAttachSystem::OnAttachRequest, this);
    const bool subDetachOk = node_.Subscribe(detachTopic_, &HookAttachSystem::OnDetachRequest, this);
    statePub_ = node_.Advertise<gz::msgs::Boolean>(outputTopic_);

    gzwarn << "[HookAttach] LOADED"
           << " subAttachOk=" << (subAttachOk ? "true" : "false")
           << " subDetachOk=" << (subDetachOk ? "true" : "false")
           << " parent_link=" << parentLinkName_
           << " child_link=" << childLinkName_
           << " attach_topic=" << attachTopic_
           << " detach_topic=" << detachTopic_
           << " output_topic=" << outputTopic_
           << "\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    if (parentLinkEntity_ == gz::sim::kNullEntity)
      parentLinkEntity_ = FindLinkInModel(_ecm, ownModelEntity_, parentLinkName_);

    if (detachRequested_)
    {
      detachRequested_ = false;
      if (isAttached_)
      {
        _ecm.RequestRemoveEntity(jointEntity_);
        jointEntity_ = gz::sim::kNullEntity;
        childModelEntity_ = gz::sim::kNullEntity;
        childLinkEntity_ = gz::sim::kNullEntity;
        isAttached_ = false;
        PublishState(false);
        gzwarn << "[HookAttach] DETACHED\n";
      }
      else
      {
        gzwarn << "[HookAttach] Detach requested but nothing is attached; ignoring\n";
      }
    }

    if (!attachRequested_ || isAttached_)
      return;

    if (parentLinkEntity_ == gz::sim::kNullEntity)
      return; // retry next tick

    std::string childName;
    {
      std::lock_guard<std::mutex> lock(pendingMutex_);
      childName = pendingChildModelName_;
    }

    if (childModelEntity_ == gz::sim::kNullEntity)
      childModelEntity_ = FindModel(_ecm, childName);
    if (childModelEntity_ != gz::sim::kNullEntity && childLinkEntity_ == gz::sim::kNullEntity)
      childLinkEntity_ = FindLinkInModel(_ecm, childModelEntity_, childLinkName_);

    if (childModelEntity_ == gz::sim::kNullEntity || childLinkEntity_ == gz::sim::kNullEntity)
      return; // child not spawned/resolvable yet -- keep retrying, same as stock DetachableJoint

    jointEntity_ = _ecm.CreateEntity();
    _ecm.CreateComponent(jointEntity_,
        gz::sim::components::DetachableJoint({parentLinkEntity_, childLinkEntity_, "fixed"}));
    isAttached_ = true;
    attachRequested_ = false;
    PublishState(true);
    gzwarn << "[HookAttach] ATTACHED child_model=" << childName
           << " joint_entity=" << jointEntity_ << "\n";
  }

private:
  void OnAttachRequest(const gz::msgs::StringMsg &_msg)
  {
    if (isAttached_)
    {
      gzwarn << "[HookAttach] Already attached; ignoring attach request for " << _msg.data() << "\n";
      return;
    }
    {
      std::lock_guard<std::mutex> lock(pendingMutex_);
      pendingChildModelName_ = _msg.data();
    }
    attachRequested_ = true;
    gzwarn << "[HookAttach] Attach requested for child_model=" << _msg.data() << "\n";
  }

  void OnDetachRequest(const gz::msgs::Boolean &_msg)
  {
    if (_msg.data())
      detachRequested_ = true;
  }

  void PublishState(bool _attached)
  {
    gz::msgs::Boolean msg;
    msg.set_data(_attached);
    statePub_.Publish(msg);
  }

  gz::sim::Entity FindModel(gz::sim::EntityComponentManager &_ecm, const std::string &name)
  {
    gz::sim::Entity out = gz::sim::kNullEntity;
    _ecm.Each<gz::sim::components::Name, gz::sim::components::Model>(
      [&](const gz::sim::Entity &e,
          const gz::sim::components::Name *n,
          const gz::sim::components::Model *) -> bool
      {
        if (n && n->Data() == name) { out = e; return false; }
        return true;
      });
    return out;
  }

  gz::sim::Entity FindLinkInModel(gz::sim::EntityComponentManager &_ecm,
                                   gz::sim::Entity modelEnt,
                                   const std::string &linkName)
  {
    gz::sim::Entity out = gz::sim::kNullEntity;
    _ecm.Each<gz::sim::components::Name,
              gz::sim::components::Link,
              gz::sim::components::ParentEntity>(
      [&](const gz::sim::Entity &e,
          const gz::sim::components::Name *n,
          const gz::sim::components::Link *,
          const gz::sim::components::ParentEntity *p) -> bool
      {
        if (!n || !p) return true;
        if (p->Data() != modelEnt) return true;

        const std::string &nn = n->Data();
        const bool exact = (nn == linkName);
        const bool suff =
          (nn.size() >= (linkName.size() + 2)) &&
          (nn.rfind("::" + linkName) == (nn.size() - (2 + linkName.size())));

        if (exact || suff) { out = e; return false; }
        return true;
      });
    return out;
  }

private:
  gz::transport::Node node_;
  gz::transport::Node::Publisher statePub_;

  std::string parentLinkName_{"hook_rope_link"};
  std::string childLinkName_{"link"};
  std::string attachTopic_{"/hook/attach"};
  std::string detachTopic_{"/hook/detach"};
  std::string outputTopic_{"/hook/state"};

  gz::sim::Entity ownModelEntity_{gz::sim::kNullEntity};
  gz::sim::Entity parentLinkEntity_{gz::sim::kNullEntity};
  gz::sim::Entity childModelEntity_{gz::sim::kNullEntity};
  gz::sim::Entity childLinkEntity_{gz::sim::kNullEntity};
  gz::sim::Entity jointEntity_{gz::sim::kNullEntity};

  std::mutex pendingMutex_;
  std::string pendingChildModelName_;

  std::atomic<bool> attachRequested_{false};
  std::atomic<bool> detachRequested_{false};
  bool isAttached_{false};
};

} // namespace hook_attach

GZ_ADD_PLUGIN(hook_attach::HookAttachSystem,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)
