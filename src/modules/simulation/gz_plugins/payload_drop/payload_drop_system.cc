#include <string>
#include <atomic>
#include <sstream>
#include <iomanip>

#include <gz/common/Console.hh>
#include <gz/plugin/Register.hh>

#include <gz/msgs/boolean.pb.h>
#include <gz/msgs/entity_factory.pb.h>
#include <gz/msgs/stringmsg.pb.h>
#include <gz/transport/Node.hh>

#include <gz/sim/System.hh>
#include <gz/sim/EntityComponentManager.hh>
#include <gz/sim/Util.hh>

#include <gz/sim/components/Name.hh>
#include <gz/sim/components/Model.hh>
#include <gz/sim/components/Link.hh>
#include <gz/sim/components/ParentEntity.hh>
#include <gz/sim/components/Visual.hh>
#include <gz/sim/components/Collision.hh>

namespace payload
{

class PayloadDropSystem :
  public gz::sim::System,
  public gz::sim::ISystemConfigure,
  public gz::sim::ISystemPreUpdate
{
public:
  void Configure(const gz::sim::Entity &,
                 const std::shared_ptr<const sdf::Element> &_sdf,
                 gz::sim::EntityComponentManager &,
                 gz::sim::EventManager &) override
  {
    if (_sdf && _sdf->HasElement("topic"))        topic_     = _sdf->Get<std::string>("topic");
    if (_sdf && _sdf->HasElement("world_name"))   worldName_ = _sdf->Get<std::string>("world_name");

    if (_sdf && _sdf->HasElement("uav_model"))    uavModel_  = _sdf->Get<std::string>("uav_model");
    if (_sdf && _sdf->HasElement("uav_link"))     uavLink_   = _sdf->Get<std::string>("uav_link");

    // RED
    if (_sdf && _sdf->HasElement("dummy_payload_link_red"))
      dummyRedLink_ = _sdf->Get<std::string>("dummy_payload_link_red");
    if (_sdf && _sdf->HasElement("offset_red_x")) rx_ = _sdf->Get<double>("offset_red_x");
    if (_sdf && _sdf->HasElement("offset_red_y")) ry_ = _sdf->Get<double>("offset_red_y");
    if (_sdf && _sdf->HasElement("offset_red_z")) rz_ = _sdf->Get<double>("offset_red_z");

    // BLUE
    if (_sdf && _sdf->HasElement("dummy_payload_link_blue"))
      dummyBlueLink_ = _sdf->Get<std::string>("dummy_payload_link_blue");
    if (_sdf && _sdf->HasElement("offset_blue_x")) bx_ = _sdf->Get<double>("offset_blue_x");
    if (_sdf && _sdf->HasElement("offset_blue_y")) by_ = _sdf->Get<double>("offset_blue_y");
    if (_sdf && _sdf->HasElement("offset_blue_z")) bz_ = _sdf->Get<double>("offset_blue_z");

    if (_sdf && _sdf->HasElement("spawn_name_prefix"))
      spawnNamePrefix_ = _sdf->Get<std::string>("spawn_name_prefix");

    // COLOR-ADDRESSED DROP (v31_3rd_mission): the legacy <topic> above only
    // ever carries a bare Boolean with no color -- this plugin's own
    // stage_ counter unilaterally decides RED-then-BLUE regardless of what
    // the mission side believes it's requesting (see OnDrop/stage_ below,
    // left completely unchanged for existing missions). color_topic is a
    // SEPARATE, additive subscription: a StringMsg of "red" or "blue"
    // drops exactly that payload, addressed by color instead of by a fixed
    // sequence, so a mission can choose BLUE first / RED second (or any
    // order) without touching the legacy path at all.
    if (_sdf && _sdf->HasElement("color_topic"))
      colorTopic_ = _sdf->Get<std::string>("color_topic");

    // RUNTIME DEBUG FIX (KURSAD40 second-payload investigation): this plugin
    // previously had NO way to tell payload.py whether a drop actually
    // spawned anything -- Python's "success" signal (see payload.py's
    // _gazebo_boolean_drop) only ever confirmed that `gz topic pub` itself
    // ran, not that this plugin received the message or that the
    // EntityFactory "create" call below actually succeeded. That gap is
    // exactly what let a genuinely failed spawn (e.g. BLUE) look identical
    // to a successful one from the mission's point of view. output_topic
    // mirrors the same pattern HookAttachSystem already uses for its own
    // attach/detach confirmation (see hook_attach/HookAttachSystem.cc).
    if (_sdf && _sdf->HasElement("output_topic"))
      outputTopic_ = _sdf->Get<std::string>("output_topic");

    // Physical shape params (optional)
    if (_sdf && _sdf->HasElement("radius")) radius_ = _sdf->Get<double>("radius");
    if (_sdf && _sdf->HasElement("length")) length_ = _sdf->Get<double>("length");
    if (_sdf && _sdf->HasElement("mass"))   mass_   = _sdf->Get<double>("mass");

    createSrv_ = "/world/" + worldName_ + "/create";
    const bool subOk = node_.Subscribe(topic_, &PayloadDropSystem::OnDrop, this);
    const bool colorSubOk = node_.Subscribe(colorTopic_, &PayloadDropSystem::OnDropColor, this);
    statePub_ = node_.Advertise<gz::msgs::StringMsg>(outputTopic_);

    gzwarn << "[PayloadDrop] LOADED (SDF-SPAWN)"
           << " subOk=" << (subOk ? "true" : "false")
           << " topic=" << topic_
           << " colorSubOk=" << (colorSubOk ? "true" : "false")
           << " color_topic=" << colorTopic_
           << " output_topic=" << outputTopic_
           << " world=" << worldName_
           << " createSrv=" << createSrv_
           << " uav=" << uavModel_ << "::" << uavLink_
           << " redDummy=" << dummyRedLink_
           << " blueDummy=" << dummyBlueLink_
           << " box(w=" << radius_*2.0 << " d=" << radius_*1.5 << " h=" << length_ << " m=" << mass_ << ")"
           << "\n";
  }

  void PreUpdate(const gz::sim::UpdateInfo &,
                 gz::sim::EntityComponentManager &_ecm) override
  {
    // Find entities once (needed by both the legacy stage_ path and the
    // color-addressed path below).
    if (uavLinkEnt_ == gz::sim::kNullEntity ||
        dummyRedEnt_ == gz::sim::kNullEntity ||
        dummyBlueEnt_ == gz::sim::kNullEntity)
    {
      const gz::sim::Entity uavModelEnt = FindModel(_ecm, uavModel_);
      if (uavModelEnt == gz::sim::kNullEntity) return;

      if (uavLinkEnt_ == gz::sim::kNullEntity) {
        uavLinkEnt_ = FindLinkInModel(_ecm, uavModelEnt, uavLink_);
        if (uavLinkEnt_ != gz::sim::kNullEntity)
          gzwarn << "[PayloadDrop] Found UAV link entity=" << uavLinkEnt_ << "\n";
      }

      if (dummyRedEnt_ == gz::sim::kNullEntity) {
        dummyRedEnt_ = FindLinkInModel(_ecm, uavModelEnt, dummyRedLink_);
        if (dummyRedEnt_ != gz::sim::kNullEntity)
          gzwarn << "[PayloadDrop] Found RED dummy link entity=" << dummyRedEnt_ << "\n";
      }

      if (dummyBlueEnt_ == gz::sim::kNullEntity) {
        dummyBlueEnt_ = FindLinkInModel(_ecm, uavModelEnt, dummyBlueLink_);
        if (dummyBlueEnt_ != gz::sim::kNullEntity)
          gzwarn << "[PayloadDrop] Found BLUE dummy link entity=" << dummyBlueEnt_ << "\n";
      }
    }

    // COLOR-ADDRESSED DROP: fully independent of the legacy stage_ counter
    // below -- tracked by its own colorRedDropped_/colorBlueDropped_ flags,
    // so a mission using this path can drop BLUE first / RED second (or
    // either alone) without disturbing stage_ for missions still using the
    // legacy Boolean topic.
    if (colorDropRequested_) {
      colorDropRequested_ = false;
      if (uavLinkEnt_ == gz::sim::kNullEntity) {
        gzwarn << "[PayloadDrop] COLOR-SELECT: UAV link not found yet, dropping request for '"
               << pendingColor_ << "'\n";
      } else if (pendingColor_ == "red") {
        if (colorRedDropped_) {
          gzwarn << "[PayloadDrop] COLOR-SELECT: RED already dropped, ignoring\n";
        } else {
          gzwarn << "[PayloadDrop] COLOR-SELECT: dropping RED (explicit request)\n";
          if (DoOneDrop(dummyRedEnt_, rx_, ry_, rz_, "red", 1.0, 0.0, 0.0, 1.0, _ecm))
            colorRedDropped_ = true;
        }
      } else if (pendingColor_ == "blue") {
        if (colorBlueDropped_) {
          gzwarn << "[PayloadDrop] COLOR-SELECT: BLUE already dropped, ignoring\n";
        } else {
          gzwarn << "[PayloadDrop] COLOR-SELECT: dropping BLUE (explicit request)\n";
          if (DoOneDrop(dummyBlueEnt_, bx_, by_, bz_, "blue", 0.0, 0.0, 1.0, 1.0, _ecm))
            colorBlueDropped_ = true;
        }
      } else {
        gzwarn << "[PayloadDrop] COLOR-SELECT: unknown color '" << pendingColor_ << "', ignoring\n";
      }
      return;
    }

    // --- Legacy Boolean-triggered path (unchanged) ---
    if (stage_ >= 2) {
      if (dropRequested_) {
        dropRequested_ = false;
        gzwarn << "[PayloadDrop] INFO: both payloads already dropped, ignoring"
               << " [internal_index=" << stage_ << "]\n";
      }
      return;
    }

    if (!dropRequested_) return;
    if (uavLinkEnt_ == gz::sim::kNullEntity) return;

    dropRequested_ = false;
    gzwarn << "[PayloadDrop] Request received. internal_index(stage)=" << stage_
           << " selected_payload=" << (stage_ == 0 ? "red" : "blue") << "\n";

    if (stage_ == 0) {
      gzwarn << "[PayloadDrop] Stage0: dropping RED (SDF) [internal_index=0]\n";
      const bool ok = DoOneDrop(dummyRedEnt_, rx_, ry_, rz_, "red", 1.0, 0.0, 0.0, 1.0, _ecm);
      if (ok) {
        stage_ = 1;
        gzwarn << "[PayloadDrop] Completion: RED spawn confirmed. Advancing internal_index 0 -> 1 (BLUE next)\n";
      } else {
        gzwarn << "[PayloadDrop] Completion: RED spawn FAILED. Staying at internal_index=0 -- "
               << "will retry RED on the next drop request instead of silently skipping to BLUE\n";
      }
      return;
    }

    if (stage_ == 1) {
      gzwarn << "[PayloadDrop] Stage1: dropping BLUE (SDF) [internal_index=1]\n";
      const bool ok = DoOneDrop(dummyBlueEnt_, bx_, by_, bz_, "blue", 0.0, 0.0, 1.0, 1.0, _ecm);
      if (ok) {
        stage_ = 2;
        gzwarn << "[PayloadDrop] Completion: BLUE spawn confirmed. Advancing internal_index 1 -> 2 (all payloads dropped)\n";
      } else {
        gzwarn << "[PayloadDrop] Completion: BLUE spawn FAILED. Staying at internal_index=1 -- "
               << "will retry BLUE on the next drop request instead of silently marking it done\n";
      }
      return;
    }
  }

private:
  void OnDrop(const gz::msgs::Boolean &_msg)
  {
    if (_msg.data()) {
      dropRequested_ = true;
      gzwarn << "[PayloadDrop] Drop command received\n";
    }
  }

  void OnDropColor(const gz::msgs::StringMsg &_msg)
  {
    pendingColor_ = _msg.data();
    colorDropRequested_ = true;
    gzwarn << "[PayloadDrop] Color-select drop command received: " << pendingColor_ << "\n";
  }

  std::string BuildPayloadSdf(const std::string &modelName,
                               double r, double l, double m,
                               double dr, double dg, double db, double da)
  {
    // inertia roughly for cylinder around center axis (not super critical here)
    // We'll keep simple stable values.
    std::ostringstream ss;
    ss << std::fixed << std::setprecision(6);
    ss <<
      "<?xml version=\"1.0\"?>"
      "<sdf version=\"1.9\">"
        "<model name=\"" << modelName << "\">"
          "<static>false</static>"
          "<link name=\"link\">"
            "<gravity>true</gravity>"
            "<inertial>"
              "<mass>" << m << "</mass>"
              "<inertia>"
                "<ixx>0.001</ixx><iyy>0.001</iyy><izz>0.001</izz>"
                "<ixy>0</ixy><ixz>0</ixz><iyz>0</iyz>"
              "</inertia>"
            "</inertial>"

            "<collision name=\"collision\">"
              "<geometry><box><size>" << r*2.0 << " " << r*1.5 << " " << l << "</size></box></geometry>"
            "</collision>"

            "<visual name=\"visual_" << modelName << "\">"
              "<geometry><box><size>" << r*2.0 << " " << r*1.5 << " " << l << "</size></box></geometry>"
              "<material>"
                "<ambient>" << dr << " " << dg << " " << db << " " << da << "</ambient>"
                "<diffuse>" << dr << " " << dg << " " << db << " " << da << "</diffuse>"
                "<specular>0.2 0.2 0.2 1</specular>"
              "</material>"
            "</visual>"
          "</link>"
        "</model>"
      "</sdf>";
    return ss.str();
  }

  bool DoOneDrop(gz::sim::Entity dummyEnt,
                 double ox, double oy, double oz,
                 const std::string &tag,
                 double cr, double cg, double cb, double ca,
                 gz::sim::EntityComponentManager &_ecm)
  {
    // UAV pose + offset
    const auto uavPose = gz::sim::worldPose(uavLinkEnt_, _ecm);
    const gz::math::Vector3d offBody(ox, oy, oz);
    const gz::math::Vector3d offWorld = uavPose.Rot().RotateVector(offBody);

    // Lower the spawn point significantly to ensure the new 30x22cm rectangular box
    // completely clears the x500 landing gear and camera link. A violent spawn collision
    // will cause the EKF to accumulate a massive position error (e.g. 20m drift).
    gz::math::Pose3d spawnPose;
    spawnPose.Pos() = uavPose.Pos() + offWorld;
    // Force Z offset to be even lower if offWorld isn't enough
    spawnPose.Pos().Z() -= 0.40;
    spawnPose.Rot() = uavPose.Rot();

    // Remove dummy visuals
    if (dummyEnt != gz::sim::kNullEntity) {
      RemoveChildren<gz::sim::components::Visual>(_ecm, dummyEnt);
      RemoveChildren<gz::sim::components::Collision>(_ecm, dummyEnt);
      gzwarn << "[PayloadDrop] " << tag << " dummy visuals removed\n";
    } else {
      gzwarn << "[PayloadDrop] WARN " << tag << " dummy link not found; spawning anyway\n";
    }

    // Build SDF (unique model name each time to avoid any reuse)
    const std::string modelName =
      spawnNamePrefix_ + tag + "_model_" + std::to_string(spawnCounter_);

    const std::string sdfStr =
      BuildPayloadSdf(modelName, radius_, length_, mass_, cr, cg, cb, ca);

    gz::msgs::EntityFactory req;
    req.set_name(spawnNamePrefix_ + tag + "_" + std::to_string(spawnCounter_++));
    req.set_sdf(sdfStr);

    auto *p = req.mutable_pose();
    p->mutable_position()->set_x(spawnPose.Pos().X());
    p->mutable_position()->set_y(spawnPose.Pos().Y());
    p->mutable_position()->set_z(spawnPose.Pos().Z());
    p->mutable_orientation()->set_w(spawnPose.Rot().W());
    p->mutable_orientation()->set_x(spawnPose.Rot().X());
    p->mutable_orientation()->set_y(spawnPose.Rot().Y());
    p->mutable_orientation()->set_z(spawnPose.Rot().Z());

    gzwarn << "[PayloadDrop] Spawn request: " << tag
           << " model=" << req.name() << " pose_z=" << spawnPose.Pos().Z() << "\n";

    gz::msgs::Boolean rep;
    bool result = false;
    const unsigned int timeoutMs = 3000;

    const bool ok = node_.Request(createSrv_, req, timeoutMs, rep, result);
    const bool spawnSucceeded = ok && result && rep.data();

    gzwarn << "[PayloadDrop] CREATE " << tag
           << " ok=" << ok
           << " result=" << result
           << " rep=" << (rep.data() ? "true" : "false")
           << " name=" << req.name()
           << " pose_z=" << spawnPose.Pos().Z()
           << " spawn_success=" << (spawnSucceeded ? "true" : "false")
           << "\n";

    // Real confirmation back to payload.py (relayed by hook_sensor_service.py),
    // mirroring HookAttachSystem's own output_topic pattern. Previously
    // there was NO feedback channel at all: payload.py's "success" only
    // ever meant "gz topic pub exited 0", i.e. a message was published,
    // never that this plugin actually received it or that the
    // EntityFactory create call above actually spawned anything -- a
    // silently failed spawn (e.g. under increased world load) looked
    // identical to a successful one from the mission's side.
    gz::msgs::StringMsg stateMsg;
    stateMsg.set_data(tag + ":" + (spawnSucceeded ? "true" : "false"));
    statePub_.Publish(stateMsg);

    return spawnSucceeded;
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

  template<typename ChildComp>
  void RemoveChildren(gz::sim::EntityComponentManager &_ecm, gz::sim::Entity parent)
  {
    _ecm.Each<gz::sim::components::ParentEntity, ChildComp>(
      [&](const gz::sim::Entity &e,
          const gz::sim::components::ParentEntity *p,
          const ChildComp *) -> bool
      {
        if (p && p->Data() == parent) {
          _ecm.RequestRemoveEntity(e);
        }
        return true;
      });
  }

private:
  gz::transport::Node node_;
  gz::transport::Node::Publisher statePub_;
  std::string topic_{"/payload_drop"};
  std::string colorTopic_{"/payload_drop_color"};
  std::string outputTopic_{"/payload_drop_state"};
  std::string worldName_{"default"};
  std::string createSrv_{"/world/default/create"};

  std::string uavModel_{"x500_mono_cam_down_0"};
  std::string uavLink_{"base_link"};

  // Dummy links
  std::string dummyRedLink_{"payload_red_link"};
  std::string dummyBlueLink_{"payload_blue_link"};

  // Offsets
  double rx_{0.0}, ry_{0.0}, rz_{-0.60};
  double bx_{0.0}, by_{0.0}, bz_{-0.55};

  // Physical params
  double radius_{0.15};
  double length_{0.05};
  double mass_{0.85};

  std::string spawnNamePrefix_{"payload_drop_"};

  std::atomic<bool> dropRequested_{false};
  int stage_{0};
  int spawnCounter_{0};

  // COLOR-ADDRESSED DROP state (independent of stage_/dropRequested_ above).
  std::atomic<bool> colorDropRequested_{false};
  std::string pendingColor_;
  bool colorRedDropped_{false};
  bool colorBlueDropped_{false};

  gz::sim::Entity uavLinkEnt_{gz::sim::kNullEntity};
  gz::sim::Entity dummyRedEnt_{gz::sim::kNullEntity};
  gz::sim::Entity dummyBlueEnt_{gz::sim::kNullEntity};
};

} // namespace payload

GZ_ADD_PLUGIN(payload::PayloadDropSystem,
              gz::sim::System,
              gz::sim::ISystemConfigure,
              gz::sim::ISystemPreUpdate)
