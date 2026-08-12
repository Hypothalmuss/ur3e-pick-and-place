// Robust grasp plugin for a parallel gripper in Gazebo Classic.
//
// Manages EVERY graspable object in the world, not one. The original version
// bound to the first non-static model it found and kept it forever, which was
// invisible while the world held a single cube: with three, only that one was
// kinematic and grabbable and the other two were ordinary dynamic bodies the
// gripper could merely shove. The symptom was a tidy sweep that reliably
// delivered exactly one cube and pushed the rest out of the workspace.
//
// Gazebo Classic contact between a light box and a moving gripper is unstable
// (the box gets knocked away or flung by stiff contacts). To make pick-and-place
// reliable, the target object is held KINEMATIC and managed by this plugin:
//   * a kinematic body ignores contact forces, so the gripper can never knock it;
//   * while the gripper is open it is held at its rest pose;
//   * when the driver joint closes past <attach_threshold> with the object within
//     <grasp_radius> of the finger-tip midpoint, it is carried with the gripper;
//   * when the joint opens past <detach_threshold> it is set down on the ground
//     at its current x/y (rest height <rest_z>).
// The fingers still physically close around the object, so the motion looks and
// behaves like a real grasp; only the hold is plugin-assisted, exactly like the
// contact-based grasp-fix plugins used in production Gazebo setups.
//
// SDF parameters (gripper_joint and palm_link required):
//   <gripper_joint> <palm_link> <left_finger_link> <right_finger_link>
//   <attach_threshold> <detach_threshold> <grasp_radius> <palm_offset_z>
//   <rest_z> <update_rate>

#include <functional>
#include <string>
#include <algorithm>
#include <vector>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <ignition/math/Pose3.hh>

namespace gazebo {

class GripperGraspPlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    world_ = model->GetWorld();

    joint_name_ = get(sdf, "gripper_joint", "");
    palm_name_ = get(sdf, "palm_link", "");
    left_finger_name_ = get(sdf, "left_finger_link", "");
    right_finger_name_ = get(sdf, "right_finger_link", "");
    attach_thresh_ = getd(sdf, "attach_threshold", 0.30);
    detach_thresh_ = getd(sdf, "detach_threshold", 0.15);
    grasp_radius_ = getd(sdf, "grasp_radius", 0.15);
    palm_offset_z_ = getd(sdf, "palm_offset_z", 0.16);
    rest_z_ = getd(sdf, "rest_z", 0.03);
    double rate = getd(sdf, "update_rate", 20.0);
    period_ = rate > 0.0 ? 1.0 / rate : 0.05;

    joint_ = model->GetJoint(joint_name_);
    palm_ = model->GetLink(palm_name_);
    left_finger_ = model->GetLink(left_finger_name_);
    right_finger_ = model->GetLink(right_finger_name_);
    if (!joint_ || !palm_) {
      gzerr << "[gripper_grasp] joint '" << joint_name_ << "' or palm '"
            << palm_name_ << "' not found; plugin disabled.\n";
      return;
    }

    update_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&GripperGraspPlugin::OnUpdate, this));
    gzmsg << "[gripper_grasp] ready (joint=" << joint_name_
          << ", palm=" << palm_name_ << ")\n";
  }

  void OnUpdate() {
    // Objects can be spawned after the plugin loads, so keep looking until the
    // world stops growing rather than binding once.
    if (objs_.size() != graspable_count()) FindObjects();
    if (objs_.empty()) return;

    for (size_t i = 0; i < objs_.size(); ++i) {
      if (carrying_ && i == carried_) {
        objs_[i].link->SetWorldPose(
            ignition::math::Pose3d(GraspPoint(), carry_rot_));
      } else {
        objs_[i].link->SetWorldPose(objs_[i].rest);  // immune to bumps
      }
    }

    common::Time now = world_->SimTime();
    if ((now - last_check_).Double() < period_) return;
    last_check_ = now;

    const double closure = joint_->Position(0);
    if (!carrying_ && closure > attach_thresh_) {
      // Take the NEAREST object inside the radius. Picking the first match
      // would grab whichever happened to be listed first, which is how the
      // single-object version behaved and why it only ever worked for one cube.
      const ignition::math::Vector3d gp = GraspPoint();
      size_t best = 0;
      double best_d = grasp_radius_;
      bool found = false;
      for (size_t i = 0; i < objs_.size(); ++i) {
        const double d = (objs_[i].link->WorldPose().Pos() - gp).Length();
        if (d < best_d) { best_d = d; best = i; found = true; }
      }
      if (found) {
        carried_ = best;
        carry_rot_ = objs_[best].link->WorldPose().Rot();
        carrying_ = true;
        gzmsg << "[gripper_grasp] grasped '" << objs_[best].model->GetName()
              << "' at " << best_d << " m\n";
      }
    } else if (carrying_ && closure < detach_thresh_) {
      ignition::math::Vector3d p = objs_[carried_].link->WorldPose().Pos();
      objs_[carried_].rest = ignition::math::Pose3d(
          ignition::math::Vector3d(p.X(), p.Y(), rest_z_), carry_rot_);
      carrying_ = false;
      gzmsg << "[gripper_grasp] placed '" << objs_[carried_].model->GetName()
            << "' at (" << p.X() << ", " << p.Y() << ")\n";
    }
  }

private:
  static std::string get(sdf::ElementPtr s, const std::string &k,
                         const std::string &d) {
    return s->HasElement(k) ? s->Get<std::string>(k) : d;
  }
  static double getd(sdf::ElementPtr s, const std::string &k, double d) {
    return s->HasElement(k) ? s->Get<double>(k) : d;
  }

  bool graspable(const physics::ModelPtr &m) const {
    return m && m != model_ && !m->IsStatic() && m->GetName() != "ground_plane";
  }

  size_t graspable_count() const {
    size_t n = 0;
    for (const auto &m : world_->Models()) if (graspable(m)) ++n;
    return n;
  }

  void FindObjects() {
    for (const auto &m : world_->Models()) {
      if (!graspable(m)) continue;
      bool known = false;
      for (const auto &o : objs_) if (o.model == m) { known = true; break; }
      if (known) continue;

      physics::LinkPtr l = m->GetLink();
      if (!l) { for (const auto &x : m->GetLinks()) { l = x; break; } }
      if (!l) continue;

      l->SetKinematic(true);
      objs_.push_back({m, l, l->WorldPose()});
      gzmsg << "[gripper_grasp] managing object '" << m->GetName()
            << "' (" << objs_.size() << " total)\n";
    }
  }

  ignition::math::Vector3d GraspPoint() const {
    if (left_finger_ && right_finger_) {
      return 0.5 * (left_finger_->WorldPose().Pos() +
                    right_finger_->WorldPose().Pos());
    }
    ignition::math::Pose3d p = palm_->WorldPose();
    return p.Pos() + p.Rot().RotateVector(
        ignition::math::Vector3d(0, 0, palm_offset_z_));
  }

  struct Managed {
    physics::ModelPtr model;
    physics::LinkPtr link;
    ignition::math::Pose3d rest;
  };

  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::JointPtr joint_;
  physics::LinkPtr palm_, left_finger_, right_finger_;
  std::vector<Managed> objs_;
  size_t carried_{0};
  ignition::math::Quaterniond carry_rot_;
  event::ConnectionPtr update_;
  std::string joint_name_, palm_name_, left_finger_name_, right_finger_name_;
  double attach_thresh_{0.30}, detach_thresh_{0.15}, grasp_radius_{0.15};
  double palm_offset_z_{0.16}, rest_z_{0.03}, period_{0.05};
  common::Time last_check_;
  bool carrying_{false};
};

GZ_REGISTER_MODEL_PLUGIN(GripperGraspPlugin)

}  // namespace gazebo
