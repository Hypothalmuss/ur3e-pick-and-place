// Robust grasp plugin for a parallel gripper in Gazebo Classic.
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
    // Lazily bind the graspable object once it exists in the world, and make it
    // kinematic so nothing can knock it around.
    if (!obj_link_) { FindObject(); if (!obj_link_) return; }

    if (carrying_) {
      obj_link_->SetWorldPose(ignition::math::Pose3d(GraspPoint(), carry_rot_));
    } else {
      obj_link_->SetWorldPose(rest_pose_);   // hold in place; immune to bumps
    }

    common::Time now = world_->SimTime();
    if ((now - last_check_).Double() < period_) return;
    last_check_ = now;

    const double closure = joint_->Position(0);
    if (!carrying_ && closure > attach_thresh_) {
      const ignition::math::Vector3d gp = GraspPoint();
      if ((obj_link_->WorldPose().Pos() - gp).Length() < grasp_radius_) {
        carry_rot_ = obj_link_->WorldPose().Rot();
        carrying_ = true;
        gzmsg << "[gripper_grasp] grasped '" << obj_model_->GetName() << "'\n";
      }
    } else if (carrying_ && closure < detach_thresh_) {
      ignition::math::Vector3d p = obj_link_->WorldPose().Pos();
      rest_pose_ = ignition::math::Pose3d(
          ignition::math::Vector3d(p.X(), p.Y(), rest_z_), carry_rot_);
      carrying_ = false;
      gzmsg << "[gripper_grasp] placed '" << obj_model_->GetName()
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

  void FindObject() {
    for (const auto &m : world_->Models()) {
      if (!m || m == model_ || m->IsStatic()) continue;
      if (m->GetName() == "ground_plane") continue;
      physics::LinkPtr l = m->GetLink();
      if (!l) { for (const auto &x : m->GetLinks()) { l = x; break; } }
      if (!l) continue;
      obj_model_ = m;
      obj_link_ = l;
      obj_link_->SetKinematic(true);
      rest_pose_ = obj_link_->WorldPose();
      gzmsg << "[gripper_grasp] managing object '" << m->GetName() << "'\n";
      return;
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

  physics::ModelPtr model_, obj_model_;
  physics::WorldPtr world_;
  physics::JointPtr joint_;
  physics::LinkPtr palm_, left_finger_, right_finger_, obj_link_;
  ignition::math::Quaterniond carry_rot_;
  ignition::math::Pose3d rest_pose_;
  event::ConnectionPtr update_;
  std::string joint_name_, palm_name_, left_finger_name_, right_finger_name_;
  double attach_thresh_{0.30}, detach_thresh_{0.15}, grasp_radius_{0.15};
  double palm_offset_z_{0.16}, rest_z_{0.03}, period_{0.05};
  common::Time last_check_;
  bool carrying_{false};
};

GZ_REGISTER_MODEL_PLUGIN(GripperGraspPlugin)

}  // namespace gazebo
