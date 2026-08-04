// Lightweight Gazebo model plugin that couples a "mimic" joint to a driver
// joint using PID force control, so a mimicked finger physically pushes against
// a grasped object (unlike kinematic SetPosition, which penetrates contacts).
//
// SDF parameters:
//   <joint>        name of the driver joint (required)
//   <mimicJoint>   name of the mimic joint  (required)
//   <multiplier>   mimic = multiplier * driver + offset   (default 1.0)
//   <offset>       (default 0.0)
//   <maxEffort>    force clamp in N·m                      (default 30.0)
//   <kp> <ki> <kd> PID gains                               (default 30/0/1)

#include <functional>
#include <string>
#include <algorithm>

#include <gazebo/common/Plugin.hh>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>

namespace gazebo {

class MimicJointPlugin : public ModelPlugin {
public:
  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override {
    model_ = model;
    world_ = model->GetWorld();

    if (!sdf->HasElement("joint") || !sdf->HasElement("mimicJoint")) {
      gzerr << "[mimic_joint] <joint> and <mimicJoint> are required.\n";
      return;
    }
    joint_name_ = sdf->Get<std::string>("joint");
    mimic_name_ = sdf->Get<std::string>("mimicJoint");
    multiplier_ = sdf->HasElement("multiplier") ? sdf->Get<double>("multiplier") : 1.0;
    offset_     = sdf->HasElement("offset")     ? sdf->Get<double>("offset")     : 0.0;
    max_effort_ = sdf->HasElement("maxEffort")  ? sdf->Get<double>("maxEffort")  : 30.0;
    kp_ = sdf->HasElement("kp") ? sdf->Get<double>("kp") : 30.0;
    ki_ = sdf->HasElement("ki") ? sdf->Get<double>("ki") : 0.0;
    kd_ = sdf->HasElement("kd") ? sdf->Get<double>("kd") : 1.0;
    // Default to kinematic (SetPosition) coupling; set <usePID>true</usePID>
    // for force-based coupling.
    use_pid_ = sdf->HasElement("usePID") ? sdf->Get<bool>("usePID") : false;

    joint_ = model->GetJoint(joint_name_);
    mimic_ = model->GetJoint(mimic_name_);
    if (!joint_ || !mimic_) {
      gzerr << "[mimic_joint] could not find joint '" << joint_name_
            << "' or '" << mimic_name_ << "'.\n";
      return;
    }

    update_ = event::Events::ConnectWorldUpdateBegin(
        std::bind(&MimicJointPlugin::OnUpdate, this));
    gzmsg << "[mimic_joint] " << mimic_name_ << " mimics " << joint_name_
          << " (x" << multiplier_ << ")\n";
  }

  void OnUpdate() {
    const double target = joint_->Position(0) * multiplier_ + offset_;
    if (use_pid_) {
      const double dt = world_->Physics()->GetMaxStepSize();
      if (dt <= 0.0) return;
      const double current = mimic_->Position(0);
      const double error = target - current;
      integral_ += error * dt;
      const double deriv = (error - prev_error_) / dt;
      prev_error_ = error;
      double effort = kp_ * error + ki_ * integral_ + kd_ * deriv;
      effort = std::max(-max_effort_, std::min(max_effort_, effort));
      mimic_->SetForce(0, effort);
    } else {
      // Kinematic follow: reliably couples the joint without adding solver
      // forces that can fight gazebo_ros2_control's position control.
      mimic_->SetPosition(0, target, true);
      // Zero the driven child link's residual velocity so the kinematic teleport
      // can't inject momentum that flings the finger off.
      physics::LinkPtr child = mimic_->GetChild();
      if (child) {
        child->SetLinearVel(ignition::math::Vector3d::Zero);
        child->SetAngularVel(ignition::math::Vector3d::Zero);
      }
    }
  }

private:
  physics::ModelPtr model_;
  physics::WorldPtr world_;
  physics::JointPtr joint_, mimic_;
  event::ConnectionPtr update_;
  std::string joint_name_, mimic_name_;
  double multiplier_{1.0}, offset_{0.0}, max_effort_{30.0};
  double kp_{30.0}, ki_{0.0}, kd_{1.0};
  double integral_{0.0}, prev_error_{0.0};
  bool use_pid_{false};
};

GZ_REGISTER_MODEL_PLUGIN(MimicJointPlugin)

}  // namespace gazebo
