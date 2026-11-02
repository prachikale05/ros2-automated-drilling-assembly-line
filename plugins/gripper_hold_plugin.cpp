#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>

#include <ignition/math/Pose3.hh>

#include <memory>
#include <cmath>
#include <functional>

namespace gazebo
{

class GripperHoldPlugin : public ModelPlugin
{
public:

  void Load(physics::ModelPtr model, sdf::ElementPtr sdf) override
  {
    this->workpiece_model_ = model;
    this->world_ = model->GetWorld();

    gzmsg << "[GripperHold] Plugin loaded on workpiece\n";

    this->update_connection_ =
        event::Events::ConnectWorldUpdateBegin(
            std::bind(&GripperHoldPlugin::OnUpdate, this));
  }

private:

  void OnUpdate()
  {
    if (!this->world_)
      return;

    // --------------------------------------------------
    // FIND ROBOT
    // --------------------------------------------------

    physics::ModelPtr robot =
        this->world_->ModelByName("workstation1_robot");

    if (!robot)
      return;

    // --------------------------------------------------
    // FIND GRIPPER LINKS
    // --------------------------------------------------

    physics::LinkPtr link6 =
        robot->GetLink("link6");

    physics::LinkPtr left_finger =
        robot->GetLink("gripper_left_finger");

    physics::LinkPtr right_finger =
        robot->GetLink("gripper_right_finger");

    physics::LinkPtr workpiece_link =
        this->workpiece_model_->GetLink("workpiece_link");

    physics::JointPtr left_joint =
        robot->GetJoint("gripper_left_finger_joint");

    physics::JointPtr right_joint =
        robot->GetJoint("gripper_right_finger_joint");

    if (!link6 ||
        !left_finger ||
        !right_finger ||
        !workpiece_link ||
        !left_joint ||
        !right_joint)
    {
      return;
    }

    // --------------------------------------------------
    // CHECK GRIPPER STATE
    // --------------------------------------------------

    double left_position =
        left_joint->Position(0);

    double right_position =
        right_joint->Position(0);

    bool gripper_closed =
        std::fabs(left_position) < 0.02 &&
        std::fabs(right_position) < 0.02;

    // --------------------------------------------------
    // GET POSITIONS
    // --------------------------------------------------

    ignition::math::Vector3d left_pos =
        left_finger->WorldPose().Pos();

    ignition::math::Vector3d right_pos =
        right_finger->WorldPose().Pos();

    ignition::math::Vector3d workpiece_pos =
        workpiece_link->WorldPose().Pos();

    double left_distance =
        left_pos.Distance(workpiece_pos);

    double right_distance =
        right_pos.Distance(workpiece_pos);

    double closest_distance =
        std::min(left_distance, right_distance);

    // Print only while not attached
    if (!this->attached_)
    {
      gzmsg << "[GripperHold] "
            << "Left: " << left_distance
            << " | Right: " << right_distance
            << " | Closed: " << gripper_closed
            << "\n";
    }

    // --------------------------------------------------
    // ATTACH
    // --------------------------------------------------

    if (!this->attached_ &&
        gripper_closed &&
        closest_distance < 0.45)
    {
      gzmsg << "\n";
      gzmsg << "[GripperHold] =============================\n";
      gzmsg << "[GripperHold] GRASP DETECTED\n";
      gzmsg << "[GripperHold] Creating fixed joint...\n";

      this->fixed_joint_ =
          this->world_->Physics()->CreateJoint("fixed");

      if (this->fixed_joint_)
      {
        // Attach workpiece to robot link6
        this->fixed_joint_->Load(
            link6,
            workpiece_link,
            ignition::math::Pose3d());

        this->fixed_joint_->Init();

        this->attached_ = true;

        gzmsg << "[GripperHold] *** WORKPIECE ATTACHED ***\n";
        gzmsg << "[GripperHold] =============================\n";
      }
    }

    // --------------------------------------------------
    // RELEASE
    // --------------------------------------------------

    if (this->attached_ &&
        !gripper_closed)
    {
      gzmsg << "[GripperHold] Gripper OPENED\n";
      gzmsg << "[GripperHold] Releasing workpiece...\n";

      if (this->fixed_joint_)
      {
        this->fixed_joint_->Detach();
        this->fixed_joint_->Fini();
        this->fixed_joint_.reset();
      }

      this->attached_ = false;

      gzmsg << "[GripperHold] *** WORKPIECE RELEASED ***\n";
    }
  }

private:

  physics::ModelPtr workpiece_model_;
  physics::WorldPtr world_;

  physics::JointPtr fixed_joint_;

  event::ConnectionPtr update_connection_;

  bool attached_ = false;
};

GZ_REGISTER_MODEL_PLUGIN(GripperHoldPlugin)

}
