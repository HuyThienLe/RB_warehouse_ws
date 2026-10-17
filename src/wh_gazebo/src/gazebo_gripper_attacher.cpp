#include "wh_gazebo/gazebo_gripper_attacher.hpp"

#include <gazebo/physics/Entity.hh>
#include <gazebo/physics/Joint.hh>
#include <gazebo/physics/Link.hh>
#include <gazebo/physics/Model.hh>
#include <gazebo/physics/World.hh>

#include <ignition/math/Pose3.hh>
#include <ignition/math/Vector3.hh>

namespace wh_gazebo
{

GazeboGripperAttacher::GazeboGripperAttacher()
{
}

GazeboGripperAttacher::~GazeboGripperAttacher()
{
  if (attach_joint_ && model_) {
    attach_joint_->Detach();
    model_->RemoveJoint("wh_gripper_box_joint");
    attach_joint_.reset();
  }
}

void GazeboGripperAttacher::Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf)
{
  model_ = _model;
  world_ = _model->GetWorld();

  // Read SDF parameters
  if (_sdf->HasElement("link_name")) {
    link_name_ = _sdf->Get<std::string>("link_name");
  }
  if (_sdf->HasElement("max_distance")) {
    max_distance_ = _sdf->Get<double>("max_distance");
  }

  // Entities to ignore
  if (_sdf->HasElement("fixed")) {
    sdf::ElementPtr fixed_elem = _sdf->GetElement("fixed");
    while (fixed_elem) {
      fixed_entities_.insert(fixed_elem->Get<std::string>());
      fixed_elem = fixed_elem->GetNextElement("fixed");
    }
  }

  gripper_link_ = model_->GetLink(link_name_);
  if (!gripper_link_) {
    // Fallback: search links
    for (auto link : model_->GetLinks()) {
      if (link->GetName().find(link_name_) != std::string::npos) {
        gripper_link_ = link;
        break;
      }
    }
  }

  // ROS 2 Node
  ros_node_ = gazebo_ros::Node::Get(_sdf);

  pub_grasping_ = ros_node_->create_publisher<std_msgs::msg::Bool>("grasping", 10);

  srv_switch_ = ros_node_->create_service<std_srvs::srv::SetBool>(
    "switch",
    std::bind(&GazeboGripperAttacher::OnSwitch, this, std::placeholders::_1, std::placeholders::_2));

  update_connection_ = gazebo::event::Events::ConnectWorldUpdateBegin(
    std::bind(&GazeboGripperAttacher::OnUpdate, this));

  RCLCPP_INFO(
    ros_node_->get_logger(),
    "GazeboGripperAttacher (Zero-Jitter Rigid Joint) loaded for link: %s (max_dist: %.2f m)",
    link_name_.c_str(), max_distance_);
}

void GazeboGripperAttacher::OnUpdate()
{
  std::lock_guard<std::mutex> lock(lock_);
  std_msgs::msg::Bool msg;
  msg.data = is_grasping_;
  pub_grasping_->publish(msg);
}

void GazeboGripperAttacher::OnSwitch(
  const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
  std::shared_ptr<std_srvs::srv::SetBool::Response> res)
{
  std::lock_guard<std::mutex> lock(lock_);

  if (!req->data) {
    // Detach
    if (attach_joint_) {
      attach_joint_->Detach();
      model_->RemoveJoint("wh_gripper_box_joint");
      attach_joint_.reset();
    }
    if (attached_model_) {
      for (auto l : attached_model_->GetLinks()) {
        l->SetEnabled(true);
        l->SetLinearVel(ignition::math::Vector3d::Zero);
        l->SetAngularVel(ignition::math::Vector3d::Zero);
      }
    }
    attached_model_.reset();
    attached_box_name_ = "";
    is_grasping_ = false;

    std_msgs::msg::Bool msg;
    msg.data = false;
    pub_grasping_->publish(msg);

    res->success = true;
    res->message = "Gripper released / detached";
    RCLCPP_INFO(ros_node_->get_logger(), "Gripper detached box.");
    return;
  }

  // Attach: Find closest box
  if (attach_joint_) {
    res->success = true;
    res->message = "Already attached to " + attached_box_name_;
    return;
  }

  if (!gripper_link_) {
    gripper_link_ = model_->GetLink(link_name_);
    if (!gripper_link_) {
      res->success = false;
      res->message = "Gripper link not found: " + link_name_;
      return;
    }
  }

  ignition::math::Pose3d g_pose = gripper_link_->WorldPose();

  gazebo::physics::ModelPtr closest_box = nullptr;
  double min_dist = max_distance_;

  for (auto m : world_->Models()) {
    std::string m_name = m->GetName();
    if (m_name == model_->GetName() || fixed_entities_.count(m_name) > 0) {
      continue;
    }
    // Only attach box models
    if (m_name.find("box") == std::string::npos) {
      continue;
    }

    ignition::math::Pose3d m_pose = m->WorldPose();
    double d = (m_pose.Pos() - g_pose.Pos()).Length();
    if (d < min_dist) {
      min_dist = d;
      closest_box = m;
    }
  }

  if (!closest_box) {
    res->success = false;
    res->message = "No box found within " + std::to_string(max_distance_) + "m";
    RCLCPP_WARN(ros_node_->get_logger(), "%s", res->message.c_str());
    return;
  }

  gazebo::physics::LinkPtr box_link = closest_box->GetLink();
  if (!box_link) {
    auto links = closest_box->GetLinks();
    if (!links.empty()) {
      box_link = links[0];
    }
  }

  if (!box_link) {
    res->success = false;
    res->message = "Box link not found on model: " + closest_box->GetName();
    return;
  }

  // Align box cleanly to gripper_link_ pose before locking to eliminate any offset or tilt
  closest_box->SetWorldPose(g_pose);
  box_link->SetWorldPose(g_pose);

  // Create rigid constraint joint between gripper_link_ and box_link
  std::string jname = "wh_gripper_box_joint";
  attach_joint_ = model_->CreateJoint(jname, "revolute", gripper_link_, box_link);
  if (!attach_joint_) {
    res->success = false;
    res->message = "Failed to create joint: " + jname;
    return;
  }

  attach_joint_->Attach(gripper_link_, box_link);
  attach_joint_->Load(gripper_link_, box_link, ignition::math::Pose3d());
  attach_joint_->SetAxis(0, ignition::math::Vector3d(0, 0, 1));
  attach_joint_->SetUpperLimit(0, 0.0);
  attach_joint_->SetLowerLimit(0, 0.0);
  attach_joint_->Init();
  model_->Update();

  attached_model_ = closest_box;
  attached_box_name_ = closest_box->GetName();
  is_grasping_ = true;

  res->success = true;
  res->message = "Successfully attached " + attached_box_name_ + " via dynamic joint (Zero Jitter)";
  RCLCPP_INFO(ros_node_->get_logger(), "%s", res->message.c_str());
}

GZ_REGISTER_MODEL_PLUGIN(wh_gazebo::GazeboGripperAttacher)

}  // namespace wh_gazebo
