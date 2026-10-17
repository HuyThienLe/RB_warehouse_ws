#ifndef WH_GAZEBO__GAZEBO_GRIPPER_ATTACHER_HPP_
#define WH_GAZEBO__GAZEBO_GRIPPER_ATTACHER_HPP_

#include <gazebo/common/Plugin.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo_ros/node.hpp>
#include <rclcpp/rclcpp.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_srvs/srv/set_bool.hpp>

#include <memory>
#include <mutex>
#include <string>
#include <unordered_set>

namespace wh_gazebo
{

class GazeboGripperAttacher : public gazebo::ModelPlugin
{
public:
  GazeboGripperAttacher();
  ~GazeboGripperAttacher() override;

  void Load(gazebo::physics::ModelPtr _model, sdf::ElementPtr _sdf) override;

private:
  void OnUpdate();
  void OnSwitch(
    const std::shared_ptr<std_srvs::srv::SetBool::Request> req,
    std::shared_ptr<std_srvs::srv::SetBool::Response> res);

  gazebo::physics::ModelPtr model_;
  gazebo::physics::WorldPtr world_;
  gazebo::physics::LinkPtr gripper_link_;
  gazebo::physics::ModelPtr attached_model_;
  gazebo::physics::JointPtr attach_joint_;

  std::string link_name_{"gripper_grasp_link"};
  std::string attached_box_name_{""};
  double max_distance_{0.15};
  bool is_grasping_{false};
  std::unordered_set<std::string> fixed_entities_;
  std::mutex lock_;

  gazebo_ros::Node::SharedPtr ros_node_;
  rclcpp::Publisher<std_msgs::msg::Bool>::SharedPtr pub_grasping_;
  rclcpp::Service<std_srvs::srv::SetBool>::SharedPtr srv_switch_;
  gazebo::event::ConnectionPtr update_connection_;
};

}  // namespace wh_gazebo

#endif  // WH_GAZEBO__GAZEBO_GRIPPER_ATTACHER_HPP_
