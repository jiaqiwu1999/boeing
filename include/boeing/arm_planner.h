#ifndef arm_planner_h
#define arm_planner_h
#include <cmath>
#include <chrono>
#include <iostream>
#include <memory>
#include <string>
#include <vector>
#include <limits>
#include <utility>
#include <eigen3/Eigen/Dense>
#include <stdafx.h>
#include <interpolation.h>
// #include "lifecycle_msgs/msg/transition.hpp"
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/publisher.hpp>

// #include "rclcpp_lifecycle/lifecycle_node.hpp"
// #include "rclcpp_lifecycle/lifecycle_publisher.hpp"

//#include "rcutils/logging_macros.h"

#include <std_msgs/msg/string.hpp>

#include <moveit/move_group_interface/move_group_interface.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/robot_trajectory/robot_trajectory.h>
#include <geometry_msgs/msg/wrench_stamped.hpp>
#include <geometry_msgs/msg/quaternion.hpp>
#include <geometry_msgs/msg/pose.hpp>

using namespace std::chrono_literals;

class ArmPlanner : public rclcpp::Node {
  public:

  ArmPlanner(double step_size, double setheight, double zheight, double force_th, bool verbose);

  void m_home();

  void run_calibration_process();

  Eigen::Vector3d m_touch_bed();

  void m_step_down(double step);

  void m_level_bed();

  alglib::spline2dinterpolant get_plate_equation_new(Eigen::MatrixXd three_points);

  void m_plan_execute(std::vector<geometry_msgs::msg::Pose> pose_waypoints);


  std::vector<Eigen::Vector3d> adjust_waypoints(std::vector<Eigen::Vector4d> waypoints);

  moveit_msgs::msg::RobotTrajectory plan_speed_constrain(std::vector<Eigen::Vector3d> waypoints, double feedrate, bool follow_surface);


  void m_waypoint_xyz(std::vector<Eigen::Vector3d> waypoints);

  void gcode_motion(std::vector<Eigen::Vector4d> waypoints, double feedrate);

  moveit::planning_interface::MoveGroupInterface move_group_;

  private:
  rclcpp::Subscription<geometry_msgs::msg::WrenchStamped>::SharedPtr subscription;
  //TODO: keeps giving error (DisplayTrajectory type error)
  //rclcpp::Publisher<moveit_msgs::msg::DisplayTrajectory>::SharedPtr publisher;
  //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  Eigen::Matrix4d gst;
  geometry_msgs::msg::Wrench wrench;
  std::string group_name = "manipulator";
  geometry_msgs::msg::Quaternion ori_quat;
  std::vector<Eigen::Vector4d> calibration_pts;
  double step_size = 0.003;
  double setheight = 1e-3;
  double zheight = -0.055;
  double force_th = 1;
  double z_deviation;
  bool verbose = false;
  bool calibrated = false;
  alglib::spline2dinterpolant bedlevel;

  void wrench_cb(const geometry_msgs::msg::WrenchStamped::SharedPtr msg) {
    this->wrench = msg->wrench;
    //const geometry_msgs::msg::Wrench w = msg->wrench;
    
  }


  // Eigen::Vector4d get_quat_vec(Eigen::Vector4d normal, Eigen::Vector4d q0) {
  //   Eigen::Vector3d vv(normal[0], normal[1], normal[2]);
  //   vv.normalize();
  //   Eigen::Vector3d zz(0, 0, 1);
  //   Eigen::Vector3d axis = zz.cross(vv);
  //   double theta = asin(axis.norm());
  //   if (theta != 0) {
  //     Eigen::Array4d ret = q0.array() * axis.array() * theta / axis.norm();
  //     Eigen::Map<Eigen::Vector4d> ret2(ret.data());
  //     return ret2;
  //   }
  //   else {
  //     return q0;
  //   }

  // }

  Eigen::Vector4d transfer(Eigen::Vector3d v) {
    Eigen::Vector4d vv(v[0], v[1], v[2], 1);
    return gst * vv;
  }
  Eigen::Vector4d transfer_inv(Eigen::Vector4d v) {
    Eigen::Vector4d vv(v[0], v[1], v[2], 1);
    //TODO
    //cv version:
    // cv::Mat cv_matrix;
    // cv::eigen2cv(this->gst, cv_matrix);
    // cv::Mat output_matrix;
    // cv::invert(cv_matrix, output_matrix, cv::DECOMP_SVD);

    // //convert cv mat back to eig
    // Eigen::Matrix4d pinv;
    // cv::cv2eigen(output_matrix, pinv);

    Eigen::MatrixXd pinv = this->gst.completeOrthogonalDecomposition().pseudoInverse();
    Eigen::Vector4d result = pinv * vv;
    return result;
  }



};



// class ArmPlanner : public rclcpp_lifecycle::LifecycleNode
// {
// public:

//   explicit ArmPlanner(const std::string & node_name, bool intra_process_comms = false)
//   : rclcpp_lifecycle::LifecycleNode(node_name,
//       rclcpp::NodeOptions().use_intra_process_comms(intra_process_comms))
//   {};

//     rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
//     on_configure(const rclcpp_lifecycle::State & previous_state);

//     rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
//     on_activate(const rclcpp_lifecycle::State & previous_state);
//     rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
//     on_deactivate(const rclcpp_lifecycle::State & previous_state);

//     rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
//     on_cleanup(const rclcpp_lifecycle::State & previous_state);
//     rclcpp_lifecycle::node_interfaces::LifecycleNodeInterface::CallbackReturn
//     on_shutdown(const rclcpp_lifecycle::State & previous_state);   


// private:
//   std::shared_ptr<rclcpp_lifecycle::LifecyclePublisher<std_msgs::msg::String>> pub_;

//   std::shared_ptr<rclcpp::TimerBase> timer_; 

// };


#endif