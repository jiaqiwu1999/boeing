#include "arm_planner.h"
#define _USE_MATH_DEFINES

ArmPlanner::ArmPlanner(double step_size=0.0003, double setheight=1e-3, double zheight=-0.055, double force_th=1, bool verbose=false) :
    Node("arm_planner"),
    move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), PLANNING_GROUP),
    step_size(step_size),
    setheight(setheight),
    zheight(zheight),
    force_th(force_th),
    verbose(verbose)
{
    this->group_name = PLANNING_GROUP;
    this->subscription_ = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
      "wrench", 10, std::bind(&ArmPlanner::wrench_cb, this, _1));
    this->publisher_ = this->create_publisher<moveit_msgs::msg::DisplayTrajectory>("move_group/display_planned_path", 20);
}

void ArmPlanner::m_plan_execute(std::vector<geometry_msgs::msg::Pose> pose_waypoints) {
    moveit_msgs::msg::RobotTrajectory traj;
    this->move_group_.computeCartesianPath(pose_waypoints, 0.001, 0.0, traj);
    this->move_group_.execute(traj);
}

void ArmPlanner::m_home() {
    geometry_msgs::msg::Pose wpose = this->move_group_.getCurrentPose().pose;
    wpose.position.x = 0.134;
    wpose.position.y = 0.565;
    wpose.position.z = 0.747;
    
    std::vector<geometry_msgs::msg::Pose> pose_wp;
    pose_wp.push_back(wpose);
    this->m_plan_execute(pose_wp);
    std::vector<double> joint_goal(-M_PI_2, -M_PI_2, -M_P, 0, M_PI_2, 0);
    this->move_group_.setJointValueTarget(joint_goal);
    this->move_group_.move();
    this->move_group_.stop();
}

alglib::real_1d_array convert_eigen_to_alg(Eigen::VectorXd eigen_vec) {
    std::vector<double> vec;
    for (int i = 0; i < eigen_vec.size(); ++i) {
        vec.push_back(eigen_vec[i]);
    }
    alglib::real_1d_array res;
    res.setcontent(vec.size(), &(vec[0]));
    return res;
}

alglib::spline2dinterpolant ArmPlanner::get_plate_equation_new(Eigen::MatrixXd three_points) {
    Eigen::VectorXd x = three_points.col(0);
    Eigen::VectorXd y = three_points.col(1);
    Eigen::VectorXd z = three_points.col(2);
    alglib::real_1d_array x_ = convert_eigen_to_alg(x);
    alglib::real_1d_array y_ = convert_eigen_to_alg(y);
    alglib::real_1d_array z_ = convert_eigen_to_alg(z);
    alglib::spline2dinterpolant fx;

    //build spline
    alglib::spline2dbuildbicubicv(x_, x_.size(), y_, y_.size(), f, 1, s);

    this->z_deviation = z.maxCoeff() - z.minCoeff();

    std::cout << "z deviation: " << this->z_deviation << "\n";

    return fx;


}



void ArmPlanner::m_waypoint_xyz(std::vector<Eigen::Vector3d> waypoints) {
    geometry_msgs::msg::Quaternion empty_quat;
    if (this->ori_quat == empty_quat) {
        std::cout << "Orientation quaternion not initialized! NOT MOVING\n";
        return;
    }
    std::vector<geometry_msgs::msg::Pose> pose_waypoints;
    for (auto &wp : waypoints) {
        Eigen::Vector4d wp2 = this->transfer(wp);
        geometry_msgs::msg::Pose pose();
        pose.position.x = (float)wp2[0];
        pose.position.y = (float)wp2[1];
        pose.position.z = (float)wp2[2];
        pose.orientation = this->ori_quat;
        pose_waypoints.push_back(pose);
    }
    this->m_plan_execute(pose_waypoints);

}

Eigen::Vector3d ArmPlanner::m_touch_bed() {
    double force_init = this->wrench.force.z;
    double force = this->wrench.force.z - force_init;//0?

    while (rclcpp::ok() && force < this->force_tr) {
        this->m_step_down();
        force = std::abs(this->wrench.force.z - force_init);
    }

    geometry_msgs::msg::Pose final_pose = this->move_group_.getCurrentPose().pose;
    double x = (double)final_pose.position.x;
    double y = (double)final_pose.position.y;
    double z = (double)final_pose.position.z;

    Eigen::Vector3d res(x, y, z);
    return res;

}

void ArmPlanner::m_level_bed() {
    double x_origin = 0.38;
    double y_origin = -0.13;
    double dx = -0.038;
    double dy = 0.04;

    //std::vector<Eigen::Vector4d> sample_points;

    Eigen::ArrayXd xs = Eigen::ArrayXf::LinSpaced(5, x_origin, x_origin + 4 * dx);
    Eigen::ArrayXd ys = Eigen::ArrayXf::LinSpaced(5, y_origin, y_origin + 4 * dy);

    Eigen::MatrixXd sample_points = Eigen::MatrixXd::Zero(xs.size() * ys.size(), 4);
    int count = 0;
    for (int i = 0; i < xs.size(); ++i) {
        for (int j = 0; j < ys.size(); ++j) {
            Eigen::Vector3d v(xs[i], yx[j], this->zheight);
            std::vector<Eigen::Vector3d> vec;
            this->m_waypoint_xyz(vec.push_back(v));
            Eigen::Vector3d new_vec = this->m_touch_bed();
            this->m_waypoint_xyz(vec);

            Eigen::Vector4d pose(new_vec[0], new_vec[1], new_vec[2], 1);
            //sample_points.push_back(this->transfer_inv(pose));
            sample_points.row(count) = transfer_inv(pose);
            count += 1;
        }
    }
    std::cout << sample_points;
    std::cout << this->ori_quat;
    this->calibration_pts = sample_points;
    //TODO
    //Interpolation check
    this->bedlevel = this->get_plate_equation_new(sample_points);
    this->calibrated = true;

}

void ArmPlanner::m_step_down(double step=0) {
    if (step == 0) {
        step = this->step_size;
    }
    geometry_msgs::msg::Pose pose = this->move_group_.getCurrentPose().pose;
    pose.position.y += step;
    std::vector<geometry_msgs::msg::Pose> v;
    v.push_back(pose)
    this->m_plan_execute(v);

}

void ArmPlanner::run_calibration_process() {
    this->m_home();
    geometry_msgs::msg::Quaternion ori_cpose = this->move_group_.getCurrentPose().pose.orientation;
    this->ori_quat = ori_cpose;
    this->m_level_bed();
    this->m_home();
    //Initialize mesh builder
    //TODO, skip for now
}

std::vector<Eigen::Vector3d> ArmPlanner::adjust_waypoints(std::vector<Eigen::Vector4d> waypoints) {
    std::vector<Eigen::Vector3d> new_wps;
    if (this->calibrated) {
        double x_origin = -0.38;
        double y_origin = -0.13;

        for (auto &point : waypoints) {
            double x = x_origin + point[0] * 0.001;
            double y = y_origin + point[1] * 0.001;
            //TODO, interpolation
            double v = (double)alglib::spline2dcalc(this->bedlevel, x, y);
            double z = this->setheight + point[2] * 0.001 + v;
            Eigen::Vector3d vec(x, y, z);
            new_wps.push_back(vec);

        }
    }
    return new_wps;
}

moveit_msgs::RobotTrajectory& ArmPlanner::plan_speed_constrain(Eigen::MatrixXd waypoints, double feedrate, bool follow_surface=false) {
    if (this->ori_quat == geometry_msgs::msg::Quaternion()) {
        std::cout << "Orientation quaternion not initialized! NOT PLANNING";
    }
    auto zs = waypoints.cols(2);
    bool zconst = (zs.array().max() - zs.array().min() < this->z_deviation);
    if (zconst) {
        follow_surface = false;
    }
    else {
        std::cout << "TRAJECTORY IS NON CONSTANT, diff > " << this->z_deviation << "\n";
        std::cout << "num points: " << waypoints.size() << "\n";
    }

    std::vector<geometry_msgs::msg::Pose> pose_waypoints;
    for (int i = 0; i < waypoints.rows(); ++i) {
        Eigen::Map<Eigen::Vector3d> vec(waypoints.rows(i).data());
        Eigen::Vector4d wp2 = this->transfer(vec);
        geometry_msgs::msg::Pose pose;
        pose.position.x = (float)wp2[0]
        pose.position.y = (float)wp2[1]
        pose.position.z = (float)wp2[2]
        pose.orientation = (float)this->ori_quat;
        pose_waypoints.push_back(pose);
    }
    moveit_msgs::msg::RobotTrajectory trajectory;
    this->move_group_.computeCartesianPath(pose_waypoints, 0.001, 0.0, trajectory);
    return trajectory;

}



void ArmPlanner::gcode_motion(std::vector<Eigen::Vector3d> waypoints, double feedrate) {
    std::vector<Eigen::Vector3d> tr_waypoints = this->adjust_waypoints(waypoints);
    moveit_msgs::RobotTrajectory& plan = this->plan_speed_constrain(waypoints, feedrate);
    this->move_group_.execute(plan);
}


