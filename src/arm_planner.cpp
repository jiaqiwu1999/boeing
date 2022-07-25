#include "boeing/arm_planner.h"
#define _USE_MATH_DEFINES

ArmPlanner::ArmPlanner(double step_size, double setheight, double zheight, double force_th, bool verbose) :
    Node("arm_planner"),
    move_group_(std::shared_ptr<rclcpp::Node>(std::move(this)), "manipulator"),
    step_size(step_size),
    setheight(setheight),
    zheight(zheight),
    force_th(force_th),
    verbose(verbose)
{
    this->gst << 0, 1, 0, 0.134,
                 0, 0, -1, 0.565,
                 1, 0, 0, 0.747,
                 0, 0, 0, 1;
    this->group_name = "manipulator";
    this->subscription = this->create_subscription<geometry_msgs::msg::WrenchStamped>(
        "wrench", 10, std::bind(&ArmPlanner::wrench_cb, this, std::placeholders::_1)
    );
    // std::function<void(std::shared_ptr<geometry_msgs::msg::WrenchStamped>)> func = std::bind(&ArmPlanner::wrench_cb, this, std::placeholders::_1);
    // this->subscription = this->create_subscription<geometry_msgs::msg::WrenchStamped>("wrench", 10, func);
    //this->publisher_ = this->create_publisher<moveit_msgs::msg::DisplayTrajectory>("move_group/display_planned_path", 20);
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
    std::vector<double> joint_goal{-M_PI_2, -M_PI_2, -M_PI, 0.0, M_PI_2, 0.0};
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
    alglib::spline2dbuildbicubicv(x_, x.size(), y_, y.size(), z_, 1, fx);

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
        //geometry_msgs::msg::Pose pose((float)wp2[0], (float)wp2[1], (float)wp2[2]);
        geometry_msgs::msg::Pose pose;
        pose.position.x = (float)wp2[0];
        pose.position.y = (float)wp2[1];
        pose.position.z = (float)wp2[2];
        pose.orientation = this->ori_quat;
        pose_waypoints.push_back(pose);
    }
    this->m_plan_execute(pose_waypoints);

}


/**
 * @brief 
 * 
 * @return Eigen::Vector3d 
 */
Eigen::Vector3d ArmPlanner::m_touch_bed() {
    double force_init = this->wrench.force.z;
    double force = this->wrench.force.z - force_init;//0?

    while (rclcpp::ok() && force < this->force_th) {
        this->m_step_down(0);
        force = std::abs(this->wrench.force.z - force_init);
    }

    geometry_msgs::msg::Pose final_pose = this->move_group_.getCurrentPose().pose;
    double x = (double)final_pose.position.x;
    double y = (double)final_pose.position.y;
    double z = (double)final_pose.position.z;

    Eigen::Vector3d res(x, y, z);
    return res;

}

/**
 * @brief run a set of equidistance points
 * set this->calibration_pts to a set of inverted points
 * set this->bedlevel interpolation equation
 */
void ArmPlanner::m_level_bed() {
    double x_origin = 0.38;
    double y_origin = -0.13;
    double dx = -0.038;
    double dy = 0.04;

    //std::vector<Eigen::Vector4d> sample_points;

    Eigen::ArrayXd xs = Eigen::ArrayXd::LinSpaced(5, x_origin, x_origin + 4 * dx);
    Eigen::ArrayXd ys = Eigen::ArrayXd::LinSpaced(5, y_origin, y_origin + 4 * dy);

    //Eigen::MatrixXd sample_points = Eigen::MatrixXd::Zero(xs.size() * ys.size(), 4);
    std::vector<Eigen::Vector4d> sample_points;
    //int count = 0;
    for (int i = 0; i < xs.size(); ++i) {
        for (int j = 0; j < ys.size(); ++j) {
            Eigen::Vector3d v(xs[i], ys[j], this->zheight);
            std::vector<Eigen::Vector3d> vec;
            vec.push_back(v);
            this->m_waypoint_xyz(vec);
            Eigen::Vector3d new_vec = this->m_touch_bed();
            this->m_waypoint_xyz(vec);

            Eigen::Vector4d pose(new_vec[0], new_vec[1], new_vec[2], 1);
            //sample_points.push_back(this->transfer_inv(pose));
            Eigen::Vector4d transferred = transfer_inv(pose);
            sample_points.push_back(transferred);
            //count += 1;
        }
    }
    std::cout << "There are " << sample_points.size() << " sample points \n";
    //std::cout << this->ori_quat;
    this->calibration_pts = sample_points;
    //TODO, change code structure probably
    //Interpolation check
    //vector<Vector4d> -> matrix
    Eigen::MatrixXd matrix = Eigen::MatrixXd::Zero(sample_points.size(), sample_points[0].size());
    for (int i = 0; i < matrix.rows(); ++i) {
        matrix.row(i) = sample_points[i];
    }
    this->bedlevel = this->get_plate_equation_new(matrix);
    this->calibrated = true;

}

void ArmPlanner::m_step_down(double step) {
    if (step == 0) {
        step = this->step_size;
    }
    geometry_msgs::msg::Pose pose = this->move_group_.getCurrentPose().pose;
    pose.position.y += step;
    std::vector<geometry_msgs::msg::Pose> v;
    v.push_back(pose);
    this->m_plan_execute(v);

}

/**
 * @brief set ori_quat to current pose orientation -> 
 * 
 */
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

moveit_msgs::msg::RobotTrajectory ArmPlanner::plan_speed_constrain(std::vector<Eigen::Vector3d> waypoints, double feedrate, bool follow_surface) {
    if (this->ori_quat == geometry_msgs::msg::Quaternion()) {
        std::cout << "Orientation quaternion not initialized! NOT PLANNING";
    }
    //find max and min of z values
    double zmax = std::numeric_limits<double>::min();
    double zmin = std::numeric_limits<double>::max();
    for (auto &v : waypoints) {
        zmin = std::min(v[2], zmin);
        zmax = std::max(v[2], zmax);
    }
    std::cout << "Inside plan_speed_constrain....\n";
    std::cout << "Z max is " << zmax << " Z min is " << zmin << "\n";
    bool zconst = (zmax - zmin < this->z_deviation);
    if (zconst) {
        follow_surface = false;
    }
    else {
        std::cout << "TRAJECTORY IS NON CONSTANT, diff > " << this->z_deviation << "\n";
        std::cout << "num points: " << waypoints.size() << "\n";
    }

    std::vector<geometry_msgs::msg::Pose> pose_waypoints;
    for (int i = 0; i < waypoints.size(); ++i) {
        Eigen::Vector4d wp2 = this->transfer(waypoints[i]);
        geometry_msgs::msg::Pose pose;
        pose.position.x = (float)wp2[0];
        pose.position.y = (float)wp2[1];
        pose.position.z = (float)wp2[2];
        pose.orientation = this->ori_quat;
        pose_waypoints.push_back(pose);
    }
    moveit_msgs::msg::RobotTrajectory trajectory;
    this->move_group_.computeCartesianPath(pose_waypoints, 0.001, 0.0, trajectory);
    return trajectory;

}



void ArmPlanner::gcode_motion(std::vector<Eigen::Vector4d> waypoints, double feedrate) {
    std::vector<Eigen::Vector3d> tr_waypoints = this->adjust_waypoints(waypoints);
    moveit_msgs::msg::RobotTrajectory plan = this->plan_speed_constrain(tr_waypoints, feedrate, false);
    this->move_group_.execute(plan);
}


