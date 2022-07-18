#include "boeing/traj_perc.h"

TrajectoryTracker::TrajectoryTracker(std::vector<Eigen::VectorXd> traj, double thr=0.00001) :
    t(traj),
    thr(thr)
{
    std::vector<Eigen::VectorXd> next = traj;
    next.erase(next.begin());

    typedef std::pair<Eigen::VectorXd, Eigen::VectorXd> vec_pair;

    std::vector<vec_pair> pairs;

    for (int i = 0; i < next.size(); ++i) {
        traj_len += (next[i] - traj[i]).norm();
        vec_pair curr_pair(traj[i], next[i]);
        pairs.push_back(curr_pair);
    }

    this->iter = pairs.begin();

    this->pair_points = pairs;

    vec_pair curr_pair = *iter;

    this->p0 = curr_pair.first;
    this->p1 = curr_pair.second;
};

double TrajectoryTracker::get_perc(std::vector<double> newpt) {
    Eigen::Map<Eigen::VectorXd> eigen_pt(newpt.data(), newpt.size()); 
    //Eigen::VectorXf eigen_pt = eigen_pt.cast<float>();
    while (true) {
        Eigen::VectorXd p00 = p0;
        Eigen::VectorXd p01 = p1;
        Eigen::VectorXd v_traj = p1 - p0;
        Eigen::VectorXd v_pt = eigen_pt - p0;
        double dist_along_traj = v_pt.dot(v_traj) / sqrt(v_traj.dot(v_traj));
        double dist_from_line = sqrt(v_pt.dot(v_pt) - dist_along_traj * dist_along_traj);
        double percentage = (dist_along_traj + this->dist_so_far) / this->traj_len;
        double perc_acc = 2 * this->thr / this->traj_len;

        if (dist_from_line > thr || dist_along_traj > v_traj.norm() || percentage - this->max_perc < (-1)*perc_acc) {
            dist_so_far += v_traj.norm();
            if (++iter != pair_points.end()) {
                vec_pair next_pair = *iter;
                this->p0 = next_pair.first;
                this->p1 = next_pair.second;
                continue;
            }
            else {
                break;
            }
        }
        max_perc = std::max(percentage, this->max_perc);
        return percentage;
    }
    return 1;

};