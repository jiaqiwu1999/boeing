#ifndef traj_perc_h
#define traj_perc_h
#include <vector>
#include <algorithm>
#include <cmath>
#include <eigen3/Eigen/Dense>

class TrajectoryTracker {
    typedef std::pair<Eigen::VectorXd, Eigen::VectorXd> vec_pair;
    public:
        TrajectoryTracker(std::vector<Eigen::VectorXd> traj, double thr);

        double get_perc(std::vector<double> newpt);

    private:
        std::vector<Eigen::VectorXd> t;

        std::vector<vec_pair>::iterator iter;
        std::vector<vec_pair> pair_points;
        Eigen::VectorXd p0;
        Eigen::VectorXd p1;

        double traj_len = 0;
        double thr = 0.00001;
        double dist_so_far = 0;
        double max_perc = 0;

};


#endif