#include <stdlib.h>
#include <string>
#include <vector>
#include <cmath>
#include <iostream>
#include <limits>
#include <time.h>
#include <eigen3/Eigen/Dense>
#include <rclcpp/rclcpp.hpp>
#include <boost/asio.hpp>

#include "boeing/serial_manager.h"
#include "boeing/arm_planner.h"
#include "boeing/gcode_reader.h"
#include "boeing/traj_perc.h"
// #include <std_msgs/double64.h>

int main(int argc, char * argv[])
{
  int timeout_seconds = 30;

  //Bed leveling parameters
  double zheight = -0.058;
  double height_t = 0.000;
  double step_size = .0003;
  double setheight = 0.0001;

  std::string port = "/dev/ttyUSB0";

  unsigned int baud_rate = 250000;

  //verbose makes the program print a lot more information
  //set to false if you only want matrix output
  bool verbose = true;

  // disableExtrusions - prevents G1 extrusions from being sent, mainly so
  // another command can deal with the movement/extrusion
  bool disableExtrusions = false;

  //Extrusion 'response' time. When does filament start moving after extr command sent.
  double extr_lead = .1;  //s
  double extr_freq = 1;  //s (actually a period)
  double force_th = 1;
  ExtrusionManager *ex_man_ptr;

  //-------------------------------------
  rclcpp::init(argc, argv);

  auto ap = std::make_shared<ArmPlanner>(step_size, setheight, zheight, force_th, verbose);
  //rclcpp::spin(ap);
  //ArmPlanner ap(verbose, zheight, setheight, step_size);
  ap->m_home();

  rclcpp::shutdown();
  return 0;

  ap->run_calibration_process();

  if (verbose) {
    std::cout << "Stargin readGcoddeV3.cpp\n";
  }

  // static boost::asio::io_service ios;
  // boost::asio::serial_port sp(ios, "TODO");
  // sp.set_option(boost::asio::serial_port::baud_rate(250000));

  SerialGcode ser = SerialGcode(port, baud_rate, verbose, timeout_seconds);
  

  if (verbose) {
    std::cout << "Ready\n";
  }

  std::string filePath = "~/Desktop/short_test.gcode";

  if (argc == 2) {
    filePath = argv[1];
  }

  GcodeReader greader(filePath, verbose);

  std::vector<GcodeReaderOutput> reader_output = greader.iter_cmds();

  for (GcodeReaderOutput &output : reader_output) {
    if (output.cmd_traj == CMD_TRAJ) {
      double feedrate = output.feedrate;
      feedrate = std::min(feedrate, 3000.0);
      double ex_len = output.extrude;
      std::vector<Eigen::Vector3d> waypoints = ap->adjust_waypoints(output.traj);
      //TODO, ?
      double tf = 10;
      moveit_msgs::msg::RobotTrajectory plan = ap->plan_speed_constrain(waypoints, feedrate, (ex_len > 0));
     // double extr_feed = ex_len * 60 / tf;
      double extr_feed = feedrate;
      //TODO
      std::vector<Eigen::VectorXd> combined_points; //waypoints_np = []
      for (auto &pt : plan.joint_trajectory.points) {
          auto vec = Eigen::VectorXd(pt.positions.size());
          for (int i = 0; i < pt.positions.size(); ++i) {
            vec[i] = (double)pt.positions[i];

          }
          combined_points.push_back(vec);
      }
      TrajectoryTracker tt = TrajectoryTracker(combined_points, 0.005);
      
      if (ex_len > 0) {
        //ap->mesh_build.feed_points(combined_points);
        ExtrusionManager ex_man(&ser, ex_len, extr_feed, extr_lead, extr_freq, false);
        ex_man_ptr = &ex_man;
        ex_man_ptr->send();

        clock_t wait_start = clock();

        while (ser.E == 0 && (clock() - wait_start) / CLOCKS_PER_SEC < 1) {
          ser.monitor();
        }
      }

      if (ex_len < 0) {
        std::string ss = "G92 E0.0\n";
        ser.send_gcode(ss, true); //reset extr
        ser.monitor();
        double ex_status = ser.E;
        ser.send_extr(ex_len, extr_feed);
        clock_t wait_start = clock();
        while (ser.E == ex_status && (clock() - wait_start) / CLOCKS_PER_SEC < 1) {
          ser.monitor();
        }
      }

      std::cout << "Sending plan to execution!\n";
      //std::cout << "Finish time (est): " << tf << "\n";
      ap->move_group_.execute(plan);

      clock_t t0 = clock();
      //TODO tf is hardcoded
      while ((clock() - t0) / CLOCKS_PER_SEC < 1.2 * tf) {
        std::vector<double> joint_values = ap->move_group_.getCurrentJointValues();
        double percent = tt.get_perc(joint_values);
        //check whether percent is close to 1
        if (std::abs(percent - 1) <= (1e-4 + 1e-5)) {
          std::cout << "Detected end-of-trajectory!\n";
          std::cout << "Finish time (act): " << (clock() - t0) / CLOCKS_PER_SEC << "\n";
          break;
        }
        if (ex_len > 0) {
          if (ex_man_ptr->checkup()) {
            double time_left = (1 - percent) * tf;
            std::cout << "time_left: " << time_left << "\n";
            ex_man_ptr->update(time_left);
            ex_man_ptr->send();
          }
        }
      }
      if (ex_len != 0) {
        int prv = INFINITY;
        while (ser.E != prv) {
          prv = ser.E;
          ser.monitor();
        }
        std::cout << "Final E: " << ser.E << "\n";
      }
    }
    else if (output.cmd_traj == CMD_GCODE) {
      if (disableExtrusions) {
        continue;
      }
      ser.send_gcode(output.str, false);
    }
  }
  rclcpp::shutdown();
  return 0;
}