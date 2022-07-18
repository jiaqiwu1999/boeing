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

  //verbose makes the program print a lot more information
  //set to false if you only want matrix output
  bool verbose = true;

  // disableExtrusions - prevents G1 extrusions from being sent, mainly so
  // another command can deal with the movement/extrusion
  bool disableExtrusions = false;

  //Extrusion 'response' time. When does filament start moving after extr command sent.
  double extr_lead = .1;  //s
  double extr_freq = 1  //s (actually a period)

  //-------------------------------------
  rclcpp::init(argc, argv);

  auto ap = std::make_shared<ArmPlanner>(verbose, zheight, setheight, step_size);
  //rclcpp::spin(ap);
  //ArmPlanner ap(verbose, zheight, setheight, step_size);
  ap->m_home();

  ap->run_calibration_process();

  if (verbose) {
    std::cout << "Stargin readGcoddeV3.cpp\n";
  }

  static boost::asio::io_service ios;
  boost::asio::serial_port sp(ios, "TODO");
  sp.set_option(boost::asio::serial_port::baud_rate(250000));

  SerialGcode ser = SerialGcode(sp, verbose, timeout_seconds);
  

  if (verbose) {
    std::cout << "Ready\n";
  }

  std::string filePath = "C:\\Users\\Kevin\\Desktop\\boeing\\gcodes\\test1.gcode";

  if (argc == 2) {
    filePath = argv[1];
  }

  GcodeReader greader(filePath, verbose);

  std::vector<GcodeReaderOutput> reader_output = greader.iter_cmds();

  for (GcodeReaderOutput &output : reader_output) {
    if (output.cmd_traj == CMD_TRAJ) {
      double feedrate = output.feedrate;
      feedrate = std::min(feedrate, 3000);
      double ex_len = output.extrude;
      std::vector<Eigen::Vector4d> traj = output.traj;
      std::vector<Eigen::Vector3d> waypoints = ap->adjust_waypoints(traj);
      //TODO pass tf by reference? ..
      double tf;
      moveit_msgs::RobotTrajectory plan = ap->plan_speed_constrain(waypoints, feedrate, (ex_len > 0));
      double extr_feed = ex_len * 60 / tf;
      //TODO
      std::vector<Eigen::VectorXd> combined_points;
      for (auto &pt : plan.joint_trajectory.points) {
          Eigen::VectorXf pt_f(pt.data());
          Eigen::VectorXd curr_points = pt_f.cast<double>();
          combined_points.push_back(curr_points);
      }
      TrajectoryTracker tt = TrajectoryTracker(combined_points, 0.005);
      
      if (ex_len > 0) {
        //ap->mesh_build.feed_points(combined_points);
        ExtrusionManager ex_man(&ser, ex_len, extr_feed, extr_lead, extr_freq);
        ex_man.send();

        clock_t wait_start = clock();

        while (ser.E == 0 && (clock() - wait_start) / CLOCKS_PER_SEC < 1) {
          ser.monitor();
        }
      }

      if (ex_len < 0) {
        ser.send_gcode("G92 E0.0\n") //reset extr
        ser.monitor();
        int ex_status = ser.E;
        ser.send_extr(ex_len, extr_feed);
        clock_t wait_start = clock();
        while (ser.E == ex_status && (clock() - wait_start) / CLOCKS_PER_SEC < 1) {
          ser.monitor();
        }
      }

      std::cout << "Sending plan to execution!\n";
      std::cout << "Finish time (est): " << tf << "\n";
      ap->move_group_.execute(plan);

      clock_t t0 = clock();
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
          if (ex_man.checkup()) {
            double time_left = (1 - percent) * tf;
            std::cout << "time_left: " << time_left << "\n";
            ex_man.update(time_left);
            ex_man.send();
          }
        }

        if (ex_len != 0) {
          int prv = std::numeric_limits<int>::infinity;
          while (ser.E != prev) {
            prv = ser.E;
            ser.monitor();
          }
          std::cout << "Final E: " << ser.E << "\n";
        }
      }

      elif (output.cmd_traj == CMD_GCODE) {
        if (disableExtrusions) {
          continue;
        }
        ser.send_gcode(output.str, false);

      }

    }
  }
    rclcpp::shutdown();
  return 0;
}