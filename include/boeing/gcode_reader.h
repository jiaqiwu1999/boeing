#ifndef gcode_reader_h
#define gcode_reader_h
#include <string>
#include <iostream>
#include <fstream>
#include <stdio.h>
#include <ctype.h>
#include <string>
#include <vector>
#include <cmath>
#include <eigen3/Eigen/Dense>


int CMD_GCODE = 0;
int CMD_TRAJ = 1;

struct GcodeReaderOutput {
    int cmd_traj = CMD_TRAJ;
    std::vector<Eigen::Vector4d> traj; //2D array
    double extrude;
    double feedrate;
    std::string str;
};


class GcodeReader {
    private:
        std::string filepath;
        bool verbose;
        std::vector<std::string> lines;

    public:
        GcodeReader(std::string filepath, bool verbose);
        std::vector<GcodeReaderOutput> iter_cmds();

};

#endif