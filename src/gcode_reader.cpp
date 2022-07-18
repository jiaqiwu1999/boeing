#include "boeing/gcode_reader.h"

GcodeReader::GcodeReader(std::string filepath, bool verbose) : 
    filepath(filepath), 
    verbose(verbose) 
    {
    std::ifstream fin(filepath);
    std::string line;
    if (!fin.is_open()) {
        std::cout << "Could not open file, something's wrong.." << "\n";
    }
    else {
        while (std::getline(fin, line)) {
            lines.push_back(line);
        }
        fin.close();
    }
    
}

std::vector<GcodeReaderOutput> GcodeReader::iter_cmds() {
    int xcoord = 0;
    int ycoord = 0;
    int zcoord = 3;
    double feedrate = 1200;

    double extrudeLen_sum = 0;
    bool relative = false;
    bool relativeExtruder = false;

    std::vector<GcodeReaderOutput> outputs;
    std::vector<Eigen::Vector4d> current_traj;
    // auto *p = &current_traj;
    // p = NULL;

    for (auto line : this->lines) {
        if (line == "\n" || line == "\r\n") {
            continue;
        }
        std::size_t pos = line.find(';');
        std::string command = line.substr(0, pos);
        bool send_flag = false;

        //if line is empty, skip
        if (command.size() <= 1) {
            continue;
        }

        //if line doesn't end in \n, make it so
        //note: the array shouldn't make unauthorized accesses b/c 
        //valid gcode is always at least 2 chars long

        while (command[command.size() - 1] != '\n') {
            if (isalnum(command[command.size() - 1])) {
                command = command + '\n';
                break;
            }
            else {
                command.pop_back();
            }
        }
        if (this->verbose) {
            printf("Parsing: %s \n", command);
        }

        //Parse commands that change relativity of commands
        if (command.find("G90") != std::string::npos) {
            relative = false;
            relativeExtruder = false;
        }
        if (command.find("G91") != std::string::npos) {
            relative = true;
            relativeExtruder = true; 
        }
        if (command.find("M82") != std::string::npos) {
            relativeExtruder = false;
        }
        if (command.find("M83") != std::string::npos) {
            relativeExtruder = true;    
        }
        //G28 - Home Command
        if (command.find("G28") != std::string::npos) {
            //TODO
            std::cout << "[0, 0, 0, 0]" << "\n";
            GcodeReaderOutput curr_output;
            Eigen::Vector4d t(0, 0, 3, 1);
            curr_output.traj.push_back(t);
            curr_output.extrude = 0;
            curr_output.feedrate = feedrate;
            curr_output.cmd_traj = CMD_TRAJ;
            outputs.push_back(curr_output);
            continue;
        }
        double extrudeLen = 0;
        double feedrate_nxt = std::nanf("");
        Eigen::Vector4d next_pt;

        //G0-3 - Move command
        if (command.find("G0") != std::string::npos || command.find("G1") != std::string::npos || command.find("G2") != std::string::npos || command.find("G3") != std::string::npos) {
            std::vector<std::string> words;
            size_t pos = 0;
            std::string space_delimiter = " ";
            while ((pos = command.find(space_delimiter)) != std::string::npos) {
                words.push_back(command.substr(0, pos));
                command.erase(0, pos + space_delimiter.length());
            }
            for (const auto &term : words) {
                if (term[0] == 'X') {
                    double x = std::stod((term.substr(1)));
                    if (!relative) {
                        xcoord = x;
                    }
                    else {
                        xcoord = xcoord + x;
                    }
                }      
                else if (term[0] == 'Y') {
                    double y = std::stod((term.substr(1)));
                    if (!relative) {
                        ycoord = y;
                    }   
                    else {
                        ycoord = ycoord + y;
                    }
                }
                else if (term[0] == 'Z') {
                    double z = std::stod((term.substr(1)));
                    if (!relative) {
                        zcoord = z;
                    }
                    else {
                        zcoord = zcoord + z;
                    }   
                }
                else if (term[0] == 'E') {
                    if (relativeExtruder) {
                        extrudeLen = std::stod((term.substr(1)));
                    }
                    else {
                        double e = std::stod(term.substr(1));
                        //extrudeLen = e - ecord;
                        double ecord = e; //TODO ?
                    }
                 }
                else if (term[0] == 'F') {
                    send_flag = true;
                    feedrate_nxt = std::stod(term.substr(1));
                }
                next_pt << xcoord, ycoord, zcoord, 1;
                current_traj.push_back(next_pt);
            }
        }
        // G92 marks end of layer; send trajectory
        if (!current_traj.empty() && command.find('G92') != std::string::npos) {
            send_flag = true;
        }
        if (command.find('G0') != std::string::npos) {
            send_flag = true;
        }   
        if (extrudeLen_sum > 850) {
            send_flag = true;
        }
            
        if (send_flag) {
            send_flag = false;
            if (!current_traj.empty()) {
                if (verbose) {
                    printf("Total extrusion: %f; feedrate: %f", extrudeLen_sum, feedrate);
                }
                GcodeReaderOutput curr_output;
                curr_output.extrude = extrudeLen_sum;
                curr_output.traj = current_traj;
                curr_output.feedrate = feedrate;
                curr_output.cmd_traj = CMD_TRAJ;
                outputs.push_back(curr_output);
            }
            current_traj.clear();
            extrudeLen_sum = 0;

        }

        //G1 saves to a group
        if (command.find('G1') != std::string::npos) {
            extrudeLen_sum += extrudeLen;
            current_traj.push_back(next_pt);
            if (std::isnan(feedrate_nxt)) {
                feedrate = feedrate_nxt;
            }
        }

        if (command.find('G0') != std::string::npos) {
            GcodeReaderOutput curr_output;
            curr_output.feedrate = feedrate;
            curr_output.extrude = 0;
            curr_output.traj.push_back(next_pt);
            curr_output.cmd_traj = CMD_TRAJ;
            outputs.push_back(curr_output);
        }
        //TODO
        GcodeReaderOutput cmd_output;
        cmd_output.cmd_traj = CMD_GCODE;
        cmd_output.str = command;
        outputs.push_back(cmd_output);
    }
    if (!current_traj.empty()) {
        if (verbose) {
            std::cout << "Sending trajectory points...\n";
            for (auto & t : current_traj) {
                std::cout << t << "\n";
            }
            printf("Total extrusion: %f; feedrate: %f", extrudeLen_sum, feedrate);
        }
        GcodeReaderOutput curr_output;
        curr_output.extrude = extrudeLen_sum;
        curr_output.feedrate = feedrate;
        curr_output.traj = current_traj;
        curr_output.cmd_traj = CMD_TRAJ;
        outputs.push_back(curr_output);
    }

    return outputs;
}

