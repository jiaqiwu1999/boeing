#include "boeing/serial_manager.h"


SerialGcode::SerialGcode(std::string port, unsigned int baud_rate, bool verbose, double timeout) : 
    io(),
    sp(io, port), 
    verbose(verbose), 
    timeout_s(timeout) 
{
    this->sp.set_option(boost::asio::serial_port_base::baud_rate(baud_rate));
    if (verbose) {
        std::cout << "Waiting...\n";
    }
    sleep(3);
    if (verbose) {
        std::cout << "Flushing input now...\n";
    }
    //serial read here?
    
};

// SerialGcode::SerialGcode(const SerialGcode &s) : 
//     {
//         this->sp = s.sp;
        

//     }

SerialGcode::~SerialGcode() {
    this->cleanup();
}

std::vector<std::string> SerialGcode::send_gcode(std::string command, bool allow_extrude=true) {
    if (!allow_extrude) {
        if (command.find("G1") != std::string::npos && command.find("E") != std::string::npos) {
            return;
        }
    }
    clock_t t0 = clock();
    //serial write
    std::vector<std::string> outlines;
    //TODO
    //add end of file detection
    while ((clock() - t0) / CLOCKS_PER_SEC < timeout_s) {
        std::string line;
        //TODO serial read line
        char c;
        for (;;) {
            boost::asio::read(this->sp, boost::asio::buffer(&c, 1));
            if (c == '\n') {
                break;
            }
            else {
                line += c;
                if (line == "TODO - END OF FILE") {
                    break;
                }
            }
        }
        if (line == "END OF FILE") {
            break;
        }
        outlines.push_back(line);
        if (verbose) {
            std::cout << line;
        }
        if (line.find("ok") != std::string::npos) {
            return outlines;
        }
        if (line.find("echo:busy: processing") != std::string::npos) {
            t0 = clock();
        }
        else if (line.find("error") != std::string::npos || line.find("Error") != std::string::npos) {
            std::cout << "Error message received: Quitting print \n";
            std::cout << line << std::endl;
            //serial write
            boost::asio::write(this->sp, boost::asio::buffer(line.c_str(), line.size()));
            exit(1);
        }

    }
    std::cout << "Timed Out: Quitting print \n";
    //serial write
    std::string line = "M107\n";
    boost::asio::write(this->sp, boost::asio::buffer(line.c_str(), line.size()));
    exit(1);
};


void SerialGcode::monitor() {
    bool v = verbose;
    this->verbose = false;
    std::vector<std::string> response = send_gcode("M114 R\n");
    if (response.empty()) {
        std::cout << "Somehow 'ok' is missing from the transaction\n";
        return;
    }
    std::string zz = response[response.size() - 2];
    std::string delim = " ";
    std::vector<std::string> blocks;
    size_t pos = 0;
    while ((pos = zz.find(delim)) != std::string::npos) {
        blocks.push_back(zz.substr(0, pos));
        zz.erase(0, pos + delim.length());
    }
    for (std::string block : blocks) {
        if (block[0] == 'E') {
            Etarg = std::stod(block.substr(2));
        }

    }
    this->verbose = v;
};

void SerialGcode::cleanup() {
    this->sp.close();
    if (this->verbose) {
        std::cout << "Closed serialVar!";
    }

};


void SerialGcode::send_extr(double ex_len, double feedrate) {
    std::stringstream gcode_stream;
    std::setprecision(5);
    gcode_stream << "G1 E" << ex_len << " F" << feedrate << "\n";
    std::string gcode_str = gcode_stream.str();
    bool v = verbose;
    verbose = false;
    auto resp = send_gcode(gcode_str);
    verbose = v;
};


ExtrusionManager::ExtrusionManager(SerialGcode* s, double ex_len, double feed_init, double lead_time, double send_freq, bool absolute_mode) :
    ser(s), 
    ex_len(ex_len), 
    feed_init(feed_init), 
    lead_time(lead_time), 
    send_freq(send_freq), 
    absolute(absolute_mode) 
{
    ser->monitor();
    while (ser->E != 0) {
        ser->send_gcode("G92 E0.0\n", true);
        ser->monitor();
    }
    Esend = ex_len;
    feed = feed_init;

}

void ExtrusionManager::update(double time_left) {
    Esend = ex_len - ser->E;
}

bool ExtrusionManager::checkup() {
    this->ser->monitor();

    if (ser->E >= ex_len) {
        return false;
    }

    if (ser->E == lastStall) {
        return false;
    }

    printf("-------- STATUS %.2f / (%.2f,%.2f)-------", this->ser->E, EAbsTarg, this->ser->Etarg);

    if (this->ser->E == lastE && this->ser->E != this->ser->Etarg) {
        stall_count += 1;
        if (stall_count < 1) { //?
            return false;
        }
        std::cout << "EXECUTION STALLED~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~~````";
        lastStall = this->ser->E;
        return true;
    }

    lastE = this->ser->E;
    stall_count = 0;
    return false;

}

void ExtrusionManager::send() {
    std::setprecision(4);
    std::stringstream ss;
    ss << "Sending [" << Esend << "] at [" << feed << "]";
    this->ser->send_extr(Esend, feed);

}
