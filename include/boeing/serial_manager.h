#ifndef serial_manager_h
#define serial_manager_h
#include <unistd.h>
#include <vector>
#include <iostream>
#include <string>
#include <time.h> 
#include <stdlib.h>
#include <sstream>
#include <iomanip>
#include <cmath>
#include <boost/asio.hpp>

// static boost::asio::io_service ios;
// boost::asio::serial_port sp(ios, "/dev/ttyS2");
// sp.set_option(boost::asio::serial_port::baud_rate(115200));
// char tmp[64];
// auto length = sp.read_some(boost::asio::buffer(tmp));
// // process the info received
// std::string message = "hello, world";
// sp.write_some(boost::asio::buffer(message));
// sp.close();

const static std::string END_OF_GCODE = "##END##";

class SerialGcode {
    public:
        SerialGcode(std::string port, unsigned int baud_rate, bool verbose, int timeout);
        // SerialGcode(const SerialGcode &s);
        ~SerialGcode();


        std::vector<std::string> send_gcode(std::string command, bool allow_extrude);

        void monitor();

        void cleanup();

        void send_extr(double ex_len, double feedrate);

        boost::asio::io_service io;
        boost::asio::serial_port sp;
        double E = 0;
        double Etarg = 0;
        bool verbose;
        double timeout_s;


};


class ExtrusionManager {
    public:
        ExtrusionManager();
        ExtrusionManager(SerialGcode* ser, double ex_len, double feed_init, double lead_time, double send_freq, bool absolute_mode);
        ~ExtrusionManager() {this->ser->cleanup();}

        void update(double time_left);

        bool checkup();

        void send();

    private:
        SerialGcode* ser;
        double Esend;
        double ex_len;
        double feed_init;
        double lead_time = 0.1;
        double send_freq = 0.5;
        bool absolute = false;
        double feed;
        double lastE = std::nanf("");
        double lastStall = std::nanf("");
        double EAbsTarg = 0;
        int stall_count = 0;
    

};




#endif