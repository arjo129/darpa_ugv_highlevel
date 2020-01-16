#include <ros/ros.h>
#include <wireless_msgs/LoraPacket.h>
#include <unordered_map>
#include <iostream>
#include <fstream>
#include <thread> 
#include <chrono>
#include <teensy_bridge/SerialPackets.h>
#include <fcntl.h> // Contains file controls like O_RDWR
#include <errno.h> // Error integer and strerror() function
#include <termios.h> // Contains POSIX terminal control definitions
#include <unistd.h> 





int openSerialPort(const char* port) {
    int serial_port = open(port, O_RDWR);

    // Create new termios struc, we call it 'tty' for convention
    struct termios tty;
    memset(&tty, 0, sizeof tty);

    // Read in existing settings, and handle any error
    if(tcgetattr(serial_port, &tty) != 0) {
        printf("Error %i from tcgetattr: %s\n", errno, strerror(errno));
    }

    tty.c_cflag &= ~PARENB; // Clear parity bit, disabling parity (most common)
    tty.c_cflag &= ~CSTOPB; // Clear stop field, only one stop bit used in communication (most common)
    tty.c_cflag |= CS8; // 8 bits per byte (most common)
    tty.c_cflag &= ~CRTSCTS; // Disable RTS/CTS hardware flow control (most common)
    tty.c_cflag |= CREAD | CLOCAL; // Turn on READ & ignore ctrl lines (CLOCAL = 1)

    tty.c_lflag &= ~ICANON;
    tty.c_lflag &= ~ECHO; // Disable echo
    tty.c_lflag &= ~ECHOE; // Disable erasure
    tty.c_lflag &= ~ECHONL; // Disable new-line echo
    tty.c_lflag &= ~ISIG; // Disable interpretation of INTR, QUIT and SUSP
    tty.c_iflag &= ~(IXON | IXOFF | IXANY); // Turn off s/w flow ctrl
    tty.c_iflag &= ~(IGNBRK|BRKINT|PARMRK|ISTRIP|INLCR|IGNCR|ICRNL); // Disable any special handling of received bytes

    tty.c_oflag &= ~OPOST; // Prevent special interpretation of output bytes (e.g. newline chars)
    tty.c_oflag &= ~ONLCR; // Prevent conversion of newline to carriage return/line feed
    // tty.c_oflag &= ~OXTABS; // Prevent conversion of tabs to spaces (NOT PRESENT ON LINUX)
    // tty.c_oflag &= ~ONOEOT; // Prevent removal of C-d chars (0x004) in output (NOT PRESENT ON LINUX)

    tty.c_cc[VTIME] = 10;    // Wait for up to 1s (10 deciseconds), returning as soon as any data is received.
    tty.c_cc[VMIN] = 0;

    // Set in/out baud rate to be 9600
    cfsetispeed(&tty, B9600);
    cfsetospeed(&tty, B9600);

    // Save tty settings, also checking for error
    if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
        printf("Error %i from tcsetattr: %s\n", errno, strerror(errno));
    }
    return serial_port;
}

/**
 * Temporary TEENSY 4.0 workaround
 */ 
void writeSerialPort(int serial_port, uint8_t* buffer, int length) {
    for(int i = 0; i <length; i+=64){
        if(length < i+64)
            write(serial_port, buffer+i, length-i*64);
        else
            write(serial_port, buffer+i, 64);
    }
}

class TeensyBridgeNode {
    ros::NodeHandle nh;
    ros::Publisher pub;
    ros::Subscriber sub;
    std::shared_ptr<NameRecords> names;
    WirelessMessageHandler handler;
    int serialPort;
    SerialParser parser;
    void onWirelessMessageRecieved(wireless_msgs::LoraPacket pkt) {
        uint8_t buffer[255];
        int length = handler.serializeMessage(pkt, buffer);
        std::cout << "sending " <<std::endl;
        for (int i = 0; i < length; i++){
            std::cout << " " <<(int) buffer[i] ;
        }
        std::cout << std::endl;
        writeSerialPort(serialPort, buffer, length);
    }

   
public:

    void spin() {
        char buffer[260];
        int length = read(serialPort, buffer, 255);
        if(length == 0) {
            std::cout << "no data recv" << std::endl;
        }
        for (int i = 0; i < length; i++){
            
            if(parser.addByteToPacket(buffer[i])){
                if(parser.getMessageType() == SerialResponseMessageType::PACKET_RECIEVED) {
                    wireless_msgs::LoraPacket pkt = parser.retrievePacket();
                    pub.publish(pkt);
                }
            }
        }
    }

    TeensyBridgeNode(ros::NodeHandle _nh): nh(_nh), names(new NameRecords), handler(names), parser(names){
        pub = this->nh.advertise<wireless_msgs::LoraPacket>("/rx",10);
        sub = this->nh.subscribe("/tx", 10, &TeensyBridgeNode::onWirelessMessageRecieved, this);
        std::string port;
        this->nh.getParam("serial_port", port);
        serialPort = openSerialPort("/dev/ttyACM0");
        names->addNameRecord("husky1", 1);
    }
};

int main(int argc, char** argv) {

    ros::init(argc, argv, "teensy_bridge");
    ros::NodeHandle nh("~");
    TeensyBridgeNode node(nh);
    ros::Rate rate(100);
    while(ros::ok()){
        ros::spinOnce();
        node.spin();
        rate.sleep();
    }
    return 0;
}