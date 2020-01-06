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





int openSerialPort() {
    int serial_port = open("/dev/ttyACM0", O_RDWR);

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

int main(int argc, char** argv) {
    std::shared_ptr<NameRecords> name(new NameRecords);
    WirelessMessageHandler handler(name);
    name->addNameRecord("husky1", 1);
    wireless_msgs::LoraPacket packet;
    packet.to.data = "husky1";
    for(int i = 0; i< 61; i++)
        packet.data.push_back('a');
  
    uint8_t buffer[255];
    int length = handler.serializeMessage(packet, buffer);
    std::cout <<  "buffer length "<< length <<std::endl;
    int serial_port = openSerialPort();
    //std::thread reader(readThread, serial_port);
    for(char j = 0; j  < 10; j++){
        
        char readString[255];
        int length = read(serial_port, readString, 255);
        if(length > 0)
            std::cout << readString;
        else
            std::cout<< "no response" <<std::endl;
     }
    return 0;
}