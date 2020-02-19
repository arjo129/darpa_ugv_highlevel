#include <ros/ros.h>
#include <wireless_msgs/LoraPacket.h>
#include <wireless_msgs/WifiArray.h>
#include <wireless_msgs/Co2.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
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
#include <tf/tf.h>
#include <tf/transform_listener.h>

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
	std::cout <<"writing stuff" <<std::endl; 
    for(int i = 0; i <length; i+=63){
        std::cout <<buffer[i] <<std::endl;
        if(length < i+63)
            write(serial_port, buffer+i, length-i);
        else
            write(serial_port, buffer+i, 63);
    }
    std::cout << "hi"<< std::endl;
}

class TeensyBridgeNode {
    ros::NodeHandle nh;
    ros::Publisher pub, addressPublisher, co2Publisher;
    ros::Subscriber sub;
    tf::TransformListener* transformListener;
    image_transport::Publisher pubThermal;
    std::shared_ptr<NameRecords> names;
    WirelessMessageHandler handler;
    wireless_msgs::LoraPacket mostRecentPacket;
    int serialPort;
    bool messageQueueEmpty = false;
    bool new_msg = false;
   
    void onWirelessMessageRecieved(wireless_msgs::LoraPacket pkt) {
        if(pkt.data.size() > 255){
            ROS_ERROR("packet size greater than 255. Not sending.");
            return;
        }
        ROS_INFO("sending packet");
        mostRecentPacket = pkt;
        new_msg = true;
    }


    void sendMsg() {
        uint8_t buffer[255];
        int length = handler.serializeMessage(mostRecentPacket, buffer);
        std::cout << "status "<< new_msg<< std::endl;
        if(this->messageQueueEmpty & new_msg){
            std::cout << "hello " <<std::endl;
            writeSerialPort(serialPort, buffer, length);
            this->messageQueueEmpty =false;
            new_msg = false;
        }
    }

   
public:

    void spin() {
        static SerialParser parser(names);
        char buffer[260];
        int length = read(serialPort, buffer, 255);
        if(length == 0) {
            std::cout << "no data recv" << std::endl;
        }
        for (int i = 0; i < length; i++){
            std::cout <<  buffer[i] << " ";
            if(parser.addByteToPacket(buffer[i])){
                if(parser.getMessageType() == SerialResponseMessageType::PACKET_RECIEVED) {
                    wireless_msgs::LoraPacket pkt = parser.retrievePacket();
                    pub.publish(pkt);
                }
                if(parser.getMessageType() == SerialResponseMessageType::LORA_STATUS_READY) {
                    this->messageQueueEmpty = true;
                    std::cout << "Queue Empty"<< std::endl;
                    parser.reset();

                }
                if (parser.getMessageType() == SerialResponseMessageType::THERMAL_FRONT) {
                    std::cout << "retrieve packet" << std::endl;
                    cv::Mat img = parser.retrieveThermalPacket();
                    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "mono8", img).toImageMsg();
		            pubThermal.publish(msg);                
                }
                if (parser.getMessageType() == SerialResponseMessageType::PHYSICAL_ADDRESS) {
                    addressPublisher.publish(parser.retrieveLoraInfo());
		            parser.reset();
                }
             /*   if(parser.getMessageType() == SerialResponseMessageType::CO2_SENSOR_READING) {
                    wireless_msgs::Co2 reading;
                    reading.concentration = parser.retrieveCo2Packet();
                    try{
                        tf::StampedTransform stampedTransform;
                        ros::Time time = ros::Time::now();
                        transformListener->waitForTransform("base_link", "darpa", time, ros::Duration(1.0));
                        transformListener->lookupTransform("base_link", "darpa", time, stampedTransform);
                        reading.position.x = stampedTransform.getOrigin().x();
                        reading.position.y = stampedTransform.getOrigin().y();
                        reading.position.z = stampedTransform.getOrigin().z();
                    } catch(tf::LookupException ex) {
                        ROS_WARN("Failed go get odometry, sending default");
                    }
                    co2Publisher.publish(reading);

                }*/
            }
        }
        std::cout << std::endl;
        sendMsg();
    }

    TeensyBridgeNode(ros::NodeHandle _nh): nh(_nh), names(new NameRecords), handler(names) {
        pub = this->nh.advertise<wireless_msgs::LoraPacket>("/lora/rx",10);
        addressPublisher = this->nh.advertise<wireless_msgs::LoraInfo>("/lora/info",10);
        co2Publisher = this->nh.advertise<wireless_msgs::Co2>("/co2", 10);
        sub = this->nh.subscribe("/lora/tx", 10, &TeensyBridgeNode::onWirelessMessageRecieved, this);
        std::string port;
        this->nh.getParam("serial_port", port);
        serialPort = openSerialPort("/dev/teensy");
        names->addNameRecord("base_station", 0);
        //thermal camera image publishing stuff
        image_transport::ImageTransport it(nh);
	    pubThermal = it.advertise("/thermal_front/image_raw", 1);
        transformListener = new tf::TransformListener();
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
