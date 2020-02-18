#!/usr/bin/env python3
import rospy
import subprocess
from wifi_sensor.msg import wifi, wifiArray
from std_msgs.msg import String
from nav_msgs.msg import Odometry

odom = Odometry()

# Change the wifi device as shown in the ifconfig
wifi_device = "wlp2s0"
cmd = "nmcli device wifi rescan && iwlist %s scan | egrep 'Cell |Encryption|Quality|Last beacon|ESSID'" % wifi_device

def odom_callback(msg):
    odom = msg.data

def talker():
    pub = rospy.Publisher('/wifi_ap', wifiArray, queue_size=10)
    sub = rospy.Subscriber('/odom',Odometry,odom_callback)
    rospy.init_node('wifi_sensor_node', anonymous=True)
    print("wifi_sensor_node Initialised")

    while not rospy.is_shutdown():
        try:
            process = subprocess.Popen(cmd,
                                       stdout=subprocess.PIPE,
                                       shell=True)
            # Shell=True : Allows the command to be a single string

            stdout = process.communicate()
            stdout = str(stdout).split("Cell")
            entries = []
            for line in stdout[1:]:
                entry = wifi()
                data = line.split("\\n")
                for item in data:
                    item = item.strip()
                    if "Address:" in item:
                        temp = String()
                        temp.data = item.split(" ")[-1]
                        entry.mac_address = temp

                    elif "Quality=" in item:
                        i = item.split(" ")
                        temp = String()
                        temp.data = i[0].split("=")[-1]
                        entry.quality = temp
                        temp2 = String()
                        temp2.data = i[3].split("=")[-1]
                        entry.signal = temp2

                    elif "ESSID:" in item:
                        temp = String()
                        temp.data = item.split(":")[1].replace("\"","")
                        entry.ssid = temp

                    elif "Extra:" in item:
                        temp = String()
                        temp.data = item.split(":")[-1].strip()
                        entry.last_beacon = temp

                entry.odom = odom
                # Reduces the number of wifi entries to only darpa related ones
                if "darpa" in entry.ssid.data:
                    entries.append(entry)

            msg = wifiArray()
            msg.count = len(entries)
            msg.data = entries

            pub.publish(msg)
            print("new message published")
            
            rospy.sleep(20)

        except KeyboardInterrupt:
            print("Shutting down wifi_sensor_node")


if __name__ == '__main__':
    try:
        talker()
    except rospy.ROSInterruptException:
        print("Wifi_sensor_node stopped.")