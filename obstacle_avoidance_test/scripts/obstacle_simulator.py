from geometry_msgs.msg import Twist
from sensor_msgs.msg import LaserScan
from PIL import Image
from rospkg import RosPack
from os.path import join
from bresenham import bresenham
from math import pi, sin, cos, atan2
import numpy as np
import rospy


def rgb2gray(rgb):
    return np.dot(rgb[...,:3], [0.2989, 0.5870, 0.1140])

class ObstacleAvoidanceMap:
    def __init__(self, filepath):
        im = np.array(Image.open(filepath))
        self.map = rgb2gray(im).T
        w,h = np.shape(self.map)
        self.obstacles = set()
        for x in range(w):
            for y in range(h):
                if self.map[x,y] == 0:
                    self.obstacles.add((x,y))  
    
    def get_size(self):
        return np.shape(self.map)
    
    def get_first_collision(self, start, end, radius=3):
        """
        Given a start and an end, return the first collision 
        """
        points = bresenham(int(start[0]), int(start[1]), int(end[0]), int(end[1]))
        inflated_path = []
        
        for p in points:
            inflated_path.append(p)
            for i in range(radius):
                for j in range(radius):
                    inflated_path.append((p[0]+i, p[1]+j))
        
        collisions = []
        for p in inflated_path:
            if p in self.obstacles:
                collisions.append(p)

        min_dist = float("inf")
        for p in collisions:
            dist = ((start[0] - p[0])**2 + (start[1] - p[1])**2)**0.5
            if dist < min_dist:
                min_dist = dist
        return min_dist        

    def to_scan(self, position, num_scan=300, pixels_per_meter=10):
        """
        Simulates a laser scan at a given position on the map
        """
        scan = LaserScan()
        scan.header.frame_id = "laser"
        scan.header.stamp = rospy.Time()
        scan.angle_increment = (2*pi/num_scan)
        scan.angle_min = -pi
        scan.angle_max = pi
        scan.range_max = 100
        w,h = self.get_size()
        curr_angle = scan.angle_min
        for i in range(num_scan):
            dist = self.get_first_collision((pixels_per_meter*position[0]+w//2, pixels_per_meter*position[1]+h//2),
                                            (pixels_per_meter*position[0]+w//2+w*cos(curr_angle), pixels_per_meter*position[1]+h//2+w*sin(curr_angle)))
            scan.ranges.append(dist/pixels_per_meter)
            scan.intensities.append(47)
            curr_angle += scan.angle_increment
        return scan


class ObstacleAvoidanceTest:
    def __init__(self, obstaclemap, pixels_per_meter=10):
        self.sub = rospy.Subscriber("/cmd_vel", Twist, self.on_twist_recieve)
        self.pub = rospy.Publisher("/scan", LaserScan, queue_size=10)
        self.current_position = np.array([0,0,0]) #Roll, Pitch, Yaw of robot
        self.obstacle_map = obstaclemap
        self.pixels_per_meter = 10
        self.last_time = 0
        self.last_twist = Twist()
        self.exit = False

    def set_obstacle_map(self, obstacle_map):
        self.obstacle_map = obstacle_map

    def publish_scan(self):
        scan = self.obstacle_map.to_scan(self.current_position, pixels_per_meter=self.pixels_per_meter)
        self.pub.publish(scan)
        if self.exit:
            rospy.signal_shutdown("test complete")

    def on_twist_recieve(self, msg): 

        if self.last_time == 0:
            self.last_twist = msg
            self.last_time = rospy.get_rostime()
            return
        
        curr_time = rospy.get_rostime()
        dt = (curr_time - self.last_time).to_sec()
        linear_vel = (self.last_twist.linear.x**2 + self.last_twist.linear.y**2)**0.5
        theta = atan2(self.last_twist.linear.y, self.last_twist.linear.x)
        dx = dt*linear_vel*cos(self.current_position[2]+theta)
        dy = dt*linear_vel*sin(self.current_position[2]+theta)
        dtheta = dt*self.last_twist.angular.z
        new_position = self.current_position + np.array([dx,dy,dtheta])
        if self.current_position[0] > (290-200)/self.pixels_per_meter:
            print("Collision detected")
            self.end_test()
        self.current_position = new_position
        self.last_twist = msg

        if self.current_position[0] < -200/self.pixels_per_meter or self.current_position[1] < -200/self.pixels_per_meter or self.current_position[0] > 200/self.pixels_per_meter or self.current_position[1] > 200/self.pixels_per_meter:
           self.end_test()
    
    def end_test(self):
        self.sub.unregister()
        self.exit = True
        
        

if __name__ == "__main__":
    rospy.init_node("simple_obstacle_avoidance_tester")
    rp = RosPack()
    test_case_path = join(rp.get_path("obstacle_avoidance_test"), "resources", "test.png")
    obstacle_map = ObstacleAvoidanceMap(test_case_path)
    simulator = ObstacleAvoidanceTest(obstacle_map)
    r = rospy.Rate(10)
    while not rospy.is_shutdown():
        simulator.publish_scan()
        r.sleep()
