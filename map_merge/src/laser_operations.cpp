#include <ros/ros.h>
#include <map_merge/laser_operations.h>
#include <amapper/grid.h>
#include <amapper/RayTracer.h>
#include <fftw3.h>

struct point_sorter {
    inline bool operator() (const pcl::PointXYZ& p1, const pcl::PointXYZ& p2){
        return atan2(p1.y, p1.x) < atan2(p2.y, p2.x);
    }
};

sensor_msgs::LaserScan toLaserScan(pcl::PointCloud<pcl::PointXYZ>& pc) {
    
    sensor_msgs::LaserScan scan;
    scan.header.frame_id = pc.header.frame_id;
    scan.header.stamp = ros::Time(pc.header.stamp);
    double min_diff = INFINITY;

    std::sort(pc.begin(), pc.end(), point_sorter());
    pcl::PointXYZ prev_pt = pc[pc.size()-1];
    double max_range = 0;
    for(int i = 0; i < pc.size(); i++) {
        Eigen::Vector3f v1(prev_pt.x , prev_pt.y, prev_pt.z);
        Eigen::Vector3f v2(pc[i].x, pc[i].y, pc[i].z);
        auto angle = acos(v1.dot(v2)/(v1.norm()*v2.norm()));
        if(angle < min_diff && angle > 0) {
            min_diff = angle;
        }
        if(v2.norm() > max_range) {
            max_range = v2.norm();
        }
        prev_pt = pc[i];
    }

    scan.angle_increment = min_diff;
    scan.angle_min = -M_PI;
    scan.angle_max = M_PI-min_diff;
    scan.range_max = max_range +1;
    auto num_entries = ceil(2*M_PI/min_diff);
    scan.ranges = std::vector<float>((size_t)num_entries, INFINITY);
    scan.intensities = std::vector<float>((size_t)num_entries, 47);
    for(int i = 0; i < pc.size(); i++) {
        Eigen::Vector3f v1(pc[i].x , pc[i].y, 0);
        Eigen::Vector3f x_axis(1,0,0);
        auto angle = atan2(pc[i].y, pc[i].x);
        auto index = round((angle - scan.angle_min)/scan.angle_increment);
        //std::cout << __LINE__<< " "<<index <<std::endl;
        if(index >= scan.ranges.size() || index < 0){
            continue;
        }
        scan.ranges[(size_t)index] = Eigen::Vector3f(pc[i].x, pc[i].y, 0).norm();
    }
    return scan;
}

float* lookup(sensor_msgs::LaserScan& scan, int index) {
    auto length = scan.ranges.size();
    if(index < 0) {
        return &scan.ranges[length - abs(index)%length];
    }
    else {
        return &scan.ranges[index%length];
    }
}

void fillGaps(sensor_msgs::LaserScan& scan, size_t max_gap) {
    size_t gap_count = 0;
    for(int i = 0; i < scan.ranges.size(); i++) {
        if(scan.ranges[i] == INFINITY) {
            gap_count++;
            continue;
        }

        if(gap_count <= max_gap) for(int j = 1; j <= gap_count; j++){
            float v1 = *lookup(scan,i);
            float v0 = *lookup(scan,i-gap_count-1);
            *lookup(scan, i-j) = (j*(v0 -v1))/(gap_count +1) + v1;
        }
        gap_count = 0;
        
    }
}

void naiveCornerDetector(sensor_msgs::LaserScan& scan, pcl::PointCloud<pcl::PointXYZ>& corners, std::vector<int>& indices, int skip) {
    fillGaps(scan, 2);
    double curr_angle = scan.angle_min;
  
    std::vector<float> angle;
    for(int i = 0; i < scan.ranges.size(); i++) {
        float r_prev = *lookup(scan, i-skip);
        float r_curr = *lookup(scan, i);
        float r_next = *lookup(scan, i+skip);
        
        if(r_prev == INFINITY || r_curr == INFINITY || r_next == INFINITY) continue;
        
        Eigen::Vector2d v_prev(r_prev*cos(curr_angle - skip*scan.angle_increment), r_prev*sin(curr_angle - skip*scan.angle_increment));
        Eigen::Vector2d v_next(r_next*cos(curr_angle + skip*scan.angle_increment), r_next*sin(curr_angle + skip*scan.angle_increment));
        Eigen::Vector2d v_curr(r_curr*cos(curr_angle), r_curr*sin(curr_angle));

        auto incoming = v_curr - v_prev;
        auto out_going = v_next - v_curr;
        
        auto projection = incoming.dot(out_going)/(incoming.norm()*out_going.norm());
        if(projection > 1) projection = 1; //Fix stupid float problems
        auto angle = acos(projection);

        if(angle > M_PI*0.4) {
            pcl::PointXYZ point(v_curr.x(), v_curr.y(), 0);
            corners.push_back(point);
            indices.push_back(i);
        }
        curr_angle += scan.angle_increment;
    }
}

void downsample(const sensor_msgs::LaserScan& scan, sensor_msgs::LaserScan& out, int skip) {
    out.angle_max = scan.angle_max;
    out.header = scan.header;
    out.angle_increment = scan.angle_increment*skip;
    out.angle_min = scan.angle_min;
    out.range_max = scan.range_max;
    out.range_min = scan.range_min;
    out.scan_time = scan.scan_time;
    out.time_increment = scan.time_increment;

    for(int i = 0; i < scan.ranges.size(); i+= skip) {
        out.ranges.push_back(scan.ranges.at(i));
        if(i < scan.intensities.size()) {
            out.intensities.push_back(scan.intensities.at(i));
        }
        else {
            ROS_WARN("Laser scan intensity and ranges length mismatch");
        }
    }
}

class LoggingBressenham: public AMapper::RayTracer {
private:
Eigen::Vector2f lastGoodPoint;
bool lastGoodPointAvailable =false;
void plotFreeSpace(AMapper::Grid& grid, Eigen::Vector2i centroid, int x, int y) override {
    if(grid.data[y][x] == 1 && !lastGoodPointAvailable){
        lastGoodPoint = Eigen::Vector2f(grid.fromXIndex(x), grid.fromYIndex(y));
        lastGoodPointAvailable = true;
    } 
}
public:
Eigen::Vector2f getLastGoodPoint() {
    lastGoodPointAvailable = false;
    return lastGoodPoint;
}
};

void centroidNormalization(const sensor_msgs::LaserScan& scan, sensor_msgs::LaserScan& recalculate, float resolution) {
    
    AMapper::Grid grid(0, 0, (scan.range_max*2)/resolution, (scan.range_max*2)/resolution, resolution);
    
    //Convert to cartesian
    float curr_angle = scan.angle_min;
    for(int i = 0; i < scan.ranges.size(); i++) {
        auto y = grid.toYIndex(scan.ranges.at(i)*sin(curr_angle));
        auto x = grid.toXIndex(scan.ranges.at(i)*cos(curr_angle));
        grid.data[y][x] = 1;
        curr_angle += scan.angle_increment;
    }

    //Get centroid 
    Eigen::Vector2f centroid(0,0);
    int num_points = 0;
    for(int i = 0; i < grid.gridWidth; i++) {
        for(int j = 0; j < grid.gridWidth; j++) {
           if(grid.data[i][j] == 1){
               centroid += Eigen::Vector2f(grid.fromXIndex(j), grid.fromYIndex(i));
               num_points++;
           }
        }
    }
    centroid /= num_points;
    //std::cout << centroid <<std::endl;

    recalculate.angle_max = scan.angle_max;
    recalculate.header = scan.header;
    recalculate.angle_increment = scan.angle_increment;
    recalculate.angle_min = scan.angle_min;
    recalculate.range_max = scan.range_max;
    recalculate.range_min = scan.range_min;
    recalculate.scan_time = scan.scan_time;
    recalculate.time_increment = scan.time_increment;
    //Ray tracing after transform
    LoggingBressenham raytracer;
    curr_angle = scan.angle_min;
    for(int i = 0 ;i < scan.ranges.size(); i++) {
        raytracer.rayTrace(grid, centroid, Eigen::Vector2f(scan.range_max*cos(curr_angle), scan.range_max*sin(curr_angle)));
        auto point = raytracer.getLastGoodPoint();
        recalculate.ranges.push_back(point.norm());
        recalculate.intensities.push_back(47);
        curr_angle += scan.angle_increment;
    }

    fillGaps(recalculate);
}

void FFT1D(const sensor_msgs::LaserScan& scan, std::vector<float>& spectra){
    
    fftw_complex* signal = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * scan.ranges.size());
    fftw_complex* result = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * scan.ranges.size());
    fftw_plan plan = fftw_plan_dft_1d(scan.ranges.size(), signal, result, FFTW_FORWARD, FFTW_ESTIMATE);

    for(int i =0; i < scan.ranges.size(); i++) {
        signal[i][0] = scan.ranges.at(i);
        signal[i][1] = 0;
    }

    fftw_execute(plan);

    for(int i =0; i < scan.ranges.size(); i++) {
       float magnitude = result[i][0]*result[i][0] + result[i][1]*result[i][1];
       spectra.push_back(magnitude);
    }

    fftw_destroy_plan(plan);
    fftw_free(signal);
    fftw_free(result);
}