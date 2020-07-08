#include <map_merge/laser_operations.h>
#include <amapper/grid.h>
#include <amapper/RayTracer.h>
#include <unordered_map>
#include <unordered_set>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>

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
    scan.range_max = max_range + 1;
    auto num_entries = ceil(2*M_PI/min_diff);
    scan.ranges = std::vector<float>((size_t)num_entries, INFINITY);
    scan.intensities = std::vector<float>((size_t)num_entries, 47);
    for(int i = 0; i < pc.size(); i++) {
        Eigen::Vector3f v1(pc[i].x , pc[i].y, 0);
        //Eigen::Vector3f x_axis(1,0,0);
        auto angle = atan2(pc[i].y, pc[i].x);
        auto index = round((angle - scan.angle_min)/scan.angle_increment);
        //std::cout << __LINE__<< " "<<index <<std::endl;
        if(index >= scan.ranges.size() || index < 0){
            continue;
        }
        scan.ranges[(size_t)index] = v1.norm();
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

int lookupAngle(sensor_msgs::LaserScan& scan, float angle) {
    return (int)((angle-scan.angle_min)/scan.angle_increment);
}

pcl::PointXYZ scanPointToPointCloud(pcl::PointXYZ point, double azimuth) {
    return pcl::PointXYZ(point.x, point.y, Eigen::Vector2f(point.x, point.y).norm()/tan(azimuth));
}

pcl::PointXYZ scanPointToPointCloud(sensor_msgs::LaserScan& scan, int index, double azimuth) {
    auto r = *lookup(scan, index);
    auto yaw = index*scan.angle_increment + scan.angle_min;
    
    if(!std::isfinite(r)) return pcl::PointXYZ(0,0,0);

    pcl::PointXYZ pt(r*cos(yaw), r*sin(yaw), 0);
    return scanPointToPointCloud(pt, azimuth);
}

pcl::PointXYZ scanPointToPointCloudWInf(sensor_msgs::LaserScan& scan, int index, double azimuth) {
    
    assert(std::isfinite(scan.range_max));

    auto r = *lookup(scan, index);
    auto yaw = index*scan.angle_increment + scan.angle_min;
    
    if(!std::isfinite(r)) r = scan.range_max;

    pcl::PointXYZ pt(r*cos(yaw), r*sin(yaw), 0);
    return scanPointToPointCloud(pt, azimuth);
}
 
bool isPointInside(LidarScan& scan, pcl::PointXYZ pt){
    
    auto r = Eigen::Vector3d(pt.x, pt.y, pt.z).norm();
    if(r == 0) return true;
    
    auto yaw = atan2(pt.y, pt.x);
    float azimuth = atan2(r, pt.z);    

    //TODO: Implement binary search
    int lt_index = -1;
    int geq_index = -1;
    for(int i = 0; i < scan.size(); i++) {
        if(scan[i].azimuth >= azimuth) {
            geq_index = i;
            break;
        }
        lt_index = i;
    }

    if(geq_index < 0) return false;

    if(scan[geq_index].azimuth == azimuth) {
        int index = lookupAngle(scan[geq_index].scan, yaw);
        return r <= scan[geq_index].scan.ranges[index];
    }

    if(scan[lt_index].azimuth < 0) return false;


    auto i1 = lookupAngle(scan[lt_index].scan, yaw);
    auto i2 = i1+1;
    auto i3 = lookupAngle(scan[geq_index].scan, yaw);

    //TODO: *CORRECT* implementation should estimate planes;
    auto pt1 = scanPointToPointCloudWInf(scan[lt_index].scan, i1, scan[lt_index].azimuth);
    auto pt2 = scanPointToPointCloudWInf(scan[lt_index].scan, i2, scan[lt_index].azimuth);
    auto pt3 = scanPointToPointCloudWInf(scan[geq_index].scan, i3, scan[geq_index].azimuth);

    auto p1 = Eigen::Vector3d(pt1.x, pt1.y, pt1.z);
    auto p2 = Eigen::Vector3d(pt2.x, pt2.y, pt2.z); 
    auto p3 = Eigen::Vector3d(pt3.x, pt3.y, pt3.z);

    auto v1 = p1 - p2;
    auto v2 = p3 - p2;
    auto norm = v1.cross(v2);
    auto k = norm.dot(Eigen::Vector3d(pt1.x, pt1.y, pt1.z));
    //Calculate unit ray in the direction of the original point
    auto ray = Eigen::Vector3d(pt.x, pt.y, pt.z)/r;
    //Get intersection 
    // N.r(t)  = k =>  t= k/N.r
    auto t = k/norm.dot(ray);
    return r<=t;
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
LoggingBressenham(): lastGoodPoint(INFINITY, INFINITY) {}

bool isPointAvailable() {
    return lastGoodPointAvailable;
}

Eigen::Vector2f getLastGoodPoint() {
    lastGoodPointAvailable = false;
    return lastGoodPoint;
}
};

Eigen::Vector2f centroidNormalization(const sensor_msgs::LaserScan& scan, sensor_msgs::LaserScan& recalculate, float resolution, int default_interpolation) {
    
    AMapper::Grid grid(0, 0, (scan.range_max*2)/resolution, (scan.range_max*2)/resolution, resolution);
    
    //Convert to cartesian
    float curr_angle = scan.angle_min;
    for(int i = 0; i < scan.ranges.size(); i++) {
        auto y = grid.toYIndex(scan.ranges.at(i)*sin(curr_angle));
        auto x = grid.toXIndex(scan.ranges.at(i)*cos(curr_angle));
        if(!grid.isWithinGridCellMap(x, y)) continue;
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
    
    if (num_points != 0) centroid /= num_points;
    //std::cout << "Centroid: " << centroid <<std::endl;

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
        Eigen::Vector2f point(INFINITY, INFINITY); 
        if(raytracer.isPointAvailable())
           point = raytracer.getLastGoodPoint();
        recalculate.ranges.push_back(point.norm());
        recalculate.intensities.push_back(47);
        curr_angle += scan.angle_increment;
    }

    fillGaps(recalculate, default_interpolation);
    return centroid;
}

void FFT1D(const sensor_msgs::LaserScan& scan, std::vector<std::complex<double>>& spectra){
    
    fftw_complex* signal = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * scan.ranges.size());
    fftw_complex* result = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * scan.ranges.size());
    fftw_plan plan = fftw_plan_dft_1d(scan.ranges.size(), signal, result, FFTW_FORWARD, FFTW_ESTIMATE);

    for(int i =0; i < scan.ranges.size(); i++) {
        auto range = scan.ranges.at(i);
        signal[i][0] = range == INFINITY ? 0 :range;
        signal[i][1] = 0;
    }

    fftw_execute(plan);

    for(int i =0; i < scan.ranges.size(); i++) {
       std::complex<double> c(result[i][0]/scan.ranges.size(), result[i][1]/scan.ranges.size());
       spectra.push_back(c);
    }

    fftw_destroy_plan(plan);
    fftw_free(signal);
    fftw_free(result);
}

void IFFT1D(std::vector<std::complex<double>>& spectra, std::vector<std::complex<double>>& out){
    
    fftw_complex* signal = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * spectra.size());
    fftw_complex* result = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * spectra.size());
    fftw_plan plan = fftw_plan_dft_1d(spectra.size(), signal, result, FFTW_BACKWARD, FFTW_ESTIMATE);

    for(int i =0; i < spectra.size(); i++) {
        signal[i][0] = spectra[i].real();
        signal[i][1] = spectra[i].imag();
    }
    fftw_execute(plan);

    for(int i =0; i < spectra.size(); i++) {
       std::complex<double> c(result[i][0]/spectra.size(), result[i][1]/spectra.size());
       out.push_back(c);
    }

    fftw_destroy_plan(plan);
    fftw_free(signal);
    fftw_free(result);
}



void multiply(fftw_complex& complex1, fftw_complex& complex2, fftw_complex& out) {
    out[0] = complex1[0] * complex1[0] - complex2[1] * complex2[1]; 
    out[1] = complex1[0] * complex2[1] + complex2[0] * complex2[1];  
}

float magnitude(fftw_complex& input) {
    return input[0] * input[0] + input[1] * input[1];
}

float magnitude(std::vector<std::complex<double>>& complex) {
    
    float total = 0;
    
    for(int i = 0; i < complex.size(); i++) {
        auto mag = std::norm(complex[i]);
        total += mag*mag;
    }

    return sqrt(total);
}


float compareScansEuclid(std::vector<std::complex<double>>& s1, std::vector<std::complex<double>>& s2){

    assert(s1.size() == s2.size());

    float sum = 0;
    for(int i = 0 ; i < s1.size(); i++){
        auto magnitude_s1 = std::norm(s1[i]);
        auto magnitude_s2 = std::norm(s2[i]);
        auto diff = magnitude_s1 - magnitude_s2;
        sum += diff*diff;
    }

    return sum;
}

float compareScansCosine(std::vector<fftw_complex>& s1, std::vector<fftw_complex>& s2) {

    assert(s1.size() == s2.size());

    float sum = 0;
    for(int i = 0 ; i < s1.size(); i++){
        auto magnitude_s1 = s1[i][0]*s1[i][0] + s1[i][1]*s1[i][1];
        auto magnitude_s2 = s2[i][0]*s2[i][0] + s2[i][1]*s2[i][1];
        auto diff = magnitude_s1 * magnitude_s2;
        sum += diff*diff;
    }

    for(int i = 0 ; i < s1.size(); i++){
        
    }
    return sum;
}

void decomposeLidarScanIntoPlanes(pcl::PointCloud<pcl::PointXYZ>& points, std::vector<sensor_msgs::LaserScan>& scan_stack) {
    std::unordered_map<long, pcl::PointCloud<pcl::PointXYZ> > planar_scans;
    /**
     * Decompose into scan planes
     */ 
    for(auto pt: points){
        if(!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            continue;
        }
        float r = sqrt(pt.x*pt.x + pt.y*pt.y);
        float angle = atan2(r,pt.z);
        float res  = angle/M_PI*180;
        long plane_hash = res;
        planar_scans[plane_hash].push_back(pt);
        planar_scans[plane_hash].header = points.header;
    }

    std::vector<double> scan_angles;
    for(auto scan: planar_scans){
        scan_angles.push_back((double)scan.first/180.f*M_PI); 
    }
    std::sort(scan_angles.begin(), scan_angles.end());

    for(auto angle: scan_angles) {
        scan_stack.push_back(toLaserScan(planar_scans[angle]));
    }
}



void decomposeLidarScanIntoPlanes(const pcl::PointCloud<pcl::PointXYZ>& points, LidarScan& scan_stack) {
    std::unordered_map<long, pcl::PointCloud<pcl::PointXYZ> > planar_scans;
    /**
     * Decompose into scan planes
     */ 
    for(auto pt: points){
        if(!std::isfinite(pt.x) || !std::isfinite(pt.y) || !std::isfinite(pt.z)) {
            continue;
        }
        float r = sqrt(pt.x*pt.x + pt.y*pt.y);
        float angle = atan2(r, pt.z);
        float res  = angle/(2*M_PI)*360;
        long plane_hash = res;
        planar_scans[plane_hash].push_back(pt);
        planar_scans[plane_hash].header = points.header;
    }

    std::vector<long> scan_angles;
    for(auto scan: planar_scans){
        scan_angles.push_back(scan.first); 
    }
    std::sort(scan_angles.begin(), scan_angles.end());

    for(auto angle: scan_angles) {
        LidarRing ring;
        ring.azimuth = angle*M_PI/180;
        ring.scan = toLaserScan(planar_scans[angle]);
        scan_stack.push_back(ring);
    }
}

float EstimateYawCorrection(sensor_msgs::LaserScan scan1, sensor_msgs::LaserScan scan2, std::vector<double>& accumulator){
    sensor_msgs::LaserScan normalized1, normalized2;
    centroidNormalization(scan1, normalized1, 0.05);
    centroidNormalization(scan2, normalized2, 0.05);
    
    sensor_msgs::LaserScan downsampled1,  downsampled2; 
    downsample(normalized1, downsampled1, 100);
    downsample(normalized2, downsampled2, 100);

    std::vector<std::complex<double>> fft1, fft2;
    FFT1D(downsampled1, fft1);
    FFT1D(downsampled2, fft2);

    assert(fft1.size() == fft2.size());

    std::vector<std::complex<double>> res;
    
    if(accumulator.size() == 0) for(int i = 0 ; i < fft1.size(); i ++)  { 
        accumulator.push_back(0);
    }

    assert(accumulator.size() == fft1.size());

    for(int i = 0; i < fft1.size(); i++) {
        auto r = fft1[i] * fft2[i];
        res.push_back(r);
       
    }
  
    std::vector<std::complex<double>> dec;
    IFFT1D(res, dec);

    for(int i = 0; i < accumulator.size(); i++) {
        auto mag  = std::norm(dec[i]);
        accumulator[i] += mag;
    }
    return 0;
}

void extractCorners(const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointCloud<pcl::PointXYZ>& corners) {
    LidarScan scan;
    decomposeLidarScanIntoPlanes(cloud, scan);
    for(auto plane: scan) {
        pcl::PointCloud<pcl::PointXYZ> cloud;
        std::vector<int> index;
        naiveCornerDetector(plane.scan, cloud, index, 100);
        for(auto point: cloud) {
            auto res = scanPointToPointCloud(point, plane.azimuth);
            corners.push_back(res);
        }
    }
}

Eigen::Vector2f getCentroid(sensor_msgs::LaserScan& scan, double angle, double resolution) {
    AMapper::Grid grid(0, 0, (scan.range_max*2)/resolution, (scan.range_max*2)/resolution, resolution);
    std::unordered_set<long> z_axis;
    //Convert to cartesian
    float curr_angle = scan.angle_min;
    for(int i = 0; i < scan.ranges.size(); i++) {
        auto y = grid.toYIndex(scan.ranges.at(i)*sin(curr_angle));
        auto x = grid.toXIndex(scan.ranges.at(i)*cos(curr_angle));
        if(!grid.isWithinGridCellMap(x, y)) continue;
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

    return centroid;
}

void getCentroid(LidarScan& scan, double resolution){
    Eigen::Vector3f centroid(0,0,0);
    for(auto& ring: scan) {
        
    }
}

pcl::PointXYZ convertToCartesian(double r, double azimuth, double yaw) {
    auto x = r * cos(yaw);
    auto y = r * sin(yaw);
    return scanPointToPointCloud(pcl::PointXYZ(x,y,0), azimuth);
}

pcl::PointXYZ convertToCartesian(sensor_msgs::LaserScan& scan, int index, double azimuth) {
    auto r = *lookup(scan, index);
    auto yaw = scan.angle_min + index*scan.angle_increment;
    return convertToCartesian(r, azimuth, yaw);
}


pcl::Normal estimateNormalRANSAC(pcl::PointCloud<pcl::PointXYZ>::Ptr points) {
    Eigen::VectorXf coefficients;
    pcl::SampleConsensusModelPlane<pcl::PointXYZ>::Ptr
    model_p (new pcl::SampleConsensusModelPlane<pcl::PointXYZ>(points));
    pcl::RandomSampleConsensus<pcl::PointXYZ> ransac (model_p);
    ransac.setDistanceThreshold (.05);
    ransac.setMaxIterations(10);
    ransac.computeModel();
    ransac.getModelCoefficients(coefficients);
    return pcl::Normal(coefficients[0], coefficients[1], coefficients[2]);
}

/**
 * Get nearest lidar neighbors on the surface
 */ 
void getSurfaceNeighbors(LidarScan& layers, int stack, int pointIndex, int radius_x, int radius_y, pcl::PointCloud<pcl::PointXYZ>::Ptr output) {
    for(int y = -radius_y; y <= radius_y; y++) {
        for(int x = -radius_x; x <= radius_x; x++) {
            auto index_z= stack+y;
            if(index_z < 0) continue;
            if(index_z >= layers.size()) continue;
            if(*lookup(layers[index_z].scan, pointIndex+x) > layers[index_z].scan.range_max) continue;
            auto res = convertToCartesian(layers[index_z].scan, pointIndex+x, layers[index_z].azimuth);
            output->push_back(res);
        }    
    }
} 
/**
 * Fast lidar normal 
 * This function utilizes the symetries in a Lidar to make fast normal estimation
 */ 
void fastLidarNormal(LidarScan& stack, pcl::PointCloud<pcl::PointXYZ>::Ptr point_cloud, pcl::PointCloud<pcl::Normal>::Ptr normals) {
    for(int i = 1 ; i < stack.size() - 1; i++) {
        for(int j = 0; j < stack[i].scan.ranges.size(); j++) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr output(new pcl::PointCloud<pcl::PointXYZ>);
            getSurfaceNeighbors(stack, i, j, 10, 1, output);
            pcl::Normal normal = estimateNormalRANSAC(output);
            pcl::PointXYZ pt = convertToCartesian(stack[i].scan, j, stack[i].azimuth);
            normals->push_back(normal);
            point_cloud->push_back(pt);
        }
    }
}


/**
 * Used for educated initial guess. Used with *very* sparse point clouds.
 */ 
Eigen::Matrix4f ICPMatchPointToPoint(const pcl::PointCloud<pcl::PointXYZ>& pt1,const pcl::PointCloud<pcl::PointXYZ>& pt2, int max_iter, double max_error) {
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>(pt1));
    pcl::PointCloud<pcl::PointXYZ>::Ptr reference(new pcl::PointCloud<pcl::PointXYZ>(pt2));
    pcl::IterativeClosestPoint<pcl::PointXYZ, pcl::PointXYZ> icp;

    icp.setInputSource(cloud);
    icp.setInputTarget(reference);
    icp.setMaximumIterations(max_iter);
    pcl::PointCloud<pcl::PointXYZ> Final;
    icp.align(Final);
    
    return icp.getFinalTransformation();
}