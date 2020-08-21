#ifndef _LASER_OPERATIONS_H_
#define _LASER_OPERATIONS_H_
#include <ros/ros.h>
#include <sensor_msgs/LaserScan.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <complex>
#include <vector>
#include <fftw3.h>


struct LidarRing {
    double azimuth;
    sensor_msgs::LaserScan scan;
};

typedef std::vector<LidarRing> LidarScan;

void decomposeLidarScanIntoPlanes(const pcl::PointCloud<pcl::PointXYZ>& points, LidarScan& scan_stack);
sensor_msgs::LaserScan toLaserScan(pcl::PointCloud<pcl::PointXYZ>& pc);
pcl::PointXYZ scanPointToPointCloud(pcl::PointXYZ point, double azimuth);

pcl::PointXYZ scanPointToPointCloud(sensor_msgs::LaserScan& scan, int index, double azimuth);
float* lookup(sensor_msgs::LaserScan& scan, int index); 

/**
 * Performs linear interpolation of lasercans
 */ 
void fillGaps(sensor_msgs::LaserScan& scan, size_t max_gap=2);

/**
 * Naive corner detector for a single plane laserscan
 */
void naiveCornerDetector(sensor_msgs::LaserScan& scan, pcl::PointCloud<pcl::PointXYZ>& corners, std::vector<int>& indices, int skip=1);

/**
 * Extract Corners from a 3D lidar scan
 */ 
void extractCorners(const pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointCloud<pcl::PointXYZ>& corners); 

/**
 * Normalize centroid of laserscan
 */
Eigen::Vector2f centroidNormalization(const sensor_msgs::LaserScan& scan, sensor_msgs::LaserScan& recalculate, float resolution, int interpolation=8); 

/**
 * FFT of laserscan
 */
void FFT1D(const sensor_msgs::LaserScan& scan, std::vector<std::complex<double> >& spectra);

/**
 * decompose 
 */ 
void  decomposeLidarScanIntoPlanes(pcl::PointCloud<pcl::PointXYZ>& points, std::vector<sensor_msgs::LaserScan>& scan_stack);

void downsample(const sensor_msgs::LaserScan& scan, sensor_msgs::LaserScan& out, int skip);

float compareScansEuclid(std::vector<std::complex<double>>& s1, std::vector<std::complex<double>>& s2);


float EstimateYawCorrection(sensor_msgs::LaserScan scan1, sensor_msgs::LaserScan scan2, std::vector<double>& accumulator);


Eigen::Matrix4f ICPMatchPointToPoint(const pcl::PointCloud<pcl::PointXYZ>& pt1, const pcl::PointCloud<pcl::PointXYZ>& pt2, int max_iter=200, double max_error=0);

bool isPointInside(LidarScan& scan, pcl::PointXYZ pt);

class FFTFeatureExtractor {
    private:
    fftw_complex* signal;
    fftw_complex* result;
    fftw_plan plan;
    size_t size;
    
    public:
    FFTFeatureExtractor(size_t sz) {
        size = sz;
        signal = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * size);
        result = (fftw_complex*) fftw_malloc(sizeof(fftw_complex) * size);
        plan = fftw_plan_dft_1d(size, signal, result, FFTW_FORWARD, FFTW_ESTIMATE);

    }

    void extractFeatures(const sensor_msgs::LaserScan& scan, std::vector<std::complex<double> >& spectra) {
    
        assert(scan.ranges.size() == size);
        if(spectra.size() != size) {
            spectra.reserve(size);
            for(size_t i = 0 ; i < size; i++){
                spectra.push_back(std::complex<double>(0,0));
            }
        }

        for(int i =0; i < scan.ranges.size(); i++) {
            auto range = scan.ranges[i];
            signal[i][0] = range == INFINITY ? 0 :range;
            signal[i][1] = 0;
        }

        fftw_execute(plan);

        for(int i =0; i < spectra.size(); i++) {
            std::complex<double> c(result[i][0]/spectra.size(), result[i][1]/spectra.size());
            spectra[i] = c;
        } 
    }
    ~FFTFeatureExtractor() {
        fftw_destroy_plan(plan);
        fftw_free(signal);
        fftw_free(result);
    }
};
#endif