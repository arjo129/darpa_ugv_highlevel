#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/sample_consensus/ransac.h>
#include <pcl/sample_consensus/sac_model_plane.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <visualization_msgs/MarkerArray.h>
#include <queue>
#include <random>
#include <unordered_map>
#include <Eigen/Eigen>


typedef pcl::PointCloud<pcl::PointXYZRGB> PointCloud;

/**
 * Utility method to normalize ros quaternions
 */ 
geometry_msgs::Quaternion normalize(geometry_msgs::Quaternion qt) {
    geometry_msgs::Quaternion res; 
    float norm = sqrt(qt.x*qt.x + qt.y*qt.y + qt.z*qt.z + qt.w*qt.w);
    res.x = qt.x / norm;
    res.y = qt.y / norm;
    res.z = qt.z / norm;
    res.w = qt.w / norm;
    return res;
}

/**
 * Gets the principal components of the point cloud
 */ 
std::vector<Eigen::Vector3f> getPrincipalComponents(PointCloud pointCloud, Eigen::Vector3f mean) {
    Eigen::Matrix3f covariance_matrix = Eigen::Matrix3f::Zero();

    for(int j = 0; j < 3; j++){
        for(int k = 0; k < 3; k++){
            for(pcl::PointXYZRGB pt: pointCloud) {
                //std::cout << pt.x << ", "<< pt.y << std::endl;
                float _pt[2];
                _pt[0] = pt.x;
                _pt[1] = pt.y;
                _pt[2] = pt.z;
                covariance_matrix(j,k) += (_pt[j] - mean[j]) * (_pt[k] - mean[k]);
            }
            covariance_matrix(j,k) /= pointCloud.size() - 1;
        }
    }

    Eigen::SelfAdjointEigenSolver<Eigen::Matrix<float, 3,3> > solver(covariance_matrix); 
    Eigen::Matrix3f eigenVectorsPCA = solver.eigenvectors();
    
    std::vector<Eigen::Vector3f> res;
    for(int i  = 0; i < 3; i++){
        res.push_back(eigenVectorsPCA.col(i));
    }
    return res;
}


float getMaxAlongVector(PointCloud::Ptr inliers, Eigen::Vector3f vec, Eigen::Vector3f centroid){
    float maxAlongVector = std::numeric_limits<float>::min();
    for(int i = 0 ; i < inliers->size(); i++) {
        pcl::PointXYZRGB pt = inliers->at(i);
        Eigen::Vector3f pointVec(pt.x, pt.y, pt.z);
        pointVec -= centroid;
        float dist = pointVec.dot(vec);
        maxAlongVector = (dist > maxAlongVector) ? dist: maxAlongVector;
    }
    return maxAlongVector;
}

float getMinAlongVector(PointCloud::Ptr inliers, Eigen::Vector3f vec, Eigen::Vector3f centroid){
    float minAlongVector = std::numeric_limits<float>::max();
    for(int i = 0 ; i < inliers->size(); i++) {
        pcl::PointXYZRGB pt = inliers->at(i);
        Eigen::Vector3f pointVec(pt.x, pt.y, pt.z);
        pointVec -= centroid;
        float dist = pointVec.dot(vec);
        minAlongVector = (dist < minAlongVector) ? dist: minAlongVector;
    }
    return minAlongVector;
}

bool isWithinRange(float val, float min, float max) {
    return min < val && val < max;
}

class Plane {

private:
    std::string frame; /// Describes frame name
    PointCloud::Ptr inliers;
    Eigen::Vector3f centroid, normal;
    double width, height;
    std::vector<Eigen::Vector3f> principalComponents;

public:
    
    Plane(PointCloud::Ptr result, Eigen::Vector3f _normal) {
        
        inliers = PointCloud::Ptr(new PointCloud());
        frame = result->header.frame_id;
        normal = _normal;
        centroid = Eigen::Vector3f(0,0,0);
        for(int i =0 ; i < result->size(); i++){
            pcl::PointXYZRGB pt = result->at(i);
            centroid += Eigen::Vector3f(pt.x, pt.y, pt.z);
            inliers->push_back(pt);
        }
        centroid *= 1/(float)inliers->size();

        principalComponents = getPrincipalComponents(*inliers, centroid);
        
        std::priority_queue<float> pq;
        for(int i = 0; i < principalComponents.size(); i++) {
            float max = getMaxAlongVector(inliers, principalComponents[i], centroid);
            float min = getMinAlongVector(inliers, principalComponents[i], centroid);
            float length = max-min;
            pq.push(length);
        }

        height = pq.top();
        pq.pop();
        width = pq.top();
        pq.pop();        
    }


    visualization_msgs::Marker toVisuallizationMarker() {
        visualization_msgs::Marker marker;
        marker.header.frame_id = frame;
        marker.type = visualization_msgs::Marker::CUBE;
        
        marker.color.r = 1.0;
        marker.color.a = 1.0;
        marker.scale.x = width;
        marker.scale.y = height;
        marker.scale.z = 0.01;
        
        marker.pose.position.x = centroid.x();
        marker.pose.position.y = centroid.y();
        marker.pose.position.z = centroid.z();
        
        Eigen::Vector3f unitNormal = 1/normal.norm()*normal;
        Eigen::Vector3f rotationArc = Eigen::Vector3f(0,0,1).cross(unitNormal);
        marker.pose.orientation.x = rotationArc.x();
        marker.pose.orientation.y = rotationArc.y();
        marker.pose.orientation.z = rotationArc.z();
        marker.pose.orientation.w = sqrt(unitNormal.norm()+1) + unitNormal.dot(Eigen::Vector3f(0,0,1));
        marker.pose.orientation = normalize(marker.pose.orientation);
        return marker;
    }

    Eigen::Vector3f getBottomLeft() {
        Eigen::Vector3f bl(centroid);
        std::vector<Eigen::Vector3f> components = getScaledPrincipalComponents();
        for(int i =0 ; i < components.size() ; i++){
            bl -= 0.5*components[i];
        }
        return bl;
    }

    std::vector<Eigen::Vector3f> getScaledPrincipalComponents() {
        std::vector<Eigen::Vector3f> scaledComponents;
        for(int i = 0; i < principalComponents.size(); i++) {
            float max = getMaxAlongVector(inliers, principalComponents[i], centroid);
            float min = getMinAlongVector(inliers, principalComponents[i], centroid);
            float length = max-min;
            Eigen::Vector3f scaled_component = principalComponents[i]*length;
            scaledComponents.push_back(scaled_component);
        }
        return scaledComponents;
    }

    bool isInside(Eigen::Vector3f point){
        point = point - getBottomLeft();
        std::vector<Eigen::Vector3f> components = getScaledPrincipalComponents();
        for(int i = 0; i < components.size(); i++) {
            float proj = components[i].dot(point)/components[i].norm();
            if(!isWithinRange(proj, 0, components[i].norm())) {
                return false;
            }
        }
        return true;
    }

    std::vector<Eigen::Vector3f> samplePoints(float num_points) {
        std::vector<Eigen::Vector3f> result;
        Eigen::Vector3f bottomLeft = getBottomLeft();
        std::vector<Eigen::Vector3f> spc = getScaledPrincipalComponents();
        
        std::default_random_engine generator;
        std::uniform_real_distribution<float> v1(0, 1);
        std::uniform_real_distribution<float> v2(0, 1);
        std::uniform_real_distribution<float> v3(0, 1);

        for(int i = 0; i < num_points; i++) {
            Eigen::Vector3f particle(bottomLeft);
            float dist = v1(generator);
            particle += dist*spc[0];
            dist = v2(generator);
            particle += dist*spc[1];
            dist = v3(generator);
            particle += dist*spc[2];
            result.push_back(particle);
        }
        return result;
    }
};

Eigen::Vector3f sphericalToCartesian(Eigen::Vector3f sphere) {
    float radius = sphere.x();
    float azimuth = sphere.y();
    float inclination = sphere.z();

    Eigen::Vector3f cartesian;
    cartesian.x() = radius * sin(inclination) * cos(azimuth);
    cartesian.y() = radius * sin(inclination) * sin(azimuth);
    cartesian.z() = radius * cos(inclination);

    return cartesian;
}

Eigen::Vector3f cartesianToSpherical(Eigen::Vector3f cartesian) {
    float radius = cartesian.norm();
    float azimuth = atan2(cartesian.y(), cartesian.x());
    float inclination = acos(cartesian.z()/radius);
    return Eigen::Vector3f(radius, azimuth, inclination);
}

class PlaneTracker {
private:
    std::vector<Eigen::Vector3f> particles;
    std::vector<int> particleId;
    std::unordered_map<int, std_msgs::ColorRGBA> colorMap;
    int maxId;
    int numParticles;
    bool first = false;
    
    std::vector<int> instantiateTrackers(std::vector<Plane> planes) {
        std::vector<int> id_s;
        std::default_random_engine generator;
        std::uniform_real_distribution<float> v1(0, 1);
        
        for(int i = 0; i < planes.size(); i++) {
            std::vector<Eigen::Vector3f> tmpParticles = planes[i].samplePoints(numParticles);
            for(int j = 0; j < tmpParticles.size(); j++) {
                particles.push_back(tmpParticles[j]);
                particleId.push_back(maxId + i);
            }
            id_s.push_back(maxId + i);
            std_msgs::ColorRGBA randomColor;
            randomColor.a = 0.5;
            randomColor.r = v1(generator);
            randomColor.g = v1(generator);
            randomColor.b = v1(generator);
            colorMap[maxId+i] = randomColor;
        }
        maxId += planes.size();
        return id_s;
    }

    void updateBeliefs() {
        std::default_random_engine generator;
        std::uniform_real_distribution<float> angularUncertainty(-1, 1);
        std::uniform_real_distribution<float> radialUncertainty(-0.1, 0.1);
        for(int i = 0; i < particles.size(); i++) {
            Eigen::Vector3f particleInSphericalCoord = cartesianToSpherical(particles[i]);
            Eigen::Vector3f noiseModel(radialUncertainty(generator), angularUncertainty(generator), angularUncertainty(generator));
            particles[i] = sphericalToCartesian(particleInSphericalCoord + noiseModel);
        }
    }

    std::vector<int> resample(std::vector<Plane> planes) {
        /*std::default_random_engine generator;
        std::vector<int> init;
        //Perform matching
        std::unordered_map<int,std::vector<int>> particleMapping;
        for(int i = 0; i < particles.size(); i++) {
            for(int j = 0; j < planes.size(); j++) {
                planes[j].isInside(particles[i]);
                particleMapping[j].push_back(particleId[i]);
            }
        }

        //Randomly sample a particle
        std::unordered_map<int,int> plane_mapping;
        for(int j = 0; j < planes.size(); j++) {
            if(planes.size() == 0) {
                //TODO: Spawn new particles for this plane
            }
            std::uniform_int_distribution<int> sampler(0, planes.size());
            int index = sampler(generator);
        }
        */
        //Re-assign all particles
    }
public:
    PlaneTracker(int num_particles) {
        numParticles = num_particles;
        maxId = 0;
    }
    std::vector<int> track(std::vector<Plane> planes) {
        if (first) {
            first = false;
            return instantiateTrackers(planes);
        }
        updateBeliefs();
        return resample(planes);
    }
    visualization_msgs::MarkerArray visualize() {
        visualization_msgs::MarkerArray particleMarkers;
        for(int i  = 0; i < particles.size(); i++) {
            visualization_msgs::Marker marker;
            marker.id = i;
            marker.type = visualization_msgs::Marker::SPHERE;
            marker.color = colorMap[particleId[i]];
            marker.pose.orientation.w =1;
            marker.pose.position.x = particles[i].x();
            marker.pose.position.y = particles[i].y();
            marker.pose.position.z = particles[i].z();
            particleMarkers.markers.push_back(marker);
        }
        return particleMarkers;
    }
};

class PlaneDetector {
private:
    ros::Publisher debug_out;
    ros::Subscriber input_sub;
    void callback(const PointCloud::ConstPtr& msg) {
        std::vector<Plane> planeList;
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients ());
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices ());
        // Create the segmentation object
        pcl::SACSegmentation<pcl::PointXYZRGB> seg;
        // Optional
        seg.setOptimizeCoefficients (true);
        // Mandatory
        seg.setModelType (pcl::SACMODEL_PLANE);
        seg.setMethodType (pcl::SAC_RANSAC);
        seg.setMaxIterations (1000);
        seg.setDistanceThreshold (0.1);

        // Create the filtering object
        pcl::ExtractIndices<pcl::PointXYZRGB> extract;
        PointCloud::Ptr cloud_filtered(new PointCloud);
        *cloud_filtered += *msg;
        int i = 0, nr_points = (int) cloud_filtered->points.size ();
        // While 30% of the original cloud is still there
        while (cloud_filtered->points.size () > 0.3 * nr_points)
        {
            PointCloud::Ptr cloud_p(new PointCloud);
            PointCloud::Ptr cloud_f(new PointCloud);
            // Segment the largest planar component from the remaining cloud
            seg.setInputCloud (cloud_filtered);
            seg.segment (*inliers, *coefficients);
            if (inliers->indices.size () == 0)
            {
                std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
                break;
            }

            // Extract the inliers
            extract.setInputCloud (cloud_filtered);
            extract.setIndices (inliers);
            extract.setNegative (false);
            extract.filter (*cloud_p);
            cloud_p->header.frame_id = msg->header.frame_id;

            //Convert to plane object
            Eigen::Vector3f vec;
            vec.x() = coefficients->values[0];
            vec.y() = coefficients->values[1];
            vec.z() = coefficients->values[2];
            Plane plane(cloud_p, vec);
            planeList.push_back(plane);

            // Create the filtering object
            extract.setNegative (true);
            extract.filter (*cloud_f);
            cloud_filtered.swap (cloud_f);
            i++;
        }
        visuallize(planeList);
    }

    void visuallize(std::vector<Plane> planeList) {
        visualization_msgs::MarkerArray markers;
        for(int i = 0 ; i < planeList.size(); i++){
            visualization_msgs::Marker marker =  planeList[i].toVisuallizationMarker();
            marker.id = i;
            //marker.header.frame_id = "camera_link";
            markers.markers.push_back(marker);
        }
        debug_out.publish(markers);
    }
public:
    PlaneDetector(ros::NodeHandle nh, std::string input_topic) {
        input_sub = nh.subscribe<PointCloud>(input_topic, 1, &PlaneDetector::callback, this);
        debug_out = nh.advertise<visualization_msgs::MarkerArray>("/pointcloud_pipeline/PlaneDetector",10);
    }
};


class PointCloudDownSampler {

private:
    ros::Subscriber input_sub;
    ros::Publisher debug_out;

    void callback(const PointCloud::ConstPtr& msg) {
        //Downsample the point cloud to keep things efficient 
        PointCloud::Ptr filtered(new PointCloud);
        pcl::VoxelGrid<pcl::PointXYZRGB> voxel_grid;
        voxel_grid.setInputCloud(msg);
        voxel_grid.setLeafSize(0.05, 0.05, 0.05);
        voxel_grid.filter(*filtered);
        filtered->header.frame_id = msg->header.frame_id;
        debug_out.publish(filtered);
    }

public:    
    PointCloudDownSampler(ros::NodeHandle nh, std::string input_topic) {
        input_sub = nh.subscribe<PointCloud>(input_topic, 1, &PointCloudDownSampler::callback, this);
        debug_out = nh.advertise<PointCloud>("/pointcloud_pipeline/downsample", 10);
    }
};


int main(int argc, char** argv)
{
  ros::init(argc, argv, "sub_pcl");
  ros::NodeHandle nh;
  PointCloudDownSampler downSampler(nh, "/camera/depth/color/points");
  PlaneDetector planedet(nh, "/pointcloud_pipeline/downsample");
  ros::spin();
}