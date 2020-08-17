#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QLegendMarker>
#include <QtGui/QImage>
#include <QtGui/QPainter>
#include <QtCore/QtMath>
#include <QtCore/QDir>
#include <QtCore/QTimer>
#include <geometry_msgs/TransformStamped.h>
#include <map_merge/laser_operations.h>
#include <map_merge/centroids.h>
#include <sensor_msgs/LaserScan.h>
#include <nlohmann_json/json.h>
#include <fstream>
#include <thread>
#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>
#include <visualization_msgs/MarkerArray.h>
#include "visualization/chartview.h"
#include "visualization/canvas.h"

ChartView::ChartView(QWidget *parent) :
    QChartView(new QChart(), parent)
{
    //![1]
    QScatterSeries *series0 = new QScatterSeries();
    series0->setName("Un-normallized");
    series0->setMarkerShape(QScatterSeries::MarkerShapeCircle);
    series0->setMarkerSize(15.0);

    //![2]
    series0->append(-4, 6);
    series0->append(2, 4);
    series0->append(3, 8);
    series0->append(7, 4);
    series0->append(10, 5);
   
    setRenderHint(QPainter::Antialiasing);
    chart()->addSeries(series0);


    chart()->setTitle("Simple scatterchart example");
    chart()->createDefaultAxes();
    chart()->setDropShadowEnabled(false);
    //![4]

    //![5]
    chart()->legend()->setMarkerShape(QLegend::MarkerShapeFromSeries);
    //![5]
}

void retrievePointCloud(nlohmann::json& j, pcl::PointCloud<pcl::PointXYZ>& pointCloud) {
    for(auto point: j["points"]) {
        pcl::PointXYZ pt;          
        pt.x = point[0];
        pt.y = point[1];
        pt.z = point[2];
        pointCloud.push_back(pt);
    }
}

Eigen::Vector3f current_pose(0,0,0), prev_pose(0,0,0);
Eigen::Quaternionf current_orientation, prev_orientation;
ros::Publisher centroid_viz;
void getFeatureVector(pcl::PointCloud<pcl::PointXYZ>& pointCloud, std::vector<std::complex<double>>& d) {
    
    auto time = ros::Time();
    static LidarScan msgs;
    decomposeLidarScanIntoPlanes(pointCloud, msgs);
    std::vector<sensor_msgs::LaserScan> normalized_scans;
    Eigen::Vector2f corrected_centroid(0,0);

    visualization_msgs::Marker marker;
    marker.header.frame_id = "world";//pointCloud.header.frame_id;
    marker.header.stamp = time;
    marker.id - 0;
    marker.type = visualization_msgs::Marker::SPHERE;
    marker.action = visualization_msgs::Marker::ADD;

    visualization_msgs::Marker marker2;
    marker2.header.frame_id = pointCloud.header.frame_id;
    marker2.header.stamp = time;
    marker2.id = 1;
    marker2.type = visualization_msgs::Marker::SPHERE;
    marker2.action = visualization_msgs::Marker::ADD;
    int count = 0;
    for(auto& scan: msgs) {
        sensor_msgs::LaserScan normalized_scan, downsampled_scan;
        
        if(abs(scan.azimuth-1.57) < 0.2) {
            auto centroid = centroidNormalization(scan.scan, normalized_scan, 0.1);
            count++;
            corrected_centroid += centroid;
              // downsample(normalized_scan, downsampled_scan, 50);
      //  FFT1D(downsampled_scan, d);
        }
    }
    corrected_centroid/=count;

    std::cout << "---" << std::endl;
    tf::Transform trans;
    tf::Vector3 pos(current_pose.x(), current_pose.y(), current_pose.z());
    tf::Quaternion qt(current_orientation.x(), current_orientation.y(), current_orientation.z(), current_orientation.w());
    trans.setOrigin(pos);
    trans.setRotation(qt);
    

    tf::Vector3 t(corrected_centroid.x(), corrected_centroid.y(), 0);

    auto r = trans*t;
   // if((current_pose-prev_pose).norm() > 0.001) {
        std::cout << r.x() << ", " << r.y() << ", "<< r.z() << std::endl;
        std::cout << current_pose <<std::endl;
        std::cout << current_orientation.x() << current_orientation.y() << current_orientation.z() <<current_orientation.w() <<std::endl;
    //} 
    marker.pose.position.x = r.x();
    marker.pose.position.y = r.y();
    marker.pose.position.z = 0;
    marker.pose.position.z = 1;
    marker.pose.orientation.x = 0.0;
    marker.pose.orientation.y = 0.0;
    marker.pose.orientation.z = 0.0;
    marker.pose.orientation.w = 1.0;
    marker.scale.x = 1;
    marker.scale.y = 1;
    marker.scale.z = 1;
    marker.color.a = 1.0; // Don't forget to set the alpha!
    marker.color.r = 1.0;
    marker.color.g = 0.0;
    marker.color.b = 0.0;


    marker2.pose.position.x = t.x();
    marker2.pose.position.y = t.y();
    marker2.pose.position.z = 0;
    marker2.pose.position.z = 1;
    marker2.pose.orientation.x = 0.0;
    marker2.pose.orientation.y = 0.0;
    marker2.pose.orientation.z = 0.0;
    marker2.pose.orientation.w = 1.0;
    marker2.scale.x = 1;
    marker2.scale.y = 1;
    marker2.scale.z = 1;
    marker2.color.a = 1.0; // Don't forget to set the alpha!
    marker2.color.r = 0.0;
    marker2.color.g = 1.0;
    marker2.color.b = 0.0;

    visualization_msgs::MarkerArray mk;
    mk.markers.push_back(marker);
    mk.markers.push_back(marker2);
    centroid_viz.publish(mk);
}


void randomYaw(pcl::PointCloud<pcl::PointXYZ>& cloud, pcl::PointCloud<pcl::PointXYZ>& out) {

    float offset = static_cast <float> (rand()) / (static_cast <float> (RAND_MAX/M_PI));;     
    for(auto pt: cloud) {
        pcl::PointXYZ new_pt;
        auto radius = sqrt(pt.x*pt.x + pt.y*pt.y);
        auto yaw = atan2(pt.y, pt.x);
        new_pt.z = pt.z;
        new_pt.x = radius*cos(yaw + offset);
        new_pt.y = radius*sin(yaw + offset);
        out.push_back(new_pt);
    }
}

void getFeatureWithRandomRotationVector(nlohmann::json& j, std::vector<std::complex<double>>& d) {
    pcl::PointCloud<pcl::PointXYZ> cloud, rotated_cloud;
    retrievePointCloud(j, cloud);
    randomYaw(cloud, rotated_cloud);
    getFeatureVector(rotated_cloud, d);
} 

void getFeatureVector(nlohmann::json& j, std::vector<std::complex<double>>& d) {
    pcl::PointCloud<pcl::PointXYZ> cloud;
    retrievePointCloud(j, cloud);
    getFeatureVector(cloud, d);
}

void evaluate_test() {
    QDir directory("/media/arjo/ba3893af-3249-49c4-8441-fb741cff5e26/triples");
    QStringList images = directory.entryList(QStringList() << "*.json",QDir::Files);
    foreach(QString filename, images) {
        std::ifstream i(directory.filePath(filename).toStdString());
        nlohmann::json j;
        i >> j;
        int index = 0; 
        for(auto k: j){
            std::vector<std::complex<double>> anchor_vector; 
            getFeatureWithRandomRotationVector(k["anchor"], anchor_vector);
            

            std::vector<std::complex<double>> neg_vector; 
            getFeatureWithRandomRotationVector(k["neg"], neg_vector);

            std::vector<std::complex<double>> pos_vector; 
            getFeatureWithRandomRotationVector(k["pos"], pos_vector);

            float pos = compareScansEuclid(pos_vector, anchor_vector);
            float neg = compareScansEuclid(neg_vector, anchor_vector);
            std::cout << pos << "\t" << neg;
            if(pos > neg) std::cout << "\t" << filename.toStdString() << "\t" << index; 
            std::cout << std::endl; 
            index++;
        }
    }
}


void index(std::vector<Centroid> centroids) {
    std::ifstream i("/media/arjo/ba3893af-3249-49c4-8441-fb741cff5e26/out.json");
    std::string str;
    while(std::getline(i, str)) {
        nlohmann::json j = nlohmann::json::parse(str);
        std::vector<std::complex<double>> vec; 
        getFeatureVector(j, vec);
        if(centroids.size() == 0) {
            Centroid cen;
            cen.centroid_vector = vec;
            cen.x = j["pose"]["pos"][0];
            cen.y = j["pose"]["pos"][1];
            cen.count = 1;
            centroids.push_back(cen);
            continue;
        }
        auto last_index = centroids.size()-1;
        auto dist = compareScansEuclid(centroids[last_index].centroid_vector, vec);
        //std::cout <<dist <<std::endl;
        if(dist < 90000) {
            centroids[last_index].count++;
            centroids[last_index].x += (float)j["pose"]["pos"][0];
            centroids[last_index].y += (float)j["pose"]["pos"][1];
            continue;
        }
        centroids[last_index].x /= centroids[last_index].count;
        centroids[last_index].y /= centroids[last_index].count;
        std::cout << centroids[last_index].seriallize().dump() <<std::endl;
        Centroid new_centroid;
        new_centroid.x = j["pose"]["pos"][0];
        new_centroid.y = j["pose"]["pos"][1];
        new_centroid.count += 1;
        new_centroid.centroid_vector = vec;
        centroids.push_back(new_centroid);   
    }
}

void loadIndices(std::string filepath, std::vector<Centroid>& centroids) {
    std::ifstream i(filepath);
    std::string str;
    while(std::getline(i, str)) {
        nlohmann::json j = nlohmann::json::parse(str);
        centroids.push_back(Centroid::deserialize(j));
    }
}

void lookupCentroid(std::vector<Centroid>& centroids, std::vector<double>& score, std::vector<std::complex<double> >& d) {
   for(int i = 0; i < centroids.size(); i++) {
       score.push_back(compareScansEuclid(centroids[i].centroid_vector, d));
   } 
}


std::vector<Centroid> centroids;
Helper *helper;
Canvas *canvas;

void onRecievePointcloud(pcl::PointCloud<pcl::PointXYZ> points) {
    std::vector<double> score;
    std::vector<std::complex<double>> featureVector;

    getFeatureVector(points, featureVector);

    //lookupCentroid(centroids, score, featureVector);

    helper->setScores(score);

    canvas->update();
}

void onGroundTruth(geometry_msgs::TransformStamped stamp) {

    if(stamp.header.frame_id != "simple_cave_01") return;
    
    auto x = stamp.transform.translation.x;
    auto y = stamp.transform.translation.y;
    auto z = stamp.transform.translation.z;

    
    prev_pose = current_pose;
    current_pose = Eigen::Vector3f(x, y, z);
    
    helper->setLocation(x,y);

    x = stamp.transform.rotation.x;
    y = stamp.transform.rotation.y;
    z = stamp.transform.rotation.z;
    auto w = stamp.transform.rotation.w;
    
    prev_orientation = current_orientation;
    current_orientation = Eigen::Quaternionf(w,x,y,z);
}

void applicationThread() {
    while(ros::ok()) ros::spinOnce();
}

int main(int argc, char *argv[])
{
    srand(1000);
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/X1/points/", 1, &onRecievePointcloud);
    ros::Subscriber positionSub = n.subscribe("/X1/pose", 1, &onGroundTruth);
    centroid_viz = n.advertise<visualization_msgs::MarkerArray>("/centroid",1);

    //evaluate_test();
    
    //index(centroids);
    loadIndices("/home/arjo/Desktop/catkin_ws/index2.json", centroids);
     std::cout << centroids.size() <<std::endl; 

    QApplication a(argc, argv);
    QMainWindow window;
    helper = new Helper(centroids);
    canvas = new Canvas(helper, nullptr);


    std::thread app(applicationThread);

   // window.setCentralWidget(canvas);
    //window.resize(400, 300);
    //window.show();
    return a.exec();
}
