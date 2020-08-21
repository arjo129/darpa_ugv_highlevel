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

void getFeatureVector(pcl::PointCloud<pcl::PointXYZ>& pointCloud, std::vector<std::complex<double>>& d) {
    std::vector<sensor_msgs::LaserScan> msgs;
    std::cout << __LINE__ <<std::endl;
    decomposeLidarScanIntoPlanes(pointCloud, msgs);
    std::cout << __LINE__ <<std::endl;
    std::vector<sensor_msgs::LaserScan> normalized_scans;
    std::cout << __LINE__ <<std::endl;
    for(auto& scan: msgs) {
        std::cout << __LINE__ <<std::endl;
        sensor_msgs::LaserScan normalized_scan, downsampled_scan;
        std::cout << __LINE__ <<std::endl;
        centroidNormalization(scan, normalized_scan, 0.1);
        std::cout << __LINE__ <<std::endl;
        downsample(normalized_scan, downsampled_scan, 50);
        std::cout << __LINE__ <<std::endl;
        FFT1D(downsampled_scan, d);
        std::cout << __LINE__ <<std::endl;
    }
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
    std::cout << __LINE__ <<std::endl;

    getFeatureVector(points, featureVector);
        std::cout << __LINE__ <<std::endl;

    lookupCentroid(centroids, score, featureVector);
    std::cout << __LINE__ <<std::endl;

    helper->setScores(score);
    std::cout << __LINE__ <<std::endl;

    canvas->update();

    std::cout << __LINE__ <<std::endl;
}

void onGroundTruth(geometry_msgs::TransformStamped stamp) {

    if(stamp.header.frame_id != "simple_cave_01") return;
    
    auto x = stamp.transform.translation.x;
    auto y = stamp.transform.translation.z;
    helper->setLocation(x,y);
}

void applicationThread() {
    while(ros::ok()) ros::spinOnce();
}

int main(int argc, char *argv[])
{
    srand(1000);
    ros::init(argc, argv, "talker");
    ros::NodeHandle n;
    ros::Subscriber sub = n.subscribe("/X1/points/", 10, &onRecievePointcloud);
    ros::Subscriber positionSub = n.subscribe("/X1/pose", 10, &onGroundTruth);
    //evaluate_test();
    
    //index(centroids);
    loadIndices("/home/arjo/Desktop/catkin_ws/index2.json", centroids);
     std::cout << centroids.size() <<std::endl; 
        std::cout << __LINE__ <<std::endl;

    QApplication a(argc, argv);
    QMainWindow window;
    helper = new Helper(centroids);
    canvas = new Canvas(helper, nullptr);

    std::cout << __LINE__ <<std::endl;

    std::thread app(applicationThread);

    window.setCentralWidget(canvas);
    window.resize(400, 300);
    window.show();
    return a.exec();
}
