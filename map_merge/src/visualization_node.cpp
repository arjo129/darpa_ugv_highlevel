#include <QtWidgets/QApplication>
#include <QtWidgets/QMainWindow>
#include <QtCharts/QScatterSeries>
#include <QtCharts/QLegendMarker>
#include <QtGui/QImage>
#include <QtGui/QPainter>
#include <QtCore/QtMath>
#include <QtCore/QDir>
#include <map_merge/laser_operations.h>
#include <sensor_msgs/LaserScan.h>
#include <nlohmann_json/json.h>
#include <fstream>
#include "visualization/chartview.h"

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
    decomposeLidarScanIntoPlanes(pointCloud, msgs);
    std::vector<sensor_msgs::LaserScan> normalized_scans;
    for(auto& scan: msgs) {
        sensor_msgs::LaserScan normalized_scan, downsampled_scan;
        centroidNormalization(scan, normalized_scan, 0.1);
        downsample(normalized_scan, downsampled_scan, 50);
        FFT1D(downsampled_scan, d);
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
void getFeatureVector(nlohmann::json& j, std::vector<std::complex<double>>& d) {
    pcl::PointCloud<pcl::PointXYZ> cloud, rotated_cloud;
    retrievePointCloud(j, cloud);
    randomYaw(cloud, rotated_cloud);
    getFeatureVector(rotated_cloud, d);
} 

int main(int argc, char *argv[])
{
    srand(1000);
    QDir directory("/media/arjo/ba3893af-3249-49c4-8441-fb741cff5e26/triples");
    QStringList images = directory.entryList(QStringList() << "*.json",QDir::Files);
    foreach(QString filename, images) {
        std::ifstream i(directory.filePath(filename).toStdString());
        nlohmann::json j;
        i >> j;
        for(auto k: j){
            std::vector<std::complex<double>> anchor_vector; 
            getFeatureVector(k["anchor"], anchor_vector);

            std::vector<std::complex<double>> neg_vector; 
            getFeatureVector(k["neg"], neg_vector);

            std::vector<std::complex<double>> pos_vector; 
            getFeatureVector(k["pos"], pos_vector);

            float pos = compareScansEuclid(pos_vector, anchor_vector);
            float neg = compareScansEuclid(neg_vector, anchor_vector);
            std::cout << pos << "\t| " << neg << std::endl; 
        }
    }

    //std::ifstream i("/media/arjo/ba3893af-3249-49c4-8441-fb741cff5e26/out.json");
    /*std::string str;
    while(std::getline(i, str)) {
        nlohmann::json j = nlohmann::json::parse(str);
        std::cout << j["pose"]["pos"] << std::endl;

        pcl::PointCloud<pcl::PointXYZ> pointCloud;
        for(auto point: j["points"]) {
            pcl::PointXYZ pt;          
            pt.x = point[0];
            pt.y = point[1];
            pt.z = point[2];
        
            pointCloud.push_back(pt);
        }
        std::vector<sensor_msgs::LaserScan> msgs;
        decomposeLidarScanIntoPlanes(pointCloud, msgs);
        std::vector<std::complex<double> > d;
        std::vector<sensor_msgs::LaserScan> normalized_scans;
        for(auto& scan: msgs) {
            sensor_msgs::LaserScan normalized_scan, downsampled_scan;
            centroidNormalization(scan, normalized_scan, 0.1);
            downsample(normalized_scan, downsampled_scan, 50);
            FFT1D(downsampled_scan, d);
            std::cout << d.size() <<std::endl;
            
        }

    }*/
    /*

    QApplication a(argc, argv);
    ChartView *chartView = new ChartView();
    QMainWindow window;
    window.setCentralWidget(chartView);
    window.resize(400, 300);
    window.show();
    return a.exec();*/
}
