#include "../include/ThermalPointCloudMapper.h"

const double THERMAL_HFOV = toRad(55.0); //110x75
const double THERMAL_VFOV = toRad(35.0);
const double DEPTH_HFOV  = toRad(87.0);
const double DEPTH_VFOV  = toRad(58.0);

ThermalPointCloudMapper::ThermalPointCloudMapper() : frameTransform({0}) {
    // define the subscriber and publisher
    sub_thermal_img = _nh.subscribe("/thermal_front/image_raw", 1, &ThermalPointCloudMapper::thermal_img_callback, this);
    sub_depth_img = _nh.subscribe("/camera/depth/image_rect_raw", 1, &ThermalPointCloudMapper::depth_img_callback, this);

    _filteredPoints_visual = _nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/thermal/actual", 1);

    try {
        tf2_ros::Buffer tfBuffer;
        tf2_ros::TransformListener tfListener(tfBuffer);
        auto tfStamped = tfBuffer.lookupTransform("camera_depth_optical_frame", "thermal_optical_frame", ros::Time(0));
        frameTransform = getAffinityMatrix(tfStamped.transform);
        std::cout << tfStamped << std::endl;
    } catch (tf2::TransformException &ex) {
        ROS_WARN("Failed to get transform(s): %s", ex.what());
        frameTransform.Rxx = 1;
        frameTransform.Ryy = 1;
        frameTransform.Rzz = 1;
    }

    ROS_INFO("Ready!");
    image_transport::ImageTransport it(_nh);
    pub_thermal_image = it.advertise("/thermal_front/actual", 1);
    
    _filteredPoints_visual = _nh.advertise<pcl::PointCloud<pcl::PointXYZRGB>>("/thermal/filtered", 1);
}

void ThermalPointCloudMapper::setBgrToArray(uchar* arr, uchar blue, uchar green, uchar red) {
    arr[0] = blue;
    arr[1] = green;
    arr[2] = red;
}

bool ThermalPointCloudMapper::isAcceptableRange(int z) {
    return isBounded(z, 100, 50000);
}

bool ThermalPointCloudMapper::isBounded(int val, int lower, int upper) {
    return (lower <= val && val <= upper);
}

void ThermalPointCloudMapper::depth_img_callback(const sensor_msgs::ImageConstPtr& msg) {
    depthImgPtr = msg;
}

void ThermalPointCloudMapper::thermal_img_callback(const sensor_msgs::ImageConstPtr& msg) {

    cv::Mat rawThermalMonoImg = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO8)->image;
    cv::Mat depthMonoImg = cv_bridge::toCvCopy(depthImgPtr, sensor_msgs::image_encodings::TYPE_16UC1)->image;

    auto begin = std::chrono::steady_clock::now();

    cv::Mat directThermalMonoImg = getDirectImageMap(depthMonoImg, rawThermalMonoImg,
                                                        DEPTH_HFOV, DEPTH_VFOV,
                                                        THERMAL_HFOV, THERMAL_VFOV,
                                                        &frameTransform);
    cv::Mat smartThermalMonoImg = getSmartThermalOverlay(depthMonoImg, directThermalMonoImg);

    auto end = std::chrono::steady_clock::now();

    ROS_INFO("Elapsed time: %dms", (int)std::chrono::duration_cast<std::chrono::milliseconds>(end - begin).count());

    sensor_msgs::ImagePtr output_msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", smartThermalMonoImg).toImageMsg();
    output_msg->header.frame_id = "camera_depth_optical_frame";
    pub_thermal_image.publish(msg);
}

int ThermalPointCloudMapper::getWatershedMarkers(const cv::Mat &src, cv::Mat &markers) {

    cv::Mat bw(src.rows, src.cols, CV_8UC1);
    cv::Mat gray(src.rows, src.cols, CV_8UC3);

    const int input_size = src.rows * src.cols;
    uchar* input_data = src.data;
    uchar* input_endData = input_data + input_size;

    uchar* gray_data = gray.data;
    uchar* bw_data = bw.data;

    while (input_data < input_endData) {
        const uchar pix = (*input_data != 0) ? (255 - *input_data) : 0;

        *bw_data = pix;
        bw_data++;

        setBgrToArray(gray_data, pix, pix, pix);
        gray_data += 3;

        input_data++;
    }

    cv::Point anchor(-1,-1);
    cv::Mat kernel3x3 = cv::Mat::ones(3, 3, CV_8U);
    cv::Mat kernel5x5 = cv::Mat::ones(5, 5, CV_8U);

    cv::erode(bw, bw, kernel5x5, anchor, 3);
    cv::morphologyEx(bw, bw, cv::MORPH_OPEN, kernel3x3, anchor, 5);
    cv::dilate(bw, bw, kernel3x3, anchor, 2);

    // Perform the distance transform algorithm
    cv::Mat dist;
    cv::distanceTransform(bw, dist, cv::DIST_L2, 3);

    // Normalize the distance image for range = {0.0, 1.0}
    // so we can visualize and threshold it
    cv::normalize(dist, dist, 0, 1.0, cv::NORM_MINMAX);
    cv::threshold(dist, dist, 0.01, 1.0, cv::THRESH_BINARY);

    // Dilate a bit the dist image
    cv::dilate(dist, dist, kernel3x3);
    
    // Create the CV_8U version of the distance image
    // It is needed for findContours()
    cv::Mat dist_8u;
    dist.convertTo(dist_8u, CV_8U);

    // Find total markers
    vector<vector<cv::Point> > contours;
    cv::findContours(dist_8u, contours, cv::RETR_EXTERNAL, cv::CHAIN_APPROX_SIMPLE);

    // Create the marker image for the watershed algorithm
    markers.release();
    markers = cv::Mat::zeros(dist.size(), CV_32S);

    // Draw the foreground markers
    for (size_t i = 0; i < contours.size(); i++) {
        cv::drawContours(markers, contours, static_cast<int>(i), cv::Scalar(static_cast<int>(i)+1), -1);
    }

    // Draw the background marker
    cv::circle(markers, cv::Point(1,1), 1, cv::Scalar(255), 0);

    // Perform the watershed algorithm
    cv::watershed(gray, markers);

    // Visualize the final image
    //cv::imshow("Src", src * 100);
    //cv::imshow("Segmentation", markers * 20000);
    //cv::waitKey(10);
    //! [watershed]

    return contours.size() + 1;
}

Affine ThermalPointCloudMapper::getAffinityMatrix(const geometry_msgs::Transform &tf) {
    Affine transform;

    const double x = tf.rotation.x;
    const double y = tf.rotation.y;
    const double z = tf.rotation.z;
    const double w = tf.rotation.w;

    const double xx = x * x;
    const double xy = x * y;
    const double xz = x * z;
    const double xw = x * w;

    const double yy = y * y;
    const double yz = y * z;
    const double yw = y * w;

    const double zw = z * w;
    const double zz = z * z;

    // Affine
    transform.Rxx = 1 - 2 * (yy + zz);
    transform.Rxy = 2 * ( xy - zw );
    transform.Rxz = 2 * ( xz + yw );

    transform.Ryx = 2 * ( xy + zw );
    transform.Ryx = 1 - 2 * ( xx + zz );
    transform.Ryx = 2 * ( yz - xw );

    transform.Rzx = 2 * ( xz - yw );
    transform.Rzx = 2 * ( yz + xw );
    transform.Rzx = 1 - 2 * ( xx + yy );

    // Translation
    transform.tx = tf.translation.x;
    transform.ty = tf.translation.y;
    transform.tz = tf.translation.z;
    
    return transform;
}


uchar ThermalPointCloudMapper::map(uchar val, uchar lowerbound, uchar upperbound) {
    if (val <= lowerbound) {
        return 0;
    } else if (upperbound <= val) {
        return 255;
    } else {
        return ((val - lowerbound) / (upperbound - lowerbound + 0.01)) * 255;
    }
}

cv::Point2i ThermalPointCloudMapper::project3dToPixel(const cv::Point3d &pt,
                                                        double HFOV, double VFOV, int img_h, int img_w) {
    const double fx = img_w / (2 * tan(HFOV / 2.0));
    const double fy = img_h / (2 * tan(VFOV / 2.0));
    const double cx = (img_w / 2.0);
    const double cy = (img_h / 2.0);

    cv::Point2i uv_rect;
    uv_rect.x = (fx * pt.x) / pt.z + cx;
    uv_rect.y = (fy * pt.y) / pt.z + cy;
    return uv_rect;
}

cv::Point3d ThermalPointCloudMapper::pixelToProject3d(
                                        int x, int y, double depth, double HFOV, double VFOV, int img_h, int img_w) {
    const double fx = img_w / (2 * tan(HFOV / 2.0));
    const double fy = img_h / (2 * tan(VFOV / 2.0));
    const double cx = (img_w / 2.0);
    const double cy = (img_h / 2.0);

    cv::Point3d pt;
    pt.z = depth;
    pt.x = (depth * (x - cx)) / fx;
    pt.y = (depth * (y - cy)) / fy;

    return pt;
}

cv::Point3d ThermalPointCloudMapper::transform(cv::Point3d &pt, const Affine *tf) {
    cv::Point3d pt_res;
    pt_res.x = pt.x * tf->Rxx + pt.y * tf->Rxy + pt.z * tf->Rxz + tf->tx;
    pt_res.y = pt.x * tf->Ryx + pt.y * tf->Ryy + pt.z * tf->Ryz + tf->ty;
    pt_res.z = pt.x * tf->Rzx + pt.y * tf->Rzy + pt.z * tf->Rzz + tf->tz;

    return pt_res;
}

cv::Mat ThermalPointCloudMapper::getDirectImageMap(
                                const cv::Mat &depth_img, const cv::Mat &ref_img,
                                double depth_HFOV, double depth_VFOV,
                                double ref_HFOV, double ref_VFOV,
                                const Affine *sourceToRefTransform) {

    const int depth_w = depth_img.cols;
    const int depth_h = depth_img.rows;

    const int ref_w = ref_img.cols;
    const int ref_h = ref_img.rows;

    cv::Mat res(depth_img.rows, depth_img.cols, CV_8UC1);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr processed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);

    for (int dy = 0; dy < depth_h; dy++) {
        for (int dx = 0; dx < depth_w; dx++) {
            const double depth = depth_img.at<int16_t>(dy, dx); // unit: mm -> cm

            if (!isAcceptableRange(depth)) {
                res.at<uchar>(dy, dx) = 0;
                continue;
            }

            cv::Point3d pt_pre = pixelToProject3d(dx, dy, depth / 1000.0, depth_HFOV, depth_VFOV, depth_h, depth_w);

            cv::Point3d pt_post = transform(pt_pre, sourceToRefTransform);
            cv::Point2d uv = project3dToPixel(pt_post, ref_HFOV, ref_VFOV, ref_h, ref_w);
            if (!isBounded(uv.x, 0, ref_w) || !isBounded(uv.y, 0, ref_h)) {
                continue;
            }

            const uchar intensity = ref_img.at<uchar>(uv);
            res.at<uchar>(dy, dx) = intensity;

            // For Debugging
            const uchar red = map(intensity, 0, 30);
            pcl::PointXYZRGB out_pt(red, 0, 255 - red);
            out_pt.x = pt_pre.x;
            out_pt.y = pt_pre.y;
            out_pt.z = pt_pre.z;

            //ROS_INFO("PT [%f, %f, %f] -> [%d, %d]", pt_pre.x, pt_pre.y, pt_pre.z, (int)uv.x, (int)uv.y);
            processed_cloud->push_back(out_pt);
        }
    }

    //For debugging
    ROS_INFO("PC size: %d", (int)processed_cloud->size());
    processed_cloud->header.frame_id = "camera_depth_optical_frame";
    _filteredPoints_visual.publish(processed_cloud);

    return res;
}

cv::Mat ThermalPointCloudMapper::getSmartThermalOverlay(const cv::Mat &depth_img, const cv::Mat &mapped_thermal_img) {

    cv::Mat markers;
    cv::Mat depth_img_8u;
    depth_img.convertTo(depth_img_8u, CV_8U, 255.0 / UINT16_MAX);

    const int numOfMarkers = getWatershedMarkers(depth_img_8u, markers);
    float* sum_intensity = new float[numOfMarkers] { 0 };
    int* pix_count = new int[numOfMarkers] { 0 };

    const int marker_size = markers.rows * markers.cols;
    uchar* thermal_data = mapped_thermal_img.data;
    int* marker_data = reinterpret_cast<int*>(markers.data);
    int* marker_endData = marker_data + marker_size;
    while (marker_data < marker_endData) {

        if (isBounded(*marker_data, 1, numOfMarkers - 1)) {
            sum_intensity[*marker_data] += *thermal_data;
            pix_count[*marker_data]++;
        }

        marker_data++;
        thermal_data++;
    }

    for (int idx = 1; idx < numOfMarkers; idx++) {
        if (pix_count[idx] != 0) {
            sum_intensity[idx] = sum_intensity[idx] / pix_count[idx];
        }
    }

    cv::Mat res(mapped_thermal_img.rows, mapped_thermal_img.cols, CV_8UC3);
    
    thermal_data = mapped_thermal_img.data;
    marker_data = reinterpret_cast<int*>(markers.data);
    uchar* output_data = res.data;

    while (marker_data < marker_endData) {

        //const uchar intensity = (*marker_data == 0) ? *thermal_data : (uchar)sum_intensity[*marker_data];
        const uchar intensity = (isBounded(*marker_data, 1, numOfMarkers - 1))
                                    ? (uchar)sum_intensity[*marker_data] : *thermal_data;
        const uchar red = map(intensity, 0, 60);
        //ROS_INFO("%d -> %d", *marker_data, intensity);
        
        marker_data++;
        thermal_data++;

        setBgrToArray(output_data, 255 - red, 0, red);
        output_data += 3;
    }

    delete[] sum_intensity;
    delete[] pix_count;

    //cv::imshow("Raw Thermal Overlay", mapped_thermal_img);
    //cv::imshow("Processed Thermal Overlay", res);
    return res;
}
