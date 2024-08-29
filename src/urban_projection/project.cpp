#include "../../include/urban_projection/project.h"
#include <ros/package.h>
#include <opencv2/calib3d.hpp>
#include <opencv2/imgproc.hpp>
#include <message_filters/subscriber.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

namespace Urban_Projection {

Urban_Projector::Urban_Projector(const std::string& config_file_path) : it_(nh_), config_file_path_(config_file_path)
{
    initParams();

    message_filters::Subscriber<sensor_msgs::Image> image_sub(nh_, i_params.camera_topic, 1);
    message_filters::Subscriber<sensor_msgs::PointCloud2> pcl_sub(nh_, i_params.lidar_topic, 1);

    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image, sensor_msgs::PointCloud2> MySyncPolicy;
    message_filters::Synchronizer<MySyncPolicy> sync(MySyncPolicy(10), image_sub, pcl_sub);
    sync.registerCallback(boost::bind(&Urban_Projector::projection_Callback, this, _1, _2));

    image_pub_ = it_.advertise("/urban_sensor/project_pc_image", 1);
    colored_cloud_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("/urban_sensor/colored_point_cloud", 1);
    ROS_INFO("Urban_Projector initialized. Waiting for data...");
    ros::spin();
}

void Urban_Projector::initParams()
{
    //std::string pkg_loc = ros::package::getPath("urban_projection");
    std::ifstream infile(config_file_path_);
    
    if (!infile.is_open()) {
        ROS_ERROR("Failed to open config file. Please check the file path.");
        ros::shutdown();
        return;
    }

    std::string line, key, value;
    while (std::getline(infile, line)) {
        std::istringstream iss(line);
        if (std::getline(iss, key, ':')) {
            std::getline(iss, value);
            value.erase(0, value.find_first_not_of(" \t"));  // Trim leading spaces

            if (key == "RectifyImage") {
                i_params.rectify_image = (value == "True" || value == "true");
            } else if (key == "show_ROI"){
                i_params.show_ROI = (value == "True" || value == "true");
            }
              else if (key == "CameraType") {
                i_params.camera_type = value;
            } else if (key == "CameraTopic") {
                i_params.camera_topic = value;
            } else if (key == "LidarTopic") {
                i_params.lidar_topic = value;
            } else if (key == "CameraMatrix") {
                i_params.camera_matrix = cv::Mat::eye(3, 3, CV_64F);
                for (int i = 0; i < 3; i++) {
                    std::getline(infile, line);
                    std::istringstream matrix_iss(line);
                    for (int j = 0; j < 3; j++) {
                        matrix_iss >> i_params.camera_matrix.at<double>(i, j);
                    }
                }
            } else if (key == "DistCoeffs") {
                i_params.dist_coeffs = cv::Mat::zeros(1, 4, CV_64F);
                std::istringstream dist_iss(value);
                for (int i = 0; i < 4; i++) {
                    dist_iss >> i_params.dist_coeffs.at<double>(0, i);
                }
            } else if (key == "ExtrinsicMat") {
                i_params.extrinsic_mat = cv::Mat::eye(4, 4, CV_64F);
                for (int i = 0; i < 4; i++) {
                    std::getline(infile, line);
                    std::istringstream matrix_iss(line);
                    for (int j = 0; j < 4; j++) {
                        matrix_iss >> i_params.extrinsic_mat.at<double>(i, j);
                    }
                }
            }
        }
    }

    infile.close();

    ROS_INFO("Parameters loaded successfully.");
    ROS_INFO("Camera type: %s", i_params.camera_type.c_str());
    ROS_INFO("show_ROI: %s", i_params.show_ROI ? "True" : "False");
    ROS_INFO("Rectify image: %s", i_params.rectify_image ? "True" : "False");
    ROS_INFO("Camera topic: %s", i_params.camera_topic.c_str());
    ROS_INFO("Lidar topic: %s", i_params.lidar_topic.c_str());
    ROS_INFO("Extrinsic matrix:");
    for (int i = 0; i < 4; i++) {
        ROS_INFO("%f %f %f %f", i_params.extrinsic_mat.at<double>(i,0), 
                                i_params.extrinsic_mat.at<double>(i,1),
                                i_params.extrinsic_mat.at<double>(i,2),
                                i_params.extrinsic_mat.at<double>(i,3));
}

}


cv::Mat Urban_Projector::undistortImage(const cv::Mat& img)
{
    cv::Mat undistorted;
    cv::undistort(img, undistorted, i_params.camera_matrix, i_params.dist_coeffs);
    return undistorted;
}

cv::Mat Urban_Projector::undistortFisheyeImage(const cv::Mat& img)
{
    cv::Mat undistorted;
    //cv::Mat new_camera_matrix;
    //cv::fisheye::estimateNewCameraMatrixForUndistortRectify(i_params.camera_matrix, i_params.dist_coeffs, img.size(), cv::Matx33d::eye(), new_camera_matrix, 1);
    cv::fisheye::undistortImage(img, undistorted, i_params.camera_matrix, i_params.dist_coeffs, i_params.camera_matrix);
    return undistorted;
}
cv::Vec3b Urban_Projector::getJetColor(float v, float vmin, float vmax)
{
    cv::Vec3b c = {255,255,255}; // white
    float dv;

    if (v < vmin)
        v = vmin;
    if (v > vmax)
        v = vmax;
    dv = vmax - vmin;

    if (v < (vmin + 0.25 * dv)) {
        c[2] = 0;
        c[1] = static_cast<uchar>(255 * (4 * (v - vmin) / dv));
    } else if (v < (vmin + 0.5 * dv)) {
        c[2] = 0;
        c[0] = static_cast<uchar>(255 * (1 + 4 * (vmin + 0.25 * dv - v) / dv));
    } else if (v < (vmin + 0.75 * dv)) {
        c[2] = static_cast<uchar>(255 * (4 * (v - vmin - 0.5 * dv) / dv));
        c[0] = 0;
    } else {
        c[1] = static_cast<uchar>(255 * (1 + 4 * (vmin + 0.75 * dv - v) / dv));
        c[0] = 0;
    }

    return c;
}

void Urban_Projector::projection_Callback(const sensor_msgs::Image::ConstPtr &img, 
                                   const sensor_msgs::PointCloud2::ConstPtr &pc)
{
    cv_bridge::CvImagePtr cv_ptr;
    try {
        cv_ptr = cv_bridge::toCvCopy(img, "bgr8");
    }
    catch (cv_bridge::Exception &e) {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
    cv::Mat raw_img = cv_ptr->image;

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZI>);
    pcl::fromROSMsg(*pc, *cloud);
    
    cv::Mat overlay = raw_img.clone();
    
    std::vector<cv::Point3f> pts_3d;
    std::vector<float> intensities;
    float max_intensity = 0;
    float min_intensity = std::numeric_limits<float>::max();

    for (const auto& point : cloud->points) {
        if (point.x > maxX || point.x < 0.0 || std::abs(point.y) > maxY || point.z < minZ) {
            continue;
        }
        pts_3d.emplace_back(cv::Point3f(point.x, point.y, point.z));
        intensities.push_back(point.intensity);
        max_intensity = std::max(max_intensity, point.intensity);
        min_intensity = std::min(min_intensity, point.intensity);
    }

    cv::Mat r_vec, t_vec;
    cv::Rodrigues(i_params.extrinsic_mat(cv::Rect(0, 0, 3, 3)), r_vec);
    t_vec = i_params.extrinsic_mat(cv::Rect(3, 0, 1, 3));

    // Ensure r_vec and t_vec are continuous
    r_vec = r_vec.clone();
    t_vec = t_vec.clone();

    std::vector<cv::Point2f> pts_2d;
    try {
        if (i_params.camera_type == "pinhole") {
            cv::projectPoints(pts_3d, r_vec, t_vec, i_params.camera_matrix, i_params.dist_coeffs, pts_2d);
        } else if (i_params.camera_type == "fisheye") {
            cv::fisheye::projectPoints(pts_3d, pts_2d, r_vec, t_vec, i_params.camera_matrix, i_params.dist_coeffs);
        }
    } catch (cv::Exception &e) {
        ROS_ERROR("OpenCV exception: %s", e.what());
        return;
    }
    if (i_params.show_ROI){
        cv::Point2f min_pt(std::numeric_limits<float>::max(), std::numeric_limits<float>::max());
        cv::Point2f max_pt(std::numeric_limits<float>::lowest(), std::numeric_limits<float>::lowest());

        for (const auto& pt : pts_2d) {
            min_pt.x = std::min(min_pt.x, pt.x);
            min_pt.y = std::min(min_pt.y, pt.y);
            max_pt.x = std::max(max_pt.x, pt.x);
            max_pt.y = std::max(max_pt.y, pt.y);
        }

        // Ensure the bounding box is within the image boundaries
        min_pt.x = std::max(0.0f, min_pt.x);
        min_pt.y = std::max(0.0f, min_pt.y);
        max_pt.x = std::min(static_cast<float>(raw_img.cols - 1), max_pt.x);
        max_pt.y = std::min(static_cast<float>(raw_img.rows - 1), max_pt.y);
        float width = max_pt.x - min_pt.x;
        float height = max_pt.y - min_pt.y;
        
        ROS_INFO_THROTTLE(1, "Calculated ROI: x_offset: %.2f, y_offset: %.2f, width: %.2f, height: %.2f", 
                      min_pt.x, min_pt.y, width, height);

    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colored_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    colored_cloud->points.reserve(pts_2d.size());
    for (size_t i = 0; i < pts_2d.size(); ++i) {
        if (pts_2d[i].x >= 0 && pts_2d[i].x < overlay.cols && pts_2d[i].y >= 0 && pts_2d[i].y < overlay.rows) {
            // Get color from the image
            cv::Vec3b PCcolor = raw_img.at<cv::Vec3b>(cv::Point(pts_2d[i].x, pts_2d[i].y));
            
            // Create a new colored point
            pcl::PointXYZRGB colored_point;
            colored_point.x = pts_3d[i].x;
            colored_point.y = pts_3d[i].y;
            colored_point.z = pts_3d[i].z;
            colored_point.r = PCcolor[2];  // OpenCV uses BGR, PCL uses RGB
            colored_point.g = PCcolor[1];
            colored_point.b = PCcolor[0];
            
            // Add the point to the new cloud
            colored_cloud->points.push_back(colored_point);

            // Draw on the overlay image (optional, for visualization)
            cv::Vec3b IMGcolor = getJetColor(intensities[i], min_intensity, max_intensity);
            cv::circle(overlay, pts_2d[i], 2, cv::Scalar(IMGcolor[0], IMGcolor[1], IMGcolor[2]), -1);  // BGR format
        }
    }

    colored_cloud->width = colored_cloud->points.size();
    colored_cloud->height = 1;
    colored_cloud->is_dense = false;

    // Publish the colored point cloud
    sensor_msgs::PointCloud2 colored_cloud_msg;
    pcl::toROSMsg(*colored_cloud, colored_cloud_msg);
    colored_cloud_msg.header = pc->header;  // Use the same header as the input cloud
    colored_cloud_pub_.publish(colored_cloud_msg);

    // Publish the overlay image (optional, for visualization)
    sensor_msgs::ImagePtr msg = cv_bridge::CvImage(std_msgs::Header(), "bgr8", overlay).toImageMsg();
    image_pub_.publish(msg);

    ROS_INFO_THROTTLE(10, "Processed and published colored point cloud and projected image");
}


} // namespace Urban_Projection

int main(int argc, char **argv) {
    ros::init(argc, argv, "Projection_pc_image");
    std::string config_file_path;
    ros::NodeHandle private_nh("~");
    if (!private_nh.getParam("config_file", config_file_path)) {
        ROS_ERROR("Config file path not provided!");
        return 1;
    }
    Urban_Projection::Urban_Projector projector(config_file_path);
    return 0;
}
