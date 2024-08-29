#ifndef URBAN_PROJECTION_H
#define URBAN_PROJECTION_H

#include <ros/ros.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/PointCloud2.h>
#include <cv_bridge/cv_bridge.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <image_transport/image_transport.h>
#include <opencv2/opencv.hpp>
#include <Eigen/Dense>
#include <fstream>

namespace Urban_Projection {

struct InitialParams {
    std::string camera_topic;
    std::string lidar_topic;
    bool show_ROI;
    bool rectify_image;
    std::string camera_type;
    cv::Mat camera_matrix;
    cv::Mat dist_coeffs;
    cv::Mat extrinsic_mat;
};

class Urban_Projector {
public:
    Urban_Projector(const std::string& config_file_path);

private:
    void initParams();
    void projection_Origin_Callback(const sensor_msgs::Image::ConstPtr &img, 
                                   const sensor_msgs::PointCloud2::ConstPtr &pc);
    void projection_Callback(const sensor_msgs::Image::ConstPtr &img, 
                            const sensor_msgs::PointCloud2::ConstPtr &pc);
    cv::Vec3b getJetColor(float v, float vmin, float vmax);
    cv::Mat undistortImage(const cv::Mat& img);
    cv::Mat undistortFisheyeImage(const cv::Mat& img);

    ros::NodeHandle nh_;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub_;
    ros::Subscriber image_sub_;
    ros::Subscriber pcl_sub_;
    ros::Publisher colored_cloud_pub_;
    InitialParams i_params;
    std::string config_file_path_;

    const float maxX = 50.0;
    const float maxY = 50.0;
    const float minZ = -1.5;
};

} // namespace Urban_Projection

#endif // PROJECT_H
