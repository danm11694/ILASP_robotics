#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <Eigen/Dense>

#include <sensor_msgs/CameraInfo.h>
#include <cv_bridge/cv_bridge.h>
#include <opencv2/core/core.hpp>
#include <opencv2/opencv.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <std_srvs/Empty.h>

#define degreesToRadians(angleDegrees) (angleDegrees * M_PI / 180.0) // Converts degrees to radians
#define radiansToDegrees(angleRadians) (angleRadians * 180.0 / M_PI) // Converts radians to degrees

class NeedleSemanticSeg
{
public:
    NeedleSemanticSeg(ros::NodeHandle &nh);
    ~NeedleSemanticSeg();

    void grabImage(const sensor_msgs::ImageConstPtr &msg);
    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg);
    void cameraSemanticCallback(const sensor_msgs::ImageConstPtr &msg);
    void cameraSemanticCallbackComp(const sensor_msgs::CompressedImageConstPtr &msg);
    void PublishRenderedImage(image_transport::Publisher pub, cv::Mat image, std::string encoding);
    cv::Mat eul2rotm(geometry_msgs::Vector3 eul);

    void morphOperation(cv::Mat img, cv::Mat &morphMask, int op, int morph_size);
    cv::Point findRightmostPoint(const std::vector<cv::Point> &points);

    cv::Mat createROI(std::vector<cv::Point2f> coord, cv::Mat &img);
    bool readCameraParameters(std::string filename);
    cv::Mat poseFromPoints(std::vector<cv::Point3f> objectPoints, std::vector<cv::Point> corners);
    cv::Mat increaseRightPartOfMask(const cv::Mat &mask, int numPixels);
    cv::Mat increaseMaskDimension(const cv::Mat &mask, int numPixels);
    void adapt_to_template(cv::Mat &in, cv::Mat &out, int morph_op);
    bool initNeedleTrack(cv::Mat &templateImg, cv::Mat &img, float recThreshold, std::vector<cv::Point2f> &point2Track, cv::Point &corner_template);

    bool startProcedure(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);
    bool saveData(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);

    std::vector<cv::Point3d> getPoints3d();
    std::vector<cv::Point3d> getMaskPoints3d();

    cv::Rect detectRegions(cv::Mat &img, int min_area);
    void filterContour(cv::Mat &img, cv::Mat &mask);

private:
    // Ros handler
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    tf::TransformBroadcaster *br;
    tf::TransformListener listener;

    std::string camera_image_frame, reference_frame, template_img_str, camera_filename, log_points;
    std::string config_path, vocab_path;

    ros::ServiceServer service_start;
    ros::ServiceServer service_save;
    bool start_procedure, save_data;
    double start_time;
    std::vector<double> act_time;
    std::vector<cv::Point3d> points3d;
    std::vector<cv::Point3d> mask_points3d;

    std::vector<cv::Point2f> tracked_features;
    cv::Point track_point;
    double rec_threshold;
    double pixel_value;
    bool is_initialized, image_calib;
    
    double pix2mm;
    cv::Mat R_camera;

    cv::Mat D, K;
    cv::Mat H;

    cv::Size pattern_size;
    int pattern_width, pattern_height;
    float square_size;

    int height, width;
    double fx;
    double fy;
    double cx;
    double cy;
    double baseline;
    double k1;
    double k2;
    double p1;
    double p2;
    double k3;

    ros::Publisher cloud_pub;
    ros::Subscriber camera_image_sub, camera_info_sub;

    image_transport::ImageTransport it_;
    image_transport::Publisher debug_image_publisher_;
    image_transport::Publisher rendered_image_publisher_, rendered_image_publisher_two;

    ros::Subscriber camera_semantic_sub;

    cv::Mat template_img;

    void convertToMat(geometry_msgs::PoseStamped &pose);
    cv::Mat getCameraPose();

    std::vector<std::string> anatomies;
    std::vector<std::vector<int>> colors;
    std::vector<int> idx_color;
    int index_color;
    std::vector<std::string> mask_anatomy;
    cv::Mat semantic_img;
    int slam_color;
};