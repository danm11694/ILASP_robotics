#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Float32MultiArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <typeinfo>
#include <sensor_msgs/CameraInfo.h>
#include <image_transport/image_transport.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcd_utils.hpp>

#include <opencv2/core.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/calib3d.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/aruco.hpp>

#include <iostream>
#include <fstream>

class ChessboardRecognition
{
    typedef pcl::PointXYZRGB Point;

private:
    ros::NodeHandle nh_, private_nh_;
    ros::Publisher color_pcl_pub_, marker_pose_pub_;
    ros::Subscriber image_sub_, camera_info_sub;
    image_transport::Publisher image_pub_;
    image_transport::ImageTransport it_;
    cv_bridge::CvImage cv_ptr2send;

    std::string pcl_topic, image_topic, camera_color_frame, camera_info_name, marker_frame, encoding;
    int camera_w, camera_h;
    cv::Mat image_rgb;
    cv::Mat D, K;

    std::vector<float> camera_K, camera_R, camera_P, camera_D;
    cv::Size patternSize;
    int pattern_width, pattern_height;
    float squareSize;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergedCloud;

    enum Pattern
    {
        CHESSBOARD,
        CIRCLES_GRID,
        ASYMMETRIC_CIRCLES_GRID
    };

public:
    ChessboardRecognition(ros::NodeHandle nh) : nh_(nh), private_nh_("~"),
                                                it_(nh)
    {
        std::string node_name = ros::this_node::getName();

        private_nh_.param("camera_color_name", image_topic, std::string(""));
        private_nh_.param("camera_info_name", camera_info_name, std::string(""));
        private_nh_.param("marker_frame", marker_frame, std::string(""));
        private_nh_.param("pattern_width", pattern_width, 8);
        private_nh_.param("pattern_height", pattern_height, 6);
        private_nh_.param("square_size", squareSize, 0.005f);

        color_pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(nh_.resolveName("/camera/depth_registered/points"), 10); // why 10

        image_sub_ = nh_.subscribe(image_topic, 1, &ChessboardRecognition::imageCb, this);
        camera_info_sub = nh_.subscribe(camera_info_name, 1, &ChessboardRecognition::cameraInfoCallback, this);

        marker_pose_pub_ = nh_.advertise<geometry_msgs::PoseStamped>(node_name + "/pose", 1);
        image_pub_ = it_.advertise(node_name + "/result", 1);

        mergedCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    }

    ~ChessboardRecognition() {}

    template <typename T>
    cv::Mat vector2Mat(std::vector<T> vec, size_t N)
    {
        cv::Mat M = cv::Mat::eye(N, N, CV_32FC1);
        memcpy(M.data, vec.data(), vec.size() * sizeof(T));
        return M;
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
    {
        camera_h = msg->height;
        camera_w = msg->width;
        camera_color_frame = msg->header.frame_id;
        for (int i = 0; i < 12; i++)
        {
            if (i < 5)
            {
                camera_D.push_back(msg->D[i]);
                camera_K.push_back(msg->K[i]);
                camera_R.push_back(msg->R[i]);
                camera_P.push_back(msg->P[i]);
            }
            else if (i >= 5 && i < 9)
            {
                camera_K.push_back(msg->K[i]);
                camera_R.push_back(msg->R[i]);
                camera_P.push_back(msg->P[i]);
            }
            else
                camera_P.push_back(msg->P[i]);
        }

        K = vector2Mat(camera_K, 3);
        cv::Mat camDist_temp = vector2Mat(camera_D, 5);
        D = camDist_temp.row(0);
        camera_info_sub.shutdown();
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            if (msg->encoding == "mono16")
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::MONO16);
            }
            else
            {
                cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            }
            encoding = msg->encoding;
            image_rgb = cv_ptr->image;
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    void calcChessboardCorners(cv::Size boardSize, float squareSize, std::vector<cv::Point3f> &corners, Pattern patternType = CHESSBOARD)
    {
        corners.resize(0);
        switch (patternType)
        {
        case CHESSBOARD:
        case CIRCLES_GRID:
            for (int i = 0; i < boardSize.height; i++)
                for (int j = 0; j < boardSize.width; j++)
                    corners.push_back(cv::Point3f(float(j * squareSize),
                                                  float(i * squareSize), 0));
            break;
        case ASYMMETRIC_CIRCLES_GRID:
            for (int i = 0; i < boardSize.height; i++)
                for (int j = 0; j < boardSize.width; j++)
                    corners.push_back(cv::Point3f(float((2 * j + i % 2) * squareSize),
                                                  float(i * squareSize), 0));
            break;
        default:
            CV_Error(cv::Error::StsBadArg, "Unknown pattern type\n");
        }
    }

    void poseEstimationFromCoplanarPoints(cv::Mat img, const cv::Size &patternSize, const float squareSize)
    {
        cv::Mat img_corners = img.clone(), img_pose = img.clone();
        std::vector<cv::Point2f> corners;
        if (encoding == "mono16")
        {
            double minVal, maxVal;
            minMaxLoc(img, &minVal, &maxVal);
            img.convertTo(img, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));
        }

        bool found = cv::findChessboardCorners(img, patternSize, corners);
        if (!found)
        {
            ROS_INFO_STREAM("Cannot find chessboard corners.");
            return;
        }
        cv::drawChessboardCorners(img_corners, patternSize, corners, found);
        // cv::imshow("Chessboard corners detection", img_corners);
        std::vector<cv::Point3f> objectPoints;
        calcChessboardCorners(patternSize, squareSize, objectPoints);
        std::vector<cv::Point2f> objectPointsPlanar;
        for (size_t i = 0; i < objectPoints.size(); i++)
        {
            objectPointsPlanar.push_back(cv::Point2f(objectPoints[i].x, objectPoints[i].y));
        }
        std::vector<cv::Point2f> imagePoints;
        cv::undistortPoints(corners, imagePoints, K, D);
        cv::Mat H = cv::findHomography(objectPointsPlanar, imagePoints);
        // Normalization to ensure that ||c1|| = 1
        double norm = std::sqrt(H.at<double>(0, 0) * H.at<double>(0, 0) +
                                H.at<double>(1, 0) * H.at<double>(1, 0) +
                                H.at<double>(2, 0) * H.at<double>(2, 0));
        H /= norm;
        cv::Mat c1 = H.col(0);
        cv::Mat c2 = H.col(1);
        cv::Mat c3 = c1.cross(c2);
        cv::Mat tvec = H.col(2);
        cv::Mat R(3, 3, CV_64F);
        for (int i = 0; i < 3; i++)
        {
            R.at<double>(i, 0) = c1.at<double>(i, 0);
            R.at<double>(i, 1) = c2.at<double>(i, 0);
            R.at<double>(i, 2) = c3.at<double>(i, 0);
        }
        cv::Mat W, U, Vt;
        cv::SVDecomp(R, W, U, Vt);
        R = U * Vt;
        cv::Mat rvec;
        // cv::Rodrigues(R, rvec);
        cv::Mat rot_x_180 = (cv::Mat_<double>(3, 3) << 1.0000000, 0.0000000, 0.0000000,
                             0.0000000, -1.0000000, -0.0000000,
                             0.0000000, 0.0000000, -1.0000000);
        cv::Mat fix_R = R * rot_x_180;
        cv::Rodrigues(fix_R, rvec);
        // cv::drawFrameAxes(img_pose, K, D, rvec, tvec, 2 * squareSize);
        cv::aruco::drawAxis(img_pose, K, D, rvec, tvec, 2 * squareSize);
        std::cout << " tvec " << tvec << std::endl;
        // auto rotationVectors;
        // auto translationVectors;

        // cv::aruco::estimatePoseSingleMarkers(img_corners, stagDimension, K, 
        // D, rotationVectors, translationVectors);
                    

        cv_ptr2send.image = img_pose;
        cv_ptr2send.encoding = encoding;
        image_pub_.publish(cv_ptr2send.toImageMsg());

        publish_marker(fix_R, tvec);

        /// Scrivere su file i punti in imagePoints
        std::ofstream out_file;
        out_file.open("/home/altair/ars_nephrectomy/src/utils/util_pkg/scripts/chessboard_pixel.txt");

        int nrow = patternSize.width;
        int ncol = patternSize.height;

        for (int i = 0; i < ncol; i++)
        {
            for (int j = 0; j < nrow ; j++)
            {
                out_file << i << " " << j << " " << corners[j + i * nrow].x << " " << corners[j + i * nrow].y;
                out_file << "\n";
            }
        }
        // ROS_INFO_STREAM("saved");
        out_file.close();

        // for (int i = 0; i < imagePoints.size; i++)
        // {
        //
        // out_file << imagePoints[i].x << " " << imagePoints[i].y;
        // out_file << "\n";
        // }
    }

    geometry_msgs::Quaternion matrix2quat(cv::Mat rotMatrix)
    {
        geometry_msgs::Quaternion q;
        double m00, m11, m22, m21, m12, m02, m20, m10, m01;
        m00 = rotMatrix.at<double>(0, 0);
        m11 = rotMatrix.at<double>(1, 1);
        m22 = rotMatrix.at<double>(2, 2);
        m21 = rotMatrix.at<double>(2, 1);
        m12 = rotMatrix.at<double>(1, 2);
        m02 = rotMatrix.at<double>(0, 2);
        m20 = rotMatrix.at<double>(2, 0);
        m10 = rotMatrix.at<double>(1, 0);
        m01 = rotMatrix.at<double>(0, 1);
        q.w = std::sqrt(1.0 + m00 + m11 + m22) / 2.0;
        double w4 = (4.0 * q.w);
        q.x = (m21 - m12) / w4;
        q.y = (m02 - m20) / w4;
        q.z = (m10 - m01) / w4;
        return q;
    }

    void publish_marker(cv::Mat rotMatrix, cv::Mat traslMatrix)
    {
        geometry_msgs::Quaternion q = matrix2quat(rotMatrix); // check if is correct in all the situations
        geometry_msgs::PoseStamped marker_pose;
        marker_pose.header.frame_id = camera_color_frame;
        marker_pose.header.stamp = ros::Time::now();
        marker_pose.pose.orientation.x = q.x;
        marker_pose.pose.orientation.y = q.y;
        marker_pose.pose.orientation.z = q.z;
        marker_pose.pose.orientation.w = q.w;
        marker_pose.pose.position.x = traslMatrix.at<double>(0);
        marker_pose.pose.position.y = traslMatrix.at<double>(1);
        marker_pose.pose.position.z = traslMatrix.at<double>(2);

        marker_pose_pub_.publish(marker_pose);
    }

    template <typename PointT>
    boost::shared_ptr<pcl::PointCloud<PointT>>
    point3fToPoinXYZ(std::vector<cv::Point3f> OpencVPointCloud, boost::shared_ptr<pcl::PointCloud<PointT>> &cld)
    {
        for (int i = 0; i < OpencVPointCloud.size(); i++)
        {
            pcl::PointXYZ point;
            point.x = OpencVPointCloud.at(i).x;
            point.y = OpencVPointCloud.at(i).y;
            point.z = OpencVPointCloud.at(i).z;
            cld->points.push_back(point);
        }
        cld->width = (int)cld->points.size();
        cld->height = 1;
        return cld;
    }

    void update()
    {
        if (!image_rgb.empty() && !K.empty() && !D.empty())
        {
            patternSize = cv::Size(pattern_width, pattern_height);
            poseEstimationFromCoplanarPoints(image_rgb, patternSize, squareSize);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "chessboard_detection");
    ros::NodeHandle nh;

    ChessboardRecognition node(nh);

    while (ros::ok())
    {
        node.update();
        ros::spinOnce();
    }

    return 0;
}
