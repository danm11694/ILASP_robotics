#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Float32MultiArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <image_transport/image_transport.h>

#include <tf/transform_listener.h>
#include <tf/transform_broadcaster.h>
#include <geometry_msgs/TransformStamped.h>
#include <typeinfo>
#include <sensor_msgs/CameraInfo.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcd_utils.hpp>
#include <opencv2/highgui.hpp>
#include <opencv2/imgcodecs.hpp>
#include <opencv2/imgproc.hpp>
#include <opencv2/video/tracking.hpp>

#define BOUNDING_BOX 100

class evaluateCalibration
{
    typedef pcl::PointXYZRGB Point;

private:
    ros::NodeHandle nh_, private_nh_;
    ros::Subscriber image_sub_, image_depth_sub_, camera_info_sub, point_pose_sub;
    ros::Publisher point_ee_pub, marker_ee_pub, marker_3d_pub;
    image_transport::ImageTransport it_;
    image_transport::Publisher image_pub, image_pub_color;

    tf::TransformBroadcaster *broadcaster;
    tf::TransformListener *listener;
    cv_bridge::CvImage cv_ptr, cv_ptr_color;

    geometry_msgs::PoseStamped poi_pose, marker_3d_pose;

    sensor_msgs::Image currImg;
    std_msgs::Float32MultiArray depthBuff;
    geometry_msgs::Point point_ee, marker_ee;

    std::string pcl_topic, depth_topic, image_topic, camera_color_frame, camera_info_name, arm_topic;
    int camera_h, camera_w;

    cv::Mat image_rgb, image_depth, K, P, D, debug_image;
    std::vector<float> camera_K, camera_R, camera_P, camera_D;

    // Kalman Filter stuff
    int stateSize = 6;
    int measSize = 3;
    int contrSize = 0;
    unsigned int type = CV_32F;
    cv::KalmanFilter *kf;

    bool isInitialized;
    // std::vector<cv::Point2f> pt_bounding_box{cv::Point2f(0, 0), cv::Point2f(0, BOUNDING_BOX), cv::Point2f(BOUNDING_BOX, BOUNDING_BOX), cv::Point2f(BOUNDING_BOX, 0)};
    std::vector<cv::Point2f> pt_bounding_box{cv::Point2f(-BOUNDING_BOX / 2, -BOUNDING_BOX / 2), cv::Point2f(0, BOUNDING_BOX), cv::Point2f(BOUNDING_BOX, BOUNDING_BOX), cv::Point2f(BOUNDING_BOX, 0)};

public:
    evaluateCalibration(ros::NodeHandle &nh) : nh_(nh),
                                               private_nh_("~"),
                                               it_(nh),
                                               isInitialized(false)
    {
        private_nh_.param("camera_color_name", image_topic, std::string("/camera/color/image_rect_color"));
        private_nh_.param("camera_info_name", camera_info_name, std::string("/camera/color/camera_info"));
        private_nh_.param("camera_depth_name", depth_topic, std::string("/camera/depth/image_rect_color"));
        private_nh_.param("arm_topic", arm_topic, std::string("/dvrk/PSM1/position_cartesian_current"));

        image_sub_ = nh_.subscribe(image_topic, 1, &evaluateCalibration::imageCb, this);
        image_depth_sub_ = nh_.subscribe(depth_topic, 1, &evaluateCalibration::imageDepthCb, this);
        camera_info_sub = nh_.subscribe(camera_info_name, 1, &evaluateCalibration::cameraInfoCallback, this);
        point_pose_sub = nh_.subscribe(arm_topic, 1, &evaluateCalibration::pointOfInterestCallback, this);

        image_pub = it_.advertise("/debug/output_video", 1);
        image_pub_color = it_.advertise("/debug/output_video_color", 1);
        point_ee_pub = nh_.advertise<geometry_msgs::Point>("debug/point_ee", 1, this);
        marker_ee_pub = nh_.advertise<geometry_msgs::Point>("debug/marker_ee", 1, this);
        marker_3d_pub = nh_.advertise<geometry_msgs::PoseStamped>("debug/3d_marker_ee", 1, this);

        listener = new tf::TransformListener();
        broadcaster = new tf::TransformBroadcaster();
        kf = new cv::KalmanFilter(stateSize, measSize, contrSize, type);

        // state space model
        // it is assigned later in the initialization step of the filter
        kf->transitionMatrix = cv::Mat::zeros(stateSize, stateSize, type);

        // this is the matrix which maps the measured states to the internal
        kf->measurementMatrix = cv::Mat::zeros(measSize, stateSize, type);
        kf->measurementMatrix.at<float>(0, 0) = 1.0f;
        kf->measurementMatrix.at<float>(1, 2) = 1.0f;
        kf->measurementMatrix.at<float>(2, 4) = 1.0f;

        // the velocity is more noisy
        kf->processNoiseCov.at<float>(0, 0) = 1e-1;
        kf->processNoiseCov.at<float>(1, 1) = 1e2;
        kf->processNoiseCov.at<float>(2, 2) = 1e-1;
        kf->processNoiseCov.at<float>(3, 3) = 1e2;
        kf->processNoiseCov.at<float>(4, 4) = 1e-1;
        kf->processNoiseCov.at<float>(5, 5) = 1e2;

        // this should be enough
        cv::setIdentity(kf->measurementNoiseCov, cv::Scalar(1e-2));
    }

    ~evaluateCalibration()
    {
        delete listener;
        delete broadcaster;
    }

    template <typename T>
    cv::Mat vector2Mat(std::vector<T> vec, size_t N)
    {
        cv::Mat M = cv::Mat(N, N, CV_32FC1);
        memcpy(M.data, vec.data(), vec.size() * sizeof(T));
        return M;
    }

    template <typename T, size_t N>
    std::vector<T> makeVector(const T (&data)[N])
    {
        return std::vector<T>(data, data + N);
    }

    cv::Mat hue_blue(cv::Mat bgr)
    {
        cv::Mat hueMask_blue, HSV;
        cv::cvtColor(bgr, HSV, cv::COLOR_BGR2HSV);
        inRange(HSV, cv::Scalar(100, 50, 50), cv::Scalar(120, 255, 255), hueMask_blue);

        return hueMask_blue;
    }

    cv::Mat hue_red(cv::Mat HSV)
    {
        cv::Mat hueMask_red_upper;
        inRange(HSV, cv::Scalar(160, 50, 90), cv::Scalar(180, 255, 255), hueMask_red_upper);
        cv::Mat hueMask_red_lower;
        inRange(HSV, cv::Scalar(0, 50, 90), cv::Scalar(10, 255, 255), hueMask_red_lower);

        return hueMask_red_lower | hueMask_red_upper;
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
        P = vector2Mat(camera_P, 4);
        cv::Mat camDist_temp = vector2Mat(camera_D, 5);
        D = camDist_temp.row(0);
        camera_info_sub.shutdown();
    }

    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        currImg = *msg;

        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //msg->encoding  sensor_msgs::image_encodings::BGR8) if BGR8 assertion fail for cvtColor. ...dno if 16
            image_rgb = cv_ptr->image;

            debug_image = image_rgb.clone();
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    void imageDepthCb(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::TYPE_16UC1); //msg->encoding  sensor_msgs::image_encodings::BGR8) if BGR8 assertion fail for cvtColor. ...dno if 16
            image_depth = cv_ptr->image;
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    void pointOfInterestCallback(const geometry_msgs::PoseStampedPtr &msg)
    {
        poi_pose = *msg;
    }

    cv::Mat createROI(std::vector<cv::Point2f> coord, cv::Mat &img)
    {
        std::vector<cv::Point> pt;
        for (int ao = 0; ao < coord.size(); ao++)
        {
            pt.push_back(coord.at(ao));
        }
        cv::polylines(img, pt, true, cv::Scalar(255, 0, 0), 1, 8, 0);

        std::vector<cv::Point> ROI_Poly;
        cv::approxPolyDP(pt, ROI_Poly, 1.0, true);
        cv::Mat mask(img.size(), CV_8UC1);

        for (int i = 0; i < mask.cols; i++)
            for (int j = 0; j < mask.rows; j++)
                mask.at<uchar>(cv::Point(i, j)) = 0;

        cv::fillConvexPoly(mask, &ROI_Poly[0], ROI_Poly.size(), 255, 8, 0);
        return mask;
    }

    cv::Point2f convert3D2D(const geometry_msgs::PoseStamped &pose, cv::Mat P, cv::Mat K, cv::Mat D, std::string camera_frame)
    {
        tf::StampedTransform transform;
        cv::Point2f img_point;
        std::string source_frame = pose.header.frame_id;
        std::vector<cv::Point2f> img;
        try
        {
            listener->waitForTransform(camera_frame, source_frame, ros::Time(0), ros::Duration(10.0));
            listener->lookupTransform(camera_frame, source_frame, ros::Time(0), transform); // target , source (frame where the data originated)
            geometry_msgs::PoseStamped A, B;
            A.header.frame_id = source_frame;
            A.header.stamp = ros::Time(0);
            A.pose.position = pose.pose.position;
            A.pose.orientation = pose.pose.orientation;
            listener->transformPose(camera_frame, A, B); // target_frame,in , out

            cv::Mat t_vec = (cv::Mat_<float>(1, 3) << 0, 0, 0);
            cv::Mat r_vec = (cv::Mat_<float>(1, 3) << 0, 0, 0);

            std::vector<cv::Point3f> objectPoints;
            cv::Point3f p(B.pose.position.x, B.pose.position.y, B.pose.position.z);
            objectPoints.push_back(p);
            cv::projectPoints(objectPoints, r_vec, t_vec, K, D, img);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
        return img.at(0);
    }

    void adaptiveBB(std::vector<cv::Point2f> &pt_bounding_box, cv::Point2f p)
    {
        cv::Point2f p_prev;
        // cv::Point2f p = convert3D2D(poi_pose, P_, K_, D_, id);
        for (size_t i = 0; i < pt_bounding_box.size(); i++)
        {
            p_prev.x += pt_bounding_box.at(i).x;
            p_prev.y += pt_bounding_box.at(i).y;
        }
        p_prev.x /= pt_bounding_box.size();
        p_prev.y /= pt_bounding_box.size();

        // float d = std::sqrt((p.x - p_prev.x) * (p.x - p_prev.x) + (p.y - p_prev.y) * (p.y - p_prev.y));

        // if (d < 75)
        for (size_t i = 0; i < pt_bounding_box.size(); i++)
            pt_bounding_box.at(i) += (p - p_prev);
    }

    void convert2D3D(cv::Mat bin_img, std::string target_frame, pcl::PointCloud<pcl::PointXYZ>::Ptr &marker_cld)
    {

        // pcl::PointCloud<pcl::PointXYZ>::Ptr marker_cld(new pcl::PointCloud<pcl::PointXYZ>());
        std::vector<cv::Point> locations;
        cv::findNonZero(bin_img, locations);

        float cx = K.at<float>(0, 2);
        float cy = K.at<float>(1, 2);
        float fx_inv = 1.0 / K.at<float>(0, 0);
        float fy_inv = 1.0 / K.at<float>(1, 1);

        for (size_t i = 0; i < locations.size(); i++)
        {
            uint16_t z_raw = image_depth.at<uint16_t>(locations.at(i));

            if (z_raw != 0.0)
            {
                float z_metric = z_raw * 0.001;
                if (z_metric < 0.35)
                {
                    pcl::PointXYZ out_points;
                    out_points.x = z_metric * ((locations.at(i).x - cx) * fx_inv);
                    out_points.y = z_metric * ((locations.at(i).y - cy) * fy_inv);
                    out_points.z = z_metric;
                    marker_cld->push_back(out_points);
                }
            }
        }
    }

    void kalman_filter_init(tf::Vector3 &centroid)
    {
        // [x, v_x, y, v_y, z, v_z]
        auto state = cv::Mat(stateSize, 1, type);

        kf->errorCovPre.at<float>(0, 0) = 1; // px
        kf->errorCovPre.at<float>(1, 1) = 1; // px per second
        kf->errorCovPre.at<float>(2, 2) = 1; // px
        kf->errorCovPre.at<float>(3, 3) = 1; // px per second
        kf->errorCovPre.at<float>(4, 4) = 1; // px
        kf->errorCovPre.at<float>(5, 5) = 1; // px per second

        // the very first value in the filter is the current
        // tracked catheter position
        // [x, y, z]
        state.at<float>(0) = centroid.x();
        state.at<float>(1) = 0;
        state.at<float>(2) = centroid.y();
        state.at<float>(3) = 0;
        state.at<float>(4) = centroid.z();
        state.at<float>(5) = 0;

        kf->statePost = state;
    }

    cv::Mat kalman_filter_track(tf::Vector3 &centroid, bool is_tracking)
    {
        // [x, y, z]
        auto meas = cv::Mat(measSize, 1, type);

        kf->transitionMatrix.at<float>(0, 0) = 1;
        kf->transitionMatrix.at<float>(0, 1) = 0.01504;
        kf->transitionMatrix.at<float>(1, 0) = 0;
        kf->transitionMatrix.at<float>(1, 1) = 0.5488;

        kf->transitionMatrix.at<float>(2, 2) = 1;
        kf->transitionMatrix.at<float>(2, 3) = 0.01504;
        kf->transitionMatrix.at<float>(3, 2) = 0;
        kf->transitionMatrix.at<float>(3, 3) = 0.5488;

        kf->transitionMatrix.at<float>(4, 4) = 1;
        kf->transitionMatrix.at<float>(4, 5) = 0.01504;
        kf->transitionMatrix.at<float>(5, 4) = 0;
        kf->transitionMatrix.at<float>(5, 5) = 0.5488;

        // if we are tracking the catheter with the optical flow
        // let's update the measurement
        if (is_tracking)
        {
            meas.at<float>(0) = centroid.x();
            meas.at<float>(1) = centroid.y();
            meas.at<float>(2) = centroid.z();
            kf->correct(meas);
        }

        // let's predict the catheter pose using the kalman filter
        return kf->predict();
    }

    //-------------------------------------------

    void update()
    {
        if (!image_rgb.empty() && !K.empty())
        {
            cv::Mat mask, dest;
            mask = createROI(pt_bounding_box, image_rgb);
            image_rgb.copyTo(dest, mask);

            // std::cout << P << " " << K << " " << D << " " << pt_bounding_box << std::endl;
            cv::Point2f p = convert3D2D(poi_pose, P, K, D, "camera_color_optical_frame");
            point_ee.x = p.x;
            point_ee.y = p.y;
            point_ee_pub.publish(point_ee);

            adaptiveBB(pt_bounding_box, p);
            cv::Mat HSV;
            cv::cvtColor(dest, HSV, CV_BGR2HSV);

            cv::Mat _mask = hue_red(HSV);
            cv::Mat mask_opening;
            int morph_elem = 2; // Element:\n 0: Rect - 1: Cross - 2: Ellipse"
            int morph_size = 2;
            cv::Mat element = getStructuringElement(morph_elem, cv::Size(2 * morph_size + 1, 2 * morph_size + 1), cv::Point(morph_size, morph_size));
            morphologyEx(_mask, mask_opening, 3, element);

            cv::Mat new_mask = cv::Mat::zeros(mask_opening.size(), CV_8UC1);
            if (!isInitialized)
            {
                pcl::PointCloud<pcl::PointXYZ>::Ptr marker_cld(new pcl::PointCloud<pcl::PointXYZ>());

                cv::Moments m = cv::moments(mask_opening, false);
                cv::Point p_m = cv::Point(m.m10 / m.m00, m.m01 / m.m00);

                cv::circle(new_mask, p_m, 3, cv::Scalar(255, 255, 255), -1, 8);
                cv::circle(debug_image, p_m, 3, cv::Scalar(255, 0, 0), -1, 8);
                convert2D3D(new_mask, "camera_color_optical_frame", marker_cld);

                Eigen::Vector4f centroid_marker;
                pcl::compute3DCentroid<pcl::PointXYZ>(*marker_cld, centroid_marker);
                tf::Transform centroidTf;
                centroidTf.setOrigin(tf::Vector3(centroid_marker[0], centroid_marker[1], centroid_marker[2]));
                centroidTf.setRotation(tf::Quaternion::getIdentity());

                kalman_filter_init(centroidTf.getOrigin());
                isInitialized = true;
            }
            else
            {

                pcl::PointCloud<pcl::PointXYZ>::Ptr marker_cld(new pcl::PointCloud<pcl::PointXYZ>());

                cv::Moments m = cv::moments(mask_opening, false);
                cv::Point p_m = cv::Point(m.m10 / m.m00, m.m01 / m.m00);
                cv::circle(new_mask, p_m, 3, cv::Scalar(255, 255, 255), -1, 8);
                cv::circle(debug_image, p_m, 3, cv::Scalar(255, 0, 0), -1, 8);
                convert2D3D(new_mask, "camera_color_optical_frame", marker_cld);

                bool is_depth_good = marker_cld->points.size() > 0;
                if (!is_depth_good)
                {
                    ROS_WARN_STREAM("Predict stage only");
                }

                Eigen::Vector4f centroid_marker;
                pcl::compute3DCentroid<pcl::PointXYZ>(*marker_cld, centroid_marker);
                tf::Transform centroidTf;
                centroidTf.setOrigin(tf::Vector3(centroid_marker[0], centroid_marker[1], centroid_marker[2]));
                centroidTf.setRotation(tf::Quaternion::getIdentity());
                auto mesaStates = kalman_filter_track(centroidTf.getOrigin(), is_depth_good);

                marker_3d_pose.header.frame_id = "camera_color_optical_frame";
                marker_3d_pose.pose.position.x = mesaStates.at<float>(0);
                marker_3d_pose.pose.position.y = mesaStates.at<float>(2);
                marker_3d_pose.pose.position.z = mesaStates.at<float>(4);
            }

            cv_ptr.image = image_rgb;
            cv_ptr.encoding = "bgr8";
            image_pub.publish(cv_ptr.toImageMsg());

            cv_ptr_color.image = debug_image;
            cv_ptr_color.encoding = "bgr8";
            image_pub_color.publish(cv_ptr_color.toImageMsg());

            marker_3d_pub.publish(marker_3d_pose);

            geometry_msgs::TransformStamped transformStamped;
            transformStamped.header.stamp = ros::Time::now();
            transformStamped.header.frame_id = "camera_color_optical_frame";
            transformStamped.child_frame_id = "marker_3d_ee";
            transformStamped.transform.translation.x = marker_3d_pose.pose.position.x;
            transformStamped.transform.translation.y = marker_3d_pose.pose.position.y;
            transformStamped.transform.translation.z = marker_3d_pose.pose.position.z;
            transformStamped.transform.rotation.x = 0;
            transformStamped.transform.rotation.y = 0;
            transformStamped.transform.rotation.z = 0;
            transformStamped.transform.rotation.w = 1;
            broadcaster->sendTransform(transformStamped);
        }
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "evaluate_calibration");
    ros::NodeHandle nh;

    evaluateCalibration node(nh);

    ros::Rate loop_rate(50);

    while (ros::ok())
    {
        node.update();
        ros::spinOnce();
        if (!loop_rate.sleep())
        {
            ROS_WARN_STREAM("Cannot keep the update frequency");
        }
    }

    return 0;
}
