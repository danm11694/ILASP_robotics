#ifndef _PEG_RING_HPP_
#define _PEG_RING_HPP_

#include <iostream>
#include <string>
#include <vector>
#include <set>
#include <map>
#include <mutex>
#include <unordered_map>

#include <tf_conversions/tf_eigen.h>
#include <tf/transform_broadcaster.h>

#include <ros/ros.h>
#include <ros/package.h>
#include <tf/transform_listener.h>
#include <std_msgs/Bool.h>
#include <std_msgs/Int32.h>
#include <std_msgs/String.h>
#include <geometry_msgs/PoseStamped.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl_ros/transforms.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/io/ply_io.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>

#include <pcd_utils.hpp>
#include <pcl_conversion_util.h>
#include <segmentation.hpp>
#include <registration_pipeline.h>

#include <dvrk_task_msgs/CloudArray.h>

// Boost
#include <boost/multi_array.hpp>
#include <boost/lexical_cast.hpp>

#include <opencv2/core/eigen.hpp>

class peg_ring
{
private:
    // Ros handler
    ros::NodeHandle nh_;
    ros::NodeHandle private_nh_;

    tf::TransformListener listener; // The listener receives tf transformations

    std::string fixed_frame;
    std::string optical_frame;
    std::string PSM_frame;
    std::string cld_topic_name;
    std::string camera_color_name;
    std::string camera_info_name;
    std::string Current_action;
    std::string tracking_value;
    std::string torus_algorithm;
    std::vector<std::string> tool_subs_topic;

    // Current sensor info
    Eigen::Vector3d position;
    Eigen::Vector4d orientation;
    tf::StampedTransform optical2map;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyz_cld_ptr;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyz_cld_ptr_original;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_scene;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cylinder;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_ring;

    std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr> obj_surf;

    bool simulation;
    std::string processing_color, prev_processing_color;
    int color_idx;
    std::string color_topic;
    bool cloud_updated;
    ros::Time goal_completion_time;
    float orig_cld_voxel_size;

    double object_cluster_distance_;
    int max_object_cluster_size_;
    int min_object_cluster_size_;
    int torus_points;

    double threshold_tracking;
    double ee_treshold;
    int num_ee;

    bool plane_detected;
    Eigen::Vector4f plane_parameters;
    Eigen::Matrix3f R;
    Eigen::Quaternionf qb;

    bool carry_state;
    bool end_carry;
    bool grasping_point;

    //tracking
    cv::Mat image_rgb;
    bool check_left;
    cv::Point prev_circle_center;
    cv::Mat mainAreaLeft;
    image_transport::Publisher image_pub_;
    image_transport::ImageTransport it_;
    cv_bridge::CvImage cv_ptr2send;

    // Subscribers
    ros::Subscriber cloud_sub; // cloud subscriber, from realsense
    ros::Subscriber color_sub;
    ros::Subscriber camera_sub;
    ros::Subscriber cloud_transformed_sub;
    ros::Subscriber camera_info_sub;
    ros::Subscriber success_topic_sub;
    ros::Subscriber peg_pose_sub;
    ros::Subscriber ee1_pose_sub;
    ros::Subscriber ee2_pose_sub;
    std::vector<ros::Subscriber> ee_pose_subs;

    // Publishers
    ros::Publisher cloud_pub;
    ros::Publisher pub_peg_pose;
    ros::Publisher pub_peg_poses;
    ros::Publisher pub_ring_pose;
    ros::Publisher plane_pub;
    ros::Publisher cylinder_pub;
    ros::Publisher ring_pub;
    ros::Publisher converted_pub;
    ros::Publisher tracking_pub;
    ros::Publisher pub_ring_track;
    ros::Publisher pub_ring_poses;
    ros::Publisher pub_ring_points;
    ros::Publisher torus_pose;
    ros::Publisher pub_ring_centers;
    ros::Publisher plane_pose_pub;

    std::vector<float> camera_K;
    std::vector<float> camera_R;
    std::vector<float> camera_P;
    std::vector<float> camera_D;

    cv::Point ee_center;

    // Messages
    sensor_msgs::PointCloud2 output_cloud, plane_cloud_msg, cylinder_cloud_msg, ring_cloud_msg, converted_cloud_msg;
    geometry_msgs::PoseStamped peg, ring, peg_pose, ring_track, plane_pose, torus;
    dvrk_task_msgs::CloudArray ring_point_sets;
    std::vector<geometry_msgs::PoseStamped> ee_pose;
    std_msgs::Bool track;
    geometry_msgs::PoseArray all_pegs, all_rings;

    double peg_height;

    Eigen::Matrix4f transform_1;

    std::mutex mtx;
    std::unordered_map<std::string, geometry_msgs::PoseStamped> tool_poses;

protected:
    virtual ros::NodeHandle &getNodeHandle() { return nh_; }
    virtual ros::NodeHandle &getPrivateNodeHandle() { return private_nh_; }

public:
    peg_ring(ros::NodeHandle &nh); // Constructor {} needed if you dont create constructor in cpp

    ~peg_ring() {}

    int init();
    int update();

    void processCloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster);

    void clusterPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster);
    void clusterAllPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr colorSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, std::string color);
    void colorSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

    void tracking2d();

    void colorTracking(std::string color);

    void pegsInitialize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_cyl);

    double solveWhitePeg(double x, double y, double m, double q, double d, double sign);

    void extractPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_filtered);

    void geometryGrasp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &ring_cloud);

    void closestGrasp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &ring_cloud);

    void graspPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &ring_cloud);

    geometry_msgs::PoseArray sendRingPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &ring_cloud);

    unsigned int findTorus(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_pn);


    template <typename T>
    cv::Mat vector2Mat(std::vector<T> vec, size_t N)
    {
        cv::Mat M = cv::Mat::eye(N, N, CV_32FC1);
        memcpy(M.data, vec.data(), vec.size() * sizeof(T));
        return M;
    }

    //------------------------------------------------------------------------------------------------------
    //                                  CALLBACKS
    //------------------------------------------------------------------------------------------------------

    // https://answers.ros.org/question/136916/conversion-from-sensor_msgspointcloud2-to-pclpointcloudt/
    void cloudCallback(const sensor_msgs::PointCloud2Ptr &input)
    {
        pcl::PCLPointCloud2 pcl_pc2;             //struttura pc2 di pcl
        pcl_conversions::toPCL(*input, pcl_pc2); //conversione a pcl della pc2
        input->header.frame_id = "camera_depth_optical_frame";

        pcl_pc2.fields[3].datatype = sensor_msgs::PointField::FLOAT32;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
        try
        {

            // update the pose -- in V-REP it's already published respect to world BUT frame id is related to the camera
            listener.waitForTransform(optical_frame, fixed_frame, input->header.stamp, ros::Duration(5.0));
            listener.lookupTransform(fixed_frame, optical_frame, input->header.stamp, optical2map);
            // optical2map : transformation FROM optical TO fixed

            // ASSUMES optical2map and sensor2map have the same translation!!!
            tf::Vector3 position_(optical2map.getOrigin());
            position.x() = position_.x();
            position.y() = position_.y();
            position.z() = position_.z();

            tf::Quaternion opt_quat_snsr(-0.5, 0.5, -0.5, 0.5);
            // same for realsense and kinect [ x: -0.5 y: 0.5  z: -0.5 w: 0.5]
            // tf::Quaternion opt_quat_snsr(0.0,0.0,0.0,1.0);
            tf::Quaternion orientation_(optical2map * opt_quat_snsr);

            // tf::Quaternion orientation_(optical2map.getRotation());
            orientation.x() = orientation_.x();
            orientation.y() = orientation_.y();
            orientation.z() = orientation_.z();
            orientation.w() = orientation_.w();

            // bring everything to FIXED FRAME ( if real camera_link, otherwise world )
            pcl_ros::transformPointCloud(*cloud, *xyz_cld_ptr, optical2map);
            xyz_cld_ptr->header.frame_id = fixed_frame;
            pcl::toROSMsg(*xyz_cld_ptr, converted_cloud_msg);
            converted_pub.publish(converted_cloud_msg);

            // update the cloud
            // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cld_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
            // pcl::copyPointCloud(*cloud, *cld_tmp);

            // cut far away points
            // pcl::PassThrough<pcl::PointXYZRGB> pass_;
            // pass_.setFilterFieldName("z");
            // pass_.setFilterLimits(0.1, 0.9); //real
            // // pass_.setFilterLimits(0.1, 2.0);
            // pass_.setInputCloud(cld_tmp);
            // pass_.setKeepOrganized(true);
            // pass_.filter(*xyz_cld_ptr);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }
        //} // END IF
    }

    // NOT USED - Switched on python
    void cloudTransformedCallback(const sensor_msgs::PointCloud2ConstPtr &input)
    {
        pcl::PCLPointCloud2 pcl_pc2;             //struttura pc2 di pcl
        pcl_conversions::toPCL(*input, pcl_pc2); //conversione a pcl della pc2

        pcl_pc2.fields[3].datatype = sensor_msgs::PointField::FLOAT32;

        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        pcl::fromPCLPointCloud2(pcl_pc2, *cloud);

        tf::StampedTransform optical2map_transf;

        try
        {
            // ROS_INFO_STREAM("fixed_frame  : " << fixed_frame);
            // update the pose
            listener.waitForTransform(fixed_frame, PSM_frame, input->header.stamp, ros::Duration(5.0));
            listener.lookupTransform(fixed_frame, PSM_frame, input->header.stamp, optical2map_transf);

            // listener.waitForTransform(fixed_frame, "/world", ros::Time(0) /* input->header.stamp */ , ros::Duration(5.0));
            // listener.lookupTransform(fixed_frame, "/world", ros::Time(0) /* input->header.stamp */ , optical2map_transf);

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyz_f(new pcl::PointCloud<pcl::PointXYZRGB>);
            pcl_ros::transformPointCloud(*cloud, *xyz_f, optical2map_transf);

            pcl::PassThrough<pcl::PointXYZRGB> pass_;
            pass_.setFilterFieldName("z");
            pass_.setFilterLimits(0.1, 0.9); //real
            // pass_.setFilterLimits(0.1, 2.0);
            pass_.setInputCloud(xyz_f);
            pass_.setKeepOrganized(true);
            pass_.filter(*xyz_cld_ptr);

            pcl::toROSMsg(*xyz_f, converted_cloud_msg);
            converted_pub.publish(converted_cloud_msg);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
        }
    }

    void color2processCallback(const std_msgs::Int32Ptr &color)
    {
        switch (color->data)
        {
        case 0:
            processing_color = "RED";
            break;
        case 1:
            processing_color = "GREEN";
            break;
        case 2:
            processing_color = "BLUE";
            break;
        case 3:
            processing_color = "YELLOW";
            break;

        default: //Optional
            processing_color = "";
            ROS_INFO_STREAM("color not valid, 0 - RED , 1- GREEN , 2 - BLUE , 3 -YELLOW");
        }
        cloud_updated = false;
        grasping_point = false;
        prev_processing_color = processing_color;
        color_idx = color->data;
    }

    void cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
    {
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
        camera_info_sub.shutdown();
    }

    void imageCallback(const sensor_msgs::ImageConstPtr &msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8); //msg->encoding  sensor_msgs::image_encodings::BGR8) if BGR8 assertion fail for cvtColor. ...dno if 16
            image_rgb = cv_ptr->image;
        }
        catch (cv_bridge::Exception &e)
        {
            ROS_ERROR("cv_bridge exception: %s", e.what());
            return;
        }
    }

    void carryCallback(const std_msgs::String::ConstPtr &msg)
    {
        if (msg->data == "carry")
        {
            // std::cout << "[TRUE] NOTIFIED CARRY" << std::endl;
            carry_state = true;
        }
        else
        {
            // std::cout << "[FALSE] NOTIFIED CARRY" << std::endl;
            carry_state = false;
        }
    }

    void peg_poseCallback(const geometry_msgs::PoseStampedPtr &msg)
    {
        peg_pose.pose.position = msg->pose.position;
        peg_pose.pose.orientation = msg->pose.orientation;
    }

    void toolPoseUpdate(const geometry_msgs::PoseStamped::ConstPtr &msg, std::string topicname)
    {
        mtx.lock();

        auto it = tool_poses.find(topicname);
        if (it != tool_poses.end())
        {
            it->second.header = msg->header;
            it->second.pose = msg->pose;
        }

        tool_poses[topicname].header = msg->header;
        tool_poses[topicname].pose = msg->pose;

        mtx.unlock();
    }
};

#endif
