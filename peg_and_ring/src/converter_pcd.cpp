#include <iostream>
#include <string>
#include <vector>

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

std::string fixed_frame, optical_frame, cloud_pcd, converted_pcd;
tf::StampedTransform optical2map;
pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyz_cld_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
sensor_msgs::PointCloud2 converted_cloud_msg;

tf::TransformBroadcaster *broadcaster;
tf::TransformListener *listener;

void cloudCallback(const sensor_msgs::PointCloud2ConstPtr &input)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*input, pcl_pc2);

    pcl_pc2.fields[3].datatype = sensor_msgs::PointField::FLOAT32;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud);
    try
    {
        listener->waitForTransform(optical_frame, fixed_frame, input->header.stamp, ros::Duration(5.0));
        listener->lookupTransform(fixed_frame, optical_frame, input->header.stamp, optical2map);

        // bring everything to FIXED FRAME ( if real camera_link, otherwise world )
         pcl_ros::transformPointCloud(*cloud, *xyz_cld_ptr, optical2map);
         xyz_cld_ptr->header.frame_id = fixed_frame;
         pcl::toROSMsg(*xyz_cld_ptr, converted_cloud_msg);
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
    }
}

int convert_pcd_main(int argc, char **argv)
{
    ros::init(argc, argv, "convert_pcd_cpp");
    ros::NodeHandle nh;
    ros::NodeHandle pnh("~");

//    ros::Rate loop_rate(200);

    broadcaster = new tf::TransformBroadcaster();
    listener = new tf::TransformListener();

    pnh.param("target_frame", fixed_frame, std::string("")); // world
    pnh.param("source_frame", optical_frame, std::string(""));
    pnh.param("in", cloud_pcd, std::string(""));
    pnh.param("out", converted_pcd, std::string(""));
    std::cout << "------------------------------------" << std::endl;
    std::cout << fixed_frame << "  " << optical_frame << std::endl;

    ros::Subscriber slamcloud_sub = nh.subscribe(cloud_pcd, 1, &cloudCallback);
    ros::Publisher cloud_pub = nh.advertise<sensor_msgs::PointCloud2>(converted_pcd, 1);

    while (ros::ok())
    {
        if (converted_cloud_msg.data.size() > 0)
            cloud_pub.publish(converted_cloud_msg);

        ros::spinOnce();

        //loop_rate.sleep();
    }
    return 0;
}

int main(int argc, char **argv)
{
    return convert_pcd_main(argc, argv);
}