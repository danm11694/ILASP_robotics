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

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

namespace tabletop_segmentation
{

class PointCloudRGBCombiner
{
    typedef pcl::PointXYZRGB Point;

private:
    ros::NodeHandle nh_;
    ros::Publisher color_pcl_pub_;
    ros::Subscriber image_sub_, depthBuffer_sub_;

    sensor_msgs::Image currImg;
    std_msgs::Float32MultiArray depthBuff;

    std::string pcl_topic, vrep_depth_topic, image_topic;
    bool old_plugin;
    int v_res, u_res;
    double focal_length;
    double near_clip, far_clip;
    cv::Mat image_rgb;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergedCloud;

public:
    PointCloudRGBCombiner(ros::NodeHandle nh) : nh_(nh),
                                                old_plugin(false),
                                                u_res(480),
                                                v_res(640),
                                                near_clip(1.00e-02),
                                                far_clip(3.50e+00)
    {
        // image_topic = "/camera/color/image_rect_color";
        // pcl_topic = "/camera/depth/image";                          // topic from depth sensor in vrep
        // vrep_depth_topic = "/vrep/depth";
        nh_.param("camera_color_name", image_topic, std::string("camera/color/image_rect_color"));
        nh_.param("camera_depth_name", vrep_depth_topic, std::string("/vrep/depth"));
        nh_.param("pcd_topic", pcl_topic, std::string("/camera/depth/image"));
        nh_.param<bool>("old_plugin", old_plugin, true);

        color_pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(nh_.resolveName("camera/depth_registered/points"), 10); // why 10
        image_sub_ = nh_.subscribe(image_topic, 1, &PointCloudRGBCombiner::imageCb, this);

        depthBuffer_sub_ = nh_.subscribe(vrep_depth_topic, 1, &PointCloudRGBCombiner::detphBuffCb, this);

        mergedCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    }

    ~PointCloudRGBCombiner() {}

    void imageCb(const sensor_msgs::ImageConstPtr &msg)
    {
        currImg = *msg;

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

    void detphBuffCb(const std_msgs::Float32MultiArray &msg)
    {
        depthBuff = msg;
    }

    void update()
    {
        cv_bridge::CvImagePtr cv_color;
        std::vector<uint8_t> color_vect;
        ros::Time start_time = ros::Time::now();

        sensor_msgs::PointCloud2::ConstPtr recent_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(pcl_topic, nh_, ros::Duration(10.0));

        if (old_plugin)
        {
            while (!recent_cloud)
            {
                ROS_ERROR("Waiting for point cloud2 and image");
                sensor_msgs::PointCloud2::ConstPtr recent_cloud = ros::topic::waitForMessage<sensor_msgs::PointCloud2>(pcl_topic, nh_, ros::Duration(3.0));
            }

            // ROS_INFO_STREAM("pcl received after " << ros::Time::now() - start_time << " seconds. Start Combining");

            if (currImg.data.size() > 0)
            {
                merge(*const_cast<sensor_msgs::PointCloud2 *>(recent_cloud.get()), currImg.data);

                color_pcl_pub_.publish(*recent_cloud);
                cv_color.reset();
                recent_cloud.reset();
                color_vect.clear();
            }
        }
        else
        {
            if (depthBuff.data.size() > 0)
            {
                sensor_msgs::PointCloud2 cld_msg;
                mergedCloud = img2cloud(image_rgb, depthBuff);
                pcl::toROSMsg(*mergedCloud, cld_msg);
                cld_msg.header.frame_id = "camera_depth_optical_frame";
                color_pcl_pub_.publish(cld_msg);
            }
        }
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr img2cloud(cv::Mat rgb, std_msgs::Float32MultiArray depth_msg)
    {
        std::vector<float> depth = depth_msg.data;
        if (rgb.cols != v_res or rgb.rows != u_res or depth.size() != u_res * v_res)
        {
            std::cout << "RGB size " << rgb.cols << ", " << rgb.rows << "   rgb and depth image size mismatch" << '\n';
            exit(0);
        }

        focal_length = std::max(u_res, v_res) / tan(0.785398 / 2); //FOV 0.785398  (45 in radians)
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
        for (size_t u = 0; u < u_res; u++)
        {
            for (size_t v = 0; v < v_res; v++)
            {
                auto rgb_ints = rgb.at<cv::Vec3b>(u, v);
                int w = v_res * u + v;
                float z = far_clip * depth[w] + near_clip;
                float y = (u - u_res / 2) * z / focal_length;
                float x = (v - v_res / 2) * z / focal_length;
                pcl::PointXYZRGB p;
                p.x = x, p.y = y, p.z = z;
                p.r = (int)rgb_ints[0];
                p.g = (int)rgb_ints[1];
                p.b = (int)rgb_ints[2];
                cloud->points.push_back(p);
            }
        }
        cloud->width = cloud->points.size();
        cloud->height = 1;
        return cloud;
    }

    void merge(sensor_msgs::PointCloud2 &cloud, const std::vector<uint8_t> &colors)
    {

        size_t size = size_t(colors.size() / 3);
        size_t col = size_t(640);
        size_t row = size_t(480);
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_r(cloud, std::string("r"));
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_g(cloud, std::string("g"));
        sensor_msgs::PointCloud2Iterator<uint8_t> iter_b(cloud, std::string("b"));
        int count = 0;
        for (size_t j = 0; j < row; ++j)
        {
            for (size_t i = 0; i < col; ++i, ++iter_r, ++iter_g, ++iter_b)
            {
                count++;
                *iter_r = colors[3 * (i + (479 - j) * 640) + 0];
                *iter_g = colors[3 * (i + (479 - j) * 640) + 1];
                *iter_b = colors[3 * (i + (479 - j) * 640) + 2];
                //point cloud count from left to right, bottom to up while color image count from left to right, up to bottom
                //above conversion to force counting consistency of color with point cloud
            }
        }
    }
};
} // namespace tabletop_segmentation

int main(int argc, char **argv)
{
    ros::init(argc, argv, "combine_depth_rgb");
    ros::NodeHandle nh;

    tabletop_segmentation::PointCloudRGBCombiner node(nh);

    while (ros::ok())
    {
        node.update();
        ros::spinOnce();
    }

    return 0;
}
