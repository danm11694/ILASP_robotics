#include <ros/ros.h>
#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>
#include <sensor_msgs/PointCloud2.h>
#include <std_srvs/Empty.h>

#include <pcl_conversions/pcl_conversions.h>
#include <Eigen/Dense>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/features/fpfh.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/io/ply_io.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/common/transforms.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/transformation_estimation_svd.h>

namespace registration_pipeline
{
double computeCloudResolution(const pcl::PointCloud<pcl::PointNormal>::Ptr &inputCloud);
void keypointDetection(
    const pcl::PointCloud<pcl::PointNormal>::Ptr &inputCloud,
    pcl::PointCloud<pcl::PointNormal>::Ptr outputCloud);

void featureExtraction(
    const pcl::PointCloud<pcl::PointNormal>::Ptr &inputCloud,
    const pcl::PointCloud<pcl::PointNormal>::Ptr &inputKeypoints,
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr outputFeatures);

void registration(
    const pcl::PointCloud<pcl::PointNormal>::Ptr &inputSourceCloud,
    const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &inputSourceFeatures,
    const pcl::PointCloud<pcl::PointNormal>::Ptr &inputTargetCloud,
    const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &inputTargetFeatures);

void featureBasedRegistation(
    const pcl::PointCloud<pcl::PointNormal>::Ptr &inputCloudSource,
    const pcl::PointCloud<pcl::PointNormal>::Ptr &inputCloudTarget);

void rigidRegitration(
    const pcl::PointCloud<pcl::PointNormal>::Ptr &inputCloudSource,
    const pcl::PointCloud<pcl::PointNormal>::Ptr &inputCloudTarget);

void updateNormals(const tf::Vector3 &cP);

template <class T>
typename pcl::PointCloud<T>::Ptr voxelize(typename pcl::PointCloud<T>::Ptr cld_in);

void init();

void save();

void doReg();

void registrationLoop(const ros::Publisher &mp);

void SlamPointCloudUpdate(const sensor_msgs::PointCloud2ConstPtr &msg);

bool RegisterInvoked(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response);


} // namespace registration_pipeline
