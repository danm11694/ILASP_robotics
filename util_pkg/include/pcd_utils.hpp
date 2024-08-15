#pragma once
#include <ros/console.h>
#include <geometry_msgs/PoseStamped.h>
// PCL
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>

#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/registration/icp.h>
#include <pcl/io/png_io.h>
#include <pcl/console/parse.h>
#include <pcl/io/point_cloud_image_extractors.h>
#include <pcl/console/print.h>
#include <pcl/io/pcd_io.h>
#include <pcl/conversions.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/features/normal_3d.h>
#include <pcl/PointIndices.h>

#include <opencv2/opencv.hpp>

// Ann
#include <ANN/ANN.h>

class pcd_utils
{
private:
	class ex_VoxelGrid : public pcl::VoxelGrid<pcl::PointXYZ>
	{
	public:
		ex_VoxelGrid()
		{
			extract_removed_indices_ = true;
		}
		virtual ~ex_VoxelGrid() {}
	};

public:
	template <typename PointT>
	static boost::shared_ptr<pcl::PointCloud<PointT>>
	voxel_grid_subsample(const boost::shared_ptr<pcl::PointCloud<PointT>> &cld_in, float cell_size);

	template <typename PointT>
	static boost::shared_ptr<pcl::PointCloud<PointT>>
	rnd_subsample(const boost::shared_ptr<pcl::PointCloud<PointT>> &cld_in, int size);

	static void color_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cld_ptr,
								 pcl::PointIndices::Ptr indices, Eigen::Vector3f color);

	static void saveImage(const std::string &filename, const pcl::PCLImage &image);

	static pcl::PCLImage getPNGImage(const std::string &filename, pcl::PointCloud<pcl::PointXYZ>::Ptr proc_cloud_ptr, bool flip);

	static void PcdRGBtoHSV(pcl::PointCloud<pcl::PointXYZRGB> &in, pcl::PointCloud<pcl::PointXYZHSV> &out);

	static void PointRGBtoHSV(pcl::PointXYZRGB &in, pcl::PointXYZHSV &out);

	// static geometry_msgs::PoseStamped findTorus(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_pn, std::string torus_algorithm, double min, double max, int inliers, std::string ref_frame);
	static int findTorus(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_pn, std::string torus_algorithm, double min, double max, int inliers, std::string ref_frame);

	static void makeSphere(float radius, int r, int g, int b);
	static void makeSphere(pcl::PointCloud<pcl::PointXYZRGB> &cloud, const float &radius, const Eigen::Vector4f &c);

	static void makeEllipsoid(pcl::PointCloud<pcl::PointXYZRGB> &cloud, const Eigen::Vector3f radii, const Eigen::Vector4f &c);

	template <typename PointT>
	static void SavePointCloud(const std::string &filename, const boost::shared_ptr<pcl::PointCloud<PointT>> &cloud_ptr)
	{
		pcl::PCDWriter writer;
		writer.writeASCII(filename + ".pcd", *cloud_ptr);
	}

	/**
	 * \brief cluster keypoints in 2D image according to the logsig criterion
	 * using KdTree
	 * \param points 2D coordinate of every point
	 * \param points_count number of keypoints
	 * \param membership membership vector
	 * \param r_max max radius for adaptive radius calculation
	 * \param r_min min radius for adaptive radius calculation
	 * \param A
	 * \param K
	 */
	static int cluster_points(ANNpointArray points, int points_count,
							  std::vector<int> &membership, float r_max = 600.0f,
							  float r_min = 200.0f, float A = 800.0f, float K = 0.02f);

	template <typename PointT>
	static boost::shared_ptr<pcl::PointCloud<PointT>>
	align_clouds(boost::shared_ptr<pcl::PointCloud<PointT>> &query_cld,
				 boost::shared_ptr<pcl::PointCloud<PointT>> &base_cld);

	template <typename PointT>
	static boost::shared_ptr<pcl::PointCloud<PointT>>
	align_clouds(boost::shared_ptr<pcl::PointCloud<PointT>> &query_cld,
				 boost::shared_ptr<pcl::PointCloud<PointT>> &base_cld,
				 double max_corr_dist,
				 double &fitness_score);

	template <typename PointT>
	static boost::shared_ptr<pcl::PointCloud<PointT>>
	align_clouds(boost::shared_ptr<pcl::PointCloud<PointT>> &query_cld,
				 boost::shared_ptr<pcl::PointCloud<PointT>> &base_cld,
				 double max_corr_dist,
				 const Eigen::Matrix4f &guess,
				 double &fitness_score);

	template <typename PointT>
	static boost::shared_ptr<pcl::PointCloud<PointT>>
	align_clouds(boost::shared_ptr<pcl::PointCloud<PointT>> &query_cld,
				 boost::shared_ptr<pcl::PointCloud<PointT>> &base_cld,
				 double max_corr_dist,
				 const Eigen::Matrix4f &guess,
				 Eigen::Matrix4f &fTrans,
				 double &fitness_score);

	template <typename PointT>
	static boost::shared_ptr<pcl::PointCloud<PointT>>
	pointCloudFrom2DMask(cv::Mat binaryImage, cv::Mat image_depth, cv::Mat K, boost::shared_ptr<pcl::PointCloud<PointT>> &cld);

	template <typename PointT>
	static boost::shared_ptr<pcl::PointCloud<PointT>>
	pointCloudFrom2D(cv::Mat image_depth, cv::Mat image_rgb, cv::Mat K, boost::shared_ptr<pcl::PointCloud<PointT>> &cld);

	static double logsig(double x)
	{
		return 1.0 / (1.0 + exp(-x));
	}
};

//**************************************************************************************
//*************** Templated Function Implementations
//**************************************************************************************

//pcl::IndicesConstPtr	//pcl::PointIndices::Ptr const boost::shared_ptr<const pcl::PointCloud<PointT> > & cld_in
template <typename PointT>
boost::shared_ptr<pcl::PointCloud<PointT>>
pcd_utils::voxel_grid_subsample(const boost::shared_ptr<pcl::PointCloud<PointT>> &cld_in, float cell_size)
{
	//ex_VoxelGrid sor;
	pcl::VoxelGrid<PointT> sor;
	sor.setInputCloud(cld_in);
	sor.setLeafSize(cell_size, cell_size, cell_size); //leaf size in meters

	boost::shared_ptr<pcl::PointCloud<PointT>> final_cld(new pcl::PointCloud<PointT>);
	sor.filter(*final_cld);
	return final_cld;

	//return sor.getRemovedIndices();
}

template <typename PointT>
boost::shared_ptr<pcl::PointCloud<PointT>> //boost::shared_ptr<pcl::PointCloud<PointT> > & cld_out
pcd_utils::rnd_subsample(const boost::shared_ptr<pcl::PointCloud<PointT>> &cld_in, int size)
{
	pcl::RandomSample<PointT> sor;
	sor.setInputCloud(cld_in);
	sor.setSample(size);

	boost::shared_ptr<pcl::PointCloud<PointT>> final_cld(new pcl::PointCloud<PointT>);
	sor.filter(*final_cld);
	return final_cld;
}

template <typename PointT>
boost::shared_ptr<pcl::PointCloud<PointT>>
pcd_utils::align_clouds(boost::shared_ptr<pcl::PointCloud<PointT>> &query_cld,
						boost::shared_ptr<pcl::PointCloud<PointT>> &base_cld)
{
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setMaxCorrespondenceDistance(0.1);
	icp.setTransformationEpsilon(1e-7); // transformation convergence epsilon
	//icp.setMaximumIterations(1000);
	//icp.setEuclideanFitnessEpsilon(1.0); // maximum allowed error between two consecutive steps
	icp.setInputCloud(query_cld);
	icp.setInputTarget(base_cld);

	boost::shared_ptr<pcl::PointCloud<PointT>> final_cld(new pcl::PointCloud<PointT>);

	icp.align(*final_cld); // final cloud contains the transformed query cloud
	//*final_cld += *base_cld;		// add the two clouds

	if (icp.hasConverged())
	{
		ROS_INFO("[pcd_utils] ICP has converged with score: %f", icp.getFitnessScore());
		return final_cld;
	}
	else
	{
		ROS_WARN("[pcd_utils] ICP did not converge!");
		return query_cld;
	}
}

template <typename PointT>
boost::shared_ptr<pcl::PointCloud<PointT>>
pcd_utils::align_clouds(boost::shared_ptr<pcl::PointCloud<PointT>> &query_cld,
						boost::shared_ptr<pcl::PointCloud<PointT>> &base_cld,
						double max_corr_dist,
						double &fitness_score)
{
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setMaxCorrespondenceDistance(max_corr_dist);
	icp.setTransformationEpsilon(1e-7); // transformation convergence epsilon
	//icp.setMaximumIterations(1000);
	//icp.setEuclideanFitnessEpsilon(1.0); // maximum allowed error between two consecutive steps
	icp.setInputCloud(query_cld);
	icp.setInputTarget(base_cld);

	boost::shared_ptr<pcl::PointCloud<PointT>> final_cld(new pcl::PointCloud<PointT>);

	icp.align(*final_cld); // final cloud contains the transformed query cloud
	//*final_cld += *base_cld;		// add the two clouds

	if (icp.hasConverged())
	{
		fitness_score = icp.getFitnessScore();
		ROS_INFO("[pcd_utils] ICP has converged with score: %f", fitness_score);
		return final_cld;
	}
	else
	{
		ROS_WARN("[pcd_utils] ICP did not converge!");
		return query_cld;
	}
}

template <typename PointT>
boost::shared_ptr<pcl::PointCloud<PointT>>
pcd_utils::align_clouds(boost::shared_ptr<pcl::PointCloud<PointT>> &query_cld,
						boost::shared_ptr<pcl::PointCloud<PointT>> &base_cld,
						double max_corr_dist,
						const Eigen::Matrix4f &guess,
						double &fitness_score)
{
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setMaxCorrespondenceDistance(max_corr_dist); // 0.1
	icp.setTransformationEpsilon(1e-7);				 // transformation convergence epsilon
	//icp.setMaximumIterations(1000);
	//icp.setEuclideanFitnessEpsilon(1.0); // maximum allowed error between two consecutive steps
	icp.setInputCloud(query_cld);
	icp.setInputTarget(base_cld);

	boost::shared_ptr<pcl::PointCloud<PointT>> final_cld(new pcl::PointCloud<PointT>);

	icp.align(*final_cld, guess); // final cloud contains the transformed query cloud
	//*final_cld += *base_cld;		// add the two clouds

	if (icp.hasConverged())
	{
		fitness_score = icp.getFitnessScore();
		ROS_INFO("[pcd_utils] ICP has converged with score: %f", fitness_score);
		return final_cld;
	}
	else
	{
		ROS_WARN("[pcd_utils] ICP did not converge!");
		return query_cld;
	}
}

template <typename PointT>
boost::shared_ptr<pcl::PointCloud<PointT>>
pcd_utils::align_clouds(boost::shared_ptr<pcl::PointCloud<PointT>> &query_cld,
						boost::shared_ptr<pcl::PointCloud<PointT>> &base_cld,
						double max_corr_dist,
						const Eigen::Matrix4f &guess,
						Eigen::Matrix4f &fTrans,
						double &fitness_score)
{
	pcl::IterativeClosestPoint<PointT, PointT> icp;
	icp.setMaxCorrespondenceDistance(max_corr_dist); // 0.1
	icp.setTransformationEpsilon(1e-7);				 // transformation convergence epsilon
	//icp.setMaximumIterations(1000);
	//icp.setEuclideanFitnessEpsilon(1.0); // maximum allowed error between two consecutive steps
	icp.setInputCloud(query_cld);
	icp.setInputTarget(base_cld);

	boost::shared_ptr<pcl::PointCloud<PointT>> final_cld(new pcl::PointCloud<PointT>);

	icp.align(*final_cld, guess); // final cloud contains the transformed query cloud
	//*final_cld += *base_cld;		// add the two clouds

	if (icp.hasConverged())
	{
		fitness_score = icp.getFitnessScore();
		ROS_INFO("[pcd_utils] ICP has converged with score: %f", fitness_score);
		fTrans = icp.getFinalTransformation();
		return final_cld;
	}
	else
	{
		ROS_WARN("[pcd_utils] ICP did not converge!");
		return query_cld;
	}
}

/**
 * \brief construct point cloud from 2d images
 * \param binaryImage mask of the rgb image, e.g. where u can find object
 * \param image_depth depth image 
 * \param K  camera matrix 
 */
template <typename PointT>
boost::shared_ptr<pcl::PointCloud<PointT>>
pcd_utils::pointCloudFrom2DMask(cv::Mat binaryImage, cv::Mat image_depth, cv::Mat K, boost::shared_ptr<pcl::PointCloud<PointT>> &cld)
{
	std::vector<cv::Point> locations;
	cv::findNonZero(binaryImage, locations);

	int w = locations.size();
	int h = 1;
	// std::cout << locations.size() << std::endl;
	cld->resize(w * h);

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

			PointT out_points;
			out_points.x = z_metric * ((locations.at(i).x - cx) * fx_inv);
			out_points.y = z_metric * ((locations.at(i).y - cy) * fy_inv);
			out_points.z = z_metric;
			cld->push_back(out_points);
		}
	}
	return cld;
}

template <typename PointT>
boost::shared_ptr<pcl::PointCloud<PointT>>
pcd_utils::pointCloudFrom2D(cv::Mat image_depth, cv::Mat image_rgb, cv::Mat K, boost::shared_ptr<pcl::PointCloud<PointT>> &cld)
{
	cv::Size s = image_depth.size();
	int rows = s.height;
	int cols = s.width;

	int w = rows * cols;
	int h = 1;

	cld->resize(w * h);

	float cx = K.at<float>(0, 2);
	float cy = K.at<float>(1, 2);
	float fx_inv = 1.0 / K.at<float>(0, 0);
	float fy_inv = 1.0 / K.at<float>(1, 1);

	for (size_t u = 0; u < rows; u++)
		for (size_t v = 0; v < cols; v++)
		{
			uint16_t z_raw = image_depth.at<uint16_t>(u, v);

			if (z_raw != 0.0)
			{
				float z_metric = z_raw * 0.001;

				PointT out_points;
				out_points.x = z_metric * ((u - cx) * fx_inv);
				out_points.y = z_metric * ((v - cy) * fy_inv);
				out_points.z = z_metric;
				if (out_points.rgb && !image_rgb.empty())
				{
					out_points.r = (int)image_rgb.at<cv::Vec3b>(u,v)[2];
					out_points.g = (int)image_rgb.at<cv::Vec3b>(u,v)[1];
					out_points.b = (int)image_rgb.at<cv::Vec3b>(u,v)[0];
				}

				cld->push_back(out_points);
			}
		}
	return cld;
}
