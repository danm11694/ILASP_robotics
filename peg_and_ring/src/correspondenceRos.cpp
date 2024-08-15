#include <string>
#include <ros/ros.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/PointCloud2.h>
#include <geometry_msgs/PoseArray.h>
#include <sensor_msgs/point_cloud_conversion.h>
#include <sensor_msgs/point_cloud2_iterator.h>
#include <std_msgs/Float32MultiArray.h>
#include <pcl_conversions/pcl_conversions.h>
#include <cv_bridge/cv_bridge.h>
#include <typeinfo>

#include <tf/transform_broadcaster.h>
#include <tf/transform_listener.h>

#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/io/pcd_io.h>
#include <pcl/correspondence.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/shot_omp.h>
#include <pcl/features/board.h>
#include <pcl/recognition/cg/hough_3d.h>
#include <pcl/recognition/cg/geometric_consistency.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/kdtree/impl/kdtree_flann.hpp>
#include <pcl/common/transforms.h>
#include <pcl/console/parse.h>
#include <pcl/filters/uniform_sampling.h>

#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/features/fpfh.h>
#include <pcl/registration/ia_ransac.h>
#include <pcl/filters/random_sample.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/ply_io.h>
#include <pcl/common/transforms.h>
#include <pcl/keypoints/iss_3d.h>
#include <pcl/registration/correspondence_estimation.h>
#include <pcl/registration/correspondence_rejection_one_to_one.h>
#include <pcl/registration/correspondence_rejection_sample_consensus.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/transformation_estimation_point_to_plane.h>
#include <pcl/registration/transformation_estimation_point_to_plane_lls.h>
#include <pcl/registration/transformation_estimation_svd.h>

#include <sensor_msgs/PointField.h>

class CorrespondenceROS
{
    typedef pcl::PointXYZRGB Point;

private:
    ros::NodeHandle nh_, private_nh_;
    ros::Publisher color_pcl_pub_, rt_poses, scenecloud_pub, scene_pub, model_pub, torus_pose;
    ros::Subscriber image_sub_, depthBuffer_sub_;

    sensor_msgs::Image currImg;
    std_msgs::Float32MultiArray depthBuff;

    geometry_msgs::PoseArray registration_M2S; // model 2 scene

    float voxel_size;

    Eigen::Matrix4f featTransformation, icpTransformation;

    std::string pcl_topic, depth_topic, image_topic, modelPCD, scenePCD, algorithm_;
    tf::TransformBroadcaster *broadcaster;
    tf::TransformListener *listener;

    int v_res, u_res;
    double focal_length;
    double near_clip, far_clip;
    cv::Mat image_rgb;

    bool show_keypoints_, show_correspondences_, use_hough_, load_scene_, use_cloud_resolution_, registered;
    double model_ss_, scene_ss_, rf_rad_, descr_rad_, cg_size_, cg_thresh_;
    int k_normal_search;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr mergedCloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene;

public:
    CorrespondenceROS(ros::NodeHandle nh) : nh_(nh), private_nh_("~"), registered(false)
    {
        private_nh_.param("camera_color_name", image_topic, std::string(""));
        private_nh_.param("camera_depth_name", depth_topic, std::string(""));

        private_nh_.param("model", modelPCD, std::string(""));
        private_nh_.param("scene", scenePCD, std::string(""));

        private_nh_.param("algorithm", algorithm_, std::string(""));

        private_nh_.param<bool>("load_scene", load_scene_, false);

        private_nh_.param<bool>("use_cloud_resolution", use_cloud_resolution_, true);
        private_nh_.param<bool>("show_keypoints", show_keypoints_, false);
        private_nh_.param<bool>("show_correspondences", show_correspondences_, false);
        private_nh_.param<bool>("use_hough", use_hough_, true);
        private_nh_.param<double>("model_ss", model_ss_, 0.01);
        private_nh_.param<double>("scene_ss", scene_ss_, 0.03);
        private_nh_.param<double>("rf_rad", rf_rad_, 0.015);
        private_nh_.param<double>("descr_rad", descr_rad_, 0.02);
        private_nh_.param<double>("cg_size", cg_size_, 0.01);
        private_nh_.param<double>("cg_thresh", cg_thresh_, 5.0);

        private_nh_.param<int>("k_normal_search", k_normal_search, 10);

        color_pcl_pub_ = nh_.advertise<sensor_msgs::PointCloud2>(nh_.resolveName("/camera/depth_registered/points"), 10); // why 10
        rt_poses = nh_.advertise<geometry_msgs::PoseArray>("/registrations", 1);

        torus_pose = nh_.advertise<geometry_msgs::PoseStamped>("/torus", 1);

        scenecloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("map_pointcloud", 1);
        scene_pub = nh_.advertise<sensor_msgs::PointCloud2>("scene", 1);
        model_pub = nh_.advertise<sensor_msgs::PointCloud2>("model", 1);

        image_sub_ = nh_.subscribe(image_topic, 1, &CorrespondenceROS::imageCb, this);
        depthBuffer_sub_ = nh_.subscribe(depth_topic, 1, &CorrespondenceROS::cloudCallback, this);

        mergedCloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
        scene = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    }

    ~CorrespondenceROS() {}

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

    double computeCloudResolution(const pcl::PointCloud<pcl::PointNormal>::Ptr &inputCloud)
    {
        double resolution = 0.0;
        int numberOfPoints = 0;
        int nres;

        std::vector<int> indices(2);
        std::vector<float> squaredDistances(2);

        pcl::search::KdTree<pcl::PointNormal> tree;
        tree.setInputCloud(inputCloud);

        for (size_t i = 0; i < inputCloud->size(); ++i)
        {
            if (!pcl_isfinite((*inputCloud)[i].x))
            {
                continue;
            }

            // Considering the second neighbor since the first is the point itself.
            nres = tree.nearestKSearch(i, 2, indices, squaredDistances);
            if (nres == 2)
            {
                resolution += sqrt(squaredDistances[1]);
                ++numberOfPoints;
            }
        }

        if (numberOfPoints != 0)
        {
            resolution /= numberOfPoints;
        }

        return resolution;
    }

    void featureExtraction(
        const pcl::PointCloud<pcl::PointNormal>::Ptr &inputCloud,
        const pcl::PointCloud<pcl::PointNormal>::Ptr &inputKeypoints,
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr outputFeatures)
    {
        pcl::FPFHEstimation<pcl::PointNormal, pcl::Normal, pcl::FPFHSignature33> fpfh_est;
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>());

        pcl::PointCloud<pcl::Normal>::Ptr inputCloudNormal(new pcl::PointCloud<pcl::Normal>);
        pcl::copyPointCloud(*inputCloud, *inputCloudNormal);

        fpfh_est.setInputCloud(inputKeypoints);
        fpfh_est.setInputNormals(inputCloudNormal);

        fpfh_est.setSearchSurface(inputCloud);
        fpfh_est.setSearchMethod(tree);
        fpfh_est.setKSearch(8);

        fpfh_est.compute(*outputFeatures);
    }

    void keypointDetection(
        const pcl::PointCloud<pcl::PointNormal>::Ptr &inputCloud,
        pcl::PointCloud<pcl::PointNormal>::Ptr outputCloud)
    {
        pcl::ISSKeypoint3D<pcl::PointNormal, pcl::PointNormal> keys_det;
        pcl::search::KdTree<pcl::PointNormal>::Ptr tree(new pcl::search::KdTree<pcl::PointNormal>());

        double cloud_resol = computeCloudResolution(inputCloud);

        pcl::PointCloud<pcl::Normal>::Ptr cldNormal(new pcl::PointCloud<pcl::Normal>);
        pcl::copyPointCloud(*inputCloud, *cldNormal);

        keys_det.setInputCloud(inputCloud);
        keys_det.setNormals(cldNormal);

        keys_det.setSearchMethod(tree);
        keys_det.setMinNeighbors(3);
        keys_det.setSalientRadius(6 * cloud_resol);
        keys_det.setNonMaxRadius(4 * cloud_resol);
        keys_det.setThreshold21(0.975);
        keys_det.setThreshold32(0.975);
        keys_det.setNumberOfThreads(4);

        keys_det.compute(*outputCloud);
    }

    void registration(
        const pcl::PointCloud<pcl::PointNormal>::Ptr &inputSourceCloud,
        const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &inputSourceFeatures,
        const pcl::PointCloud<pcl::PointNormal>::Ptr &inputTargetCloud,
        const pcl::PointCloud<pcl::FPFHSignature33>::Ptr &inputTargetFeatures)
    {
        // estimate correspondences
        pcl::registration::CorrespondenceEstimation<pcl::FPFHSignature33, pcl::FPFHSignature33> est;
        pcl::CorrespondencesPtr correspondences(new pcl::Correspondences());
        est.setInputSource(inputSourceFeatures);
        est.setInputTarget(inputTargetFeatures);
        est.determineCorrespondences(*correspondences);

        // Duplication rejection
        pcl::CorrespondencesPtr correspondences_result_rej_one_to_one(new pcl::Correspondences());
        pcl::registration::CorrespondenceRejectorOneToOne corr_rej_one_to_one;
        corr_rej_one_to_one.setInputCorrespondences(correspondences);
        corr_rej_one_to_one.getCorrespondences(*correspondences_result_rej_one_to_one);

        // Correspondance rejection RANSAC
        pcl::registration::CorrespondenceRejectorSampleConsensus<pcl::PointNormal> rejector_sac;
        pcl::CorrespondencesPtr correspondences_filtered(new pcl::Correspondences());
        rejector_sac.setInputSource(inputSourceCloud);
        rejector_sac.setInputTarget(inputTargetCloud);
        rejector_sac.setInlierThreshold(2.5);
        rejector_sac.setMaximumIterations(1000000);
        rejector_sac.setRefineModel(false);
        rejector_sac.setInputCorrespondences(correspondences_result_rej_one_to_one);

        rejector_sac.getCorrespondences(*correspondences_filtered);
        correspondences.swap(correspondences_filtered);

        pcl::registration::TransformationEstimationSVD<pcl::PointNormal, pcl::PointNormal> transformation_estimation;
        transformation_estimation.estimateRigidTransformation(*inputSourceCloud, *inputTargetCloud, *correspondences, featTransformation);
    }

    void featureBasedRegistation(
        const pcl::PointCloud<pcl::PointNormal>::Ptr &inputCloudSource,
        const pcl::PointCloud<pcl::PointNormal>::Ptr &inputCloudTarget)
    {
        // ISS keypoints
        pcl::PointCloud<pcl::PointNormal>::Ptr model_keypoints(new pcl::PointCloud<pcl::PointNormal>);
        pcl::PointCloud<pcl::PointNormal>::Ptr scene_keypoints(new pcl::PointCloud<pcl::PointNormal>);
        keypointDetection(inputCloudSource, model_keypoints);
        keypointDetection(inputCloudTarget, scene_keypoints);

        // Extract features
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr slam_features_(new pcl::PointCloud<pcl::FPFHSignature33>);
        pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_features_(new pcl::PointCloud<pcl::FPFHSignature33>);
        featureExtraction(inputCloudSource, model_keypoints, slam_features_);
        featureExtraction(inputCloudTarget, scene_keypoints, scene_features_);

        std::cout << "scene_features_->size() " << scene_features_->size() << std::endl;
        std::cout << "slam_features_->size() " << slam_features_->size() << std::endl;

        // Registration
        registration(model_keypoints, slam_features_, scene_keypoints, scene_features_);
    }

    void rigidRegistration(
        const pcl::PointCloud<pcl::PointNormal>::Ptr &inputCloudSource,
        const pcl::PointCloud<pcl::PointNormal>::Ptr &inputCloudTarget,
        const pcl::PointCloud<pcl::PointNormal>::Ptr &inputCloudAlign)
    {
        pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
        typedef pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal> TE_t;
        boost::shared_ptr<TE_t> teObj(new TE_t);
        icp.setTransformationEstimation(teObj);

        auto ransac_thr = voxel_size * 0.95;

        icp.setMaximumIterations(500);
        icp.setRANSACOutlierRejectionThreshold(ransac_thr);
        icp.setMaxCorrespondenceDistance(ransac_thr * 10.0);
        icp.setTransformationEpsilon(1e-12);
        icp.setEuclideanFitnessEpsilon(1e-12);

        // transform the point cloud with the feature based initial alignment
        pcl::PointCloud<pcl::PointNormal>::Ptr inputCloudSource_Reg(new pcl::PointCloud<pcl::PointNormal>());
        pcl::transformPointCloud(*inputCloudSource, *inputCloudSource_Reg, featTransformation);

        icp.setInputSource(inputCloudSource_Reg);
        icp.setInputTarget(inputCloudTarget);

        // useless pointcloud, but obvioulsy you can't pass NULL
        pcl::PointCloud<pcl::PointNormal>::Ptr cloud_slam_icp(new pcl::PointCloud<pcl::PointNormal>());
        icp.align(*cloud_slam_icp);

        if (icp.hasConverged())
        {
            ROS_INFO("[Registration] ICP has converged with score: %f", icp.getFitnessScore());
            icpTransformation = icp.getFinalTransformation();
            std::cout << " icpTransformation " << icpTransformation << std::endl;
        }
        else
        {
            ROS_WARN("[Registration] ICP did not converge!");
        }
    }

    void findTorus(pcl::PointCloud<pcl::PointNormal>::Ptr cloud_pn)
    {

        pcl::SACSegmentation<pcl::PointNormal> seg;

        seg.setOptimizeCoefficients(true);
        seg.setInputCloud(cloud_pn);
        seg.setRadiusLimits(0.05, 0.10);
        if (algorithm_ == "RANSAC")
        {
            seg.setMethodType(pcl::SAC_RANSAC);
        }
        else if (algorithm_ == "LMEDS")
        {
            seg.setMethodType(pcl::SAC_LMEDS);
        }
        else if (algorithm_ == "MSAC")
        {
            seg.setMethodType(pcl::SAC_MSAC);
        }
        else if (algorithm_ == "RRANSAC")
        {
            seg.setMethodType(pcl::SAC_RRANSAC);
        }
        else if (algorithm_ == "RMSAC")
        {
            seg.setMethodType(pcl::SAC_RMSAC);
        }
        else if (algorithm_ == "MLESAC")
        {
            seg.setMethodType(pcl::SAC_MLESAC);
        }
        else if (algorithm_ == "PROSAC")
        {
            seg.setMethodType(pcl::SAC_PROSAC);
        }

        seg.setDistanceThreshold(0.01);
        seg.setMaxIterations(100);
        seg.setModelType(pcl::SACMODEL_CIRCLE3D);

        pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
        pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
        seg.segment(*inliers, *coefficients);

        pcl::PointCloud<pcl::PointNormal>::Ptr inlierPoints(new pcl::PointCloud<pcl::PointNormal>);

        // std::cerr << "ring coefficients: " << *coefficients << std::endl;
        if (inliers->indices.size() > 10) // min_size_
        {

            pcl::copyPointCloud<pcl::PointNormal>(*cloud_pn, *inliers, *inlierPoints);

            sensor_msgs::PointCloud2 model_pcd2;
            pcl::toROSMsg(*inlierPoints, model_pcd2);
            model_pcd2.header.frame_id = "camera_color_optical_frame";
            model_pcd2.header.stamp = ros::Time::now();
            model_pub.publish(model_pcd2);

            // check direction. Torus should direct to origin of pointcloud
            // always.
            Eigen::Vector3f dir(coefficients->values[4],
                                coefficients->values[5],
                                coefficients->values[6]);
            if (dir.dot(Eigen::Vector3f::UnitZ()) < 0)
            {
                dir = -dir;
            }

            Eigen::Affine3f pose = Eigen::Affine3f::Identity();
            Eigen::Vector3f pos = Eigen::Vector3f(coefficients->values[0],
                                                  coefficients->values[1],
                                                  coefficients->values[2]);
            Eigen::Quaternionf rot;
            rot.setFromTwoVectors(Eigen::Vector3f::UnitZ(), dir);
            pose = pose * Eigen::Translation3f(pos) * Eigen::AngleAxisf(rot);

            std::cout << " Position  " << pos << std::endl;

            geometry_msgs::PoseStamped torus;
            torus.header.frame_id = "camera_color_optical_frame";
            torus.pose.position.x = pos(0);
            torus.pose.position.y = pos(1);
            torus.pose.position.z = pos(2);
            torus.pose.orientation.x = rot.x();
            torus.pose.orientation.y = rot.y();
            torus.pose.orientation.z = rot.z();
            torus.pose.orientation.w = rot.w();
            torus_pose.publish(torus);
        }
    }

    void update()
    {
        ros::Time curr_time = ros::Time::now();
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene(new pcl::PointCloud<pcl::PointXYZRGB>());              // scene
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr model(new pcl::PointCloud<pcl::PointXYZRGB>()); // model
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr model_keypoints(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr scene_keypoints(new pcl::PointCloud<pcl::PointXYZRGB>());
        pcl::PointCloud<pcl::Normal>::Ptr model_normals(new pcl::PointCloud<pcl::Normal>());       // model_normals
        pcl::PointCloud<pcl::Normal>::Ptr scene_normals(new pcl::PointCloud<pcl::Normal>());       // scene_normals
        pcl::PointCloud<pcl::SHOT352>::Ptr model_descriptors(new pcl::PointCloud<pcl::SHOT352>()); // model_descriptor
        pcl::PointCloud<pcl::SHOT352>::Ptr scene_descriptors(new pcl::PointCloud<pcl::SHOT352>()); // scene_descriptor

        if (pcl::io::loadPCDFile(modelPCD, *model) < 0)
        {
            std::cout << "Error loading model cloud:  " << modelPCD << std::endl;
            return;
        }

        if (load_scene_)
            if (pcl::io::loadPCDFile(scenePCD, *scene) < 0)
            {
                std::cout << "Error loading scene cloud:  " << scenePCD << std::endl;
                return;
            }

        pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> norm_est;
        norm_est.setKSearch(k_normal_search);
        norm_est.setInputCloud(model);
        norm_est.compute(*model_normals);
        norm_est.setInputCloud(scene);
        norm_est.compute(*scene_normals);

        pcl::PointCloud<int> sampled_indices;

        pcl::UniformSampling<pcl::PointXYZRGB> uniform_sampling;
        uniform_sampling.setInputCloud(model);
        uniform_sampling.setRadiusSearch(model_ss_);
        uniform_sampling.filter(*model_keypoints);
        std::cout << "Model total points: " << model->size() << "; Selected Keypoints: " << model_keypoints->size() << std::endl;

        uniform_sampling.setInputCloud(scene);
        uniform_sampling.setRadiusSearch(scene_ss_);
        uniform_sampling.filter(*scene_keypoints);
        std::cout << "Scene total points: " << scene->size() << "; Selected Keypoints: " << scene_keypoints->size() << std::endl;

        // assign a 3D descriptor to each keypoint
        pcl::SHOTEstimationOMP<pcl::PointXYZRGB, pcl::Normal, pcl::SHOT352> descr_est;
        descr_est.setRadiusSearch(descr_rad_);
        //model
        descr_est.setInputCloud(model_keypoints);
        descr_est.setInputNormals(model_normals);
        descr_est.setSearchSurface(model);
        descr_est.compute(*model_descriptors);
        //scene
        descr_est.setInputCloud(scene_keypoints);
        descr_est.setInputNormals(scene_normals);
        descr_est.setSearchSurface(scene);
        descr_est.compute(*scene_descriptors);

        // correspondence point-to-point between descriptor
        pcl::CorrespondencesPtr model_scene_corrs(new pcl::Correspondences());

        pcl::KdTreeFLANN<pcl::SHOT352> match_search;
        match_search.setInputCloud(model_descriptors);

        std::cout << " scene_descriptors:  " << scene_descriptors->size() << std::endl;
        // For each keypoints in the scene, find the closest keypoint in the model, and add in the correspondence vector
        for (size_t i = 0; i < scene_descriptors->size(); ++i)
        {
            std::vector<int> neigh_indices(1);
            std::vector<float> neigh_sqr_dists(1);
            if (!pcl_isfinite(scene_descriptors->at(i).descriptor[0])) //skipping NaNs
            {
                continue;
            }
            int found_neighs = match_search.nearestKSearch(scene_descriptors->at(i), 1, neigh_indices, neigh_sqr_dists);

            // parametro neigh_sqr_dists[0] modificato rispetto a quello di partenza (0,25f)
            // add match only if the squared descriptor distance is less than 0.25 (SHOT descriptor distances are between 0 and 1 by design)
            if (found_neighs == 1 && neigh_sqr_dists[0] < 0.95f) // 0.02 // 0.25
            {
                pcl::Correspondence corr(neigh_indices[0], static_cast<int>(i), neigh_sqr_dists[0]);
                model_scene_corrs->push_back(corr);
            }
        } //fine for
        std::cout << "Correspondences found: " << model_scene_corrs->size() << std::endl;

        //----------------------------- SEGMENTAZIONE CON CLUSTERIZZAZIONE
        std::vector<Eigen::Matrix4f, Eigen::aligned_allocator<Eigen::Matrix4f>> rototranslations;
        std::vector<pcl::Correspondences> clustered_corrs;

        //  Using Hough3D (utilizza i keypoints, le normali e la PointCloud)
        if (use_hough_)
        {
            //abbinamento di modello e scena considerando valori tipici di Hugh (keypoints, le normali e la PointCloud)
            pcl::PointCloud<pcl::ReferenceFrame>::Ptr model_rf(new pcl::PointCloud<pcl::ReferenceFrame>());
            pcl::PointCloud<pcl::ReferenceFrame>::Ptr scene_rf(new pcl::PointCloud<pcl::ReferenceFrame>());

            pcl::BOARDLocalReferenceFrameEstimation<pcl::PointXYZRGB, pcl::Normal, pcl::ReferenceFrame> rf_est;
            rf_est.setFindHoles(true);
            rf_est.setRadiusSearch(rf_rad_);
            //modello
            rf_est.setInputCloud(model_keypoints);
            rf_est.setInputNormals(model_normals);
            rf_est.setSearchSurface(model);
            rf_est.compute(*model_rf);
            //scena
            rf_est.setInputCloud(scene_keypoints);
            rf_est.setInputNormals(scene_normals);
            rf_est.setSearchSurface(scene);
            rf_est.compute(*scene_rf);

            //  Clustering
            pcl::Hough3DGrouping<pcl::PointXYZRGB, pcl::PointXYZRGB, pcl::ReferenceFrame, pcl::ReferenceFrame> clusterer;
            clusterer.setHoughBinSize(cg_size_);
            clusterer.setHoughThreshold(cg_thresh_);
            clusterer.setUseInterpolation(true);
            clusterer.setUseDistanceWeight(false);

            clusterer.setInputCloud(model_keypoints);
            clusterer.setInputRf(model_rf);
            clusterer.setSceneCloud(scene_keypoints);
            clusterer.setSceneRf(scene_rf);
            clusterer.setModelSceneCorrespondences(model_scene_corrs);

            //  creazione matrice di rotazione traslazione dell'modello messo in corrispondenza
            clusterer.recognize(rototranslations, clustered_corrs);
        }
        else
        {

            pcl::PointCloud<pcl::PointNormal>::Ptr cloud_scene_normals = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
            pcl::PointCloud<pcl::PointNormal>::Ptr cloud_model_normals = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
            pcl::PointCloud<pcl::PointNormal>::Ptr cloudAligned = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);

            pcl::PointCloud<pcl::PointXYZ>::Ptr model_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::copyPointCloud(*model, *model_cloud_xyz);
            pcl::copyPointCloud(*scene, *scene_cloud_xyz);

            pcl::concatenateFields(*model_cloud_xyz, *model_normals, *cloud_model_normals);
            pcl::concatenateFields(*scene_cloud_xyz, *scene_normals, *cloud_scene_normals);

            findTorus(cloud_scene_normals);

            featureBasedRegistation(cloud_model_normals, cloud_scene_normals);
            rigidRegistration(cloud_model_normals, cloud_scene_normals, cloudAligned);
            registered = true;

            if (registered)
            {
                sensor_msgs::PointCloud2 cloud2;
                pcl::toROSMsg(*cloudAligned, cloud2);
                cloud2.header.frame_id = "world";
                cloud2.header.stamp = ros::Time::now();
                scenecloud_pub.publish(cloud2);

                // sensor_msgs::PointCloud2 model_pcd2;
                // pcl::toROSMsg(*model, model_pcd2);
                // model_pcd2.header.frame_id = "world";
                // model_pcd2.header.stamp = ros::Time::now();
                // model_pub.publish(model_pcd2);

                sensor_msgs::PointCloud2 scene_pcd2;
                pcl::toROSMsg(*scene, scene_pcd2);
                scene_pcd2.header.frame_id = "world";
                scene_pcd2.header.stamp = ros::Time::now();
                scene_pub.publish(scene_pcd2);
            }

            // pcl::GeometricConsistencyGrouping<pcl::PointXYZRGB, pcl::PointXYZRGB> gc_clusterer;
            // gc_clusterer.setGCSize(cg_size_);
            // gc_clusterer.setGCThreshold(cg_thresh_);

            // gc_clusterer.setInputCloud(model_keypoints);
            // gc_clusterer.setSceneCloud(scene_keypoints);
            // gc_clusterer.setModelSceneCorrespondences(model_scene_corrs);

            // //gc_clusterer.cluster (clustered_corrs);
            // gc_clusterer.recognize(rototranslations, clustered_corrs);
        }

        std::cout << "Model instances found: " << rototranslations.size() << std::endl;
        // for (std::size_t i = 0; i < rototranslations.size(); ++i)
        // {
        //     std::cout << "\n    Instance " << i + 1 << ":" << std::endl;
        //     std::cout << "        Correspondences belonging to this instance: " << clustered_corrs[i].size() << std::endl;

        //     // Print the rotation matrix and translation vector
        //     Eigen::Matrix3f rotation = rototranslations[i].block<3, 3>(0, 0);
        //     Eigen::Vector3f translation = rototranslations[i].block<3, 1>(0, 3);

        //     printf("\n");
        //     printf("            | %6.3f %6.3f %6.3f | \n", rotation(0, 0), rotation(0, 1), rotation(0, 2));
        //     printf("        R = | %6.3f %6.3f %6.3f | \n", rotation(1, 0), rotation(1, 1), rotation(1, 2));
        //     printf("            | %6.3f %6.3f %6.3f | \n", rotation(2, 0), rotation(2, 1), rotation(2, 2));
        //     printf("\n");
        //     printf("        t = < %0.3f, %0.3f, %0.3f >\n", translation(0), translation(1), translation(2));
        // }

        registration_M2S.header.frame_id = scene->header.frame_id;
        registration_M2S.header.stamp = ros::Time::now();

        for (size_t i = 0; i < rototranslations.size(); i++)
        {
            Eigen::Vector3f translation = rototranslations[i].block<3, 1>(0, 3);
            Eigen::Matrix3f rotation = rototranslations[i].block<3, 3>(0, 0);

            Eigen::Quaternionf q(rotation);
            geometry_msgs::Pose rt;
            rt.position.x = translation(0);
            rt.position.y = translation(1);
            rt.position.z = translation(2);
            rt.orientation.x = q.x();
            rt.orientation.y = q.y();
            rt.orientation.z = q.z();
            rt.orientation.w = q.w();
            // std::cout << " rt " << rt << std::endl;
            registration_M2S.poses.push_back(rt);
        }
        rt_poses.publish(registration_M2S);

        if (rototranslations.size() >= 1)
        {
            //------------------------------VISUALIZZAZIONE DELLA CORRISPONDENZA (CLASSIFICAZIONE VERA E PROPRIA)
            pcl::visualization::PCLVisualizer viewer("Correspondence Grouping");
            viewer.addPointCloud(scene, "scene_cloud");

            pcl::PointCloud<pcl::PointXYZRGB>::Ptr off_scene_model(new pcl::PointCloud<pcl::PointXYZRGB>());
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr off_scene_model_keypoints(new pcl::PointCloud<pcl::PointXYZRGB>());

            if (show_correspondences_ || show_keypoints_)
            {
                //  Il modello viene traslato per fare in modo che non venga visualizzato all'interno della scena
                pcl::transformPointCloud(*model, *off_scene_model, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));
                pcl::transformPointCloud(*model_keypoints, *off_scene_model_keypoints, Eigen::Vector3f(-1, 0, 0), Eigen::Quaternionf(1, 0, 0, 0));

                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> off_scene_model_color_handler(off_scene_model, 255, 255, 128);
                viewer.addPointCloud(off_scene_model, off_scene_model_color_handler, "off_scene_model");
            }

            if (show_keypoints_)
            { // Visualizzazione dei punti chiave sulla scena
                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> scene_keypoints_color_handler(scene_keypoints, 0, 0, 255);
                viewer.addPointCloud(scene_keypoints, scene_keypoints_color_handler, "scene_keypoints");
                viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "scene_keypoints");

                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> off_scene_model_keypoints_color_handler(off_scene_model_keypoints, 0, 0, 255);
                viewer.addPointCloud(off_scene_model_keypoints, off_scene_model_keypoints_color_handler, "off_scene_model_keypoints");
                viewer.setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 5, "off_scene_model_keypoints");
            }

            //visualizzazione modello
            for (size_t i = 0; i < rototranslations.size(); ++i)
            {
                pcl::PointCloud<pcl::PointXYZRGB>::Ptr rotated_model(new pcl::PointCloud<pcl::PointXYZRGB>());
                pcl::transformPointCloud(*model, *rotated_model, rototranslations[i]);

                std::stringstream ss_cloud;
                ss_cloud << "instance" << i;

                pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZRGB> rotated_model_color_handler(rotated_model, 255, 0, 0);
                viewer.addPointCloud(rotated_model, rotated_model_color_handler, ss_cloud.str());

                if (show_correspondences_)
                { // Visualizzazione delle corrispondenze tra modello e scena
                    for (size_t j = 0; j < clustered_corrs[i].size(); ++j)
                    {
                        std::stringstream ss_line;
                        ss_line << "correspondence_line" << i << "_" << j;
                        pcl::PointXYZRGB &model_point = off_scene_model_keypoints->at(clustered_corrs[i][j].index_query);
                        pcl::PointXYZRGB &scene_point = scene_keypoints->at(clustered_corrs[i][j].index_match);

                        //  Disegna una line per ogni coppia di punti corrispondente tra il modello e la scena
                        viewer.addLine<pcl::PointXYZRGB, pcl::PointXYZRGB>(model_point, scene_point, 0, 255, 0, ss_line.str());
                    }
                }
            }

            while (!viewer.wasStopped())
            {
                viewer.spinOnce();
            }
        }
    }

    void
    cloudCallback(const sensor_msgs::PointCloud2ConstPtr &input)
    {
        pcl::PCLPointCloud2 pcl_pc2;
        pcl_conversions::toPCL(*input, pcl_pc2);

        pcl_pc2.fields[3].datatype = sensor_msgs::PointField::FLOAT32;

        pcl::fromPCLPointCloud2(pcl_pc2, *scene);

        // pcl::PCDWriter writer;
        // writer.write("/home/andrea/ars_workspace/src/peg_and_ring/models/ring.pcd",*scene);
    }
};

int main(int argc, char **argv)
{
    ros::init(argc, argv, "correspondenceRosNode");
    ros::NodeHandle nh;

    CorrespondenceROS CorrespondenceROSNode(nh);

    // CorrespondenceROSNode.update();
    // ros::spin();

    while (ros::ok())
    {
        CorrespondenceROSNode.update();
        ros::spinOnce();
    }

    return 0;
}
