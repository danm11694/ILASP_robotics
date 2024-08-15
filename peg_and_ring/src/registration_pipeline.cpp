#include <registration_pipeline.h>

#define MAP_REGISTERED "/registrator/map_registered"

namespace registration_pipeline
{

tf::TransformBroadcaster *broadcaster;
tf::TransformListener *listener;

bool registered = false, slamReady = false;
Eigen::Matrix4f featTransformation, icpTransformation;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_slam;
pcl::PointCloud<pcl::PointNormal>::Ptr cloud_scene, subsample_cloud_scene, cloud_slam_normals;
pcl::PointCloud<pcl::Normal>::Ptr cloud_normals;

float voxel_size;
std::string map_path, reference_frame, camera_initial_frame;

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
    pcl::PointCloud<pcl::PointNormal>::Ptr slam_keypoints(new pcl::PointCloud<pcl::PointNormal>);
    pcl::PointCloud<pcl::PointNormal>::Ptr scene_keypoints(new pcl::PointCloud<pcl::PointNormal>);
    keypointDetection(inputCloudSource, slam_keypoints);
    keypointDetection(inputCloudTarget, scene_keypoints);

    // Extract features
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr slam_features_(new pcl::PointCloud<pcl::FPFHSignature33>);
    pcl::PointCloud<pcl::FPFHSignature33>::Ptr scene_features_(new pcl::PointCloud<pcl::FPFHSignature33>);
    featureExtraction(inputCloudSource, slam_keypoints, slam_features_);
    featureExtraction(inputCloudTarget, scene_keypoints, scene_features_);

    std::cout << "scene_features_->size() " << scene_features_->size() << std::endl;
    std::cout << "slam_features_->size() " << slam_features_->size() << std::endl;

    // Registration
    registration(slam_keypoints, slam_features_, scene_keypoints, scene_features_);
}

void rigidRegitration(
    const pcl::PointCloud<pcl::PointNormal>::Ptr &inputCloudSource,
    const pcl::PointCloud<pcl::PointNormal>::Ptr &inputCloudTarget)
{
    pcl::IterativeClosestPointWithNormals<pcl::PointNormal, pcl::PointNormal> icp;
    typedef pcl::registration::TransformationEstimationPointToPlane<pcl::PointNormal, pcl::PointNormal> TE_t;
    boost::shared_ptr<TE_t> teObj(new TE_t);
    icp.setTransformationEstimation(teObj);

    auto ransac_thr = voxel_size * 0.95;

    icp.setMaximumIterations(100);
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
        ROS_INFO("[SlamRegistration] ICP has converged with score: %f", icp.getFitnessScore());
        icpTransformation = icp.getFinalTransformation();
    }
    else
    {
        ROS_WARN("[SlamRegistration] ICP did not converge!");
    }
}

void updateNormals(const tf::Vector3 &cP)
{
    // Create the normal estimation class, and pass the input dataset to it
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> ne;
    ne.setInputCloud(cloud_slam);

    // Create an empty kdtree representation, and pass it to the normal estimation object.
    // Its content will be filled inside the object, based on the given input dataset (as no other search surface is given).
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>());
    ne.setSearchMethod(tree);

    // Use all neighbors in a sphere
    ne.setKSearch(16);

    // Set the viewpoint
    ne.setViewPoint(cP.x(), cP.y(), cP.z());

    // Compute the features
    ne.compute(*cloud_normals);
}

template <class T>
typename pcl::PointCloud<T>::Ptr voxelize(typename pcl::PointCloud<T>::Ptr cld_in)
{
    typename pcl::VoxelGrid<T> voxelizer;
    voxelizer.setInputCloud(cld_in);

    // leaf size in meters
    voxelizer.setLeafSize(voxel_size, voxel_size, voxel_size);

    typename pcl::PointCloud<T>::Ptr final_cld(new typename pcl::PointCloud<T>);
    voxelizer.filter(*final_cld);
    return final_cld;
}

void init()
{
    cloud_slam = pcl::PointCloud<pcl::PointXYZ>::Ptr(new pcl::PointCloud<pcl::PointXYZ>);
    cloud_scene = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
    subsample_cloud_scene = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
    cloud_slam_normals = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
    cloud_normals = pcl::PointCloud<pcl::Normal>::Ptr(new pcl::PointCloud<pcl::Normal>);

    pcl::PLYReader Reader;
    Reader.read(map_path, *cloud_scene);

    subsample_cloud_scene = voxelize<pcl::PointNormal>(cloud_scene);

    if (subsample_cloud_scene->size() > 0)
    {
        ROS_INFO("Model loaded");
        ROS_INFO_STREAM("Subsampling (voxel size: " << voxel_size << "): " << subsample_cloud_scene->size() << " points.");
    }
}

void save()
{
    // save last point cloud
    pcl::PLYWriter writer;
    writer.write<pcl::PointNormal>("/tmp/cloud_normals.ply", *cloud_slam_normals, false, false);
}

void doReg()
{
    // Do ICP and publish the registration
    if (slamReady)
    {
        // Lookup the initial camera position
        tf::StampedTransform cameraPose_transform;
        try
        {
            listener->waitForTransform(reference_frame, camera_initial_frame, ros::Time(0), ros::Duration(10.0));
            listener->lookupTransform(reference_frame, camera_initial_frame, ros::Time(0), cameraPose_transform);

            // align the sparse point cloud to the reference map
            // and publish the transform in ROS compute normals for vertices
            tf::Vector3 cP = cameraPose_transform.getOrigin();
            updateNormals(cP);
            pcl::concatenateFields(*cloud_slam, *cloud_normals, *cloud_slam_normals);

            // try rigid transformation with guess the previous transformation
            featureBasedRegistation(cloud_slam_normals, subsample_cloud_scene);
            rigidRegitration(cloud_slam_normals, subsample_cloud_scene);

            registered = true;
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("[Unable to register] %s", ex.what());
            return;
        }
    }
}

void registrationLoop(const ros::Publisher &mp)
{
    // Do ICP and publish the registration
    if (slamReady)
    {
#ifdef SIZE_TRIGGER
        int partialSize = (subsample_cloud_scene->size() * 0.8);
        std::cout << cloud_slam->size() << " " << partialSize << std::endl;
        if (cloud_slam->size() > partialSize)
        {
            doReg();
        }
#endif
        // if at least one registration is done still publish the last transformation
        if (registered)
        {
            /////////////////////////////////////////////
            tf::Vector3 featOrigin;
            tf::Matrix3x3 featTf3d;

            featOrigin.setValue(featTransformation(0, 3), featTransformation(1, 3), featTransformation(2, 3));

            featTf3d.setValue(static_cast<double>(featTransformation(0, 0)), static_cast<double>(featTransformation(0, 1)), static_cast<double>(featTransformation(0, 2)),
                              static_cast<double>(featTransformation(1, 0)), static_cast<double>(featTransformation(1, 1)), static_cast<double>(featTransformation(1, 2)),
                              static_cast<double>(featTransformation(2, 0)), static_cast<double>(featTransformation(2, 1)), static_cast<double>(featTransformation(2, 2)));

            tf::Quaternion featTfqt;
            featTf3d.getRotation(featTfqt);

            tf::Transform featTransform;
            featTransform.setOrigin(featOrigin);
            featTransform.setRotation(featTfqt);

            /////////////////////////////////////////////
            tf::Vector3 icpOrigin;
            tf::Matrix3x3 icpTf3d;

            icpOrigin.setValue(
                static_cast<double>(icpTransformation(0, 3)),
                static_cast<double>(icpTransformation(1, 3)),
                static_cast<double>(icpTransformation(2, 3)));

            icpTf3d.setValue(static_cast<double>(icpTransformation(0, 0)), static_cast<double>(icpTransformation(0, 1)), static_cast<double>(icpTransformation(0, 2)),
                             static_cast<double>(icpTransformation(1, 0)), static_cast<double>(icpTransformation(1, 1)), static_cast<double>(icpTransformation(1, 2)),
                             static_cast<double>(icpTransformation(2, 0)), static_cast<double>(icpTransformation(2, 1)), static_cast<double>(icpTransformation(2, 2)));

            tf::Quaternion icpTfqt;
            icpTf3d.getRotation(icpTfqt);

            tf::Transform icpTransform;
            icpTransform.setOrigin(icpOrigin);
            icpTransform.setRotation(icpTfqt);

            broadcaster->sendTransform(tf::StampedTransform(featTransform.inverse() * icpTransform.inverse(), ros::Time::now(), camera_initial_frame, MAP_REGISTERED));

            // publish the reference point cloud w.r.t the new transform
            sensor_msgs::PointCloud2 cloud2;
            pcl::toROSMsg(*subsample_cloud_scene, cloud2);
            cloud2.header.frame_id = MAP_REGISTERED;
            cloud2.header.stamp = ros::Time::now();
            mp.publish(cloud2);
        }
    }
}

void SlamPointCloudUpdate(const sensor_msgs::PointCloud2ConstPtr &msg)
{
    pcl::PCLPointCloud2 pcl_pc2;
    pcl_conversions::toPCL(*msg, pcl_pc2);
    pcl::fromPCLPointCloud2(pcl_pc2, *cloud_slam);

    // point cloud volexation to keep the same density of the map
    cloud_slam = voxelize<pcl::PointXYZ>(cloud_slam);

    // flag the output of the slam as available
    slamReady = true;
}

bool RegisterInvoked(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
    try
    {
        registered = false;
        doReg();

        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
}

} // namespace registration_pipeline

// ------------------------------------------------------------------

// int main(int argc, char **argv)
// {
//     ros::init(argc, argv, "Transform estimator");
//     ros::NodeHandle nh;
//     ros::NodeHandle pnh("~");
//     ros::Rate loop_rate(200);

//     broadcaster = new tf::TransformBroadcaster();
//     listener = new tf::TransformListener();

//     ros::Subscriber slamcloud_sub = pnh.subscribe("sparse_pointcloud", 1, &SlamPointCloudUpdate);
//     ros::Publisher scenecloud_pub = pnh.advertise<sensor_msgs::PointCloud2>("map_pointcloud", 1);
//     ros::ServiceServer service = pnh.advertiseService("register", &RegisterInvoked);

//     pnh.param("voxel_size", voxel_size, 0.008f);
//     pnh.param("reference_frame", reference_frame, std::string(""));
//     pnh.param("map_path", map_path, std::string(""));
//     pnh.param("camera_initial_frame ", camera_initial_frame, std::string("")); // camera_initial_frame

//     init();

//     while (ros::ok())
//     {
//         registrationLoop(scenecloud_pub);

//         ros::spinOnce();
//         loop_rate.sleep();
//     }

//     save();

//     return 0;
// }
