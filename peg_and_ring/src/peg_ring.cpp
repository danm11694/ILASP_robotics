#include <peg_ring.hpp>
peg_ring::peg_ring(ros::NodeHandle &nh) : nh_(nh), private_nh_("~"),
                                          goal_completion_time(ros::Time::now()),
                                          xyz_cld_ptr(new pcl::PointCloud<pcl::PointXYZRGB>),
                                          xyz_cld_ptr_original(new pcl::PointCloud<pcl::PointXYZRGB>),
                                          cloud_updated(false),
                                          plane_detected(false),
                                          carry_state(false),
                                          end_carry(false),
                                          grasping_point(false),
                                          orig_cld_voxel_size(0.0f),
                                          processing_color(""),
                                          prev_processing_color(""),
                                          num_ee(0),
                                          peg_height(0.005),
                                          it_(nh)
{
}

int peg_ring::init()
{
    ROS_INFO("[PEG_RING]  INIT ");

    private_nh_.param("simulation", simulation, false);
    private_nh_.param("fixed_frame", fixed_frame, std::string("world")); // world
    private_nh_.param("optical_frame", optical_frame, std::string("camera_color_optical_frame"));
    private_nh_.param("robot_frame", PSM_frame, std::string("/PSM1_base"));
    private_nh_.param("cld_topic_name", cld_topic_name, std::string("/camera/depth/color/points"));
    private_nh_.param("camera_color_name", camera_color_name, std::string("/camera/color/image_raw"));
    private_nh_.param("camera_info_name", camera_info_name, std::string("/camera/color/camera_info"));
    private_nh_.param("processing_color", color_topic, std::string("color"));
    private_nh_.param("Current_action", Current_action, std::string("/Current_action"));
    private_nh_.param("orig_cld_voxel_size", orig_cld_voxel_size, 0.008f);
    private_nh_.param<double>("object_cluster_distance", object_cluster_distance_, 0.008); //0.03
    private_nh_.param<int>("max_object_cluster_size", max_object_cluster_size_, 500000);
    private_nh_.param<int>("min_object_cluster_size", min_object_cluster_size_, 300);
    private_nh_.param<double>("threshold_tracking", threshold_tracking, 0.005);
    private_nh_.param<double>("ee_treshold", ee_treshold, 0.06); // centimeters
    private_nh_.param("tracking_value", tracking_value, std::string("/tracking_value"));

    private_nh_.param("torus_algorithm", torus_algorithm, std::string(""));
    private_nh_.param<int>("torus_points", torus_points, 40);

    private_nh_.getParam("tool_subscribers", tool_subs_topic); // num_ee
    for (std::string &topic : tool_subs_topic)
    {
        ros::Subscriber sub = nh_.subscribe<geometry_msgs::PoseStamped>(topic.c_str(), 1, std::bind(&peg_ring::toolPoseUpdate, this, std::placeholders::_1, topic));
        ee_pose_subs.push_back(sub);
        num_ee++;
    }

    cloud_sub = nh_.subscribe(cld_topic_name, 1, &peg_ring::cloudCallback, this);
    color_sub = nh_.subscribe(color_topic, 1, &peg_ring::color2processCallback, this);
    camera_sub = nh_.subscribe(camera_color_name, 1, &peg_ring::imageCallback, this);
    camera_info_sub = nh_.subscribe(camera_info_name, 1, &peg_ring::cameraInfoCallback, this);
    peg_pose_sub = nh_.subscribe("peg/pose", 1, &peg_ring::peg_poseCallback, this);
    success_topic_sub = nh_.subscribe(Current_action, 1, &peg_ring::carryCallback, this);

    pub_peg_pose = nh_.advertise<geometry_msgs::PoseStamped>("peg/automatic/pose", 1);
    image_pub_ = it_.advertise("/track_ring/output_video", 1);
    cloud_pub = nh_.advertise<sensor_msgs::PointCloud2>("/output_cloud", 1, this);
    plane_pub = nh_.advertise<sensor_msgs::PointCloud2>("/plane_cloud", 1, this);
    cylinder_pub = nh_.advertise<sensor_msgs::PointCloud2>("/cylinder_cloud", 1, this);
    ring_pub = nh_.advertise<sensor_msgs::PointCloud2>("/ring_cloud", 1, this);
    converted_pub = nh_.advertise<sensor_msgs::PointCloud2>("/converted_cloud", 1, this);
    tracking_pub = nh_.advertise<std_msgs::Bool>(tracking_value, 1, this);
    pub_ring_pose = nh_.advertise<geometry_msgs::PoseStamped>("/grasping_point", 1);
    pub_ring_track = nh_.advertise<geometry_msgs::PoseStamped>("/tracking/pose", 1);
    pub_ring_points = nh_.advertise<dvrk_task_msgs::CloudArray>("/ring_points", 1);
    pub_ring_poses = nh_.advertise<geometry_msgs::PoseArray>("/grasping_points", 1);
    pub_peg_poses = nh_.advertise<geometry_msgs::PoseArray>("/all_pegs", 1);
    torus_pose = nh_.advertise<geometry_msgs::PoseStamped>("/torus", 1);
    pub_ring_centers = nh_.advertise<geometry_msgs::PoseArray>("/all_rings", 1);
    plane_pose_pub = nh_.advertise<geometry_msgs::PoseStamped>("/setup_pose", 1);

    segmented_cloud = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_plane = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_scene = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_cylinder = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    cloud_ring = pcl::PointCloud<pcl::PointXYZRGB>::Ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    transform_1 = Eigen::Matrix4f::Identity();

    ee_pose.resize(num_ee);
}

int peg_ring::update()
{
    goal_completion_time = ros::Time::now();
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_cld_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    if (xyz_cld_ptr->size() > 0)
    {
        map_cld_ptr = pcd_utils::voxel_grid_subsample(xyz_cld_ptr, orig_cld_voxel_size);
        // map_cld_ptr = xyz_cld_ptr;
        if (!plane_detected)
        {
            // ROS_INFO("[PEG_RING] CREATED PLANE AND FRAME");
            extractPlane(map_cld_ptr); // output cloud_plane, cloud_cylinder and cloud_scene

            // std::cout << " map_cld_ptr " << map_cld_ptr->header.frame_id << std::endl;
            // std::cout << " cloud_scene " << cloud_scene->header.frame_id << std::endl;
            pegsInitialize(cloud_cylinder);

            // cloud_plane->header.frame_id = fixed_frame;
            // cloud_plane->header.stamp = xyz_cld_ptr->header.stamp;
        }

        all_pegs.header.frame_id = fixed_frame;
        pub_peg_poses.publish(all_pegs);

        // brute force solution, the peg base should be extracted from the scene when identifying the plane, instead
        plane_pose.pose.position.x = (all_pegs.poses[0].position.x + all_pegs.poses[1].position.x + all_pegs.poses[2].position.x + all_pegs.poses[3].position.x) / 4;
        plane_pose.pose.position.y = (all_pegs.poses[0].position.y + all_pegs.poses[1].position.y + all_pegs.poses[2].position.y + all_pegs.poses[3].position.y) / 4;
        plane_pose_pub.publish(plane_pose); //plane center

        colorSegmentation(map_cld_ptr);                                     //ring poses
        segmented_cloud = colorSegmentation(map_cld_ptr, processing_color); // output segmented_cloud

        pcl::toROSMsg(*segmented_cloud, output_cloud);
        cloud_pub.publish(output_cloud);

        // pcl::PCDWriter writer;
        // if(segmented_cloud->points.size() > 0 )
        // writer.write("/home/andrea/ars_workspace/src/peg_and_ring/models/scene_red.pcd", *segmented_cloud);

        pcl::toROSMsg(*cloud_plane, plane_cloud_msg);
        plane_pub.publish(plane_cloud_msg);

        pcl::toROSMsg(*cloud_cylinder, cylinder_cloud_msg);
        cylinder_pub.publish(cylinder_cloud_msg);

        //----------------------------------------------------------------------------------------------------------------

        if (segmented_cloud->points.size() > 0 && !cloud_updated)
        {
            int ne = 0;
            for (std::string &topic : tool_subs_topic)
            {
                ee_pose[ne] = tool_poses[topic];
                ne++;
            }
            // std::cout << " cloud_updated   " << cloud_updated << std::endl;
            clusterPoints(segmented_cloud); // output

            // pcl::toROSMsg(*segmented_cloud, output_cloud);
            // cloud_pub.publish(output_cloud);
        }
        else if (cloud_updated) // TRACKING 2d -- enter here ONLY if we use geometryGrasp method - changing treshold tracking for daVinci.
        {
            tracking2d();
        }
        else
        {
            ROS_INFO_STREAM("[PEG_RING] WAITING FOR COLOR");
        }
    }
}

void peg_ring::pegsInitialize(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_cyl)
{
    std::vector<std::string> color_seq = {"RED", "GREEN", "BLUE", "YELLOW"};
    Eigen::Vector4f centroid_peg[4], minp_peg[4], maxp_peg[4];
    for (int i = 0; i < 4; i++)
    {
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr seg_cld(new pcl::PointCloud<pcl::PointXYZRGB>);
        seg_cld = colorSegmentation(cloud_cyl, color_seq.at(i));

        pcl::getMinMax3D(*seg_cld, minp_peg[i], maxp_peg[i]);
        pcl::compute3DCentroid<pcl::PointXYZRGB>(*seg_cld, centroid_peg[i]);

        geometry_msgs::Pose pegPose_temp;
        pegPose_temp.position.x = centroid_peg[i][0];
        pegPose_temp.position.y = centroid_peg[i][1];
        pegPose_temp.position.z = maxp_peg[i][2];
        pegPose_temp.orientation.x = 0;
        pegPose_temp.orientation.y = 0;
        pegPose_temp.orientation.z = 0;
        pegPose_temp.orientation.w = 1;
        all_pegs.poses.push_back(pegPose_temp);
    }

    geometry_msgs::Pose pegPose_tempA;
    pegPose_tempA.position.x = (centroid_peg[0][0] + centroid_peg[1][0]) / 2.0;
    pegPose_tempA.position.y = (centroid_peg[0][1] + centroid_peg[1][1]) / 2.0;
    pegPose_tempA.position.z = maxp_peg[0][2];
    pegPose_tempA.orientation.x = 0;
    pegPose_tempA.orientation.y = 0;
    pegPose_tempA.orientation.z = 0;
    pegPose_tempA.orientation.w = 1;
    all_pegs.poses.push_back(pegPose_tempA);

    geometry_msgs::Pose pegPose_tempB;
    pegPose_tempB.position.x = (centroid_peg[2][0] + centroid_peg[3][0]) / 2.0;
    pegPose_tempB.position.y = (centroid_peg[2][1] + centroid_peg[3][1]) / 2.0;
    pegPose_tempB.position.z = maxp_peg[2][2];
    pegPose_tempB.orientation.x = 0;
    pegPose_tempB.orientation.y = 0;
    pegPose_tempB.orientation.z = 0;
    pegPose_tempB.orientation.w = 1;
    all_pegs.poses.push_back(pegPose_tempB);

    double d = std::sqrt((centroid_peg[0][0] - centroid_peg[1][0]) * (centroid_peg[0][0] - centroid_peg[1][0]) +
                         (centroid_peg[0][1] - centroid_peg[1][1]) * (centroid_peg[0][1] - centroid_peg[1][1]));

    // BRUTE FORCE SOLUTION
    geometry_msgs::Pose pegPose_tempC;
    pegPose_tempC.position.x = centroid_peg[1][0];
    pegPose_tempC.position.y = centroid_peg[1][1] + d / 2.0;
    pegPose_tempC.position.z = maxp_peg[0][2];
    pegPose_tempC.orientation.x = 0;
    pegPose_tempC.orientation.y = 0;
    pegPose_tempC.orientation.z = 0;
    pegPose_tempC.orientation.w = 1;
    all_pegs.poses.push_back(pegPose_tempC);

    geometry_msgs::Pose pegPose_tempD;
    pegPose_tempD.position.x = centroid_peg[2][0];
    pegPose_tempD.position.y = centroid_peg[2][1] - d / 2.0;
    pegPose_tempD.position.z = maxp_peg[2][2];
    pegPose_tempD.orientation.x = 0;
    pegPose_tempD.orientation.y = 0;
    pegPose_tempD.orientation.z = 0;
    pegPose_tempD.orientation.w = 1;
    all_pegs.poses.push_back(pegPose_tempD);

    // end OF BRUTE FORCE SOLUTION
#ifdef TEST
    double c_x1, c_x2, c_y1, c_y2, m, q;
    c_x1 = centroid_peg[0][0];
    c_y1 = centroid_peg[0][1];
    c_x2 = centroid_peg[1][0];
    c_y2 = centroid_peg[1][1];
    m = (c_y2 - c_y1) / (c_x2 - c_x1);
    q = -m + c_y1;
    std::cout << " m " << m << std::endl;
    std::cout << " q " << q << std::endl;

    std::cout << "centroid V " << centroid_peg[1][0] << " , " << centroid_peg[1][1] << std::endl;
    // since the y is close to 0 we can use brute force solution in simulation. -> DELTA is negative
    double x_b_1, x_b_2, y_b_1, y_b_2;
    x_b_1 = solveWhitePeg(centroid_peg[1][0], centroid_peg[1][1], m, q, d, 1);
    x_b_2 = solveWhitePeg(centroid_peg[1][0], centroid_peg[1][1], m, q, d, -1);
    y_b_1 = m * x_b_1 + q;
    y_b_2 = m * x_b_2 + q;

    std::cout << "Points for 1 are  " << x_b_1 << " , " << y_b_1 << std::endl;
    std::cout << "Points for 2 are " << x_b_2 << " , " << y_b_2 << std::endl;

    std::cout << "\n*************\ntest\n*************\n ";
    double simpleTest = solveWhitePeg(0, 1, 1, 1, std::sqrt(2), 1);
    std::cout << simpleTest << " , " << simpleTest + 1 << std::endl;
    simpleTest = solveWhitePeg(0, 1, 1, 1, std::sqrt(2), -1);
    std::cout << simpleTest << " , " << simpleTest + 1 << std::endl;
#endif
}

double peg_ring::solveWhitePeg(double x, double y, double m, double q, double d, double sign)
{
    double sol = 0.0;
    double a = (m * m + 1);
    double b = (2 * m * y - 2 * m * q + 2 * x);
    double c = (-2 * q * y + q * q + x * x + y * y - d * d);
    double delta = b * b - 4 * a * c;
    std::cout << "a " << a << " b  " << b << " c " << c << std::endl;
    sol = (-b + sign * std::sqrt(delta)) / (2 * a);

    return sol;
}

// USED FOR PANDA
void peg_ring::tracking2d()
{
    // std::cout << "[TRACKING]" << std::endl;
    // std::cout << " segmented_cloud->points.size()   " << segmented_cloud->points.size() << std::endl;
    if (camera_K.size() > 0) // bool inside cb cameraInfo?
    {
        cv::Mat camMatrix = vector2Mat(camera_K, 3);
        cv::Mat camRot = vector2Mat(camera_R, 3);
        cv::Mat camProj = vector2Mat(camera_P, 4);
        cv::Mat camDist_temp = vector2Mat(camera_D, 5);
        cv::Mat camDist = camDist_temp.row(0);

        tf::StampedTransform transform;
        try
        {
            listener.lookupTransform(optical_frame, PSM_frame, ros::Time(0), transform);

            cv::Mat t = (cv::Mat_<float>(1, 3) << transform.getOrigin().x(), transform.getOrigin().y(), transform.getOrigin().z());
            tf::Matrix3x3 R = transform.getBasis();
            cv::Mat Rt = cv::Mat::eye(4, 4, CV_32F);

            for (int l = 0; l < 3; l++)
                for (int j = 0; j < 3; j++)
                {
                    Rt.at<float>(l, j) = R[l][j];
                }
            for (int l = 0; l < 3; l++)
                Rt.at<float>(l, 3) = t.at<float>(l);

            cv::Mat XYZ = (cv::Mat_<float>(4, 1) << ee_pose[0].pose.position.x, ee_pose[0].pose.position.y, ee_pose[0].pose.position.z, 1.0); // TO FIX WHICH IS THE TOOL IS BRINING THE RING
            cv::Mat p_cam = (cv::Mat_<float>(4, 1) << 0.0, 0.0, 0.0, 1.0);
            p_cam = Rt * XYZ;

            // camera Matrix given by calibration tool. You can find the parameters in config/camera_parameters
            cv::Mat camMatrix2 = (cv::Mat_<float>(3, 3) << 605.859680, 0.000000, 326.386965, 0.000000, 608.103149, 251.396761, 0.000000, 0.000000, 1.000000);

            cv::Mat img = (camMatrix2 * cv::Mat::eye(3, 4, CV_32F)) * p_cam;
            float scale = 0.0;
            scale = 1.0 / img.at<float>(0, 2);
            // cv::Point ee_center;
            ee_center.x = img.at<float>(0, 0) * scale;
            ee_center.y = img.at<float>(0, 1) * scale;
            cv::circle(image_rgb, ee_center, 1, cv::Scalar(255, 0, 0), 1, 8, 0);

            colorTracking(processing_color);
        }
        catch (tf::TransformException ex)
        {
            ROS_ERROR("%s", ex.what());
            ros::Duration(1.0).sleep();
        }
    }
}

// USED FOR DA VINCI
void peg_ring::closestGrasp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &ring_cloud)
{
    double distance[num_ee];
    int max_index[num_ee];
    float temp_distance[num_ee];
    // std::vector<geometry_msgs::PoseStamped> ee_pose_out;
    // std::cout << " num_ee    " << num_ee << std::endl;

    for (size_t j = 0; j < num_ee; j++)
    {

        for (size_t i = 0; i < ring_cloud->points.size(); i++)
        {
            temp_distance[j] = std::sqrt((ring_cloud->points[i].x - ee_pose[j].pose.position.x) * (ring_cloud->points[i].x - ee_pose[j].pose.position.x) +
                                         (ring_cloud->points[i].y - ee_pose[j].pose.position.y) * (ring_cloud->points[i].y - ee_pose[j].pose.position.y) +
                                         (ring_cloud->points[i].z - ee_pose[j].pose.position.z) * (ring_cloud->points[i].z - ee_pose[j].pose.position.z));

            if (distance[j] < temp_distance[j])
            {
                distance[j] = temp_distance[j];
                max_index[j] = i;
            }
        }
    }
    // std::cout << " max_element(array , array + n) " <<  *std::max_element(distance , distance + num_ee) << std::endl;
    // std::cout << "TOOL 1 Distance is " << distance[0] << "  with index  " << max_index[0] << std::endl;
    // std::cout << "TOOL 2 Distance is " << distance[1] << "  with index  " << max_index[1] << std::endl;

    double d = 0.0;
    int idx = 0;
    for (size_t i = 0; i < num_ee - 1; i++)
    {
        if (distance[i] < distance[i + 1])
        {
            d = distance[i];
            idx = max_index[i + 1]; // to check
        }
        else
        {
            d = distance[i + 1];
            idx = max_index[i]; // to check
        }
    }
    // std::cout << " max_index[idx] " << d << " -------" << max_index[idx] << "   " << idx << std::endl;

    geometry_msgs::PoseStamped gp, gp1;
    gp.header.frame_id = fixed_frame;
    gp.header.stamp = ros::Time::now();
    gp.pose.position.x = ring_cloud->points[idx].x;
    gp.pose.position.y = ring_cloud->points[idx].y;
    gp.pose.position.z = ring_cloud->points[idx].z;
    // orientation is fixed
    gp.pose.orientation.x = 0;
    gp.pose.orientation.y = 0;
    gp.pose.orientation.z = 0;
    gp.pose.orientation.w = 1;

    pub_ring_pose.publish(gp);
    // std::cout << " gp   " << gp << std::endl << std::endl;
    grasping_point = true;
}

//TO GRASP FROM THE LEFTMOST / RIGHTMOST SIDE OF THE RING
void peg_ring::graspPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &ring_cloud)
{
    std::vector<float> cost[num_ee]; // simple policy to avoid EEs intersection at transfer: leftmost EE picks on the left, rightmost on the right. Any better one?
    std::vector<float>::iterator min_index[num_ee];
    float sign; // to choose whether to pick the ring from the left or the right

    // ring_cloud->header.frame_id = fixed_frame;
    for (size_t j = 0; j < num_ee; j++)
    {
        for (size_t i = 0; i < ring_cloud->points.size(); i++)
        {
            if (!(std::abs(ee_pose[j].pose.position.y - ee_pose[1 - j].pose.position.y) < 2e-3)) // same y-value, no matter whether right or left grasping...
            {
                sign = (ee_pose[j].pose.position.y - ee_pose[1 - j].pose.position.y) / std::abs(ee_pose[j].pose.position.y - ee_pose[1 - j].pose.position.y);
                cost[j].push_back(ring_cloud->points[i].y * sign);
            }
        }

        min_index[j] = std::max_element(std::begin(cost[j]), std::end(cost[j]));
    }

    geometry_msgs::Pose gp;
    geometry_msgs::PoseArray closest_points;
    closest_points.header.frame_id = fixed_frame;
    closest_points.header.stamp = ros::Time::now();
    for (size_t i = 0; i < num_ee; i++)
    {
        gp.position.x = ring_cloud->points[std::distance(std::begin(cost[i]), min_index[i])].x;
        gp.position.y = ring_cloud->points[std::distance(std::begin(cost[i]), min_index[i])].y;
        gp.position.z = ring_cloud->points[std::distance(std::begin(cost[i]), min_index[i])].z;
        // orientation is fixed
        gp.orientation.x = 0;
        gp.orientation.y = 0;
        gp.orientation.z = 0;
        gp.orientation.w = 1;

        closest_points.poses.push_back(gp);
    }

    pub_ring_poses.publish(closest_points);
    grasping_point = true;
}

geometry_msgs::PoseArray peg_ring::sendRingPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &ring_cloud)
{
    geometry_msgs::Pose gp;
    geometry_msgs::PoseArray cluster_points;

    ring_cloud->header.frame_id = fixed_frame;

    for (size_t i = 0; i < ring_cloud->points.size(); i++)
    {
        gp.position.x = ring_cloud->points[i].x;
        gp.position.y = ring_cloud->points[i].y;
        gp.position.z = ring_cloud->points[i].z;
        // orientation is fixed
        gp.orientation.x = 0;
        gp.orientation.y = 0;
        gp.orientation.z = 0;
        gp.orientation.w = 1;

        cluster_points.poses.push_back(gp);
    }

    return cluster_points;
}
// USED FOR PANDA
void peg_ring::geometryGrasp(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &ring_cloud)
{
    tf::StampedTransform rot_cam_transform;
    tf::StampedTransform final_transform;
    tf::StampedTransform transform;
    ring_cloud->header.frame_id = fixed_frame;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr map_cld_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr rot_cam_cld_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr final_cam_cld_ptr(new pcl::PointCloud<pcl::PointXYZRGB>);

    map_cld_ptr->header.stamp = xyz_cld_ptr->header.stamp;
    try
    {
        listener.lookupTransform(PSM_frame, fixed_frame, ros::Time(0), rot_cam_transform);
        pcl_ros::transformPointCloud(*ring_cloud, *map_cld_ptr, rot_cam_transform);

        // pcl_ros::transformPointCloud(*rot_cam_cld_ptr, *map_cld_ptr, final_transform);

        // listener.lookupTransform(PSM_frame, fixed_camera, ros::Time(0), transform);
        // pcl_ros::transformPointCloud(*final_cam_cld_ptr, *map_cld_ptr, transform);

        Eigen::Vector4f centroid_ring, minp_ring, maxp_ring;
        pcl::getMinMax3D(*map_cld_ptr, minp_ring, maxp_ring);

        pcl::compute3DCentroid<pcl::PointXYZRGB>(*map_cld_ptr, centroid_ring);

        // if (centroid_ring[2] < 0.06)
        // {
        cloud_updated = true;

        geometry_msgs::PoseStamped gp, gp1;
        gp.header.frame_id = PSM_frame;
        gp.header.stamp = ros::Time::now();
        // std::cout << " minp_ring " << minp_ring << std::endl;
        // std::cout << " maxp_ring " << maxp_ring << std::endl;
        // std::cout << " centroid_ring " << centroid_ring << std::endl;
        gp.pose.position.x = minp_ring[0];
        gp.pose.position.y = centroid_ring[1];
        gp.pose.position.z = centroid_ring[2];
        // orientation is fixed
        gp.pose.orientation.x = 0.71;
        gp.pose.orientation.y = 0.7;
        gp.pose.orientation.z = -0.02;
        gp.pose.orientation.w = 0.02;

        // listener.transformPose(PSM_frame, ros::Time(0), gp, "/world", gp1);
        pub_ring_pose.publish(gp);
        // }
    }
    catch (tf::TransformException ex)
    {
        ROS_ERROR("%s", ex.what());
        ros::Duration(1.0).sleep();
    }
}

unsigned int peg_ring::findTorus(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_pn)
{
    pcl::SACSegmentation<pcl::PointNormal> seg;

    seg.setOptimizeCoefficients(true);
    seg.setInputCloud(cloud_pn);
    seg.setRadiusLimits(0.004, 0.006);

    if (torus_algorithm == "RANSAC")
    {
        seg.setMethodType(pcl::SAC_RANSAC);
    }
    else if (torus_algorithm == "LMEDS")
    {
        seg.setMethodType(pcl::SAC_LMEDS);
    }
    else if (torus_algorithm == "MSAC")
    {
        seg.setMethodType(pcl::SAC_MSAC);
    }
    else if (torus_algorithm == "RRANSAC")
    {
        seg.setMethodType(pcl::SAC_RRANSAC);
    }
    else if (torus_algorithm == "RMSAC")
    {
        seg.setMethodType(pcl::SAC_RMSAC);
    }
    else if (torus_algorithm == "MLESAC")
    {
        seg.setMethodType(pcl::SAC_MLESAC);
    }
    else if (torus_algorithm == "PROSAC")
    {
        seg.setMethodType(pcl::SAC_PROSAC);
    }

    seg.setDistanceThreshold(0.01);
    seg.setMaxIterations(200);
    seg.setModelType(pcl::SACMODEL_CIRCLE3D);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.segment(*inliers, *coefficients);

    // std::cerr << "ring coefficients: " << *coefficients << std::endl;

    if (inliers->indices.size() > torus_points) // min_size_
    {
        // std::cout << "SIZE OF INLIERS IS " << inliers->indices.size() << std::endl;
        // check direction. Torus should direct to origin of pointcloud always.
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

        torus.header.frame_id = fixed_frame;
        torus.pose.position.x = pos(0);
        torus.pose.position.y = pos(1);
        torus.pose.position.z = pos(2);
        torus.pose.orientation.x = rot.x();
        torus.pose.orientation.y = rot.y();
        torus.pose.orientation.z = rot.z();
        torus.pose.orientation.w = rot.w();
        // torus_pose.publish(torus);
    }
    else
    {
        torus.header.frame_id = "";
    }
    return inliers->indices.size();
}

void peg_ring::clusterPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster)
{
    obj_surf.clear();
    std::vector<pcl::PointIndices> clusters;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyz_f_(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_cluster, *xyz_f_, indices);
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGB>());
    ec.setInputCloud(xyz_f_);
    ec.setClusterTolerance(object_cluster_distance_);
    ec.setMinClusterSize(min_object_cluster_size_);
    ec.setMaxClusterSize(max_object_cluster_size_);
    ec.setSearchMethod(tree2);
    ec.extract(clusters);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_;
    //ROS_INFO_STREAM("CLUSTER SIZE  " << clusters.size());

    Eigen::Vector4f centroid, minp, maxp, transformed_max, transformed_min;

    pcl::PointCloud<pcl::Normal>::Ptr scene_normals(new pcl::PointCloud<pcl::Normal>()); // scene_normals
    pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> norm_est;
    norm_est.setKSearch(10);
    norm_est.setInputCloud(xyz_f_);
    norm_est.compute(*scene_normals);
    pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::copyPointCloud(*xyz_f_, *scene_cloud_xyz);
    pcl::PointCloud<pcl::PointNormal>::Ptr cloud_scene_normals = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
    pcl::concatenateFields(*scene_cloud_xyz, *scene_normals, *cloud_scene_normals);
    findTorus(cloud_scene_normals);

    for (unsigned int i = 0; i < clusters.size(); ++i)
    {
        // std::cout << "STARTING TO POPULATE OBJ_SURF WITH NUMBER OF CLUSTERS " << clusters.size() << std::endl;
        if (clusters[i].indices.size() >= static_cast<unsigned int>(min_object_cluster_size_))
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusters_i(new pcl::PointCloud<pcl::PointXYZRGB>);
            extract_.setInputCloud(xyz_f_);
            extract_.setIndices(boost::make_shared<const pcl::PointIndices>(clusters[i]));
            extract_.setNegative(false);
            extract_.filter(*clusters_i);
            obj_surf.push_back(clusters_i);
        }
    }

    int tempS = 0;
    int index = 0;

    for (size_t st = 0; st < obj_surf.size(); st++)
    {
        // std::cout << " size " << st << " -- " << obj_surf.at(st)->points.size() << std::endl;
        if (obj_surf.at(st)->points.size() > tempS)
        {
            tempS = obj_surf.at(st)->points.size();
            index = st;
        }
    }
    int index_r = 0;
    int index_p = 0;

    // FOR GIOVANNI TASK - we dont have PEGS so we should have only 1 cluster - THE RING
    // pcl::toROSMsg(*obj_surf.at(index), ring_cloud_msg); // change this
    // ring_pub.publish(ring_cloud_msg);

    ROS_INFO_STREAM(obj_surf.size());

    if (obj_surf.size() >= 2) // CHECK THIS ! (I EXPECT TWO POINT CLOUDS! ONE FOR RING AND SECOND FOR PEG)
    {
        // pcl::PCDWriter writer;
        // writer.write("/home/andrea/Dropbox/PCD_panda/ring_" + processing_color+  ".pcd", *obj_surf.at(index), false);

        // std::cout << "SIZE OF RING CLOUD" << "-" << obj_surf.at(index)->points.size() << std::endl;
        pcl::toROSMsg(*obj_surf.at(index), ring_cloud_msg);
        ring_pub.publish(ring_cloud_msg);

        // cloud_updated = true; // change this to TRUE if you want to track in RGB - and not in 3D

        // --- tracking 3d

        // pcl::compute3DCentroid<pcl::PointXYZRGB>(*obj_surf.at(index), centroid);

        // geometry_msgs::PoseStamped ring_track;
        ring_track.header.frame_id = fixed_frame;
        ring_track.header.stamp = ros::Time::now();
        ring_track.pose.position.x = all_rings.poses[color_idx].position.x;
        ring_track.pose.position.y = all_rings.poses[color_idx].position.y;
        ring_track.pose.position.z = all_rings.poses[color_idx].position.z;
        ring_track.pose.orientation.y = all_rings.poses[color_idx].orientation.y;
        ring_track.pose.orientation.z = all_rings.poses[color_idx].orientation.z;
        ring_track.pose.orientation.x = all_rings.poses[color_idx].orientation.x;
        ring_track.pose.orientation.w = all_rings.poses[color_idx].orientation.w;

        // pub_ring_track.publish(ring_track);

        // std::cout << " grasping_point   "  << grasping_point << std::endl;
        // geometryGrasp(obj_surf.at(index));           // PANDA

        // if (!grasping_point)
        // closestGrasp(obj_surf.at(index)); // DA VINCI
        // sendRingPoints(obj_surf.at(index));

        // if (index == 0)
        // {
        //     index_p = 1;
        //     index_r = 0;
        // }
        // else
        // {
        //     index_p = 0;
        //     index_r = 1;
        // }

        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
        // pcl::PointCloud<pcl::PointXYZRGB>::Ptr clean_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

        peg.header.frame_id = fixed_frame;
        peg.pose = all_pegs.poses[color_idx];
        pub_peg_pose.publish(peg); // Publish current peg

        // double d = 0.0;
        // d = std::sqrt((ring_track.pose.position.x - all_pegs.poses[color_idx].position.x) * (ring_track.pose.position.x - all_pegs.poses[color_idx].position.x) +
        //               (ring_track.pose.position.y - all_pegs.poses[color_idx].position.y) * (ring_track.pose.position.y - all_pegs.poses[color_idx].position.y) +
        //               (ring_track.pose.position.z - (all_pegs.poses[color_idx].position.z - peg_height)) * (ring_track.pose.position.z - (all_pegs.poses[color_idx].position.z - -peg_height)));

        // // std::cout << " Distance tracking " << d << std::endl;
        // if (d < threshold_tracking)
        //     std::cout << " RING with color id " << color_idx << " IS IN RIGHT POSITION" << std::endl;
    }

    // else if (obj_surf.size() == 1)
    // {
    //     peg.header.frame_id = fixed_frame;
    //     peg.pose = all_pegs.poses[color_idx];
    //     pub_peg_pose.publish(peg); // Publish current peg
    // }
}

void peg_ring::clusterAllPoints(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_cluster)
{
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr sub_ring_cld(new pcl::PointCloud<pcl::PointXYZRGB>);
    // sub_ring_cld = pcd_utils::voxel_grid_subsample(cloud_cluster, orig_cld_voxel_size);

    geometry_msgs::PoseArray ring_set;
    ring_set.header.frame_id = fixed_frame;
    ring_set.header.stamp = ros::Time::now();

    obj_surf.clear();
    std::vector<pcl::PointIndices> clusters;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyz_f_(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud_cluster, *xyz_f_, indices);
    pcl::EuclideanClusterExtraction<pcl::PointXYZRGB> ec;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree2(new pcl::search::KdTree<pcl::PointXYZRGB>());
    ec.setInputCloud(xyz_f_);
    ec.setClusterTolerance(object_cluster_distance_);
    ec.setMinClusterSize(min_object_cluster_size_);
    ec.setMaxClusterSize(max_object_cluster_size_);
    ec.setSearchMethod(tree2);
    ec.extract(clusters);
    pcl::ExtractIndices<pcl::PointXYZRGB> extract_;
    //ROS_INFO_STREAM("CLUSTER SIZE  " << clusters.size());

    Eigen::Vector4f centroid, minp, maxp, transformed_max, transformed_min;

    for (unsigned int i = 0; i < clusters.size(); ++i)
    {
        // std::cout << "STARTING TO POPULATE OBJ_SURF WITH NUMBER OF CLUSTERS " << clusters.size() << std::endl;
        if (clusters[i].indices.size() >= static_cast<unsigned int>(min_object_cluster_size_))
        {
            pcl::PointCloud<pcl::PointXYZRGB>::Ptr clusters_i(new pcl::PointCloud<pcl::PointXYZRGB>);
            extract_.setInputCloud(xyz_f_);
            extract_.setIndices(boost::make_shared<const pcl::PointIndices>(clusters[i]));
            extract_.setNegative(false);
            extract_.filter(*clusters_i);
            obj_surf.push_back(clusters_i);
        }
    }

    int tempS = 0;
    int index = 0;

    for (size_t st = 0; st < obj_surf.size(); st++)
    {
        // std::cout << " size " << st << " -- " << obj_surf.at(st)->points.size() << std::endl;
        if (obj_surf.at(st)->points.size() > tempS)
        {
            tempS = obj_surf.at(st)->points.size();
            index = st;
        }
    }
    int index_r = 0;
    int index_p = 0;

    // FOR GIOVANNI TASK - we dont have PEGS so we should have only 1 cluster - THE RING
    // pcl::toROSMsg(*obj_surf.at(index), ring_cloud_msg); // change this
    // ring_pub.publish(ring_cloud_msg);

    if (obj_surf.size() >= 2) // CHECK THIS ! (I EXPECT TWO POINT CLOUDS! ONE FOR RING AND SECOND FOR PEG)
    {
        // std::cout << "FOUND AT LEAST TWO CLUSTERS" << std::endl;
        // pcl::PCDWriter writer;
        // writer.write("/home/andrea/Dropbox/PCD_panda/ring_" + processing_color+  ".pcd", *obj_surf.at(index), false);

        // std::cout << "SIZE OF RING CLOUD" << "-" << obj_surf.at(index)->points.size() << std::endl;
        // pcl::toROSMsg(*obj_surf.at(index), ring_cloud_msg);
        // ring_pub.publish(ring_cloud_msg);

        // cloud_updated = true; // change this to TRUE if you want to track in RGB - and not in 3D

        // --- tracking 3d
        index = -1;
        unsigned int inliers, old_inliers;
        inliers = 0;
        old_inliers = 0;
        ROS_INFO_STREAM("inside size 2");
        for (unsigned int i = 0; i < obj_surf.size(); ++i)
        {
            // TORUS
            pcl::PointCloud<pcl::Normal>::Ptr scene_normals(new pcl::PointCloud<pcl::Normal>()); // scene_normals
            pcl::NormalEstimationOMP<pcl::PointXYZRGB, pcl::Normal> norm_est;
            norm_est.setKSearch(10);
            norm_est.setInputCloud(obj_surf.at(i));
            norm_est.compute(*scene_normals);
            pcl::PointCloud<pcl::PointXYZ>::Ptr scene_cloud_xyz(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::copyPointCloud(*obj_surf.at(i), *scene_cloud_xyz);
            pcl::PointCloud<pcl::PointNormal>::Ptr cloud_scene_normals = pcl::PointCloud<pcl::PointNormal>::Ptr(new pcl::PointCloud<pcl::PointNormal>);
            pcl::concatenateFields(*scene_cloud_xyz, *scene_normals, *cloud_scene_normals);
            torus.header.frame_id = "";
            inliers = findTorus(cloud_scene_normals);
            // std::cout << "INLIERS ARE " << inliers << std::endl;
            // std::cout << "OLD_INLIERS ARE " << old_inliers << std::endl;
            if (torus.header.frame_id == fixed_frame)
            {
                if (inliers > old_inliers)
                {
                    // std::cout << "FOUND TORUS" << std::endl;
                    old_inliers = inliers;
                    index = i;
                    // break;
                }
            }
            // TORUS
        }

        if (index != -1)
        {
            pcl::compute3DCentroid<pcl::PointXYZRGB>(*obj_surf.at(index), centroid);
            Eigen::Vector4f plane_ring_parameters;
            float curvature;
            pcl::computePointNormal<pcl::PointXYZRGB>(*obj_surf.at(index), plane_ring_parameters, curvature);

            // geometry_msgs::PoseStamped ring_track;
            ring_track.header.frame_id = fixed_frame;
            ring_track.header.stamp = ros::Time::now();
            ring_track.pose.position.x = centroid[0];
            ring_track.pose.position.y = centroid[1];
            ring_track.pose.position.z = centroid[2];
            ring_track.pose.orientation.x = plane_ring_parameters[0];
            ring_track.pose.orientation.y = plane_ring_parameters[1];
            ring_track.pose.orientation.z = plane_ring_parameters[2];
            ring_track.pose.orientation.w = plane_ring_parameters[3];

            ring_set = sendRingPoints(obj_surf.at(index));   
        }
        else
        {
            ring_track.header.frame_id = fixed_frame;
            ring_track.header.stamp = ros::Time::now();
            ring_track.pose.position.x = 0;
            ring_track.pose.position.y = 0;
            ring_track.pose.position.z = 0;
            ring_track.pose.orientation.x = 0;
            ring_track.pose.orientation.y = 0;
            ring_track.pose.orientation.z = 0;
            ring_track.pose.orientation.w = 0;            
        }
    }
    //     pub_ring_track.publish(ring_track);

    //     // std::cout << " grasping_point   "  << grasping_point << std::endl;
    //     // geometryGrasp(obj_surf.at(index));           // PANDA

    //     // if (!grasping_point)
    //     // closestGrasp(obj_surf.at(index)); // DA VINCI
    //     sendRingPoints(obj_surf.at(index));

    //     if (index == 0)
    //     {
    //         index_p = 1;
    //         index_r = 0;
    //     }
    //     else
    //     {
    //         index_p = 0;
    //         index_r = 1;
    //     }

    //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr transformed_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr filtered_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    //     pcl::PointCloud<pcl::PointXYZRGB>::Ptr clean_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());

    //     peg.header.frame_id = fixed_frame;
    //     peg.pose = all_pegs.poses[color_idx];
    //     pub_peg_pose.publish(peg); // Publish current peg

    //     double d = 0.0;
    //     d = std::sqrt((ring_track.pose.position.x - all_pegs.poses[color_idx].position.x) * (ring_track.pose.position.x - all_pegs.poses[color_idx].position.x) +
    //                   (ring_track.pose.position.y - all_pegs.poses[color_idx].position.y) * (ring_track.pose.position.y - all_pegs.poses[color_idx].position.y) +
    //                   (ring_track.pose.position.z - (all_pegs.poses[color_idx].position.z - peg_height)) * (ring_track.pose.position.z - (all_pegs.poses[color_idx].position.z - - peg_height)));

    //     // std::cout << " Distance tracking " << d << std::endl;
    //     if (d < threshold_tracking)
    //         std::cout << " RING with color id " << color_idx << " IS IN RIGHT POSITION" << std::endl;
    // }

    else if (obj_surf.size() == 1)
    {
        ring_track.header.frame_id = fixed_frame;
        ring_track.header.stamp = ros::Time::now();
        ring_track.pose.position.x = 0;
        ring_track.pose.position.y = 0;
        ring_track.pose.position.z = 0;
        ring_track.pose.orientation.x = 0;
        ring_track.pose.orientation.y = 0;
        ring_track.pose.orientation.z = 0;
        ring_track.pose.orientation.w = 0;
    }

    all_rings.poses.push_back(ring_track.pose);
    ring_point_sets.sets.push_back(ring_set);
}

pcl::PointCloud<pcl::PointXYZRGB>::Ptr peg_ring::colorSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, std::string color)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr segmented_cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyz_f_(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *xyz_f_, indices);
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr HSV(new pcl::PointCloud<pcl::PointXYZHSV>);
    pcd_utils::PcdRGBtoHSV(*xyz_f_, *HSV); // RGB -> HSV
    segmented_cloud_tmp->points.clear();

    for (size_t i = 0; i < HSV->points.size(); i++)
    {
        if (color == "RED")
        {
            if ((HSV->points[i].h > 340 || HSV->points[i].h < 20) && (HSV->points[i].s >= 0.70 && HSV->points[i].s <= 1)) // red
            {
                pcl::PointXYZRGB out_points;
                out_points.x = HSV->points[i].x;
                out_points.y = HSV->points[i].y;
                out_points.z = HSV->points[i].z;
                out_points.r = xyz_f_->points[i].r;
                out_points.g = xyz_f_->points[i].g;
                out_points.b = xyz_f_->points[i].b;
                segmented_cloud_tmp->push_back(out_points);
            }
        }
        else if (color == "GREEN")
        {
            if ((HSV->points[i].h > 70 && HSV->points[i].h < 170) && (HSV->points[i].s >= 0.70 && HSV->points[i].s <= 1)) // green
            {
                pcl::PointXYZRGB out_points;
                out_points.x = HSV->points[i].x;
                out_points.y = HSV->points[i].y;
                out_points.z = HSV->points[i].z;
                out_points.r = xyz_f_->points[i].r;
                out_points.g = xyz_f_->points[i].g;
                out_points.b = xyz_f_->points[i].b;
                segmented_cloud_tmp->push_back(out_points);
            }
        }
        else if (color == "BLUE")
        {
            if ((HSV->points[i].h > 200 && HSV->points[i].h < 280) && (HSV->points[i].s >= 0.65 && HSV->points[i].s <= 1)) // blu
            {
                pcl::PointXYZRGB out_points;
                out_points.x = HSV->points[i].x;
                out_points.y = HSV->points[i].y;
                out_points.z = HSV->points[i].z;
                out_points.r = xyz_f_->points[i].r;
                out_points.g = xyz_f_->points[i].g;
                out_points.b = xyz_f_->points[i].b;
                segmented_cloud_tmp->push_back(out_points);
            }
        }
        else if (color == "YELLOW")
        {
            if ((HSV->points[i].h > 35 && HSV->points[i].h < 70) && (HSV->points[i].s >= 0.70 && HSV->points[i].s <= 1)) // yellow
            {
                pcl::PointXYZRGB out_points;
                out_points.x = HSV->points[i].x;
                out_points.y = HSV->points[i].y;
                out_points.z = HSV->points[i].z;
                out_points.r = xyz_f_->points[i].r;
                out_points.g = xyz_f_->points[i].g;
                out_points.b = xyz_f_->points[i].b;
                segmented_cloud_tmp->push_back(out_points);
            }
        }
        else
            // ROS_INFO_STREAM("colorSegmentation - COLOR NOT VALID ");
            continue;
    }
    segmented_cloud_tmp->header.frame_id = fixed_frame;
    segmented_cloud_tmp->header.stamp = xyz_cld_ptr->header.stamp;
    return segmented_cloud_tmp;
}

void peg_ring::colorSegmentation(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr red_segmented_cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr green_segmented_cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr blue_segmented_cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr yellow_segmented_cloud_tmp(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyz_f_(new pcl::PointCloud<pcl::PointXYZRGB>);
    std::vector<int> indices;
    pcl::removeNaNFromPointCloud(*cloud, *xyz_f_, indices);
    pcl::PointCloud<pcl::PointXYZHSV>::Ptr HSV(new pcl::PointCloud<pcl::PointXYZHSV>);
    pcd_utils::PcdRGBtoHSV(*xyz_f_, *HSV); // RGB -> HSV
    red_segmented_cloud_tmp->points.clear();
    blue_segmented_cloud_tmp->points.clear();
    green_segmented_cloud_tmp->points.clear();
    yellow_segmented_cloud_tmp->points.clear();

    for (size_t i = 0; i < HSV->points.size(); i++)
    {
        if ((HSV->points[i].h > 340 || HSV->points[i].h < 20) && (HSV->points[i].s >= 0.70 && HSV->points[i].s <= 1)) // red
        {
            pcl::PointXYZRGB red_out_points;
            red_out_points.x = HSV->points[i].x;
            red_out_points.y = HSV->points[i].y;
            red_out_points.z = HSV->points[i].z;
            red_out_points.r = xyz_f_->points[i].r;
            red_out_points.g = xyz_f_->points[i].g;
            red_out_points.b = xyz_f_->points[i].b;
            red_segmented_cloud_tmp->push_back(red_out_points);
        }
        else if ((HSV->points[i].h > 70 && HSV->points[i].h < 170) && (HSV->points[i].s >= 0.70 && HSV->points[i].s <= 1)) // green
        {
            pcl::PointXYZRGB green_out_points;
            green_out_points.x = HSV->points[i].x;
            green_out_points.y = HSV->points[i].y;
            green_out_points.z = HSV->points[i].z;
            green_out_points.r = xyz_f_->points[i].r;
            green_out_points.g = xyz_f_->points[i].g;
            green_out_points.b = xyz_f_->points[i].b;
            green_segmented_cloud_tmp->push_back(green_out_points);
        }
        else if ((HSV->points[i].h > 200 && HSV->points[i].h < 280) && (HSV->points[i].s >= 0.65 && HSV->points[i].s <= 1)) // blue
        {
            pcl::PointXYZRGB blue_out_points;
            blue_out_points.x = HSV->points[i].x;
            blue_out_points.y = HSV->points[i].y;
            blue_out_points.z = HSV->points[i].z;
            blue_out_points.r = xyz_f_->points[i].r;
            blue_out_points.g = xyz_f_->points[i].g;
            blue_out_points.b = xyz_f_->points[i].b;
            blue_segmented_cloud_tmp->push_back(blue_out_points);
        }
        else if ((HSV->points[i].h > 35 && HSV->points[i].h < 70) && (HSV->points[i].s >= 0.70 && HSV->points[i].s <= 1)) // yellow
        {
            pcl::PointXYZRGB yellow_out_points;
            yellow_out_points.x = HSV->points[i].x;
            yellow_out_points.y = HSV->points[i].y;
            yellow_out_points.z = HSV->points[i].z;
            yellow_out_points.r = xyz_f_->points[i].r;
            yellow_out_points.g = xyz_f_->points[i].g;
            yellow_out_points.b = xyz_f_->points[i].b;
            yellow_segmented_cloud_tmp->push_back(yellow_out_points);
        }
    }
    red_segmented_cloud_tmp->header.frame_id = fixed_frame;
    green_segmented_cloud_tmp->header.frame_id = fixed_frame;
    blue_segmented_cloud_tmp->header.frame_id = fixed_frame;
    yellow_segmented_cloud_tmp->header.frame_id = fixed_frame;
    red_segmented_cloud_tmp->header.stamp = xyz_cld_ptr->header.stamp;
    green_segmented_cloud_tmp->header.stamp = xyz_cld_ptr->header.stamp;
    blue_segmented_cloud_tmp->header.stamp = xyz_cld_ptr->header.stamp;
    yellow_segmented_cloud_tmp->header.stamp = xyz_cld_ptr->header.stamp;
    all_rings.poses.clear();
    ring_point_sets.sets.clear();
    all_rings.header.frame_id = fixed_frame;
    clusterAllPoints(red_segmented_cloud_tmp);
    clusterAllPoints(green_segmented_cloud_tmp);
    clusterAllPoints(blue_segmented_cloud_tmp);
    clusterAllPoints(yellow_segmented_cloud_tmp);
    pub_ring_centers.publish(all_rings);
    pub_ring_points.publish(ring_point_sets);
    // return segmented_cloud_tmp;
}

void peg_ring::colorTracking(std::string color)
{
    cv::Mat left_HSV;
    cv::Mat left_mask_mser;
    cv::Mat left_hueMask;
    std::vector<cv::Rect> left_circle_temp;
    double left_distance;
    int left_l;
    // prepare a black image where to print circles found from mser
    cv::Mat mask(image_rgb.rows, image_rgb.cols, CV_8UC1, cv::Scalar(0));

    // if (check_left)
    // {
    //     mainAreaLeft = findMainArea(image_rgb);
    //     check_left = false;
    // }

    if (!image_rgb.empty())
    {
        cv::Mat reduced_img; //= findMainArea(image_rgb);

        // cv::Mat ROI(image_rgb, cv::Rect(0, 110, image_rgb.size().width, image_rgb.size().height - 110));
        // ROI.copyTo(reduced_img);

        cv_ptr2send.image = image_rgb;
        cv_ptr2send.encoding = "bgr8";
        image_pub_.publish(cv_ptr2send.toImageMsg());

        cvtColor(image_rgb, left_HSV, CV_BGR2HSV);

        if (color == "RED")
        {
            left_hueMask = hue_red(left_HSV); //mask
        }
        else if (color == "GREEN")
        {
            left_hueMask = hue_green(left_HSV);
        }
        else if (color == "BLUE")
        {
            left_hueMask = hue_blue(left_HSV);
        }
        else if (color == "YELLOW")
        {
            left_hueMask = hue_yellow(left_HSV); // to fix yellow range
                                                 // centerPegL = centerBlackPegL;
        }
        else
        {
            left_hueMask = hue_red(left_HSV);
        }

        GaussianBlur(left_hueMask, left_hueMask, cv::Size(9, 9), 2, 2);

        left_mask_mser = mser(left_hueMask, mask, 2000, 4500); //& mainAreaRight;
        mser2(left_mask_mser, 2000, 6000, &left_circle_temp);

        if (!left_circle_temp.empty())
        {
            // std::cout << "color  -- " << color << std::endl;
            cv::rectangle(image_rgb, left_circle_temp.at(left_circle_temp.size() - 1), CV_RGB(0, 255, 0));
            //
            cv::Point circle_center(left_circle_temp.at(left_circle_temp.size() - 1).x + (left_circle_temp.at(left_circle_temp.size() - 1).width / 2),
                                    left_circle_temp.at(left_circle_temp.size() - 1).y + (left_circle_temp.at(left_circle_temp.size() - 1).height / 2));

            // std::cout << "circle_center " << circle_center.x << "   " << circle_center.y << std::endl;
            cv::circle(image_rgb, circle_center, 1, cv::Scalar(255, 0, 0), 1, 8, 0);
            double distance = cv::norm(cv::Point(circle_center) - cv::Point(ee_center));
            // std::cout << "distance   " << distance << std::endl;
            if (carry_state) //&& !end_carry)
            {
                if (prev_circle_center != circle_center)
                {
                    if (distance < threshold_tracking) // TO FIX RANGE and HANDLE messages
                    {
                        ROS_INFO_STREAM("RING ATTACHED");
                        // track.data = true;
                    }
                    else
                    {
                        // if (((ee_pose.pose.position.x - peg_pose.pose.position.x) * (ee_pose.pose.position.x - peg_pose.pose.position.x) +
                        // (ee_pose.pose.position.y - peg_pose.pose.position.y) * (ee_pose.pose.position.y - peg_pose.pose.position.y)) > (ee_treshold * ee_treshold))
                        // {
                        ROS_INFO_STREAM("RING NOT ATTACHED");
                        track.data = false;
                        carry_state = false;
                        tracking_pub.publish(track);
                        // }
                    }
                }
            }

            cv_ptr2send.image = reduced_img;
            cv_ptr2send.encoding = "bgr8";
            image_pub_.publish(cv_ptr2send.toImageMsg());

            prev_circle_center = circle_center;
        }
    }
}

void peg_ring::extractPlane(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud_filtered)
{
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pcl::NormalEstimation<pcl::PointXYZRGB, pcl::Normal> ne;
    pcl::SACSegmentationFromNormals<pcl::PointXYZRGB, pcl::Normal> seg;
    pcl::PCDWriter writer;
    pcl::ExtractIndices<pcl::PointXYZRGB> extract;
    pcl::ExtractIndices<pcl::Normal> extract_normals;
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>());

    // Datasets
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals(new pcl::PointCloud<pcl::Normal>);
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_filtered2(new pcl::PointCloud<pcl::PointXYZRGB>);
    pcl::PointCloud<pcl::Normal>::Ptr cloud_normals2(new pcl::PointCloud<pcl::Normal>);
    pcl::ModelCoefficients::Ptr coefficients_plane(new pcl::ModelCoefficients), coefficients_cylinder(new pcl::ModelCoefficients);
    pcl::PointIndices::Ptr inliers_plane(new pcl::PointIndices), inliers_cylinder(new pcl::PointIndices);

    // Estimate point normals
    ne.setSearchMethod(tree);
    ne.setInputCloud(cloud_filtered);
    ne.setKSearch(50);
    ne.compute(*cloud_normals);

    // Create the segmentation object for the planar model and set all the parameters
    seg.setOptimizeCoefficients(true);
    seg.setModelType(pcl::SACMODEL_NORMAL_PLANE);
    seg.setNormalDistanceWeight(0.1);
    seg.setMethodType(pcl::SAC_RANSAC);
    seg.setMaxIterations(100);
    seg.setDistanceThreshold(0.015); // seg.setDistanceThreshold(0.03);
    seg.setInputCloud(cloud_filtered);
    seg.setInputNormals(cloud_normals);
    // Obtain the plane inliers and coefficients
    seg.segment(*inliers_plane, *coefficients_plane);
    // std::cerr << "Plane coefficients: " << *coefficients_plane << std::endl;

    // Extract the planar inliers from the input cloud
    extract.setInputCloud(cloud_filtered);
    extract.setIndices(inliers_plane);
    extract.setNegative(false);

    // Write the planar inliers to disk
    // pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_plane(new pcl::PointCloud<pcl::PointXYZRGB>());
    extract.filter(*cloud_plane);
    // std::cerr << "PointCloud representing the planar component: " << cloud_plane->points.size() << " data points." << std::endl;
    // writer.write("/home/andrea/debug_Data/plane2.pcd", *cloud_plane, false);
    if (cloud_plane->size() > 0)
    {
        plane_detected = true;
        // Eigen::Vector4f plane_parameters; // SAME AS COEFFICIENTS PLANE
        Eigen::Vector4f centroid_plane;
        float curvature;
        pcl::computePointNormal(*cloud_plane, plane_parameters, curvature);
        pcl::compute3DCentroid<pcl::PointXYZRGB>(*cloud_plane, centroid_plane);
        // std::cout << " PLANE PARAM " << plane_parameters[0] << "  " << plane_parameters[1] << "  " << plane_parameters[2] << "  " << plane_parameters[3] << "  " << std::endl;
        plane_pose.header.frame_id = fixed_frame;
        plane_pose.header.stamp = ros::Time::now();
        plane_pose.pose.position.x = centroid_plane[0];
        plane_pose.pose.position.y = centroid_plane[1];
        plane_pose.pose.position.z = centroid_plane[2];
        plane_pose.pose.orientation.z = 1;
        // std::cout << " PLANE PARAM " << plane_parameters[0] << "  " << plane_parameters[1] << "  " << plane_parameters[2] << "  " << plane_parameters[3] << "  " << std::endl;
    }
    // if set true , i extract the rest of the pcds
    extract.setNegative(true);
    extract.filter(*cloud_scene);

    Eigen::Vector4f minPlane, maxPlane;
    pcl::getMinMax3D(*cloud_plane, minPlane, maxPlane);
    Eigen::Vector4f minScene, maxScene;
    pcl::getMinMax3D(*cloud_scene, minScene, maxScene);

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr new_scene(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PassThrough<pcl::PointXYZRGB> pass_;
    pass_.setFilterFieldName("z");
    // 0.005 magic number to remove rings --> peg_height
    pass_.setFilterLimits(maxScene[2] - peg_height, maxScene[2]);
    pass_.setInputCloud(cloud_scene);
    pass_.setKeepOrganized(true);
    pass_.filter(*cloud_cylinder);
}

//------------------------------------------------------------------------------------------------
//------------------------------------------------------------------------------------------------
//                                         MAIN
//------------------------------------------------------------------------------------------------
int peg_ring_main(int argc, char **argv)
{
    ros::init(argc, argv, "peg_ring_node");
    ros::NodeHandle nh;
    peg_ring peg_ring(nh);
    peg_ring.init();

    while (ros::ok())
    {
        peg_ring.update();
        ros::spinOnce();
    }
    return 0;
}

int main(int argc, char **argv)
{
    return peg_ring_main(argc, argv);
}