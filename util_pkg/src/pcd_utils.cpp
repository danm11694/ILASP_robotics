#include "pcd_utils.hpp"

void pcd_utils::color_pointcloud(pcl::PointCloud<pcl::PointXYZRGB>::Ptr cld_ptr,
                                 pcl::PointIndices::Ptr indices,
                                 Eigen::Vector3f color)
{
    for (std::vector<int>::iterator it = indices->indices.begin();
         it != indices->indices.end(); ++it)
    {
        cld_ptr->points[(*it)].r = color.x();
        cld_ptr->points[(*it)].g = color.y();
        cld_ptr->points[(*it)].b = color.z();
    }
}

int pcd_utils::cluster_points(ANNpointArray points, int points_count, std::vector<int> &membership,
                              float r_max, float r_min, float A, float K)
{
    // adaptive radius calculation
    // (see paper Fast and Robust Object Detection in Household Environments
    // Using Vocabulary Trees with SIFT Descriptors by Pangercic and Haltakov)
    double radius = (1 - logsig((points_count - A) * K)) * (r_max - r_min) + r_min;
    ANNidxArray nnIdx = new ANNidx[points_count];
    ANNkd_tree *kdTree = new ANNkd_tree(points, points_count, 2);
    membership.assign(points_count, -1);
    int last_unassigned_id = 0;
    int current_cluster = 0;

    while (last_unassigned_id < points_count)
    {
        std::vector<int> points_stack;
        points_stack.push_back(last_unassigned_id);
        while (points_stack.size() > 0)
        {
            int current_point_id = points_stack.back();
            points_stack.pop_back();
            membership[current_point_id] = current_cluster;
            int points_found = kdTree->annkFRSearch(points[current_point_id],
                                                    radius, points_count, nnIdx);

            int newPointsCount = 0;
            for (int i = 0; i < points_found; ++i)
                if (membership[nnIdx[i]] == -1)
                    ++newPointsCount;

            if (newPointsCount > 3)
            {
                for (int i = 0; i < points_found; ++i)
                    if (membership[nnIdx[i]] == -1)
                        points_stack.push_back(nnIdx[i]);
            }
        }

        ++current_cluster;
        ++last_unassigned_id;
        while (last_unassigned_id < points_count && membership[last_unassigned_id] != -1)
            ++last_unassigned_id;
    }

    delete[] nnIdx;
    delete kdTree;
    annClose();
    return current_cluster;
}

void pcd_utils::saveImage(const std::string &filename, const pcl::PCLImage &image)
{
    clock_t tic;
    tic = clock();
    pcl::console::print_highlight("Saving ");
    pcl::console::print_value("%s ", filename.c_str());
    pcl::io::savePNGFile(filename, image);
    pcl::console::print_info("[done, ");
    pcl::console::print_value("%g", (clock() - tic) * 1000 / CLOCKS_PER_SEC);
    pcl::console::print_info(" ms : ");
    pcl::console::print_value("%d", image.width * image.height);
    pcl::console::print_info(" points]\n");
}

// if flip , flip vertically 180, else just save
pcl::PCLImage
pcd_utils::getPNGImage(const std::string &filename, pcl::PointCloud<pcl::PointXYZ>::Ptr proc_cloud_ptr, bool flip)
{
    pcl::PCLImage image;
    bool extracted;
    pcl::io::PointCloudImageExtractorFromZField<pcl::PointXYZ> pcie;
    pcie.setPaintNaNsWithBlack(true);
    extracted = pcie.extract(*proc_cloud_ptr, image);

    if (flip)
    {
        pcl::PCLImage image2 = image;
        int height = image.height;
        for (int i = 0; i < image.step; ++i)
            for (int j = 0; j < height; ++j)
                image.data[(i + j * image.step)] = image2.data[(i + (height - 1 - j) * image.step)];
    }

    saveImage(filename, image);
    return image;
}

void pcd_utils::PointRGBtoHSV(pcl::PointXYZRGB &in, pcl::PointXYZHSV &out)
{
    const unsigned char max = std::max(in.r, std::max(in.g, in.b));
    const unsigned char min = std::min(in.r, std::min(in.g, in.b));
    out.x = in.x;
    out.y = in.y;
    out.z = in.z;
    out.v = static_cast<float>(max) / 255.f;

    if (max == 0) // division by zero
    {
        out.s = 0.f;
        out.h = 0.f; // h = -1.f;
        return;
    }

    const float diff = static_cast<float>(max - min);
    out.s = diff / static_cast<float>(max);

    if (min == max) // diff == 0 -> division by zero
    {
        out.h = 0;
        return;
    }

    if (max == in.r)
        out.h = 60.f * (static_cast<float>(in.g - in.b) / diff);
    else if (max == in.g)
        out.h = 60.f * (2.f + static_cast<float>(in.b - in.r) / diff);
    else
        out.h = 60.f * (4.f + static_cast<float>(in.r - in.g) / diff); // max == b

    if (out.h < 0.f)
        out.h += 360.f;
}

void pcd_utils::PcdRGBtoHSV(pcl::PointCloud<pcl::PointXYZRGB> &in,
                            pcl::PointCloud<pcl::PointXYZHSV> &out)
{
    out.width = in.width;
    out.height = in.height;
    for (size_t i = 0; i < in.points.size(); i++)
    {
        pcl::PointXYZHSV p;
        PointRGBtoHSV(in.points[i], p);
        out.points.push_back(p);
    }
}

// input : pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_pn
// torus_agorithm
// double min, double max Radius limits
// int inliersPts min size of inliers
// std::string ref_frame  reference frame for the pose of the torus
// geometry_msgs::PoseStamped pcd_utils::findTorus(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_pn, std::string torus_algorithm, double min, double max, int inliersPts, std::string ref_frame)
int pcd_utils::findTorus(pcl::PointCloud<pcl::PointNormal>::Ptr &cloud_pn, std::string torus_algorithm, double min, double max, int inliersPts, std::string ref_frame)
{
    geometry_msgs::PoseStamped torus;
    pcl::SACSegmentation<pcl::PointNormal> seg;

    seg.setOptimizeCoefficients(true);
    seg.setInputCloud(cloud_pn);
    seg.setRadiusLimits(min, max);

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

    seg.setDistanceThreshold(0.1);
    seg.setMaxIterations(100);
    seg.setModelType(pcl::SACMODEL_CIRCLE3D);

    pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
    pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
    seg.segment(*inliers, *coefficients);

    // std::cerr << "ring coefficients: " << *coefficients << std::endl;

    if (inliers->indices.size() > inliersPts) // min_size_
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

        torus.header.frame_id = ref_frame;
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
    // return torus;
}

void pcd_utils::makeSphere(float radius, int r, int g, int b)
{
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr sphere_cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
    float px, py, pz;
    for (float phi = 0; phi < M_PI; phi += M_PI / 50)
    {
        pz = radius * cos(phi);
        for (float theta = 0; theta < 2 * M_PI; theta += 2 * M_PI / 50)
        {
            px = radius * sin(phi) * cos(theta);
            py = radius * sin(phi) * sin(theta);
            pcl::PointXYZRGB point(r, g, b);
            point.x = px;
            point.y = py;
            point.z = pz;
            sphere_cloud->push_back(point);
        }
    }
}

//Add to the cloud a empty sphere centered in c
// USAGE :
// Get the minimum and maximum values on each of the 3 (x-y-z) dimensions in cloud_filtered
// pcl::PointXYZRGB min_pt, max_pt;
// pcl::getMinMax3D(*cloud_filtered, min_pt, max_pt);
// auto sphereDiameter = std::max((max_pt.z - min_pt.z), (max_pt.x - min_pt.x));
// Eigen::Vector4f c;
// pcl::compute3DCentroid(*cloud_filtered,c); // Find the cloud 's centroid to be used as the sphere' s centre
// makeSphere(*cloud_filtered, sphereDiameter/2.0f, c); // Create sphere around obstacle
void pcd_utils::makeSphere(pcl::PointCloud<pcl::PointXYZRGB> &cloud, const float &radius, const Eigen::Vector4f &c)
{
    pcl::PointXYZRGB p;
    //Color the sphere red
    p.r = 180;

    //Get centroid coordinates
    float cz = c.z();
    float cx = c.x();
    float cy = c.y();

    //Convert from spherical coordinates to cartesian coordinates
    //For performance improvement half of the total point of the surface are used
    for (int phi = 0; phi <= 180; phi += 2)
    {
        for (int theta = 0; theta <= 360; theta += 2)
        {
            p.x = cx + (radius * cos(theta * M_PI / 180) * sin(phi * M_PI / 180));
            p.y = cy + (radius * sin(theta * M_PI / 180) * sin(phi * M_PI / 180));
            p.z = cz + (radius * cos(phi * M_PI / 180));
            cloud.push_back(p);
        }
    }
}

void pcd_utils::makeEllipsoid(pcl::PointCloud<pcl::PointXYZRGB> &cloud, const Eigen::Vector3f radii, const Eigen::Vector4f &c)
{
    pcl::PointXYZRGB p;
    p.r = 180;
    p.g = 180;

    float cz = c.z();
    float cx = c.x();
    float cy = c.y();

    for (int phi = 0; phi <= 180; phi += 2)
    {
        for (int theta = 0; theta <= 360; theta += 2)
        {
            p.x = cx + (radii.x() * cos(theta * M_PI / 180) * sin(phi * M_PI / 180));
            p.y = cy + (radii.y() * sin(theta * M_PI / 180) * sin(phi * M_PI / 180));
            p.z = cz + (radii.z() * cos(phi * M_PI / 180));
            cloud.push_back(p);
        }
    }
}

int pcd_utils_test(int argc, char **argv)
{
    return 0;
}

/*
int main(int argc, char **argv)
{
    return pcd_utils_test(argc, argv);
}
*/
