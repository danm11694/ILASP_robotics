#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>

#include <pcl/surface/gp3.h>
#include <pcl/surface/poisson.h>

#include <pcl_ros/point_cloud.h>
#include <pcl_ros/transforms.h>

#include <pcl/io/ply_io.h>
#include <pcl/io/obj_io.h>


int main()
{
    // Load the PLY file into a point cloud
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>);
     pcl::io::loadPLYFile("home/andrea/ars_point_cloud.ply", *cloud);

    // Create a search tree for efficient nearest neighbor searches
    pcl::search::KdTree<pcl::PointXYZRGB>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZRGB>);
    tree->setInputCloud(cloud);
    
 // Create a surface reconstruction object
      pcl::Poisson<pcl::PointXYZRGB> poisson;
      pcl::PolygonMesh mesh;

      // Set the input point cloud
      poisson.setInputCloud(cloud);

      // Set parameters for the surface reconstruction
      poisson.setDepth(9); // Increase this value for higher resolution
      poisson.setSolverDivide(8);
      poisson.setIsoDivide(8);
      poisson.setPointWeight(4.0f);
      poisson.setScale(1.1f);
      poisson.setSamplesPerNode(1.0f);
      poisson.setOutputPolygons(false); // Set to true if you want to output polygons instead of triangles

      // Perform the surface reconstruction
      poisson.reconstruct(mesh);
    // Save the mesh to a file in OBJ format
    pcl::io::saveOBJFile("home/andrea/output_mesh.obj", mesh);

    return 0;
}