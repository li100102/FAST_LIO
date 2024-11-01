#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/gicp.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/common/transforms.h>
#include <pcl/visualization/pcl_visualizer.h>

int main(int argc, char **argv)
{
    if (argc != 4)
    {
        std::cerr << "Usage: ./gicp_example <source.pcd> <target.pcd>" << std::endl;
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr source_cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::PointCloud<pcl::PointXYZINormal>::Ptr target_cloud(new pcl::PointCloud<pcl::PointXYZINormal>());

    // Load the point clouds
    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(argv[1], *source_cloud) == -1)
    {
        std::cerr << "Couldn't read file " << argv[1] << std::endl;
        return -1;
    }

    if (pcl::io::loadPCDFile<pcl::PointXYZINormal>(argv[2], *target_cloud) == -1)
    {
        std::cerr << "Couldn't read file " << argv[2] << std::endl;
        return -1;
    }

    pcl::PointCloud<pcl::PointXYZINormal> source_cloud_xyzinormal;
    double scale_factor = atof(argv[3]);
    for (size_t i = 0; i < source_cloud->points.size(); ++i)
    {
        pcl::PointXYZINormal p;
        p.x = source_cloud->points[i].x * scale_factor;
        p.y = source_cloud->points[i].y * scale_factor;
        p.z = source_cloud->points[i].z * scale_factor;
        p.intensity = 0;
        p.curvature = 0;
        source_cloud_xyzinormal.push_back(p);
    }

    // Eigen::Matrix4f transform = Eigen::Matrix4f::Identity();
    // transform.block<3, 3>(0, 0) = Eigen::AngleAxisf(M_PI_2 * 1.0, Eigen::Vector3f::UnitZ()).toRotationMatrix() * Eigen::AngleAxisf(-M_PI_2 * 1.3, Eigen::Vector3f::UnitY()).toRotationMatrix() * Eigen::AngleAxisf(0.2, Eigen::Vector3f::UnitX()).toRotationMatrix();
    // transform.block<3, 1>(0, 3) = Eigen::Vector3f(12.0, -44.0, -8.0);
    // pcl::transformPointCloud(source_cloud_xyzinormal, source_cloud_xyzinormal, transform);

    double voxel_size = 0.3;
    pcl::VoxelGrid<pcl::PointXYZINormal> voxel_grid;
    voxel_grid.setLeafSize(voxel_size, voxel_size, voxel_size);

    voxel_grid.setInputCloud(source_cloud_xyzinormal.makeShared());
    voxel_grid.filter(source_cloud_xyzinormal);

    voxel_grid.setInputCloud(target_cloud);
    voxel_grid.filter(*target_cloud);

    // GICP alignment

    Eigen::Matrix4f initial_guess = Eigen::Matrix4f::Identity();
    initial_guess.block<3, 3>(0, 0) = Eigen::AngleAxisf(M_PI_2 * 1.0, Eigen::Vector3f::UnitZ()).toRotationMatrix() * Eigen::AngleAxisf(-M_PI_2 * 1.3, Eigen::Vector3f::UnitY()).toRotationMatrix() * Eigen::AngleAxisf(0.2, Eigen::Vector3f::UnitX()).toRotationMatrix();
    initial_guess.block<3, 1>(0, 3) = Eigen::Vector3f(12.0, -44.0, -8.0);
    pcl::GeneralizedIterativeClosestPoint<pcl::PointXYZINormal, pcl::PointXYZINormal> gicp;
    gicp.setInputSource(source_cloud_xyzinormal.makeShared());
    gicp.setInputTarget(target_cloud);

    pcl::PointCloud<pcl::PointXYZINormal> aligned_cloud;
    gicp.align(aligned_cloud, initial_guess);

    std::cout << "GICP converged:" << gicp.hasConverged() << " score: " << gicp.getFitnessScore() << std::endl;
    std::cout << gicp.getFinalTransformation() << std::endl;

    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("3D Viewer"));
    // pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZINormal> source_cloud_color(source_cloud_xyzinormal.makeShared(), 255, 0, 0); // Red
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZINormal> target_cloud_color(target_cloud, 0, 255, 0);                         // Green
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZINormal> final_cloud_color(aligned_cloud.makeShared(), 0, 0, 255);            // Blue

    // viewer->addPointCloud<pcl::PointXYZINormal>(source_cloud_xyzinormal.makeShared(), source_cloud_color, "source cloud");
    viewer->addPointCloud<pcl::PointXYZINormal>(target_cloud, target_cloud_color, "target cloud");
    viewer->addPointCloud<pcl::PointXYZINormal>(aligned_cloud.makeShared(), final_cloud_color, "final cloud");

    // viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "source cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "target cloud");
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, "final cloud");

    viewer->addCoordinateSystem(1.0);
    viewer->initCameraParameters();

    while (!viewer->wasStopped())
    {
        viewer->spinOnce(100);
    }
    return 0;
}
