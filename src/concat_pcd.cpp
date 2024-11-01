#include <ros/ros.h>
#include <Eigen/Eigen>
#include <pcl/common/transforms.h>

#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl_conversions/pcl_conversions.h>

#include <iostream>
#include <fstream>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Geometry>

struct LidarPose {
    double stamp;
    double tx, ty, tz;
    double qx, qy, qz, qw;
};

std::vector<LidarPose> loadLidarPoses(const std::string& filePath) {
    std::vector<LidarPose> poses;
    std::ifstream file(filePath);
    if (!file.is_open()) {
        std::cerr << "Failed to open file: " << filePath << std::endl;
        return poses;
    }

    double stamp, tx, ty, tz, qx, qy, qz, qw;
    while (file >> stamp >> tx >> ty >> tz >> qx >> qy >> qz >> qw) {
        LidarPose pose;
        pose.stamp = stamp;
        pose.tx = tx;
        pose.ty = ty;
        pose.tz = tz;
        pose.qx = qx;
        pose.qy = qy;
        pose.qz = qz;
        pose.qw = qw;
        poses.push_back(pose);
    }

    file.close();
    return poses;
}

int main(int argc, char** argv)
{
    if (argc < 3) {
        std::cerr << "Usage: " << argv[0] << " <lidar_poses_file> <output_pcd_file>" << std::endl;
        return 1;
    }

    const std::string lidarPosesFile = argv[1];
    const std::string pcd_file_path = argv[2];
    const std::string outputPCDFile = argv[3];

    const std::vector<LidarPose> poses = loadLidarPoses(lidarPosesFile);
    std::cout << "Loaded " << poses.size() << " lidar poses" << std::endl;

    pcl::PointCloud<pcl::PointXYZINormal> full_pc;
    full_pc.width = poses.size();
    full_pc.height = 1;
    full_pc.points.resize(full_pc.width * full_pc.height);

    for (size_t i = 0; i < poses.size(); ++i) {
        const LidarPose& pose = poses[i];
        Eigen::Quaterniond q(pose.qw, pose.qx, pose.qy, pose.qz);
        Eigen::Matrix3d R = q.toRotationMatrix();
        Eigen::Vector3d t(pose.tx, pose.ty, pose.tz);
        Eigen::Matrix4d T = Eigen::Matrix4d::Identity();
        T.block<3, 3>(0, 0) = R;
        T.block<3, 1>(0, 3) = t;
        pcl::PointCloud<pcl::PointXYZINormal> scan;
        pcl::io::loadPCDFile<pcl::PointXYZINormal>(pcd_file_path + "/scans_" + std::to_string(i) + ".pcd", scan);
        pcl::transformPointCloud(scan, scan, T);
        full_pc += scan;
    }

    pcl::io::savePCDFileBinary(outputPCDFile, full_pc);
    std::cout << "Saved " << full_pc.points.size() << " points to " << outputPCDFile << std::endl;

    return 0;
}
