#include <iostream>
#include <iomanip>
#include <fstream>
#include <sstream>
#include <string>
#include <vector>
#include <Eigen/Core>
#include <Eigen/Dense>

struct Pose
{
    double timestamp;
    Eigen::Vector3d translation;
    Eigen::Quaterniond quaternion;
};

std::vector<Pose> readPoses(const std::string &filename)
{
    std::vector<Pose> poses;
    std::ifstream file(filename);
    std::string line;

    while (std::getline(file, line))
    {
        std::istringstream iss(line);
        Pose pose;
        iss >> pose.timestamp >> pose.translation.x() >> pose.translation.y() >> pose.translation.z() >> pose.quaternion.x() >> pose.quaternion.y() >> pose.quaternion.z() >> pose.quaternion.w();
        poses.push_back(pose);
    }

    return poses;
}

void writePosesToTUMFile(const std::string &filename, const std::vector<Pose> &poses)
{
    std::ofstream file(filename);

    if (!file.is_open())
    {
        std::cerr << "Error: Unable to open file for writing" << std::endl;
        return;
    }

    for (const Pose &pose : poses)
    {
        file << std::fixed << std::setprecision(6) << pose.timestamp << " ";
        file << pose.translation.x() << " " << pose.translation.y() << " " << pose.translation.z() << " ";
        file << pose.quaternion.x() << " " << pose.quaternion.y() << " " << pose.quaternion.z() << " " << pose.quaternion.w() << std::endl;
    }

    file.close();
}

// -0.00148582   -0.989959    0.141349     7.81839
//   -0.415944   -0.127929   -0.900347    -46.0068
//    0.909389  -0.0601309   -0.411577    -5.75727
//           0           0           0           1

int main(int argc, char **argv)
{
    if (argc < 3)
    {
        std::cerr << "Usage: " << argv[0] << " <poses_file> <output_file>" << std::endl;
        return 1;
    }
    std::vector<Pose> poses = readPoses(argv[1]);

    Eigen::Matrix3d transR;
    transR << -0.00148582, -0.989959, 0.141349, -0.415944, -0.127929, -0.900347, 0.909389, -0.0601309, -0.411577;
    Eigen::Vector3d transt(7.81839, -46.0068, -5.75727);
    double scale = 7.4;

    // Now you can access the poses and their components, e.g., timestamps, translations, quaternions
    for (Pose& pose : poses) {
        Eigen::Matrix3d R = pose.quaternion.toRotationMatrix().transpose();
        Eigen::Vector3d t = - R * pose.translation * scale;
        pose.quaternion = Eigen::Quaterniond(transR * R);
        pose.translation = transR * t + transt;   
    }

    std::cout << poses.size() << " poses read from file" << std::endl;
    writePosesToTUMFile(argv[2], poses);
    return 0;
}
