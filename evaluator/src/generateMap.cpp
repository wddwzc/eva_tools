// #include "utility.h"
#include "debug_utility.h"
#include "nanoflann_pcl.h"

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <unistd.h>

class MapGenerator {

private:
    ros::NodeHandle n;

    ros::Publisher pub_semantic_map;

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr semantic_map;

public:
    MapGenerator() {
        semantic_map.reset(new pcl::PointCloud<pcl::PointXYZRGB>());

        pub_semantic_map = n.advertise<sensor_msgs::PointCloud2>("/map/semantic_map", 2);
    }

    // Generate semantic map
    void semantic(pcl::PointCloud<PointXYZIL> &laser_cloud,
                    Eigen::Quaterniond &q,
                    Eigen::Vector3d &t,
                    float &timestamp) {
        pcl::PointCloud<pcl::PointXYZRGB> temp_cloud;
        for (auto &p : laser_cloud.points) {
            if (p.label < 250) {
                int class_id = simple_classes_map[p.label];
                if (class_id == 0)  continue;
                // if (p.z > 2.0)  continue;
                if (sqrt(p.x*p.x+p.y*p.y+p.z*p.z)<1.5)  continue;
                pcl::PointXYZRGB cur_p;
                cur_p.x = p.x;
                cur_p.y = p.y;
                cur_p.z = p.z;
                cur_p.r = colors_map_simple[class_id][0];
                cur_p.g = colors_map_simple[class_id][1];
                cur_p.b = colors_map_simple[class_id][2];
                temp_cloud.push_back(cur_p);
            }
        }

        Eigen::Matrix4d T_matrix = Eigen::Matrix4d::Identity();
        T_matrix.block<3, 3>(0, 0) = q.matrix();
        T_matrix(0, 3) = t[0];
        T_matrix(1, 3) = t[1];
        T_matrix(2, 3) = t[2];
        // T_matrix(0, 3) = t[1];
        // T_matrix(1, 3) = -t[0];
        // T_matrix(2, 3) = t[2];
        pcl::transformPointCloud(temp_cloud, temp_cloud, T_matrix);
        
        *semantic_map += temp_cloud;
        
        pcl::VoxelGrid<pcl::PointXYZRGB> local_filter;
        local_filter.setLeafSize(0.5, 0.5, 0.5);
        local_filter.setInputCloud(semantic_map);
        local_filter.filter(*semantic_map);

        sensor_msgs::PointCloud2 tempLaserCloud;
        pcl::toROSMsg(*semantic_map, tempLaserCloud);
        tempLaserCloud.header.stamp = ros::Time().fromSec(timestamp);
        tempLaserCloud.header.frame_id = "map";
        pub_semantic_map.publish(tempLaserCloud);
    }


    /******************************************/
    // pcl::PointCloud<pcl::PointXYZRGB> temp_cloud;
    // for (auto &p : laser_cloud.points) {
    //     if (p.label < 250) {
    //         int class_id = simple_classes_map[p.label];
    //         if (class_id == 0)  continue;
    //         // if (p.z > 2.0)  continue;
    //         if (sqrt(p.x*p.x+p.y*p.y+p.z*p.z)<1.5)  continue;
    //         pcl::PointXYZRGB cur_p;
    //         cur_p.x = p.x;
    //         cur_p.y = p.y;
    //         cur_p.z = p.z;
    //         cur_p.r = colors_map_simple[class_id][0];
    //         cur_p.g = colors_map_simple[class_id][1];
    //         cur_p.b = colors_map_simple[class_id][2];
    //         temp_cloud.push_back(cur_p);
    //     }
    // }
    
    // sensor_msgs::PointCloud2 tempLaserCloud;
    // pcl::toROSMsg(temp_cloud, tempLaserCloud);
    // tempLaserCloud.header.stamp = ros::Time().fromSec(timestamp);
    // tempLaserCloud.header.frame_id = "map";
    // pub_semantic_map.publish(tempLaserCloud);
    /******************************************/

};


std::vector<float> read_lidar_data(const std::string lidar_data_path)
{
    std::ifstream lidar_data_file(lidar_data_path, std::ifstream::in | std::ifstream::binary);
    lidar_data_file.seekg(0, std::ios::end);
    const size_t num_elements = lidar_data_file.tellg() / sizeof(float);
    lidar_data_file.seekg(0, std::ios::beg);

    std::vector<float> lidar_data_buffer(num_elements);
    lidar_data_file.read(reinterpret_cast<char*>(&lidar_data_buffer[0]), num_elements*sizeof(float));
    return lidar_data_buffer;
}

std::vector<uint> read_label_data(const std::string label_data_path)
{
    std::ifstream label_data_file(label_data_path, std::ifstream::in | std::ifstream::binary);
    label_data_file.seekg(0, std::ios::end);
    const size_t num_elements = label_data_file.tellg() / sizeof(uint);
    label_data_file.seekg(0, std::ios::beg);

    std::vector<uint> label_data_buffer(num_elements);
    label_data_file.read(reinterpret_cast<char*>(&label_data_buffer[0]), num_elements*sizeof(uint));
    return label_data_buffer;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "generate_map");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    ROS_INFO("\033[1;32m---->\033[0m Generate Map Started.");
    
    MapGenerator mapGenerator;

    string dataset_folder;
    string sequence_number;

    nh.param<string>("dataset_folder", dataset_folder, "dataset_folder");
    nh.param<string>("sequence_number", sequence_number, "sequence_number");
    std::cout << "Reading sequence " << sequence_number << " from " << dataset_folder << endl;


    // ros::Publisher pub_laser_cloud = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 2);
    ros::Publisher pub_laser_cloud = n.advertise<sensor_msgs::PointCloud2>("/kitti/velo/pointall", 2);
    // ros::Publisher pubOdomGT = n.advertise<nav_msgs::Odometry> ("/odometry_gt", 5);
    ros::Publisher pubOdomGT = n.advertise<nav_msgs::Odometry> ("/kitti/velodyne_poses", 5);
    nav_msgs::Odometry odomGT;
    odomGT.header.frame_id = "/base_link";
    odomGT.child_frame_id = "/ground_truth";

    ros::Publisher pubPathGT = n.advertise<nav_msgs::Path> ("/path_gt", 5);
    nav_msgs::Path pathGT;
    pathGT.header.frame_id = "/base_link";


    std::string timestamp_path = "sequences/" + sequence_number + "/times.txt";
    std::ifstream timestamp_file(dataset_folder + timestamp_path, std::ifstream::in);

    std::string ground_truth_path = "poses_gt/" + sequence_number + "_gt.txt";
    std::ifstream ground_truth_file(dataset_folder + ground_truth_path, std::ifstream::in);

    cout << "File Path List:" << endl;
    cout << "Timestamps: " << dataset_folder + timestamp_path << endl;
    cout << "Groundtruth: " << dataset_folder + ground_truth_path << endl;

    std::string line;
    std::size_t line_num = 0;

    ros::Rate r(10.0 / 2.0);
    getchar();
    while (std::getline(timestamp_file, line) && ros::ok())
    {
        float timestamp = stof(line);

        // 读取真值
        std::getline(ground_truth_file, line);
        std::stringstream pose_stream(line);
        std::string s;
        Eigen::Matrix<double, 3, 4> gt_pose;
        for (std::size_t i = 0; i < 3; ++i)
        {
            for (std::size_t j = 0; j < 4; ++j)
            {
                std::getline(pose_stream, s, ' ');
                gt_pose(i, j) = stof(s);
            }
        }
        Eigen::Quaterniond q(gt_pose.topLeftCorner<3, 3>());
        q.normalize();
        Eigen::Vector3d t = gt_pose.topRightCorner<3, 1>();

        odomGT.header.stamp = ros::Time().fromSec(timestamp);
        // odomGT.header.stamp = ros::Time::now();
        odomGT.pose.pose.orientation.x = q.x();
        odomGT.pose.pose.orientation.y = q.y();
        odomGT.pose.pose.orientation.z = q.z();
        odomGT.pose.pose.orientation.w = q.w();
        odomGT.pose.pose.position.x = t(0);
        odomGT.pose.pose.position.y = t(1);
        odomGT.pose.pose.position.z = t(2);
        pubOdomGT.publish(odomGT);

        geometry_msgs::PoseStamped poseGT;
        poseGT.header = odomGT.header;
        poseGT.pose = odomGT.pose.pose;
        pathGT.header.stamp = odomGT.header.stamp;
        pathGT.poses.push_back(poseGT);
        pubPathGT.publish(pathGT);


        // read lidar point cloud and label
        std::stringstream lidar_data_path;
        lidar_data_path << dataset_folder << "sequences/" + sequence_number + "/velodyne/" 
                        << std::setfill('0') << std::setw(6) << line_num << ".bin";
        std::vector<float> lidar_data = read_lidar_data(lidar_data_path.str());
        std::stringstream label_data_path;
        label_data_path << dataset_folder << "sequences/" + sequence_number + "/labels/" 
                        << std::setfill('0') << std::setw(6) << line_num << ".label";
        std:vector<uint> label_data = read_label_data(label_data_path.str());

        pcl::PointCloud<PointXYZIL> laser_cloud;
        for (std::size_t i = 0; i < lidar_data.size() / 4; ++i)
        {
            PointXYZIL point;
            point.x = lidar_data[i * 4];
            point.y = lidar_data[i * 4 + 1];
            point.z = lidar_data[i * 4 + 2];
            point.intensity = lidar_data[i * 4 + 3];
            point.label = label_data[i] & 0xffff;
            laser_cloud.push_back(point);
        }

        sensor_msgs::PointCloud2 laser_cloud_msg;
        pcl::toROSMsg(laser_cloud, laser_cloud_msg);
        laser_cloud_msg.header.stamp = ros::Time().fromSec(timestamp);
        // laser_cloud_msg.header.stamp = ros::Time::now();
        laser_cloud_msg.header.frame_id = "/base_link";
        pub_laser_cloud.publish(laser_cloud_msg);

        line_num++;
        
        cout << "scan: " << line_num << "    " << "totally " << laser_cloud.size() << " points. " << endl;

        mapGenerator.semantic(laser_cloud, q, t, timestamp);

        // usleep(200000);
        // getchar();
    }

    return 0;
}

