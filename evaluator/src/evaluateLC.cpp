#include "utility.h"
#include "nanoflann_pcl.h"
#include "debug_utility.h"
#include "descriptor/descriptor.h"

#include <opencv2/opencv.hpp>

#include <Eigen/Core>
#include <Eigen/Geometry>
#include <eigen3/Eigen/Dense>

#include <nav_msgs/Path.h>
#include <geometry_msgs/PoseStamped.h>

#include <unistd.h>


class LCEvaluator {

private:
    ros::NodeHandle n;

    // Parameter
    string descriptorType;
    double searchRadius;

    pcl::PointCloud<PointType>::Ptr pointCloudAll;
    pcl::PointCloud<pcl::PointXYZ>::Ptr poseCloud;
    nanoflann::KdTreeFLANN<pcl::PointXYZ> kdtreeLoopClosure;

    ros::Publisher pubPointTP;
    ros::Publisher pubPointFP;
    ros::Publisher pubPointFN;
    ros::Publisher pubPointTN;
    ros::Publisher pubPointTarget;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointTP;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointFP;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointFN;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointTN;
    pcl::PointCloud<pcl::PointXYZ>::Ptr pointTarget;
    TimeRecorder make_recorder;
    TimeRecorder match_recorder;
    vector<double> timesMake;
    vector<double> timesMatch;
    double TP, TN, FP, FN, precision, recall;

    SCManager scManager;
    ISCManager iscManager;
    SSC ssc;
    // "/home/wzc/ros_ws/tools_ws/src/eva_tools/evaluator/config/config_kitti.yaml"

public:

    LCEvaluator() {
        pointCloudAll.reset(new pcl::PointCloud<PointType>());
        poseCloud.reset(new pcl::PointCloud<pcl::PointXYZ>());
        pointTP.reset(new pcl::PointCloud<pcl::PointXYZ>());
        pointFP.reset(new pcl::PointCloud<pcl::PointXYZ>());
        pointFN.reset(new pcl::PointCloud<pcl::PointXYZ>());
        pointTN.reset(new pcl::PointCloud<pcl::PointXYZ>());
        pointTarget.reset(new pcl::PointCloud<pcl::PointXYZ>());
        pubPointTP = n.advertise<sensor_msgs::PointCloud2> ("/evaluator/point_cloud_TP", 2);
        pubPointFP = n.advertise<sensor_msgs::PointCloud2> ("/evaluator/point_cloud_FP", 2);
        pubPointFN = n.advertise<sensor_msgs::PointCloud2> ("/evaluator/point_cloud_FN", 2);
        pubPointTN = n.advertise<sensor_msgs::PointCloud2> ("/evaluator/point_cloud_TN", 2);
        pubPointTarget = n.advertise<sensor_msgs::PointCloud2> ("/evaluator/point_cloud_Target", 2);
        TP = 0;
        TN = 0;
        FP = 0;
        FN = 0;

        ros::NodeHandle nh("~");
        nh.param<string>("descriptor_type", descriptorType, "scan_context");
        nh.param<double>("search_radius", searchRadius, 5.0);
        cout << "descriptor type: " << descriptorType << endl;
        cout << "search radius: " << searchRadius << endl;
    }

    ~LCEvaluator() {
        precision = TP / (TP + FP);
        recall = TP / (TP + FN);
        cout << "###############################" << endl;
        cout << "Make descriptor time " << cal_average(timesMake) << " ms." << endl;
        cout << "Match place time  " << cal_average(timesMatch) << " ms." << endl;
        cout << "Precision: " << precision << endl;
        cout << "Recall: " << recall << endl;
        cout << "TP: " << TP << " FN: " << FN << " FP: " << FP << " TN: " << TN << endl;
    }

    double cal_average(std::vector<double> &data) {
        double sum = std::accumulate(data.begin(), data.end(), 0.0);
        return sum / data.size();
    }

    vector<double> evaluate(pcl::PointCloud<PointXYZIL>::Ptr &laserCloudAll, Eigen::Vector3d& t, float timestamp) {
        // 判断是否存在闭环
        bool isLoopClosure = false;
        pcl::PointXYZ cur_pose(t[0], t[1], t[2]);
        poseCloud->push_back(pcl::PointXYZ(t[0], t[1], t[2]));
        std::vector<int> pointSearchInd;
        std::vector<float> pointSearchSqDis;
        kdtreeLoopClosure.setInputCloud(poseCloud);
        kdtreeLoopClosure.radiusSearch(cur_pose, searchRadius, pointSearchInd, pointSearchSqDis);
        for (size_t i = 0; i < pointSearchInd.size(); ++i) {
            if (poseCloud->size() - pointSearchInd[i] > 200) {
                isLoopClosure = true;
                break;
            }
        }

        std::pair<int, float> detectResult;
        if (descriptorType == "scan_context") {
            // 构建描述子
            make_recorder.recordStart();
            scManager.makeAndSaveScancontextAndKeys(*laserCloudAll);
            timesMake.push_back(make_recorder.calculateDuration());

            // 闭环检测
            match_recorder.recordStart();
            detectResult = scManager.detectLoopClosureID(); // first: nn index, second: yaw diff
            timesMatch.push_back(match_recorder.calculateDuration());
        }
        else if (descriptorType == "intensity_scan_context") {
            pcl::PointCloud<pcl::PointXYZI>::Ptr laserCloudXYZI(new pcl::PointCloud<pcl::PointXYZI>());
            for (auto &p : laserCloudAll->points) {
                pcl::PointXYZI point;
                point.x = p.x;
                point.y = p.y;
                point.z = p.z;
                point.intensity = p.intensity;
                laserCloudXYZI->push_back(point);
            }
            Eigen::Isometry3d T1 = Eigen::Isometry3d::Identity();
            T1(0,3) = t[0];
            T1(1,3) = t[1];
            T1(2,3) = t[2];
            iscManager.loopDetection(laserCloudXYZI, T1);
            for(int i = 0; i < (int)iscManager.matched_frame_id.size(); i++){
                detectResult.first = iscManager.matched_frame_id[i];
                detectResult.second = 0.0;
            }
            
        }
        

        pcl::PointXYZ temp_p = poseCloud->points[detectResult.second];
        temp_p.z += 1.0;
        pointTarget->points.push_back(temp_p);

        bool detectLoopClosure = false;
        if (detectResult.first != -1) {
            if (poseCloud->size() -  detectResult.first > 200)
                detectLoopClosure = true;
        }

        if (isLoopClosure) {
            if (detectLoopClosure) {
                TP += 1;
                cur_pose.z += 1.0;
                pointTP->points.push_back(cur_pose);
            }
            else {
                FN += 1;
                cur_pose.z += 1.5;
                pointFN->points.push_back(cur_pose);
            }
        }
        else {
            if (detectLoopClosure) {
                FP += 1;
                pointFP->points.push_back(cur_pose);
            }
            else {
                pointTN->points.push_back(cur_pose);
                TN += 1;
            }                    
        }

        visualizeCloud(pointTP, pubPointTP, ros::Time::now(),"base_link");
        visualizeCloud(pointFP, pubPointFP, ros::Time::now(),"base_link");
        visualizeCloud(pointFN, pubPointFN, ros::Time::now(),"base_link");
        visualizeCloud(pointTN, pubPointTN, ros::Time::now(),"base_link");
        visualizeCloud(pointTarget, pubPointTarget, ros::Time::now(),"base_link");

        return vector<double>{TP, FP, FN, TN, timesMake.back(), timesMatch.back()};
    }
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
    ros::init(argc, argv, "evaluate_loop_closure");
    ros::NodeHandle n;
    ros::NodeHandle nh("~");

    LCEvaluator evaluator;

    ROS_INFO("\033[1;32m---->\033[0m Evaluate Loop Closure Started.");
    
    string dataset_folder;
    string sequence_number;

    nh.param<string>("dataset_folder", dataset_folder, "dataset_folder");
    nh.param<string>("sequence_number", sequence_number, "sequence_number");
    std::cout << "Reading sequence " << sequence_number << " from " << dataset_folder << endl;

    ros::Publisher pub_semantic_map = n.advertise<sensor_msgs::PointCloud2>("/map/semantic_map", 2);

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

    pcl::PointCloud<pcl::PointXYZRGB>::Ptr semantic_map(new pcl::PointCloud<pcl::PointXYZRGB>());

    int skip = 0;

    ros::Rate r(10.0 / 2.0);
    getchar();
    while (std::getline(timestamp_file, line) && ros::ok())
    {
        // skip++;
        // if (skip < 3000)
        //     continue;

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

        pcl::PointCloud<PointXYZIL>::Ptr laser_cloud(new pcl::PointCloud<PointXYZIL>());
        for (std::size_t i = 0; i < lidar_data.size() / 4; ++i)
        {
            PointXYZIL point;
            point.x = lidar_data[i * 4];
            point.y = lidar_data[i * 4 + 1];
            point.z = lidar_data[i * 4 + 2];

            float rate = 1.0;
            // float r = sqrt(point.x * point.x + point.y * point.y + point.z * point.z);
            // if (r > 35.0)  rate = r * r / 25.0 * 25.0;

            point.intensity = lidar_data[i * 4 + 3] * rate;
            point.label = label_data[i] & 0xffff;
            laser_cloud->push_back(point);
        }

        sensor_msgs::PointCloud2 laser_cloud_msg;
        pcl::toROSMsg(*laser_cloud, laser_cloud_msg);
        laser_cloud_msg.header.stamp = ros::Time().fromSec(timestamp);
        // laser_cloud_msg.header.stamp = ros::Time::now();
        laser_cloud_msg.header.frame_id = "/base_link";
        pub_laser_cloud.publish(laser_cloud_msg);

        line_num++;

        // vector<double> result = evaluator.evaluate(laser_cloud, t, timestamp);
        
        // cout << "scan: " << line_num << "    " << "totally " << laser_cloud->size() << " points.   "
        //     << "make_time: " << result[4] << "ms  match_time: " << result[5] << endl;

        cout << "scan: " << line_num << "    " << "totally " << laser_cloud->size() << " points.   " << endl;

        // usleep(20000);
        getchar();
    }

    return 0;
}

