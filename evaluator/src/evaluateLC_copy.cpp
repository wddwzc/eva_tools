#include "utility.h"
#include "LCprovider.hpp"

#include <Eigen/Core>
#include <Eigen/Geometry>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosgraph_msgs/Clock.h>

class LCEvaluator {

private:

    ros::NodeHandle nh;

    ros::Subscriber subPointCloud;
    ros::Subscriber subPointLabel;
    ros::Subscriber subLaserOdom;

    pcl::PointCloud<pcl::PointXYZI>::Ptr pointCloud;
    pcl::PointCloud<pcl::PointXYZL>::Ptr pointLabel;
    pcl::PointCloud<PointType>::Ptr pointCloudAll;
    geometry_msgs::Point geoPose;
    geometry_msgs::Quaternion geoQuat;
    std_msgs::Header currentHeader;
    double timePointCloud;
    double timePointLabel;
    double timeLaserOdomLast;
    bool newPointCloud;
    bool newPointLabel;
    bool newLaserOdomLast;

    Eigen::Matrix4d global_pose;

    SCManager scManager;

public:

    LCEvaluator() {

        subPointCloud = nh.subscribe<sensor_msgs::PointCloud2>("/kitti/velo/none", 2, &LCEvaluator::pointCloudHandler, this);
        subPointLabel = nh.subscribe<sensor_msgs::PointCloud2>("/kitti/velo/none", 2, &LCEvaluator::pointLabelHandler, this);
        subLaserOdom = nh.subscribe<nav_msgs::Odometry>("/kitti/velodyne_poses", 5, &LCEvaluator::laserOdometryHandler, this);

        pointCloudAll.reset(new pcl::PointCloud<PointType>());
        pointCloud.reset(new pcl::PointCloud<pcl::PointXYZI>());
        pointLabel.reset(new pcl::PointCloud<pcl::PointXYZL>());

        timePointCloud = 0.0;
        timePointLabel = 0.0;
        timeLaserOdomLast = 0.0;
        newPointCloud = false;
        newPointLabel = false;
        newLaserOdomLast = false;

        global_pose = Eigen::Matrix4d::Identity();
    }

    void resetMemory() {
        pointCloud->clear();
        pointLabel->clear();
        pointCloudAll->clear();
    }

    void pointCloudHandler(const sensor_msgs::PointCloud2::ConstPtr& laserCloud) {
        currentHeader = laserCloud->header;

        timePointCloud = currentHeader.stamp.toSec();
        pointCloud->clear();
        pcl::fromROSMsg(*laserCloud, *pointCloud);
        newPointCloud = true;

        ROS_INFO("get new cloud.");
    }

    void pointLabelHandler(const sensor_msgs::PointCloud2::ConstPtr& laserCloud) {
        currentHeader = laserCloud->header;

        timePointLabel = currentHeader.stamp.toSec();
        pointLabel->clear();
        pcl::fromROSMsg(*laserCloud, *pointLabel);
        newPointLabel = true;

        ROS_INFO("get new label.");
    }

    void laserOdometryHandler(const nav_msgs::Odometry::ConstPtr& laserOdometry)
    {
        currentHeader = laserOdometry->header;

        timeLaserOdomLast = currentHeader.stamp.toSec();
        geoPose = laserOdometry->pose.pose.position;
        geoQuat = laserOdometry->pose.pose.orientation;
        newLaserOdomLast = true;

        ROS_INFO("get new odometry.");
    }



    void evaluate() {
        if (newPointCloud && newPointLabel && newLaserOdomLast && std::abs(timePointCloud - timeLaserOdomLast) < 0.005 && std::abs(timePointLabel - timeLaserOdomLast))
        {
            auto detectResult = scManager.detectLoopClosureID(); // first: nn index, second: yaw diff 
            scManager.makeAndSaveScancontextAndKeys(*pointCloudAll);
            
            newLaserOdomLast = false;
            newPointCloud = false;
            newPointLabel = false;
        }

        resetMemory();
    }
};


int main(int argc, char** argv)
{
    ros::init(argc, argv, "evaluate_loop_closure");
    
    ros::NodeHandle nh("~");

    LCEvaluator evaluator;

    ROS_INFO("\033[1;32m---->\033[0m Evaluate Loop Closure Started.");

    
    string rosBag;
    string poseTopic;
    string lidarTopic;
    string labelTopic;
    string descriptorType;
    nh.param<string>("rosbag", rosBag, "00.bag");
    nh.param<string>("pose_topic", poseTopic, "/kitti/velodyne_poses");
    nh.param<string>("lidar_topic", lidarTopic, "/kitti/velo/point");
    nh.param<string>("label_topic", labelTopic, "/kitti/velo/pointlabel");
    nh.param<string>("descriptor_type", descriptorType, "sc");

    cout << "### Parameter list ###" << endl;
    cout << "rosbag path: " << rosBag << endl;
    cout << "pose topic: " << poseTopic << endl;
    cout << "lidar topic: " << lidarTopic << endl;
    cout << "label topic: " << labelTopic << endl;
    cout << "descriptor type: " << descriptorType << endl;

    rosbag::Bag bag;
    if (!rosBag.empty()) {
        try {
            bag.open(rosBag, rosbag::bagmode::Read);
        } catch (std::exception& ex) {
            ROS_FATAL("Unable to open rosbag [%s]", rosBag.c_str());
            return 1;
        }
    }
    std::vector<std::string> topics;
    topics.push_back(lidarTopic);
    topics.push_back(labelTopic);
    topics.push_back(poseTopic);
    rosbag::View view(bag, rosbag::TopicQuery(topics));

    // auto clock_publisher = nh.advertise<rosgraph_msgs::Clock>("/clock",1);

    for(const rosbag::MessageInstance& m: view)
    {
        const sensor_msgs::PointCloud2ConstPtr &cloud = m.instantiate<sensor_msgs::PointCloud2>();
        if (cloud != NULL) {
            evaluator.pointCloudHandler(cloud);
            // ROS_INFO("cloud");
        }

        const sensor_msgs::PointCloud2ConstPtr &label = m.instantiate<sensor_msgs::PointCloud2>();
        if (label != NULL) {
            evaluator.pointLabelHandler(label);
            // ROS_INFO("label");
        }

        const nav_msgs::Odometry::ConstPtr &odom = m.instantiate<nav_msgs::Odometry>();
        if (odom != NULL) {
            evaluator.laserOdometryHandler(odom);
            // ROS_INFO("odom");
        }




        // rosgraph_msgs::Clock clock_msg;
        // clock_msg.clock = m.getTime();
        // clock_publisher.publish(clock_msg);

        ros::spinOnce();
    }

    bag.close();

    return 0;
}




#include <iostream>
#include <fstream>
#include <iterator>
#include <string>
#include <vector>
#include <opencv2/opencv.hpp>
#include <image_transport/image_transport.h>
#include <opencv2/highgui/highgui.hpp>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <ros/ros.h>
#include <rosbag/bag.h>
#include <geometry_msgs/PoseStamped.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <eigen3/Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl_conversions/pcl_conversions.h>
#include <sensor_msgs/PointCloud2.h>

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

int main(int argc, char** argv)
{
    ros::init(argc, argv, "kitti_helper");
    ros::NodeHandle n("~");
    std::string dataset_folder, sequence_number, output_bag_file;
    n.getParam("dataset_folder", dataset_folder);
    n.getParam("sequence_number", sequence_number);
    std::cout << "Reading sequence " << sequence_number << " from " << dataset_folder << '\n';
    bool to_bag;
    n.getParam("to_bag", to_bag);
    if (to_bag)
        n.getParam("output_bag_file", output_bag_file);
    int publish_delay;
    n.getParam("publish_delay", publish_delay);
    publish_delay = publish_delay <= 0 ? 1 : publish_delay;

    ros::Publisher pub_laser_cloud = n.advertise<sensor_msgs::PointCloud2>("/velodyne_points", 2);

    image_transport::ImageTransport it(n);
    image_transport::Publisher pub_image_left = it.advertise("/image_left", 2);
    image_transport::Publisher pub_image_right = it.advertise("/image_right", 2);

    ros::Publisher pubOdomGT = n.advertise<nav_msgs::Odometry> ("/odometry_gt", 5);
    nav_msgs::Odometry odomGT;
    odomGT.header.frame_id = "/camera_init";
    odomGT.child_frame_id = "/ground_truth";

    ros::Publisher pubPathGT = n.advertise<nav_msgs::Path> ("/path_gt", 5);
    nav_msgs::Path pathGT;
    pathGT.header.frame_id = "/camera_init";

    std::string timestamp_path = "sequences/" + sequence_number + "/times.txt";
    std::ifstream timestamp_file(dataset_folder + timestamp_path, std::ifstream::in);

    std::string ground_truth_path = "results/" + sequence_number + ".txt";
    std::ifstream ground_truth_file(dataset_folder + ground_truth_path, std::ifstream::in);

    rosbag::Bag bag_out;
    if (to_bag)
        bag_out.open(output_bag_file, rosbag::bagmode::Write);
    
    Eigen::Matrix3d R_transform;
    R_transform << 0, 0, 1, -1, 0, 0, 0, -1, 0;
    Eigen::Quaterniond q_transform(R_transform);

    std::string line;
    std::size_t line_num = 0;

    ros::Rate r(10.0 / publish_delay);
    while (std::getline(timestamp_file, line) && ros::ok())
    {
        float timestamp = stof(line);
        std::stringstream left_image_path, right_image_path;
        left_image_path << dataset_folder << "sequences/" + sequence_number + "/image_0/" << std::setfill('0') << std::setw(6) << line_num << ".png";
        cv::Mat left_image = cv::imread(left_image_path.str(), CV_LOAD_IMAGE_GRAYSCALE);
        right_image_path << dataset_folder << "sequences/" + sequence_number + "/image_1/" << std::setfill('0') << std::setw(6) << line_num << ".png";
        cv::Mat right_image = cv::imread(left_image_path.str(), CV_LOAD_IMAGE_GRAYSCALE);

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

        Eigen::Quaterniond q_w_i(gt_pose.topLeftCorner<3, 3>());
        Eigen::Quaterniond q = q_transform * q_w_i;
        q.normalize();
        Eigen::Vector3d t = q_transform * gt_pose.topRightCorner<3, 1>();

        odomGT.header.stamp = ros::Time().fromSec(timestamp);
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

        // read lidar point cloud
        std::stringstream lidar_data_path;
        lidar_data_path << dataset_folder << "velodyne/sequences/" + sequence_number + "/velodyne/" 
                        << std::setfill('0') << std::setw(6) << line_num << ".bin";
        std::vector<float> lidar_data = read_lidar_data(lidar_data_path.str());
        std::cout << "totally " << lidar_data.size() / 4.0 << " points in this lidar frame \n";

        std::vector<Eigen::Vector3d> lidar_points;
        std::vector<float> lidar_intensities;
        pcl::PointCloud<pcl::PointXYZI> laser_cloud;
        for (std::size_t i = 0; i < lidar_data.size(); i += 4)
        {
            lidar_points.emplace_back(lidar_data[i], lidar_data[i+1], lidar_data[i+2]);
            lidar_intensities.push_back(lidar_data[i+3]);

            pcl::PointXYZI point;
            point.x = lidar_data[i];
            point.y = lidar_data[i + 1];
            point.z = lidar_data[i + 2];
            point.intensity = lidar_data[i + 3];
            laser_cloud.push_back(point);
        }

        sensor_msgs::PointCloud2 laser_cloud_msg;
        pcl::toROSMsg(laser_cloud, laser_cloud_msg);
        laser_cloud_msg.header.stamp = ros::Time().fromSec(timestamp);
        laser_cloud_msg.header.frame_id = "/camera_init";
        pub_laser_cloud.publish(laser_cloud_msg);

        sensor_msgs::ImagePtr image_left_msg = cv_bridge::CvImage(laser_cloud_msg.header, "mono8", left_image).toImageMsg();
        sensor_msgs::ImagePtr image_right_msg = cv_bridge::CvImage(laser_cloud_msg.header, "mono8", right_image).toImageMsg();
        pub_image_left.publish(image_left_msg);
        pub_image_right.publish(image_right_msg);

        if (to_bag)
        {
            bag_out.write("/image_left", ros::Time::now(), image_left_msg);
            bag_out.write("/image_right", ros::Time::now(), image_right_msg);
            bag_out.write("/velodyne_points", ros::Time::now(), laser_cloud_msg);
            bag_out.write("/path_gt", ros::Time::now(), pathGT);
            bag_out.write("/odometry_gt", ros::Time::now(), odomGT);
        }

        line_num ++;
        r.sleep();
    }
    bag_out.close();
    std::cout << "Done \n";


    return 0;
}