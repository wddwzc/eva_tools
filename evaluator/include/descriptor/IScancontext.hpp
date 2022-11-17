#ifndef _ISC_GENERATION_CLASS_H_
#define _ISC_GENERATION_CLASS_H_

#include "utility.h"

#include <iostream>
#include <vector>
#include <string>
#include <algorithm>
#include <numeric>
#include <cmath>

//PCL
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>

//opencv
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>

//ros
#include <ros/ros.h>

//pcl
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/filters/filter.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/filters/crop_box.h>
#include <Eigen/Dense>
#include <Eigen/Core>
#include <Eigen/Geometry>


//#define INTEGER_INTENSITY
typedef cv::Mat ISCDescriptor;


class ISCManager
{
public:
    ISCManager();
    void init_param(int rings_in, int sectors_in, double max_dis_in);

    ISCDescriptor getLastISCMONO(void);
    ISCDescriptor getLastISCRGB(void);
    void loopDetection(const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_pc, Eigen::Isometry3d& odom);
    
    int current_frame_id;
    std::vector<int> matched_frame_id;

private:
    bool INTEGER_INTENSITY;
    int rings = 20;
    int sectors = 90;
    double ring_step=0.0;
    double sector_step=0.0;
    double max_dis = 60;

    //IF TRAVELLED DISTANCE IS LESS THAN THIS VALUE, SKIP FOR PLACE RECOGNTION
    double SKIP_NEIBOUR_DISTANCE = 20.0;
    //how much error will odom generate per frame 
    double INFLATION_COVARIANCE = 0.03;
    //define threshold for loop closure detection
    double GEOMETRY_THRESHOLD = 0.67;
    double INTENSITY_THRESHOLD = 0.91;

    std::vector<cv::Vec3b> color_projection;
    
    pcl::PointCloud<pcl::PointXYZI>::Ptr current_point_cloud;
    pcl::PointCloud<pcl::PointXYZI>::Ptr test_pc;

    std::vector<Eigen::Vector3d> pos_arr;
    std::vector<double> travel_distance_arr;
    std::vector<ISCDescriptor> isc_arr;

    void init_color(void);
    void print_param(void);
    bool is_loop_pair(ISCDescriptor& desc1, ISCDescriptor& desc2, double& geo_score, double& inten_score);
    ISCDescriptor calculate_isc(const pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pointcloud);
    double calculate_geometry_dis(const ISCDescriptor& desc1, const ISCDescriptor& desc2, int& angle);
    double calculate_intensity_dis(const ISCDescriptor& desc1, const ISCDescriptor& desc2, int& angle);
    void ground_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out);
};



ISCManager::ISCManager()
{
    ros::NodeHandle nh;
    nh.param<bool>("/descriptor/intensity_scan_context/integer_intensity", INTEGER_INTENSITY, false);
    nh.param<int>("/descriptor/intensity_scan_context/num_ring", rings, 20);
    nh.param<int>("/descriptor/intensity_scan_context/num_sector", sectors, 90);
    nh.param<double>("/descriptor/intensity_scan_context/ring_step", ring_step, 0.0);
    nh.param<double>("/descriptor/intensity_scan_context/sector_step", sector_step, 0.0);
    nh.param<double>("/descriptor/intensity_scan_context/max_dis", max_dis, 60);

    nh.param<double>("/descriptor/intensity_scan_context/skip_neibour_distance", SKIP_NEIBOUR_DISTANCE, 20.0);
    nh.param<double>("/descriptor/intensity_scan_context/inflation_covariance", INFLATION_COVARIANCE, 0.03);
    nh.param<double>("/descriptor/intensity_scan_context/geometry_threshold", GEOMETRY_THRESHOLD, 0.67);
    nh.param<double>("/descriptor/intensity_scan_context/intensity_threshold", INTENSITY_THRESHOLD, 0.91);

}

void ISCManager::init_param(int rings_in, int sectors_in, double max_dis_in){
    rings = rings_in;
    sectors = sectors_in;
    max_dis = max_dis_in;
    ring_step = max_dis/rings;
    sector_step = 2*M_PI/sectors;
    print_param();
    init_color();

    current_point_cloud = pcl::PointCloud<pcl::PointXYZI>::Ptr(new pcl::PointCloud<pcl::PointXYZI>());
}

void ISCManager::init_color(void){
    for(int i=0;i<1;i++){//RGB format
        color_projection.push_back(cv::Vec3b(0,i*16,255));
    }
    for(int i=0;i<15;i++){//RGB format
        color_projection.push_back(cv::Vec3b(0,i*16,255));
    }
    for(int i=0;i<16;i++){//RGB format
        color_projection.push_back(cv::Vec3b(0,255,255-i*16));
    }
    for(int i=0;i<32;i++){//RGB format
        color_projection.push_back(cv::Vec3b(i*32,255,0));
    }
    for(int i=0;i<16;i++){//RGB format
        color_projection.push_back(cv::Vec3b(255,255-i*16,0));
    }
    for(int i=0;i<64;i++){//RGB format
        color_projection.push_back(cv::Vec3b(i*4,255,0));
    }
    for(int i=0;i<64;i++){//RGB format
        color_projection.push_back(cv::Vec3b(255,255-i*4,0));
    }
    for(int i=0;i<64;i++){//RGB format
        color_projection.push_back(cv::Vec3b(255,i*4,i*4));
    }
}

void ISCManager::print_param(){
    std::cout << "The ISC parameters are:"<<rings<<std::endl;
    std::cout << "number of rings:\t"<<rings<<std::endl;
    std::cout << "number of sectors:\t"<<sectors<<std::endl;
    std::cout << "maximum distance:\t"<<max_dis<<std::endl;
}

ISCDescriptor ISCManager::calculate_isc(const pcl::PointCloud<pcl::PointXYZI>::Ptr filtered_pointcloud){
    ISCDescriptor isc = cv::Mat::zeros(cv::Size(sectors,rings), CV_8U);

    for(int i=0;i<(int)filtered_pointcloud->points.size();i++){
        ROS_WARN_ONCE("intensity is %f, if intensity showed here is integer format between 1-255, please uncomment #define INTEGER_INTENSITY in ISCManager.cpp and recompile", (double) filtered_pointcloud->points[i].intensity);
        double distance = std::sqrt(filtered_pointcloud->points[i].x * filtered_pointcloud->points[i].x + filtered_pointcloud->points[i].y * filtered_pointcloud->points[i].y);
        if(distance>=max_dis)
            continue;
        double angle = M_PI + std::atan2(filtered_pointcloud->points[i].y,filtered_pointcloud->points[i].x);
        int ring_id = std::floor(distance/ring_step);
        int sector_id = std::floor(angle/sector_step);
        if(ring_id>=rings)
            continue;
        if(sector_id>=sectors)
            continue;
        
        int intensity_temp;
        if (INTEGER_INTENSITY)
            intensity_temp = (int) (filtered_pointcloud->points[i].intensity);
        else
            intensity_temp = (int) (255*filtered_pointcloud->points[i].intensity);

        if(isc.at<unsigned char>(ring_id,sector_id)<intensity_temp)
            isc.at<unsigned char>(ring_id,sector_id)=intensity_temp;

    }

    return isc;

}


ISCDescriptor ISCManager::getLastISCMONO(void){
    return isc_arr.back();

}

ISCDescriptor ISCManager::getLastISCRGB(void){
    //ISCDescriptor isc = isc_arr.back();
    ISCDescriptor isc_color = cv::Mat::zeros(cv::Size(sectors,rings), CV_8UC3);
    for (int i = 0;i < isc_arr.back().rows;i++) {
        for (int j = 0;j < isc_arr.back().cols;j++) {
            isc_color.at<cv::Vec3b>(i, j) = color_projection[isc_arr.back().at<unsigned char>(i,j)];

        }
    }
    return isc_color;
}

void ISCManager::loopDetection(const pcl::PointCloud<pcl::PointXYZI>::Ptr& current_pc, Eigen::Isometry3d& odom){

    pcl::PointCloud<pcl::PointXYZI>::Ptr pc_filtered(new pcl::PointCloud<pcl::PointXYZI>());
    ground_filter(current_pc, pc_filtered);
    ISCDescriptor desc = calculate_isc(pc_filtered);
    Eigen::Vector3d current_t = odom.translation();
    //dont change push_back sequence
    if(travel_distance_arr.size()==0){
        travel_distance_arr.push_back(0);
    }else{
        double dis_temp = travel_distance_arr.back()+std::sqrt((pos_arr.back()-current_t).array().square().sum());
        travel_distance_arr.push_back(dis_temp);
    }
    pos_arr.push_back(current_t);
    isc_arr.push_back(desc);

    current_frame_id = pos_arr.size()-1;
    matched_frame_id.clear();
    //search for the near neibourgh pos
    int best_matched_id=0;
    double best_score=0.0;
    for(int i = 0; i< (int)pos_arr.size(); i++){
        double delta_travel_distance = travel_distance_arr.back()- travel_distance_arr[i];
        double pos_distance = std::sqrt((pos_arr[i]-pos_arr.back()).array().square().sum());
        if(delta_travel_distance > SKIP_NEIBOUR_DISTANCE && pos_distance<delta_travel_distance*INFLATION_COVARIANCE){
            double geo_score=0;
            double inten_score =0;
            if(is_loop_pair(desc,isc_arr[i],geo_score,inten_score)){
                if(geo_score+inten_score>best_score){
                    best_score = geo_score+inten_score;
                    best_matched_id = i;
                }
            }

        }
    }
    if(best_matched_id != 0) {
        matched_frame_id.push_back(best_matched_id);
        //ROS_INFO("received loop closure candidate: current: %d, history %d, total_score%f",current_frame_id,best_matched_id,best_score);
    }

}

bool ISCManager::is_loop_pair(ISCDescriptor& desc1, ISCDescriptor& desc2, double& geo_score, double& inten_score){
    int angle =0;
    geo_score = calculate_geometry_dis(desc1,desc2,angle);
    if(geo_score>GEOMETRY_THRESHOLD){
        inten_score = calculate_intensity_dis(desc1,desc2,angle);
        if(inten_score>INTENSITY_THRESHOLD){
            return true;
        }
    }
    return false;
}

double ISCManager::calculate_geometry_dis(const ISCDescriptor& desc1, const ISCDescriptor& desc2, int& angle){
    double similarity = 0.0;

    for(int i=0;i<sectors;i++){
        int match_count=0;
        for(int p=0;p<sectors;p++){
            int new_col = p+i>=sectors?p+i-sectors:p+i;
            for(int q=0;q<rings;q++){
                if((desc1.at<unsigned char>(q,p)== true && desc2.at<unsigned char>(q,new_col)== true) || (desc1.at<unsigned char>(q,p)== false && desc2.at<unsigned char>(q,new_col)== false)){
                    match_count++;
                }

            }
        }
        if(match_count>similarity){
            similarity=match_count;
            angle = i;
        }

    }
    return similarity/(sectors*rings);
    
}
double ISCManager::calculate_intensity_dis(const ISCDescriptor& desc1, const ISCDescriptor& desc2, int& angle){
    double difference = 1.0;
    double angle_temp = angle;
    for(int i=angle_temp-10;i<angle_temp+10;i++){

        int match_count=0;
        int total_points=0;
        for(int p=0;p<sectors;p++){
            int new_col = p+i;
            if(new_col>=sectors)
                new_col = new_col-sectors;
            if(new_col<0)
                new_col = new_col+sectors;
            for(int q=0;q<rings;q++){
                    match_count += abs(desc1.at<unsigned char>(q,p)-desc2.at<unsigned char>(q,new_col));
                    total_points++;
            }
            
        }
        double diff_temp = ((double)match_count)/(sectors*rings*255);
        if(diff_temp<difference)
            difference=diff_temp;

    }
    return 1 - difference;
    
}
void ISCManager::ground_filter(const pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_in, pcl::PointCloud<pcl::PointXYZI>::Ptr& pc_out){
    pcl::PassThrough<pcl::PointXYZI> pass;
    pass.setInputCloud (pc_in);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (-0.9, 30.0);
    pass.filter (*pc_out);

}

#endif

