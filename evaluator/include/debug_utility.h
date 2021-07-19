#ifndef _DEBUG_UTILITY_H_
#define _DEBUG_UTILITY_H_

#include "utility.h"
#include "kitti_label_color.h"

// record time
class TimeRecorder {
public:
    TimeRecorder() {
        begin = ros::Time::now();
        duration_ms = 0.0;
    }

    ros::Time recordStart() {
        begin = ros::Time::now();
        return begin;
    }
    
    double calculateDuration() {
        duration_ms = (ros::Time::now() - begin).toSec() * 1000;
        return duration_ms;
    }

    void printDuration(string task_name)
    {
        double duration_ms = (ros::Time::now() - begin).toSec() * 1000;

        // std::cout.precision(3); // 10 for sec, 3 for ms 
        std::cout << task_name << ": " << duration_ms << " ms." << std::endl;
    }

private:
    ros::Time begin;
    double duration_ms;
};

void visualizeCloudRGB(pcl::PointCloud<PointType>::Ptr laserCloud, ros::Publisher &pub,
                        ros::Time time, string frame_id) {
    if (pub.getNumSubscribers() != 0) {
        pcl::PointCloud<pcl::PointXYZRGB> laserCloudColor;
        for (auto &p : laserCloud->points) {
            pcl::PointXYZRGB cur_p;
            cur_p.x = p.x;
            cur_p.y = p.y;
            cur_p.z = p.z;
            int class_id = p.label;
            cur_p.r = colors_map_tran[class_id][0];
            cur_p.g = colors_map_tran[class_id][1];
            cur_p.b = colors_map_tran[class_id][2];
            laserCloudColor.push_back(cur_p);
        }
        sensor_msgs::PointCloud2 tempLaserCloud;
        pcl::toROSMsg(laserCloudColor, tempLaserCloud);
        tempLaserCloud.header.stamp = time;
        tempLaserCloud.header.frame_id = frame_id;
        pub.publish(tempLaserCloud);
    }
}

void visualizeCloud(pcl::PointCloud<pcl::PointXYZ>::Ptr laserCloud, ros::Publisher &pub,
                    ros::Time time, string frame_id) {
    if (pub.getNumSubscribers() != 0) {
        sensor_msgs::PointCloud2 tempLaserCloud;
        pcl::toROSMsg(*laserCloud, tempLaserCloud);
        tempLaserCloud.header.stamp = time;
        tempLaserCloud.header.frame_id = frame_id;
        pub.publish(tempLaserCloud);
    }
}




#endif


        // for (auto &segment_to_add : cluster_indices) {
        //     unsigned int sz = segment_to_add.indices.size();

        //     if (param.debugOctomapCluster) cout << "cur_segment_size: " << sz << endl;

            

        //     pcl::PointXYZRGB centroid(0.0, 0.0, 0.0);
        //     vector<unsigned int> class_counts(26, 0);
        //     for (auto &index : segment_to_add.indices) {
        //         pcl::PointXYZL &cur_point = laserCloud_noGround->points[index];
        //         unsigned class_id = classes_map[cur_point.label];
        //         ++class_counts[class_id];
        //         centroid.x += cur_point.x;
        //         centroid.y += cur_point.y;
        //         centroid.z += cur_point.z;
        //     }
        //     centroid.x /= (float)sz;
        //     centroid.y /= (float)sz;
        //     centroid.z /= (float)sz;

        //     unsigned int segment_class = 0;
        //     for (int i = 0; i < 25; ++i) {
        //         if (class_counts[i] / (float)sz >= 0.6)
        //             segment_class = i;
        //     }
        //     centroid.r = colors_map_tran[segment_class][0];
        //     centroid.g = colors_map_tran[segment_class][1];
        //     centroid.b = colors_map_tran[segment_class][2];

        //     // add marker
        //     geometry_msgs::Point marker_p;
        //     marker_p.x = centroid.x;
        //     marker_p.y = centroid.y;
        //     marker_p.z = centroid.z;
        //     marker_Centroid2Seg.points.push_back(marker_p);
        //     marker_p.z += 10.0;
        //     marker_Centroid2Seg.points.push_back(marker_p);

        //     classifiedCentroidRGB->push_back(centroid);
        //     centroid.z += 10.0;
        //     classifiedCentroidRGB->push_back(centroid);

        //     for (auto &index : segment_to_add.indices) {
        //         pcl::PointXYZL cur_point = laserCloud_noGround->points[index];
        //         pcl::PointXYZRGB cur_p;
        //         cur_p.x = cur_point.x;
        //         cur_p.y = cur_point.y;
        //         cur_p.z = cur_point.z;
        //         cur_p.r = colors_map_tran[segment_class][0];
        //         cur_p.g = colors_map_tran[segment_class][1];
        //         cur_p.b = colors_map_tran[segment_class][2];
        //         classifiedCloud->push_back(cur_p);
        //     }
        // }