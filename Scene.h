//
// Created by liuwei on 2021/10/24.
//

#ifndef STAIR_LW_NEW_SCENE_H
#define STAIR_LW_NEW_SCENE_H

#include <boost/filesystem.hpp>
#include <pcl/PCLHeader.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common_headers.h>

#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/passthrough.h>

#include <opencv2/opencv.hpp>
#include <string>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <pcl/io/pcd_io.h>

using namespace std;

const float factor = 1000;
//const float fx = 611.973;  // 焦距
//const float fy = 611.261;  // 焦距
//const float cx = 316.243;
//const float cy = 247.133;

// D435i的内参
const float fx = 386.992;  // 焦距
const float fy = 386.992;  // 焦距
const float cx = 320.915;
const float cy = 238.617;

class Scene {
public:
    Scene(){
        floor_cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        rotation = Eigen::Matrix4f::Identity();
        horizontalTheta = -2*M_PI;
        isStairs = false;
    }
    ~Scene(){}

    void VoxelFilter(float leaf_size, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);
    void PassthroughFilter();
    void loadPcd(string filename,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void loadDepth(cv::Mat &depth,cv::Mat &rgb,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud);
    void houghHorizontalLine(cv::Mat &color);
    void rotation_y();



    pcl::PointCloud<pcl::PointXYZ>::Ptr fcloud;
    pcl::PointCloud<pcl::PointXYZ>::Ptr floor_cloud;

    Eigen::Matrix4f rotation;  // 旋转变换矩阵
    float rotationAngle;
    float horizontalTheta;

    bool isStairs;
    string leftOrRight;

    cv::Mat mat;


};


#endif //STAIR_LW_NEW_SCENE_H
