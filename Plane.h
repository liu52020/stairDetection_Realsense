//
// Created by liuwei on 2021/10/24.
//

#ifndef STAIR_LW_NEW_PLANE_H
#define STAIR_LW_NEW_PLANE_H

#include <boost/filesystem.hpp>
#include <pcl/io/pcd_io.h>
#include <pcl/PCLHeader.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/transforms.h>
#include <pcl/common/common.h>

#include <pcl/filters/passthrough.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/common/angles.h>

#include <pcl/features/normal_3d_omp.h>
#include <pcl/segmentation/region_growing.h>
#include <pcl/filters/extract_indices.h>

#include <vector>

#include <time.h>

using namespace std;

class Plane {
public:
    Plane(){
        initial_floor = false;
        colored_cloud.reset(new pcl::PointCloud <pcl::PointXYZRGB>);
        stairFlag = true;  // 默认上楼梯
        operatorNormals.reset(new pcl::PointCloud<pcl::Normal>);
    }
    ~Plane(){}

    void findFloor(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud);

    bool subtractInitialPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Eigen::Vector4f &normal);

    void computeCamera2FloorMatrix (Eigen::Vector4f floor_normal);

    void normalOperator(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    void regionGrowing(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    void getCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    void sortCentroid();

    void getStairModel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud);

    void reset();

    bool initial_floor;
    bool new_floor;
    Eigen::Vector4f floor_normal; // 地板平面的参数
    Eigen::Affine3d c2f; // 相机到地板坐标系的转换矩阵    Affine3d T是一个4*4齐次矩阵变换
    Eigen::Affine3d f2c; // 地板到相机坐标系的转换矩阵
    float person_height; // 相机到地板的高度
    pcl::search::Search<pcl::PointXYZ>::Ptr tree;
    pcl::PointCloud<pcl::Normal>::Ptr operatorNormals;
    pcl::PointXYZ centroid;
    std::vector <pcl::PointIndices> clusters;  //  分割平面
    pcl::PointCloud <pcl::PointXYZRGB>::Ptr colored_cloud;

    bool stairFlag;
    float camera_distance;
    int realClusterNumber;


    vector<Eigen::Vector4f> clusterCentroid;
    vector<vector<float>> depthWeight;
    vector<float> centroid_y;
    vector<float> centroid_z;
    vector<float> clusterWeight;
    vector<float> clusterDepth;
    vector<float> clusterHeigth;

    std::vector<std::pair<float,pcl::PointIndices>> pairs;
};


#endif //STAIR_LW_NEW_PLANE_H
