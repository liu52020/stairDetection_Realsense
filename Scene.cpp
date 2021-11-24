//
// Created by liuwei on 2021/10/24.
//

#include "Scene.h"


void Scene::VoxelFilter(float leaf_size, pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    fcloud.reset(new pcl::PointCloud<pcl::PointXYZ>);

    pcl::VoxelGrid<pcl::PointXYZ> vg;
    vg.setInputCloud (cloud);
    vg.setLeafSize (leaf_size, leaf_size, leaf_size);
    vg.filter (*fcloud);
}

void Scene::PassthroughFilter(){

    pcl::PassThrough<pcl::PointXYZ> pass_z;
    pass_z.setInputCloud(fcloud);
    pass_z.setFilterFieldName("z");
    pass_z.setFilterLimits(0, 3.0);
    pass_z.setFilterLimitsNegative (false); // 保留还是删除滤波
    pass_z.filter(*fcloud);
}

void Scene::loadPcd(string filename,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

    if (pcl::io::loadPCDFile<pcl::PointXYZRGB>(filename,*cloud) == -1) {
        std::cout << "load source failed!" << std::endl;
    }
    std::cout << "source loaded!" << std::endl;
}

void Scene::loadDepth(cv::Mat &depth,cv::Mat &rgb,pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud){

    for (int m = 0; m < depth.rows; m++)
        for (int n = 0; n < depth.cols; n++)    {
            ushort d = depth.ptr<ushort>(m)[n];
            if (d == 0)
                continue;
            pcl::PointXYZRGB p;
            p.z = double(d) / factor;
            p.x = (n - cx) * p.z / fx;
            p.y = (m - cy) * p.z / fy;


            // 添加颜色
            p.b = rgb.ptr<uchar>(m)[n * 3];
            p.g = rgb.ptr<uchar>(m)[n * 3 + 1];
            p.r = rgb.ptr<uchar>(m)[n * 3 + 2];


            // 把p加入到点云中
            cloud->points.push_back(p);
        } // cloud 是一个三维的点云数据

    // 设置并保存点云
    cloud->height = 1;
    cloud->width = cloud->points.size();
    cloud->is_dense = false;   // 判断points中的数据是否是有限的（有限为true）或者说是判断点云中的点是否包含 Inf/NaN这种值（包含为false）。
}

void Scene::houghHorizontalLine(cv::Mat &color) {

    cv::resize(color, mat, cv::Size(640, 480));
    // Roi

    cv::Mat img_binary, img_gray, img_canny;

    cv::cvtColor(mat, img_gray, cv::COLOR_BGR2GRAY);

    cv::medianBlur(img_gray, img_gray, 5);
    cv::Canny(img_gray, img_canny, 20, 40);
    //cv::imshow("img_gray", img_gray);


    vector<cv::Vec4i> lines;
    cv::Vec4i pt;
    int num_line = 0;
    float theta;
    //clock_t startt = clock();
    cv::HoughLinesP(img_canny, lines, 1, CV_PI / 180, 171, 100, 20);
    /*clock_t endt = clock();
    double time = (endt - startt) * 1.0 / CLOCKS_PER_SEC;
    printf("hough finished in %f seconds\n", time);*/

    int min_x = 639, max_x = 0, min_y = 479, max_y = 0,length = 0;
    cv::Vec4i max_line, final_line;
    int num = 0;
    for (int i = 0; i < lines.size(); i++) {
        pt = lines[i];
        if (abs(pt[0] - pt[2]) == 0) {
            theta = CV_PI / 2;
        }
        else {
            theta = atan((pt[1] - pt[3]) * 1.0 / (pt[0] - pt[2]));
        }
        if (theta < 0.5 && theta>-0.5) {
            min_x = min(min(pt[0], pt[2]), min_x);
            max_x = max(max(pt[0], pt[2]), max_x);
            min_y = min(min(pt[1], pt[3]), min_y);
            max_y = max(max(pt[1], pt[3]), max_y);
            num_line++;
            int temp_length = sqrt(pow((pt[1] - pt[3]), 2) + pow((pt[0] - pt[2]), 2));
            if (temp_length > length) {
                length = temp_length;
                max_line = pt;
            }
            line(mat, cv::Point(pt[0], pt[1]), cv::Point(pt[2], pt[3]), cv::Scalar(0, 0, 255), 1);
//            cout << "theta: " << theta << endl;
//            cout<<"angle: "<< theta/CV_PI*180<<endl;

            horizontalTheta = max(theta,horizontalTheta);   // 阶梯线角度
            rotationAngle = horizontalTheta/M_PI*180;  // 旋转角度

            num++;
        }
    }

    if(num>1){
        isStairs = true;
        if (horizontalTheta>0.05){
            std::cout<<"左边有楼梯！"<<std::endl;
            leftOrRight = "Left";
        }else if(horizontalTheta<-0.05){
            std::cout<<"右边有楼梯！"<<std::endl;
            leftOrRight = "right";
        } else{
            std::cout<<"正前方有楼梯！"<<std::endl;
            leftOrRight = "Forward";
        }
    }
}


void Scene::rotation_y(){

    float angle;
    std::cout<<"horizontalTheta:"<<horizontalTheta<<std::endl;
    std::cout<<"rotationAngle:"<<rotationAngle<<std::endl;

    if(horizontalTheta>0){

        angle =  - (horizontalTheta + M_PI/2.) + M_PI/2.;
    }else{
        angle =  - (horizontalTheta + M_PI/2.) + M_PI/2.;
    }
    rotation(0, 0) = cos(angle);
    rotation(0, 2) = -sin(angle);
    rotation(2, 0) = sin(angle);
    rotation(2, 2) = cos(angle);
}

