//
// create by lw on 2021/11/7
// for detecting stair with the inter-realsense

#include <iostream>
#include <librealsense2/rs.hpp>
#include <opencv2/opencv.hpp>

using namespace std;

#include "Scene.h"
#include "Plane.h"

#define _width 640
#define _height 480
#define fps 30


// 执行类
class mainLoop {

public:
    mainLoop() : plane(){
        color_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        color_cloud_show.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
    }

    ~mainLoop() {}

    void reset(){
        cloud.reset(new pcl::PointCloud<pcl::PointXYZ>);
        color_cloud.reset(new pcl::PointCloud<pcl::PointXYZRGB>);
    }


    float Get_FPS()
    {
        //定义四个静态变量
        static float  s_fps = 0;					 //我们需要计算的FPS值
        static int     s_frameCount = 0;		 //帧数
        static time_t s_currentTime =0;	//当前时间
        static time_t s_lastTime = 0;		//持续时间

        s_frameCount++;           //每调用一次Get_FPS()函数，帧数自增1
        s_currentTime = clock();//获取系统时间，其中timeGetTime函数返回的是以毫秒为单位的系统时间，所以需要乘以0.001，得到单位为秒的时间

        //如果当前时间减去持续时间大于了1秒钟，就进行一次FPS的计算和持续时间的更新，并将帧数值清零
        float t_temp =(float)(s_currentTime - s_lastTime)/(float)1000.0 ;
        if( t_temp > 1.0f) //将时间控制在1秒钟
        {
            s_fps = (float)s_frameCount /t_temp; //计算这1秒钟的FPS值
            s_lastTime = s_currentTime;              //将当前时间currentTime赋给持续时间lastTime，作为下一秒的基准时间
            s_frameCount    = 0;                         //将本次帧数frameCount值清零
        }
        return s_fps;
    }


    void withoutStair(cv::Mat &mat){
        cv::Point origin;
        origin.x = 0;
        origin.y = mat.rows - 10;
        cv::putText(mat, "No stairs ahead", origin, cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(0, 255, 0), 0.5, 8, 0);
    }


    // 执行方法
    void execute(){

        // 1. 获取realsnese的深度图和rgb图
        rs2::context ctx;
        auto list = ctx.query_devices(); // Get a snapshot of currently connected devices
        if (list.size() == 0)
            throw std::runtime_error("No device detected. Is it plugged in?");
        rs2::device dev = list.front();

        rs2::frameset frames;
        //Contruct a pipeline which abstracts the device
        rs2::pipeline pipe;//创建一个通信管道//https://baike.so.com/doc/1559953-1649001.html pipeline的解释
        //Create a configuration for configuring the pipeline with a non default profile
        rs2::config cfg;//创建一个以非默认配置的配置用来配置管道
        //Add desired streams to configuration
        cfg.enable_stream(RS2_STREAM_COLOR, _width, _height, RS2_FORMAT_BGR8, fps);//向配置添加所需的流
        cfg.enable_stream(RS2_STREAM_DEPTH, _width, _height, RS2_FORMAT_Z16, fps);
        cfg.enable_stream(RS2_STREAM_INFRARED, 1, _width, _height, RS2_FORMAT_Y8, fps);
        cfg.enable_stream(RS2_STREAM_INFRARED, 2, _width, _height, RS2_FORMAT_Y8, fps);
        cfg.enable_stream(RS2_STREAM_ACCEL, RS2_FORMAT_MOTION_XYZ32F);
        cfg.enable_stream(RS2_STREAM_GYRO, RS2_FORMAT_MOTION_XYZ32F);
        // start stream
        pipe.start(cfg);//指示管道使用所请求的配置启动流

        while(1){

            std::cout<<"帧率："<<Get_FPS()<<std::endl;
            frames = pipe.wait_for_frames();//等待所有配置的流生成框架

            // Align to depth
            rs2::align align_to_depth(RS2_STREAM_COLOR);
            frames = align_to_depth.process(frames);

            //Get each frame
            rs2::frame color_frame = frames.get_color_frame();
            rs2::frame depth_frame = frames.get_depth_frame();
            rs2::video_frame ir_frame_left = frames.get_infrared_frame(1);
            rs2::video_frame ir_frame_right = frames.get_infrared_frame(2);

            // Creating OpenCV Matrix from a color image
            cv::Mat color(cv::Size(_width, _height), CV_8UC3, (void *) color_frame.get_data(), cv::Mat::AUTO_STEP);
            cv::Mat pic_right(cv::Size(_width, _height), CV_8UC1, (void *) ir_frame_right.get_data());
            cv::Mat pic_left(cv::Size(_width, _height), CV_8UC1, (void *) ir_frame_left.get_data());
            cv::Mat pic_depth(cv::Size(_width, _height), CV_16U, (void *) depth_frame.get_data(), cv::Mat::AUTO_STEP);

            //重置
            Scene scene;
            reset();
            plane.reset();

            // 输入 rgb color 深度图 pic_depth
            // 2. 霍夫变换
            scene.houghHorizontalLine(color);

            if(scene.isStairs){
                // 3. 导入数据
                scene.loadDepth(pic_depth,color,color_cloud);

                pcl::copyPointCloud(*color_cloud,*cloud);  // exexute

                // 4. 滤波处理
                scene.VoxelFilter(0.04f,cloud);
                scene.PassthroughFilter();

                // 5.平面检测
                std::cout<<"scene.fcloud:"<<scene.fcloud->points.size()<<std::endl;
//                if(scene.fcloud->points.size()<=0){
//                    continue;
//                }
                if(!plane.initial_floor) {

                    // 5.1 寻找地板
                    plane.findFloor(scene.fcloud);
                    // 5.1.1 判断是否找到地板
                    if(plane.new_floor){
                        plane.computeCamera2FloorMatrix(plane.floor_normal);   // c2f

                        // 5.2 转换坐标系
                        // 5.2.1 相机坐标系转地板坐标系
                        pcl::transformPointCloud(*scene.fcloud, *scene.floor_cloud, plane.c2f);

                        // 5.2.2
                        // 坐标系旋转  正对阶梯
                        scene.rotation_y();   // 绕y轴旋转
                        pcl::transformPointCloud(*scene.floor_cloud, *scene.floor_cloud, scene.rotation);


                        // 5.3 法向量操作
                        plane.normalOperator(scene.floor_cloud);


                        // 5.4 区域生长算法
                        plane.regionGrowing(scene.floor_cloud);

                        // 6. 获取楼梯参数
                        // 6.1 获取分割平面深度、宽度和质心
                        plane.getCentroid(scene.floor_cloud);

                        // 6.2 按照质心高度排序
                        plane.sortCentroid();

                        // 6.3 构建阶梯模型
                        plane.getStairModel(scene.floor_cloud);

                        // 7. 检测楼梯
                        int step = 0;
                        int length = plane.clusters.size();
                        float  base = plane.centroid_y[0];
                        float camera_distance = plane.clusterDepth[0];
                        float stairWeight = *min_element(plane.clusterWeight.begin(),plane.clusterWeight.end());

                        // 可视化文本参数
                        int threshold = 30;

                        if(plane.clusters.size()>1){
                            if(plane.stairFlag){ // 上阶梯
                                for(int i=1;i<length;++i){
                                    if((base+0.10<=plane.centroid_y[i])and(base+0.30>=plane.centroid_y[i])){
                                        step++;
                                    }
                                    base = plane.centroid_y[i];  // 修改base
                                }
                                if(step>=1) {
                                    camera_distance = plane.centroid_z[1];
                                    std::cout << "--- ASCENDING STAIRCASE ---\n" <<
                                              "--- Location ---"<<scene.leftOrRight<<":"<<abs(plane.centroid.x- plane.clusterCentroid[0].matrix()[0])<<"m ----\n"<<
                                              "--- Steps: " << step <<" ---\n"<<
                                              "--- Ascending stairs at "<< camera_distance<<"m ----\n"<<
                                              "--- Measurements: " <<
                                              stairWeight <<"m of width, "<< // plane.clusterWeight<<"m of width, "<<
                                              plane.clusterDepth[1]<<"m of depth, "<<
                                              abs(plane.clusterHeigth[1]-plane.clusterHeigth[0])<<"m of height"<<std::endl;
                                    cv::Point origin;
                                    origin.x = 0;
                                    origin.y = scene.mat.rows - 10;
                                    cv::putText(scene.mat, "ASCENDING STAIRCASE", origin, cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0), 0.5, 8, 0);

                                    origin.y -= threshold;
                                    cv::putText(scene.mat,"step: "+to_string(step), origin, cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 0, 0), 0.5, 8, 0);

                                }else{
                                    std::cout<< "前方没有楼梯"<<std::endl;
                                    withoutStair(scene.mat);
                                }
                            }
                            else{ // 下阶梯
                                for(int i=1;i<plane.clusters.size();++i){
                                    if((base-0.10>=plane.centroid_y[i])and(base-0.30<=plane.centroid_y[i])){
                                        step++;
                                    }
                                    base = plane.centroid_y[i];  // 修改base
                                }
                                if(step>=1){
                                    std::cout << "--- DESCENDING STAIRCASE ---\n" <<
                                              "--- Steps: " << step <<" ---\n"<<
                                              "- Descending stairs at "<< camera_distance<<"m -"<<
                                              "--- Measurements: " <<
                                              stairWeight<<"m of width, "<<  // plane.clusterWeight<<"m of width, "<<
                                              plane.clusterDepth[1]<<"m of depth, "<<
                                              abs(plane.clusterHeigth[1]-plane.clusterHeigth[0])<<"m of height, "<<std::endl;
                                    cv::Point origin;
                                    origin.x = 0;
                                    origin.y = scene.mat.rows - 10;
                                    cv::putText(scene.mat, "DESCENDING STAIRCASE", origin, cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 0), 0.5, 8, 0);

                                    origin.y -= threshold;
                                    cv::putText(scene.mat,"step: "+to_string(step), origin, cv::FONT_HERSHEY_COMPLEX, 1, cv::Scalar(255, 255, 0), 0.5, 8, 0);

                                }else {
                                    std::cout<< "前方没有楼梯"<<std::endl;
                                    withoutStair(scene.mat);
                                }
                            }
                        }else{// if(plane.clusters.size()>1){
                            std::cout<< "前方没有楼梯"<<std::endl;
                            withoutStair(scene.mat);
                        }
                    }else{
                        std::cout<< "前方没有楼梯"<<std::endl;
                        withoutStair(scene.mat);
                    }

                }// if(!plane.initial_floor)
                else{
                    std::cout<<"前方没有楼梯"<<std::endl;
                    withoutStair(scene.mat);
                }
            }// if(scene.isStairs)  预检测楼梯
             else{
                std::cout<<"前方没有楼梯"<<std::endl;
                withoutStair(scene.mat);
             }

            // 显示
            namedWindow("Display depth", cv::WINDOW_AUTOSIZE);
            imshow("Display depth", pic_depth);
            namedWindow("Display Image", cv::WINDOW_AUTOSIZE);
            imshow("Display Image", color);
            namedWindow("Hough Image", cv::WINDOW_AUTOSIZE);
            imshow("Hough Image", scene.mat);
            cv::waitKey(1);
            // 点击退出
            if((cv::getWindowProperty("Display Image",1)==-1)or(cv::getWindowProperty("Hough Image",1)==-1)){
                break;
            }

        } // while(1)

    }// void ececute()


    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud;
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr color_cloud_show;

    Plane plane;

};



int main() {

    mainLoop m;
    m.execute();

    return 0;

}
