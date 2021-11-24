//
// Created by liuwei on 2021/10/24.
//

#include "Plane.h"

void Plane::findFloor(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud) {

    float z_dist = 1.0;
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>);

    const int k_min_cluster_size = 500;
    const float k_max_z_dist = 5.0;
    const float k_z_dist_increase = 0.1f;

    while (cloud_filtered->points.size() < k_min_cluster_size)	{
        pcl::PassThrough<pcl::PointXYZ> pass;
        pass.setInputCloud(cloud);
        pass.setFilterFieldName("z");
        pass.setFilterLimits(0.0, z_dist);
        pass.setFilterLimitsNegative (false);
        pass.filter(*cloud_filtered);
        if (z_dist > k_max_z_dist)   //  k_max_z_dist  z_dist能达到的最大距离
            break;
        else
            z_dist += k_z_dist_increase;

    }

    const int k_n_planes_try = 5;
    int n_planes_try = k_n_planes_try;

    while ((!initial_floor) and (n_planes_try >= 0)) {
        if (n_planes_try == 0) {
            z_dist += k_z_dist_increase;
            if (z_dist > k_max_z_dist)
                break;

            pcl::PassThrough<pcl::PointXYZ> pass;
            pass.setInputCloud(cloud);
            pass.setFilterFieldName("z");
            pass.setFilterLimits(0.0, z_dist);
            pass.filter(*cloud_filtered);

            n_planes_try = k_n_planes_try;
        }
        else {
            initial_floor = this->subtractInitialPlane(cloud_filtered, floor_normal);
            n_planes_try -= 1;
        }
    }

}


bool Plane::subtractInitialPlane(pcl::PointCloud<pcl::PointXYZ>::Ptr &cloud, Eigen::Vector4f &normal) {

    bool initial_plane = false;

    const int k_min_plane_size = 300;
    const float k_min_percentage_size = 0.30f;

    pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZ>());

    *cloud_filtered=*cloud;


    if (cloud_filtered->points.size()>k_min_plane_size){ //
        // RANSAC
        pcl::SACSegmentation<pcl::PointXYZ> seg;  // 创建一个点云分割对象
        pcl::PointIndices::Ptr inliers (new pcl::PointIndices); // 内点索引
        pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);  // 系数
        seg.setOptimizeCoefficients (true); // 是否优化模型系数
        seg.setModelType (pcl::SACMODEL_PLANE); // 平面模型
        seg.setMethodType (pcl::SAC_RANSAC); // 随机采样一致性算法
        seg.setMaxIterations (100);
        seg.setDistanceThreshold (0.02); // aprox. Voxel size / 2 //是否在平面上的阈值

        // Segment the largest planar component from the cloud
        seg.setInputCloud (cloud_filtered); // 输入点云
        seg.segment (*inliers, *coefficients); //分割　得到平面系数　已经在平面上的点的　索引

        // 只能得到一个最终分割结果

//        std::cout<<"查看索引inliers的情况:"<<inliers->indices.size()<<std::endl;
        // 要么点云非常大  要么占据特别多
        if (inliers->indices.size() > k_min_plane_size ||
            (inliers->indices.size() > float(k_min_plane_size)*k_min_percentage_size && float(inliers->indices.size())/float(cloud_filtered->points.size()) > k_min_percentage_size)) {

            // 平面系数
            // Plane coefficients   Ax + By + Cz + D =0
            float A=(coefficients->values[0]);
            float B=(coefficients->values[1]);
            float C=(coefficients->values[2]);
            float D=(coefficients->values[3]);

            // D呈现的就是相机距离平面的高度
            if (D < 0) {A = -A; B = -B; C = -C; D = -D;} //  使平面朝向原点正常

            Eigen::Vector3f v_plane(A,B,C);  // 平面
            Eigen::Vector3f v_floor(0.0f, -sin(float(M_PI_4)), -sin(float(M_PI_4))); // 考虑到相机向下看，大约 45 度的地板法线的估计位置

            float dot = v_plane.dot(v_floor);
            dot = ( dot < -1.0f ? -1.0f : ( dot > 1.0f ? 1.0f : dot ) ); // to avoid NaNs
            float angle = pcl::rad2deg(acos(dot));  // 角度转换
//            std::cout<<"地平面的角度为："<<angle<<std::endl;
//            std::cout<<"地平面与y轴的截距为："<<D<<std::endl;
            // Parameters to consider valid floor
            const float k_angle_threshold = 45; // Valid threshold around estimated floor vector
            const float k_min_D = 0.5f; // 拍摄的时候  角度比较低
            const float k_max_D = 2.5f; // Maximum D value (i.e. maximum distance of the floor to the camera allowed)

            if (angle < k_angle_threshold && fabs(D) > k_min_D && fabs(D) < k_max_D) {

                pcl::PointCloud<pcl::PointXYZ>::Ptr inlierPoints(new pcl::PointCloud<pcl::PointXYZ>);
                //只取inliners中索引对应的点拷贝到inlierPoints中
                pcl::copyPointCloud(*cloud_filtered, *inliers, *inlierPoints);

                // 查看地板的质心
                Eigen::Vector4f vector_centroid;
                pcl::compute3DCentroid(*inlierPoints,vector_centroid);

                centroid = pcl::PointXYZ (vector_centroid[0], vector_centroid[1], vector_centroid[2]);

                std::cout << std::endl <<"-- Floor found --" << std::endl;
                std::cout << "Plane coefficients -> A: " << A << " B: " << B << " C: " << C << " D: " << D << std::endl;
                std::cout << "Angle with respect to the estimated vertical normal = " << angle << std::endl;
                std::cout << "Plane contains " << inliers->indices.size() << " points" << std::endl << std::endl;

                normal = Eigen::Vector4f(A,B,C,D);  //  法向量

                new_floor = true;
            }
        }
        else {
            new_floor = false;
        }

    }

    return (new_floor);
}
// 坐标系转换
void Plane::computeCamera2FloorMatrix (Eigen::Vector4f floor_normal) {
    //
    Eigen::Matrix3f R;
    Eigen::Vector3f t = Eigen::Vector3f::Zero();

    float a,b,c,d;
    c = sqrt(1/(1+floor_normal(2)*floor_normal(2)/(floor_normal(1)*floor_normal(1))));
    d = -c*floor_normal(2)/floor_normal(1);
    b = 1/(floor_normal(0)+(c*floor_normal(1)*(c*floor_normal(1)-d*floor_normal(2)))/floor_normal(0)-(d*floor_normal(2)*(c*floor_normal(1)-d*floor_normal(2)))/floor_normal(0));
    a = b*(c*floor_normal(1)-d*floor_normal(2))/floor_normal(0);

    R << a, -b*c, b*d, b, a*c, -a*d, 0, d, c;
    t(1) = -a*floor_normal(3);

    c2f = ( Eigen::Translation3d (t.cast<double>()) * Eigen::AngleAxisd (R.cast<double>()));
    f2c = c2f.inverse();
    person_height = float(fabs(c2f(1,3)));
//    std::cout<<"person_height:"<<person_height<<std::endl;

}

void Plane::normalOperator(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

//    std::cout<<"法向量操作前点云量"<<cloud->points.size()<<std::endl;

    clock_t  time1,time2,time5,time6;
    time1 = clock();
    pcl::PointCloud<pcl::PointNormal>::Ptr normals(new pcl::PointCloud<pcl::PointNormal>);
    pcl::NormalEstimationOMP<pcl::PointXYZ,pcl::PointNormal> est_normal;
    est_normal.setInputCloud(cloud);

//    time5 = clock();
//    cout << "法向量耗时3 : " <<(double)(time5 - time1) / CLOCKS_PER_SEC << "s" << endl;

    tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
    est_normal.setSearchMethod(tree);
    // Use all neighbors in a sphere of radius 5cm
    est_normal.setKSearch(20);  // 50个点找一次
    est_normal.compute(*normals); // 每一个点都对应一个法向量


    time2 = clock();
//    cout << "法向量耗时4 : " <<(double)(time2 - time5) / CLOCKS_PER_SEC << "s" << endl;
//    cout << "法向量耗时1 : " <<(double)(time2 - time1) / CLOCKS_PER_SEC << "s" << endl;



    for (size_t i = 0; i < cloud->points.size(); ++i)
    {
        normals->points[i].x = cloud->points[i].x;
        normals->points[i].y = cloud->points[i].y;
        normals->points[i].z = cloud->points[i].z;
    }

    // 对法向量进行处理 全部取反
    for (int i = 0; i < normals->points.size(); ++i) {
        if(normals->points[i].normal_y<0){
            normals->points[i].normal_x = -normals->points[i].normal_x;
            normals->points[i].normal_y = -normals->points[i].normal_y;
            normals->points[i].normal_z = -normals->points[i].normal_z;
        }
    }


    pcl::PointCloud<pcl::PointXYZ>::Ptr index_cloud(new  pcl::PointCloud<pcl::PointXYZ>);


    for(int i=0;i<cloud->points.size();++i){
        if((normals->points[i].normal_y>0.95)){ //0.95
            index_cloud->push_back(cloud->points[i]);
        }
    }


    pcl::copyPointCloud(*index_cloud,*cloud);

    clock_t time3,time4;
    time3 = clock();
    tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);


    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> normal_estimator;
    normal_estimator.setSearchMethod (tree);
    normal_estimator.setInputCloud (cloud);
    normal_estimator.setKSearch (50);
    normal_estimator.compute (*operatorNormals);

    time4 = clock();
//    cout << "法向量耗时2 : " <<(double)(time4 - time3) / CLOCKS_PER_SEC << "s" << endl;

    // 对法向量进行处理 全部取反
    for (int i = 0; i < operatorNormals->points.size(); ++i) {
        if(operatorNormals->points[i].normal_y<0){
            operatorNormals->points[i].normal_x = -operatorNormals->points[i].normal_x;
            operatorNormals->points[i].normal_y = -operatorNormals->points[i].normal_y;
            operatorNormals->points[i].normal_z = -operatorNormals->points[i].normal_z;
        }
    }

//    std::cout<<"法向量操作后点云量"<<cloud->points.size()<<std::endl;
}


void Plane::regionGrowing(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
    pcl::RegionGrowing<pcl::PointXYZ, pcl::Normal> reg;
    reg.setMinClusterSize (50);  // 50
    reg.setMaxClusterSize (1000000);
    tree = boost::shared_ptr<pcl::search::Search<pcl::PointXYZ> > (new pcl::search::KdTree<pcl::PointXYZ>);
    reg.setSearchMethod (tree);
    reg.setNumberOfNeighbours (10);     // 10   这个数值可以影响分割的结果
    reg.setInputCloud (cloud);
    //reg.setIndices (indices);
    reg.setInputNormals (operatorNormals);
    reg.setSmoothnessThreshold (3.0 / 180.0 * M_PI); //3.0 / 180.0 * M_PI
    reg.setCurvatureThreshold (1.0);// 1.0

    reg.extract (clusters);
    colored_cloud = reg.getColoredCloud ();
}

void Plane::getCentroid(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){

    vector<float> temp;
    Eigen::Vector4f centroid_temp;
    for (size_t Q=0; Q < clusters.size(); Q++) {
        temp.clear();
        if (clusters[Q].indices.size() > 30) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp (new pcl::PointCloud<pcl::PointXYZ>());
            pcl::ExtractIndices<pcl::PointXYZ> ex;
            ex.setInputCloud (cloud);
            pcl::PointIndices::Ptr indices_ (new pcl::PointIndices);
            *indices_ = clusters[Q];
            ex.setIndices (indices_);
            ex.setNegative (false);
            ex.filter (*cloud_temp);   // 结果


            pcl::PointXYZ minPt, maxPt;
            pcl::getMinMax3D (*cloud_temp, minPt, maxPt);

            temp.push_back(abs(maxPt.z - minPt.z));     // 深度
            temp.push_back(abs(maxPt.x - minPt.x));     // 宽度
            depthWeight.push_back(temp);

            pcl::compute3DCentroid(*cloud_temp,centroid_temp); //估计质心的坐标
            clusterCentroid.push_back(centroid_temp);

        }
    }

}

static bool cmp(std::pair<float,pcl::PointIndices> &a,std::pair<float,pcl::PointIndices> &b){
    return a.first<b.first;
}

static bool cmp_yz(std::pair<float,float> &a,std::pair<float,float> &b){
    return a.first<b.first;
}

static bool cmp_re(std::pair<float,pcl::PointIndices> &a,std::pair<float,pcl::PointIndices> &b){
    return a.first>b.first;
}

void Plane::sortCentroid(){

    int length = clusters.size();


    if(length>1){

        for(int i=0;i<length;++i){
            centroid_y.push_back(clusterCentroid[i].matrix()[1]);
            centroid_z.push_back(clusterCentroid[i].matrix()[2]);
        }

        for(int i=0;i<length;++i){
            pairs.push_back({centroid_y[i],clusters[i]});
        }

        sort(pairs.begin(),pairs.end(),cmp);  //从小到大排序

        for(int i =0;i<length;++i){
            centroid_y[i] = pairs[i].first;
            clusters[i] = pairs[i].second;
        }

        // 将centroid_z按照centroid_y排序
        std::vector<std::pair<float,float>> pairs_yz(length);

        for(int i=0;i<length;++i){
            pairs_yz[i] = {centroid_y[i],centroid_z[i]};
        }

        sort(pairs_yz.begin(),pairs_yz.end(),cmp_yz);  //从小到大排序

        for(int i =0;i<length;++i){
            centroid_y[i] = pairs_yz[i].first;
            centroid_z[i] = pairs_yz[i].second;
        }


//        // 如果第一个数是负数 说明是下楼梯
////        std::cout<<"centroid_y[0]:"<<centroid_y[0]<<std::endl;
////        std::cout<<"centroid_y[clusters.size()-1]:"<<centroid_y[clusters.size()-1]<<std::endl;
//        std::cout<<"打印质心的y值"<<std::endl;
//        for(int i=0;i<centroid_y.size();++i){
//            std::cout<<centroid_y[i]<<std::endl;
//        }
        if((centroid_y[0]<0.10)and(abs(centroid_y[clusters.size()-1]<0.10))and(centroid_z[0]>centroid_z[clusters.size()-1])){ // 说明是下阶梯
            sort(pairs.begin(),pairs.end(),cmp_re);  //从大到小排序

            for(int i =0;i<length;++i){
                centroid_y[i] = pairs[i].first;
                clusters[i] = pairs[i].second;
            }

            // 将分割好的平面按照质心——y值进行排序
            stairFlag = false;  // 下阶梯
        }
    }

    // 再按照从小到大排序
    sort(centroid_z.begin(), centroid_z.end());

}

void Plane::getStairModel(pcl::PointCloud<pcl::PointXYZ>::Ptr cloud) {

    realClusterNumber = 0;
    camera_distance = 0;
    Eigen::Vector4f centroid_temp;

    for (size_t Q = 0; Q < clusters.size(); Q++) {

        if (clusters[Q].indices.size() > 30) {
            pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_temp(new pcl::PointCloud<pcl::PointXYZ>());
            pcl::ExtractIndices<pcl::PointXYZ> ex;
            ex.setInputCloud(cloud);
            pcl::PointIndices::Ptr indices_(new pcl::PointIndices);
            *indices_ = clusters[Q];
            ex.setIndices(indices_);
            ex.setNegative(false);
            ex.filter(*cloud_temp);   // 结果

            pcl::PointXYZ minPt, maxPt;
            pcl::getMinMax3D(*cloud_temp, minPt, maxPt);
//            std::cout<<Q<<" maxPt.z:"<<maxPt.z<<" minPt.z:"<<minPt.z<<" centroid_z:"<<centroid_z[Q]<<" centroid_y:"<<centroid_y[Q]<<std::endl;

            pcl::compute3DCentroid(*cloud_temp,centroid_temp); //估计质心的坐标
            clusterCentroid[Q] = centroid_temp;

            if (Q == 0) {
                clusterWeight.push_back(abs(maxPt.x - minPt.x));
                clusterDepth.push_back(abs(maxPt.z - minPt.z));
                clusterHeigth.push_back(centroid_y[Q]);
//                camera_distance_start = maxPt.z;
//                std::cout<<"camera_distance_start"<<camera_distance_start<<std::endl;

            } else {

                if (abs(centroid_y[Q] - centroid_y[Q - 1]) < 0.05) {  // 一般都是长度缺失
                    clusterWeight[realClusterNumber] += abs(maxPt.x - minPt.x);
//                    float temp = max(clusterDepth[realClusterNumber],abs(maxPt.z - minPt.z));
                    clusterDepth[realClusterNumber] = max(clusterDepth[realClusterNumber], abs(maxPt.z - minPt.z));
                    clusterHeigth[realClusterNumber] = min(clusterHeigth[realClusterNumber], centroid_y[Q]);
                } else {
                    realClusterNumber++;
                    clusterWeight.push_back(abs(maxPt.x - minPt.x));
                    clusterDepth.push_back(abs(maxPt.z - minPt.z));
                    clusterHeigth.push_back(centroid_y[Q]);
//                    if(realClusterNumber==1){
//                        camera_distance_end = maxPt.z;
//                        std::cout<<"camera_distance_end"<<camera_distance_end<<std::endl;
//                        camera_distance = (camera_distance_end+camera_distance_start)/2;
//                    }
                }
            }
        }

//    std::cout<<"realClusterNumber:"<<realClusterNumber<<std::endl;
//    std::cout<<"clusterDepth.size():"<<clusterDepth.size()<<std::endl;
//    for (int i = 0; i < clusterDepth.size(); ++i) {
//        std::cout<<clusterDepth[i]<<std::endl;
//        std::cout<<clusterWeight[i]<<std::endl;
//        std::cout<<clusterHeigth[i]<<std::endl;
//    }


    }
}


void Plane::reset(){
    c2f.setIdentity();
    f2c.setIdentity();

    initial_floor = false;
    new_floor = false;

    floor_normal = Eigen::Vector4f::Zero();
    person_height = 0;
    stairFlag = true;

    pairs.clear();
    clusters.clear();
    clusterCentroid.clear();
    depthWeight.clear();
    centroid_y.clear();
    centroid_z.clear();
    clusterWeight.clear();
    clusterDepth.clear();
    clusterHeigth.clear();

    colored_cloud.reset(new pcl::PointCloud <pcl::PointXYZRGB>);
    operatorNormals.reset(new pcl::PointCloud<pcl::Normal>);

}
