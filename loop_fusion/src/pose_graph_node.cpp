/*******************************************************
 * Copyright (C) 2019, Aerial Robotics Group, Hong Kong University of Science and Technology
 * 
 * This file is part of VINS.
 * 
 * Licensed under the GNU General Public License v3.0;
 * you may not use this file except in compliance with the License.
 *
 * Author: Qin Tong (qintonguav@gmail.com)
 *******************************************************/

#include <vector>
#include <ros/ros.h>
#include <nav_msgs/Odometry.h>
#include <nav_msgs/Path.h>
#include <sensor_msgs/PointCloud.h>
#include <sensor_msgs/Image.h>
#include <sensor_msgs/image_encodings.h>
#include <nav_msgs/OccupancyGrid.h>

#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <visualization_msgs/Marker.h>
#include <std_msgs/Bool.h>
#include <cv_bridge/cv_bridge.h>
#include <iostream>
#include <ros/package.h>
#include <mutex>
#include <queue>
#include <thread>
#include <eigen3/Eigen/Dense>
#include <opencv2/opencv.hpp>
#include <opencv2/core/eigen.hpp>
#include "keyframe.h"
#include "utility/tic_toc.h"
#include "pose_graph.h"
#include "utility/CameraPoseVisualization.h"
#include "parameters.h"
#define SKIP_FIRST_CNT 10
using namespace std;

queue<sensor_msgs::ImageConstPtr> image_buf;
queue<sensor_msgs::ImageConstPtr> depth_buf;
queue<sensor_msgs::PointCloudConstPtr> point_buf;
queue<nav_msgs::Odometry::ConstPtr> pose_buf;
queue<Eigen::Vector3d> odometry_buf;
std::mutex m_buf;
std::mutex m_process;
int frame_index  = 0;
int sequence = 1;
PoseGraph posegraph;
int skip_first_cnt = 0;
int SKIP_CNT;
int skip_cnt = 0;
bool load_flag = 0;
bool start_flag = 0;
double SKIP_DIS = 0;
int ESTIMATE_EXTRINSIC;

int DEPTH_DIST =10;
int DEPTH_BOUNDARY = 10;
float RESOLUTION = 0.03;

int VISUALIZATION_SHIFT_X;
int VISUALIZATION_SHIFT_Y;
int ROW;
int COL;
int DEBUG_IMAGE;
int DEPTH;

camodocal::CameraPtr m_camera;
Eigen::Vector3d tic;
Eigen::Matrix3d qic;
ros::Publisher pub_match_img;
ros::Publisher pub_camera_pose_visual;
ros::Publisher pub_odometry_rect;
ros::Publisher g_map_puber;

std::string BRIEF_PATTERN_FILE;
std::string POSE_GRAPH_SAVE_PATH;
std::string VINS_RESULT_PATH;
CameraPoseVisualization cameraposevisual(1, 0, 0, 1);
Eigen::Vector3d last_t(-100, -100, -100);
double last_image_time = -1;

ros::Publisher pub_point_cloud, pub_margin_cloud;

void new_sequence()
{
    printf("new sequence\n");
    sequence++;
    printf("sequence cnt %d \n", sequence);
    if (sequence > 10)
    {
        ROS_WARN("only support 5 sequences since it's boring to copy code for more sequences.");
        ROS_BREAK();
    }
    posegraph.posegraph_visualization->reset();
    posegraph.publish();
    m_buf.lock();
    while(!image_buf.empty())
        image_buf.pop();
    while(!depth_buf.empty())
        depth_buf.pop();
    while(!point_buf.empty())
        point_buf.pop();
    while(!pose_buf.empty())
        pose_buf.pop();
    while(!odometry_buf.empty())
        odometry_buf.pop();
    m_buf.unlock();
}

void image_callback(const sensor_msgs::ImageConstPtr &image_msg, const sensor_msgs::ImageConstPtr &depth_msg)
{
    //ROS_INFO("image_callback!");
    m_buf.lock();
    image_buf.push(image_msg);
    depth_buf.push(depth_msg);
    m_buf.unlock();
    //printf(" image time %f \n", image_msg->header.stamp.toSec());

    // detect unstable camera stream
    if (last_image_time == -1)
        last_image_time = image_msg->header.stamp.toSec();
    else if (image_msg->header.stamp.toSec() - last_image_time > 3.0 )
    {
        ROS_WARN("image discontinue! >3 ! just warning!");
        // new_sequence();
    }
    else if (image_msg->header.stamp.toSec() < last_image_time)
    {
        ROS_WARN("image discontinue! <0 ! just warning!");
        // new_sequence();
    }
    last_image_time = image_msg->header.stamp.toSec();

}

void point_callback(const sensor_msgs::PointCloudConstPtr &point_msg)
{
    //ROS_INFO("point_callback!");
    m_buf.lock();
    point_buf.push(point_msg);
    m_buf.unlock();
    /*
    for (unsigned int i = 0; i < point_msg->points.size(); i++)
        printf("%d, 3D point: %f, %f, %f 2D point %f, %f \n",i , point_msg->points[i].x,point_msg->points[i].y,point_msg->points[i].z,
                                                     point_msg->channels[i].values[0],point_msg->channels[i].values[1]);
    */

    // for visualization
    sensor_msgs::PointCloud point_cloud;
    point_cloud.header = point_msg->header;
    for (unsigned int i = 0; i < point_msg->points.size(); i++)
    {
        cv::Point3f p_3d;
        p_3d.x = point_msg->points[i].x;
        p_3d.y = point_msg->points[i].y;
        p_3d.z = point_msg->points[i].z;
        Eigen::Vector3d tmp = posegraph.r_drift * Eigen::Vector3d(p_3d.x, p_3d.y, p_3d.z) + posegraph.t_drift;
        geometry_msgs::Point32 p;
        p.x = tmp(0);
        p.y = tmp(1);
        p.z = tmp(2);
        point_cloud.points.push_back(p);
    }
    pub_point_cloud.publish(point_cloud);
}

// only for visualization
void margin_point_callback(const sensor_msgs::PointCloudConstPtr &point_msg)
{
    sensor_msgs::PointCloud point_cloud;
    point_cloud.header = point_msg->header;
    for (unsigned int i = 0; i < point_msg->points.size(); i++)
    {
        cv::Point3f p_3d;
        p_3d.x = point_msg->points[i].x;
        p_3d.y = point_msg->points[i].y;
        p_3d.z = point_msg->points[i].z;
        Eigen::Vector3d tmp = posegraph.r_drift * Eigen::Vector3d(p_3d.x, p_3d.y, p_3d.z) + posegraph.t_drift;
        geometry_msgs::Point32 p;
        p.x = tmp(0);
        p.y = tmp(1);
        p.z = tmp(2);
        point_cloud.points.push_back(p);
    }
    pub_margin_cloud.publish(point_cloud);
}

void pose_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //ROS_INFO("pose_callback!");
    m_buf.lock();
    pose_buf.push(pose_msg);
    m_buf.unlock();
    /*
    printf("pose t: %f, %f, %f   q: %f, %f, %f %f \n", pose_msg->pose.pose.position.x,pose_msg->pose.pose.position.y,pose_msg->pose.pose.position.z,
                                                       pose_msg->pose.pose.orientation.w,pose_msg->pose.pose.orientation.x,
                                                       pose_msg->pose.pose.orientation.y,pose_msg->pose.pose.orientation.z);
    */
}

void vio_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    //ROS_INFO("vio_callback!");
    Vector3d vio_t(pose_msg->pose.pose.position.x, pose_msg->pose.pose.position.y, pose_msg->pose.pose.position.z);
    Quaterniond vio_q;
    vio_q.w() = pose_msg->pose.pose.orientation.w;
    vio_q.x() = pose_msg->pose.pose.orientation.x;
    vio_q.y() = pose_msg->pose.pose.orientation.y;
    vio_q.z() = pose_msg->pose.pose.orientation.z;

    vio_t = posegraph.w_r_vio * vio_t + posegraph.w_t_vio;
    vio_q = posegraph.w_r_vio *  vio_q;

    vio_t = posegraph.r_drift * vio_t + posegraph.t_drift;
    vio_q = posegraph.r_drift * vio_q;

    nav_msgs::Odometry odometry;
    odometry.header = pose_msg->header;
    odometry.header.frame_id = "world";
    odometry.pose.pose.position.x = vio_t.x();
    odometry.pose.pose.position.y = vio_t.y();
    odometry.pose.pose.position.z = vio_t.z();
    odometry.pose.pose.orientation.x = vio_q.x();
    odometry.pose.pose.orientation.y = vio_q.y();
    odometry.pose.pose.orientation.z = vio_q.z();
    odometry.pose.pose.orientation.w = vio_q.w();
    pub_odometry_rect.publish(odometry);

    Vector3d vio_t_cam;
    Quaterniond vio_q_cam;
    vio_t_cam = vio_t + vio_q * tic;
    vio_q_cam = vio_q * qic;        

    cameraposevisual.reset();
    cameraposevisual.add_pose(vio_t_cam, vio_q_cam);
    cameraposevisual.publish_by(pub_camera_pose_visual, pose_msg->header);
}

void extrinsic_callback(const nav_msgs::Odometry::ConstPtr &pose_msg)
{
    m_process.lock();
    tic = Vector3d(pose_msg->pose.pose.position.x,
                   pose_msg->pose.pose.position.y,
                   pose_msg->pose.pose.position.z);
    qic = Quaterniond(pose_msg->pose.pose.orientation.w,
                      pose_msg->pose.pose.orientation.x,
                      pose_msg->pose.pose.orientation.y,
                      pose_msg->pose.pose.orientation.z).toRotationMatrix();
    m_process.unlock();
}

void process()
{
    while (true)
    {
        sensor_msgs::ImageConstPtr image_msg = NULL;
        sensor_msgs::ImageConstPtr depth_msg = NULL;
        sensor_msgs::PointCloudConstPtr point_msg = NULL;
        nav_msgs::Odometry::ConstPtr pose_msg = NULL;

        // find out the messages with same time stamp
        m_buf.lock();
        if(!image_buf.empty() && !point_buf.empty() && !pose_buf.empty())
        {
            if (image_buf.front()->header.stamp.toSec() > pose_buf.front()->header.stamp.toSec())
            {
                pose_buf.pop();
                printf("throw pose at beginning\n");
            }
            else if (image_buf.front()->header.stamp.toSec() > point_buf.front()->header.stamp.toSec())
            {
                point_buf.pop();
                printf("throw point at beginning\n");
            }
            else if (image_buf.back()->header.stamp.toSec() >= pose_buf.front()->header.stamp.toSec() 
                && point_buf.back()->header.stamp.toSec() >= pose_buf.front()->header.stamp.toSec())
            {
                pose_msg = pose_buf.front();
                pose_buf.pop();
                while (!pose_buf.empty())
                    pose_buf.pop();
                while (image_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                {
                    image_buf.pop();
                    depth_buf.pop();
                }

                image_msg = image_buf.front();
                image_buf.pop();
                depth_msg = depth_buf.front();
                depth_buf.pop();

                while (point_buf.front()->header.stamp.toSec() < pose_msg->header.stamp.toSec())
                    point_buf.pop();
                point_msg = point_buf.front();
                point_buf.pop();
            }
        }
        m_buf.unlock();

        if (pose_msg != NULL)
        {
            //printf(" depth time %f \n", depth_msg->header.stamp.toSec());
            //printf(" pose time %f \n", pose_msg->header.stamp.toSec());
            //printf(" point time %f \n", point_msg->header.stamp.toSec());
            //printf(" image time %f \n", image_msg->header.stamp.toSec());

            // skip fisrt few
            if (skip_first_cnt < SKIP_FIRST_CNT)
            {
                skip_first_cnt++;
                continue;
            }

            if (skip_cnt < SKIP_CNT)
            {
                skip_cnt++;
                continue;
            }
            else
            {
                skip_cnt = 0;
            }

            vector<Eigen::Matrix<float, 6, 1>> point_rgbd;
            if(DEPTH)
            {
                cv_bridge::CvImageConstPtr depth_ptr;
                //if (depth_msg->encoding == "8UC1")
                {
                    sensor_msgs::Image img;
                    img.header = depth_msg->header;
                    img.height = depth_msg->height;
                    img.width = depth_msg->width;
                    img.is_bigendian = depth_msg->is_bigendian;
                    img.step = depth_msg->step;
                    img.data = depth_msg->data;
                    img.encoding = sensor_msgs::image_encodings::MONO16;
                    depth_ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO16);
                }
                cv::Mat depth = depth_ptr->image;

                cv_bridge::CvImageConstPtr ptr;
                {
                    sensor_msgs::Image img;
                    img.header = image_msg->header;
                    img.height = image_msg->height;
                    img.width = image_msg->width;
                    img.is_bigendian = image_msg->is_bigendian;
                    img.step = image_msg->step;
                    img.data = image_msg->data;
                    img.encoding = "rgb8";
                    ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::RGB8);
                }
                cv::Mat image = ptr->image;
                //cv::Mat image;
                //cout<< image.channels() <<endl;

                //cv::bilateralFilter(image2, image, 10, 10*2, 10/2);

                for (int v = DEPTH_BOUNDARY; v < ROW - DEPTH_BOUNDARY; v += DEPTH_DIST) {
                    for (int u = DEPTH_BOUNDARY; u < COL - DEPTH_BOUNDARY; u += DEPTH_DIST) {
                        Eigen::Vector2d a(u, v);
                        Eigen::Vector3d a_3d;
                        m_camera->liftProjective(a, a_3d);
                        float d = (depth.ptr<unsigned short>(v)[u]+ depth.ptr<unsigned short>(v-1)[u] +
                                depth.ptr<unsigned short>(v)[u-1] + depth.ptr<unsigned short>(v+1)[u] +
                                depth.ptr<unsigned short>(v)[u+1])/ 5000.0 * 1.15;
                        float r = image.ptr<cv::Vec3b>(v)[u][0];
                        float g = image.ptr<cv::Vec3b>(v)[u][1];
                        float b = image.ptr<cv::Vec3b>(v)[u][2];
                        if (r > 240 && g > 240 && b > 240) continue;
                        if (d < 0.1 || d >= 7.0) continue;
                        Eigen::Matrix<float, 6, 1> point;
                        point << a_3d.x() * d, a_3d.y() * d, d, r, g, b;
                        point_rgbd.push_back(point);
                        //cout <<"r:"<< r <<" g:"<< g <<" b:"<< b <<" d:"<< d << endl;
                    }
                }
            }

            cv_bridge::CvImageConstPtr ptr;
            if (image_msg->encoding == "8UC1")
            {
                sensor_msgs::Image img;
                img.header = image_msg->header;
                img.height = image_msg->height;
                img.width = image_msg->width;
                img.is_bigendian = image_msg->is_bigendian;
                img.step = image_msg->step;
                img.data = image_msg->data;
                img.encoding = "mono8";
                ptr = cv_bridge::toCvCopy(img, sensor_msgs::image_encodings::MONO8);
            }
            else
                ptr = cv_bridge::toCvCopy(image_msg, sensor_msgs::image_encodings::MONO8);

            cv::Mat image = ptr->image;


            // build keyframe
            Vector3d T = Vector3d(pose_msg->pose.pose.position.x,
                                  pose_msg->pose.pose.position.y,
                                  pose_msg->pose.pose.position.z);
            Matrix3d R = Quaterniond(pose_msg->pose.pose.orientation.w,
                                     pose_msg->pose.pose.orientation.x,
                                     pose_msg->pose.pose.orientation.y,
                                     pose_msg->pose.pose.orientation.z).toRotationMatrix();
            if((T - last_t).norm() > SKIP_DIS)
            {
                vector<cv::Point3f> point_3d; 
                vector<cv::Point2f> point_2d_uv; 
                vector<cv::Point2f> point_2d_normal;
                vector<double> point_id;

                for (unsigned int i = 0; i < point_msg->points.size(); i++)
                {
                    cv::Point3f p_3d;
                    p_3d.x = point_msg->points[i].x;
                    p_3d.y = point_msg->points[i].y;
                    p_3d.z = point_msg->points[i].z;
                    point_3d.push_back(p_3d);

                    cv::Point2f p_2d_uv, p_2d_normal;
                    double p_id;
                    p_2d_normal.x = point_msg->channels[i].values[0];
                    p_2d_normal.y = point_msg->channels[i].values[1];
                    p_2d_uv.x = point_msg->channels[i].values[2];
                    p_2d_uv.y = point_msg->channels[i].values[3];
                    p_id = point_msg->channels[i].values[4];
                    point_2d_normal.push_back(p_2d_normal);
                    point_2d_uv.push_back(p_2d_uv);
                    point_id.push_back(p_id);
                    //printf("u %f, v %f \n", p_2d_uv.x, p_2d_uv.y);
                }

                if( DEPTH )
                {
                    KeyFrame *keyframe = new KeyFrame(pose_msg->header.stamp.toSec(), frame_index, T, R, image, point_rgbd,
                                                      point_3d, point_2d_uv, point_2d_normal, point_id, sequence);
                    m_process.lock();
                    start_flag = 1;
                    posegraph.addKeyFrame(keyframe, 1);
                    m_process.unlock();
                }

                else {
                    KeyFrame* keyframe = new KeyFrame(pose_msg->header.stamp.toSec(), frame_index, T, R, image,
                                                      point_3d, point_2d_uv, point_2d_normal, point_id, sequence);
                    m_process.lock();
                    start_flag = 1;
                    posegraph.addKeyFrame(keyframe, 1);
                    m_process.unlock();
                }

                frame_index++;
                last_t = T;
            }
        }
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}

void command()
{
    while(1)
    {
        char c = getchar();
        if (c == 's')
        {
            m_process.lock();
            posegraph.savePoseGraph();
            m_process.unlock();
            printf("save pose graph finish\nyou can set 'load_previous_pose_graph' to 1 in the config file to reuse it next time\n");
            printf("program shutting down...\n");
            ros::shutdown();
        }
        if (c == 'n')
            new_sequence();
        if (c == 'd')
        {
            TicToc t_pcdfile;
            posegraph.save_cloud->width = posegraph.save_cloud->points.size();
            posegraph.save_cloud->height = 1;
            pcl::io::savePCDFileASCII("/home/xuduo/pcd_file_"+to_string(frame_index)+"keyframes.pcd", *(posegraph.save_cloud));
            printf("Save pcd file done! Time cost: %f", t_pcdfile.toc());
        }
        std::chrono::milliseconds dura(5);
        std::this_thread::sleep_for(dura);
    }
}


int main(int argc, char **argv)
{
    ros::init(argc, argv, "loop_fusion");
    ros::NodeHandle n("~");
    posegraph.registerPub(n);
    
    VISUALIZATION_SHIFT_X = 0;
    VISUALIZATION_SHIFT_Y = 0;
    SKIP_CNT = 0;
    SKIP_DIS = 0;

    if(argc != 2)
    {
        printf("please intput: rosrun loop_fusion loop_fusion_node [config file] \n"
               "for example: rosrun loop_fusion loop_fusion_node "
               "/home/tony-ws1/catkin_ws/src/VINS-Fusion/config/euroc/euroc_stereo_imu_config.yaml \n");
        return 0;
    }
    
    string config_file = argv[1];
    printf("config_file: %s\n", argv[1]);

    cv::FileStorage fsSettings(config_file, cv::FileStorage::READ);
    if(!fsSettings.isOpened())
    {
        std::cerr << "ERROR: Wrong path to settings" << std::endl;
    }

    cameraposevisual.setScale(0.1);
    cameraposevisual.setLineWidth(0.01);

    std::string IMAGE_TOPIC;
    std::string DEPTH_TOPIC;
    int LOAD_PREVIOUS_POSE_GRAPH;
    int LOAD_GRID_MAP;

    ROW = fsSettings["image_height"];
    COL = fsSettings["image_width"];
    std::string pkg_path = ros::package::getPath("loop_fusion");
    string vocabulary_file = pkg_path + "/../support_files/brief_k10L6.bin";
    cout << "vocabulary_file" << vocabulary_file << endl;
    posegraph.loadVocabulary(vocabulary_file);

    BRIEF_PATTERN_FILE = pkg_path + "/../support_files/brief_pattern.yml";
    cout << "BRIEF_PATTERN_FILE" << BRIEF_PATTERN_FILE << endl;

    int pn = config_file.find_last_of('/');
    std::string configPath = config_file.substr(0, pn);
    std::string cam0Calib;
    fsSettings["cam0_calib"] >> cam0Calib;
    std::string cam0Path = configPath + "/" + cam0Calib;
    printf("cam calib path: %s\n", cam0Path.c_str());
    m_camera = camodocal::CameraFactory::instance()->generateCameraFromYamlFile(cam0Path.c_str());

    fsSettings["image0_topic"] >> IMAGE_TOPIC;

    DEPTH = fsSettings["depth"];
    fsSettings["image1_topic"] >> DEPTH_TOPIC;

    fsSettings["pose_graph_save_path"] >> POSE_GRAPH_SAVE_PATH;
    fsSettings["output_path"] >> VINS_RESULT_PATH;
    fsSettings["save_image"] >> DEBUG_IMAGE;

    DEPTH_DIST = fsSettings["depth_dist"];
    DEPTH_BOUNDARY = fsSettings["depth_boundary"];
    //RESOLUTION = fsSettings["resolution"];


    ESTIMATE_EXTRINSIC = fsSettings["estimate_extrinsic"];
    if (ESTIMATE_EXTRINSIC == 0){
        cv::Mat cv_T;
        fsSettings["body_T_cam0"] >> cv_T;
        Eigen::Matrix4d T;
        cv::cv2eigen(cv_T, T);
        qic = (T.block<3, 3>(0, 0));
        tic = (T.block<3, 1>(0, 3));
    }

    LOAD_GRID_MAP =  fsSettings["load_grid_map"];
    string GRID_MAP_PATH = fsSettings["grid_map_save_path"];


    LOAD_PREVIOUS_POSE_GRAPH = fsSettings["load_previous_pose_graph"];
    VINS_RESULT_PATH = VINS_RESULT_PATH + "/vio_loop.csv";
    std::ofstream fout(VINS_RESULT_PATH, std::ios::out);
    fout.close();

    int USE_IMU = fsSettings["imu"];
    posegraph.setIMUFlag(USE_IMU);
    fsSettings.release();

    // 读取先验地图（位姿图）
    if (LOAD_PREVIOUS_POSE_GRAPH)
    {
        printf("load pose graph\n");
        m_process.lock();
        posegraph.loadPoseGraph();
        m_process.unlock();
        printf("load pose graph finish\n");
        load_flag = 1;
    }
    else
    {
        printf("no previous pose graph\n");
        load_flag = 1;
    }

    // 读取先验地图（栅格图）
    if (LOAD_GRID_MAP)
    {
        printf("load grid map\n");
        g_map_puber = n.advertise<nav_msgs::OccupancyGrid> ( "grid_map", 1 );
        cv::Mat grid_img = cv::imread(GRID_MAP_PATH + "map.png", CV_LOAD_IMAGE_GRAYSCALE);
        cv::flip(grid_img, grid_img, 0);
        cv::Mat grid_img2;
        grid_img.convertTo(grid_img2, CV_32F, 1/255.0, 0.0);

        // 这些参数后续可以放到config中
        int size_x = 1500;
        int size_y = 500;
        int init_x = 750;
        int init_y = 250;
        double cell_size = 0.05;

        nav_msgs::OccupancyGrid occ_grid;
        occ_grid.header.frame_id = "world";
        occ_grid.header.stamp = ros::Time::now();
        occ_grid.info.width = size_x;
        occ_grid.info.height = size_y;
        occ_grid.info.resolution = cell_size;
        occ_grid.info.origin.position.x = -init_x * cell_size;
        occ_grid.info.origin.position.y = -init_y * cell_size;

        vector<signed char> grid_data(size_x * size_y, -1);

        for(int j = 0; j < size_x; j++){
            for(int i = 0; i < size_y; i++){
                double value = 1.0 - 1.0 * grid_img2.at<float>(j,i) ;
                cout << value << " ";
                if(abs(value - 0.5) > 0.005)
                    grid_data[size_x * i + j] = value * 100;
            }
        }
        occ_grid.data = grid_data;
        g_map_puber.publish ( occ_grid );
    }


    ros::Subscriber sub_vio = n.subscribe("/vins_estimator/odometry", 2000, vio_callback);
    //ros::Subscriber sub_image = n.subscribe(IMAGE_TOPIC, 100, image_callback);
    //ros::Subscriber sub_depth = n.subscribe(DEPTH_TOPIC, 100, depth_callback);
    message_filters::Subscriber<sensor_msgs::Image> sub_image(n, IMAGE_TOPIC, 1);
    message_filters::Subscriber<sensor_msgs::Image> sub_depth(n, DEPTH_TOPIC, 1);
    typedef message_filters::sync_policies::ApproximateTime<sensor_msgs::Image,sensor_msgs::Image> syncPolicy;
    message_filters::Synchronizer<syncPolicy> sync(syncPolicy(10), sub_image, sub_depth);
    sync.registerCallback(boost::bind(&image_callback, _1, _2));

    ros::Subscriber sub_pose = n.subscribe("/vins_estimator/keyframe_pose", 2000, pose_callback);
    ros::Subscriber sub_extrinsic = n.subscribe("/vins_estimator/extrinsic", 2000, extrinsic_callback);
    ros::Subscriber sub_point = n.subscribe("/vins_estimator/keyframe_point", 2000, point_callback);
    ros::Subscriber sub_margin_point = n.subscribe("/vins_estimator/margin_cloud", 2000, margin_point_callback);

    pub_match_img = n.advertise<sensor_msgs::Image>("match_image", 1000);
    pub_camera_pose_visual = n.advertise<visualization_msgs::MarkerArray>("camera_pose_visual", 1000);
    pub_point_cloud = n.advertise<sensor_msgs::PointCloud>("point_cloud_loop_rect", 1000);
    pub_margin_cloud = n.advertise<sensor_msgs::PointCloud>("margin_cloud_loop_rect", 1000);
    pub_odometry_rect = n.advertise<nav_msgs::Odometry>("odometry_rect", 1000);

    std::thread measurement_process;
    std::thread keyboard_command_process;

    measurement_process = std::thread(process);
    keyboard_command_process = std::thread(command);
    
    ros::spin();

    return 0;
}
