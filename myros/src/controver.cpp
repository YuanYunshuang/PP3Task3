#include "ros/ros.h"
#include "std_msgs/String.h"
#include "std_msgs/Float64.h"
#include "sensor_msgs/PointCloud2.h"
#include "velodyne_msgs/VelodyneScan.h"
#include "rawdata.h"
#include "datacontainerbase.h"
#include "point_types.h"
//#include <velodyne_pointcloud/pointcloudXYZIR.h>
#include <pcl/point_types.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/PCLPointCloud2.h>
#include <pcl/conversions.h>
#include <pcl_ros/transforms.h>
#include <pcl_conversions/pcl_conversions.h>
#include <pcl/io/ply_io.h>
#include <iostream>
#include <fstream>
#include <string>
#include <boost/lexical_cast.hpp>

typedef velodyne_pointcloud::PointXYZIR Tpoint;

double time_diff=0;
bool flann=false;
double time_error;

void wtPC2txt(std::string outpath, pcl::PointCloud<pcl::PointXYZRGBA>:: Ptr cloud){
      std::ofstream outtxt(outpath);
      for (size_t i = 0; i < cloud->points.size(); ++i){
         uint16_t azi = (uint16_t)(cloud->points[i].rgba >> 16);
         uint16_t ring = (uint16_t)(cloud->points[i].rgba);
         outtxt<<cloud->points[i].x<<","<<cloud->points[i].y<<","<<cloud->points[i].z<<","
         <<azi<<","<<ring<<std::endl;
      }
      outtxt.close();

}

void controverCallback(const std_msgs::Float64::ConstPtr& msg){
 // ROS_INFO("I heard the timestamps of [%f]", msg->data);
  time_diff= ros::Time::now().toSec()-msg->data;
}
void controverCallback2(const sensor_msgs::PointCloud2::ConstPtr& msg){


  pcl::PCLPointCloud2 pcl_pc2;
  pcl_conversions::toPCL(*msg,pcl_pc2);
  pcl::PointCloud<pcl::PointXYZ>::Ptr temp_cloud(new pcl::PointCloud<pcl::PointXYZ>);
  temp_cloud->header = pcl_pc2.header;
  temp_cloud->height = pcl_pc2.height;
  temp_cloud->width = pcl_pc2.width;
  pcl::fromPCLPointCloud2(pcl_pc2,*temp_cloud);
  /*
  std::cout<<"header :"<<temp_cloud->header<<std::endl;
  std::cout<<"size :"<<temp_cloud->points.size()<<std::endl;
  std::cout<<"shape :"<<temp_cloud->height<<","<<temp_cloud->width<<std::endl;

  for(size_t i=0;i<temp_cloud->points.size();i++){

  }*/
  std::string outpath = "/home/liangwei/pointcloud/"+ boost::lexical_cast<std::string>(temp_cloud->header.stamp) + ".ply";
  pcl::io::savePLYFileASCII (outpath, *temp_cloud);
  std::cerr << "Saved points to"<< outpath << std::endl;

  if (flann==false && time_diff!=0){
    time_error=time_diff;
  flann=true;
//  ROS_INFO("header time is [%f]",msg->header.stamp);
}
  if (flann == true){
  //  ROS_INFO("time_error is [%f]",time_error);
    double secs =ros::Time::now().toSec()-time_error;
    ROS_INFO("time is [%f]",secs);
}

}

void convertPoints(const velodyne_rawdata::VPointCloud::ConstPtr& inMsg)
  {
    // allocate an PointXYZ message with same time and frame ID as
    // input data
    pcl::PointCloud<pcl::PointXYZRGBA>::Ptr outMsg(new pcl::PointCloud<pcl::PointXYZRGBA>);
    // pcl::PointCloud<pcl::PointXYZ>::Ptr ringAzi(new pcl::PointCloud<pcl::PointXYZ>);
    //RGBPointCloud::Ptr outMsg(new RGBPointCloud());
    outMsg->header.stamp = inMsg->header.stamp;
    outMsg->header.frame_id = inMsg->header.frame_id;
    outMsg->height = 1;
  //  ROS_INFO("saving files-----------------");
    for (size_t i = 0; i < inMsg->points.size(); ++i)
      {
         pcl::PointXYZRGBA p;
         //pcl::PointXYZ p2;
         p.x = inMsg->points[i].x;
         p.y = inMsg->points[i].y;
         p.z = inMsg->points[i].z;

        // color lasers with the rainbow array
        //int color = inMsg->points[i].ring % N_COLORS;
        //p.rgb = *reinterpret_cast<float*>(rainbow+color);
        uint16_t azi = inMsg->points[i].azimuth;
        uint16_t ring = inMsg->points[i].ring;
        //std::cout<<"azimut: "<<azi<<","<<"ring: "<<ring<<std::endl;
        p.rgba = ((uint32_t)azi << 16 | ring); //pack 2 uint16 into one uint32, azi is high, ring is low
         outMsg->points.push_back(p);
        //ringAziDis->points.push_back(p);
         ++outMsg->width;
        //++ringAziDis->width;
      }
      std::string outpath1 = "/home/liangwei/PLYpointcloud_azi_ring/"+ boost::lexical_cast<std::string>(outMsg->header.stamp) + ".ply";
      std::string outpath2 = "/home/liangwei/TXTpc_azi_ring/"+ boost::lexical_cast<std::string>(outMsg->header.stamp) + ".txt";
    pcl::io::savePLYFileASCII (outpath1, *outMsg);
     wtPC2txt(outpath2, outMsg);
     std::cerr << "Saved points to"<< outpath1 << std::endl;
  }

 void scanCallback(const velodyne_msgs::VelodyneScan::ConstPtr &msg){
    //to do
}



int main(int argc, char** argv) {
  ros::init(argc,argv,"controver");
  ros::NodeHandle n;
  ROS_INFO("start to read the msgs");

  //ros::Subscriber sub1 = n.subscribe("/trigger_timestamps", 1000, controverCallback);
  //ros::Subscriber sub2 = n.subscribe("/velodyne_points", 1000, controverCallback2);
  //n.subscribe("velodyne_points", 1000,&convertPoints, ros::TransportHints().tcpNoDelay(true));
  ros::Subscriber sub2 = n.subscribe("/velodyne_points", 5000, convertPoints);
  ros::spin();
  return 0;
}
