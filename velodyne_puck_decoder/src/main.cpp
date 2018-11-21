#include <stdint.h>
#include <iostream>
#include <list>
#include <vector>
#include <stdio.h>
#include <sstream>
#include "VLP16.h"
#include <chrono>

using namespace std;

VLP16* vlp;
float coordinates[3]={-99,-99,-99};

#include <ros/ros.h>
#include <pcl_ros/point_cloud.h>
#include <pcl/point_types.h>
#include <boost/foreach.hpp>
#include "std_msgs/Float32MultiArray.h"
//#include <velodyne_puck_decoder/lidarDataType.h>
//#include <velodyne_puck_decoder/lidarDataTypeArray.h>
#include <velodyne_puck_decoder/lidarDataType.h>
#include <velodyne_puck_decoder/lidarDataTypeArray.h>

// needs to be added to be able to feed the header.timestamp
#include <pcl_conversions/pcl_conversions.h>

typedef pcl::PointCloud<pcl::PointXYZ> PointCloud;
typedef pcl::PointCloud<pcl::PointXYZ> CloudType;

bool publishCones = false;
bool firstRun = true;

#define PI 3.14159265

float startAngle = 0;
float endAngle   = 180;
float curAngle   =  0;
float curAngle_temp = 0;
int i=0;
long newWidth = 0;
void callback(const PointCloud::ConstPtr& msg)
{

	  CloudType::Ptr cloud (new CloudType);

	printf ("Cloud: width = %d", msg->width);

	if(!vlp->running())
	{ 
	  	long width = msg->width;
		if(width>0)
		{	
			  curAngle = startAngle;
			  for(i=0; 3*i < width; i++) 
			  {
				CloudType::PointType p_retrieved = msg->points[3*i];
				curAngle_temp = atan2(-p_retrieved.y,p_retrieved.x)*180/PI;

				curAngle = curAngle_temp;
				
				// TURNED TO THE LEFT BY 90deg ( ! ! ! CABLE IS TO THE RIGHT SIDE ! ! ! )
				if(p_retrieved.y<=0 && p_retrieved.z>=-0.10 && p_retrieved.z<=1)// && p_retrieved.y>=-2 && p_retrieved.y<=2
				{
				  	vlp->addPoint(-p_retrieved.y,-p_retrieved.x,p_retrieved.z);
					++newWidth;
				}
			
				/* // NORMAL POSITIONING OF THE VELODYNE
				if(p_retrieved.x>=0 && p_retrieved.z>=-0.10 && p_retrieved.z<=1)// && p_retrieved.y>=-2 && p_retrieved.y<=2
				{
				  	vlp->addPoint(p_retrieved.x,p_retrieved.y,p_retrieved.z);
					++newWidth;
				}*/
			  }
			  vlp->addScan(newWidth);
			  vlp->doPub();
		}
	}
}

int main(int argc, char** argv)
{
  vlp = new VLP16();

	auto start = std::chrono::high_resolution_clock::now();
  ros::init(argc, argv, "coneFinder");
  ros::NodeHandle nh;
  ros::Subscriber sub = nh.subscribe<PointCloud>("velodyne_point_cloud", 1, callback);
  //ros::spin();

  //cout << "Preparing for pub... \n";

  // PCL publishing
  //ros::Publisher pub = nh.advertise<std_msgs::Float32MultiArray>("cone_detected_lidarX1Y1Z1,etc...", 1);

  // ONE_ARRAY publishing x1,y1,z1,x2,y2,z2,...
  //ros::Publisher pub = nh.advertise<PointCloud>("cone_detected_lidarPCL", 1);

  // XYZ_Struct_MSG_DATATYPE
  ros::Publisher pub =nh.advertise<velodyne_puck_decoder::lidarDataTypeArray>("cone_detected_lidarXYZ",1);


  // PCL publishing
  //PointCloud::Ptr msg (new PointCloud);
  //msg->header.frame_id = "velodyne";
  //msg->height = 1;
  // =======================================================================================

  // ONE_ARRAY publishing x1,y1,z1,x2,y2,z2,...
  //std_msgs::Float32MultiArray msg;
  // =======================================================================================


  ros::Rate loop_rate(500);
  while (ros::ok())
  {
    	
	// PCL publishing
	//msg->points.clear();
	// ONE_ARRAY publishing x1,y1,z1,x2,y2,z2,...
	//msg.data.clear(); 	
	// XYZ_Struct_MSG_DATATYPE
	velodyne_puck_decoder::lidarDataType data;
	velodyne_puck_decoder::lidarDataTypeArray msg; 

	if(vlp->ready())
	{
		vlp->run();
		
		int totalCones = vlp->conesLength();
		double* cones;
		cones=vlp->getCones();

		// PCL publishing
		//msg->width = totalCones;
		//pcl_conversions::toPCL(ros::Time::now(), msg->header.stamp);
		// =======================================================================================

		for (int i = 0; i < totalCones; i++)
		{	
			// PCL publishing
			// ROTATED TO THE LEFT BY 90deg... change to be rereferenced in rviz ONLY! not to the CAR!
			//msg->points.push_back(pcl::PointXYZ(-cones[3 * i+1],-cones[3 * i],cones[3 * i+2]));
			// PUBLISHING FOR THE CAR IN ANY CASE OF VLP16 ORIENTATION
			//msg->points.push_back(pcl::PointXYZ(cones[3 * i],cones[3 * i+1],cones[3 * i+2]));	// comminted this and uncommented the 3 msg.,,, lines
			// =======================================================================================

			
  			// ONE_ARRAY publishing x1,y1,z1,x2,y2,z2,...
			/*
			  msg.data.push_back(-cones[3*i+1]);	// 
			  msg.data.push_back(-cones[3*i]);	//
			  msg.data.push_back(cones[3*i+2]);	//
			*/
			// =======================================================================================

			// XYZ_Struct_MSG_DATATYPE
			  data.x = (-cones[3*i+1]);	// 
			  data.y = (-cones[3*i]);	//
			  data.z = (cones[3*i+2]);	//
			  msg.detected_cones_lidar.push_back(data);		
			// =======================================================================================
		}
		pub.publish(msg);
		vlp->donePub();
		auto finish = std::chrono::high_resolution_clock::now();
		std::chrono::duration<double> elapsed = finish - start;
  		cout << "	Elapsed time: " << elapsed.count() << " s" << "	Publishing done\n";
		start = std::chrono::high_resolution_clock::now();
	}
	ros::spinOnce();
    	loop_rate.sleep();   
  }

}
