/*****************************************************************************
 * PROJECT: ISE_INGENIA1516
 * MODULE: ise_localnav: Implements local navigation control.                 *
 *
 * (c) Copyright 2016 Miguel Cordero Sánchez. All rights reserved.
 * based on 2D Work by Reid Simmons, Joaquín López, Javier Albertos and 
 * Miguel Cordero.                                                           *
 *                                                                           *
 * FILE: ise_localnav.cpp                                                    *
 *                                                                           *
 * ABSTRACT: "main function": Calls method according to the configuration.   *
 * 
 *
 *          Subscribes to sensor_msgs/PointCloud2 and localNav/moveDir msgs.
 *          Publishes 
 *
 *****************************************************************************/

// ROS Includes
#include "ros/ros.h"
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>

//#include <string.h>
#include <cmath> 
#include <algorithm>    // std::fill
#include <vector>



//C++ and OPEN GL Includes
#include <stdio.h>
#include <stdlib.h>
#include <iostream>
#include <termios.h>
#include <signal.h>
#include <math.h>
#include <algorithm>

#include <fstream>



//#include <boost/foreach.hpp>//para suscripcion a nube de puntos

// C Reference definition
//extern "C" {
/*#include "localNav_interface.h"
#include "localNav_common.h"
#include "vectors.h"*/
}// 

#ifndef _PTCLIOPUB_H
	#define _PTCLIOPUB_H
#endif




//ros::Publisher pub_or, pub_fil, pub_cvm;

ros::Publisher cmd_publisher;
ros::Publisher dif_publisher;
ros::Publisher mov_publisher;
//ros::Publisher marker_publisher;
//ros::Publisher marker2_publisher;
//ros::Publisher odom_publisher;
//ros::Publisher nearest_publisher;
//ros::Publisher goal_publisher;

int c1=0;
static local_nav::z_LocalNav referencePos;//object which stores references from wheight local nav
static local_nav::Odometry quad_odom;//Create an object to store quadrotor odometry data
static local_nav::Other_markers rviz_markers;
std::vector<float>t;//vector which stores every clock time when a cloud is update from kinect
static float t_loop;//variable which stores time lapse between kinect cloud last update
static double in_x,in_y,theta,absolute_angle,diference_ant;
//in_* stores odometry position values after updating, is input for update odometry of cvm
//theta orientation in z for the robot, is input for update odometry of cvm
RADIANS_PER_SEC rv;
MS_PER_SEC tv;
//double variable which store angular and linear twist in z and x respectly, are content outputs for pointers used in localnav step
//static pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_examp (new pcl::PointCloud<pcl::PointXYZ>);
int counter;

class mcs_newLocalNavMain
{
  private:
  double walk_vel, run_vel, yaw_rate, yaw_rate_run,z_ref;
  int movestate,cmd_eval_z;
  
  public:
  
  void init()
  { 

//    localNav_init();

    RADIANS_PER_SEC rv;
    MS_PER_SEC tv;
    double absolute_angle;

    std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
    
    ros::NodeHandle n_/*private("~")*/;
    n_/*private*/.param("walk_vel", walk_vel, walk_vel);
    n_/*private*/.param("run_vel", run_vel, run_vel);
    n_/*private*/.param("yaw_rate", yaw_rate, yaw_rate);
    n_/*private*/.param("yaw_rate_run", yaw_rate_run, yaw_rate_run);
    n_/*private*/.param("movestate", movestate, movestate);
    n_/*private*/.param("cmd_eval_z", cmd_eval_z, cmd_eval_z);
    //n_/*private*/.param("z_ref", z_ref, z_ref)
//    n_/*private*/.setParam("z_ref", 2);
    
  }
  
  ~mcs_newLocalNavMain()  {}
  
};


//float length_funct (float max, float min){
//	float length;
//	if (min <0){
//		length=max+min;
//	}else{
//		length=max-min;
//	}
//	return length;
//}

//float range_funct (float max, float min){
//float range;
//	range=max-min;
//return range;
//}

//void printing_cloudXYZ (pcl::PointCloud<pcl::PointXYZ>::Ptr cloud){
//	std::cout<<"  ----Valores (x,y,z)----\n";
//	for (size_t i = 0; i < cloud->points.size (); ++i)
//	{
//    		std::cerr <<"    " << cloud->points[i].x << " "
//                        << cloud->points[i].y << " "
//                        << cloud->points[i].z << std::endl;
//	}
//}

double zero_to_2pi(double angle){
		double temp_set_1=std::abs(angle);
		std::cout<<"From"<<angle;
		if (std::fmod(temp_set_1,M_PI)!=0){//case is different to 0+K2PI and PI
			if (temp_set_1>M_PI){//case is

				angle=-std::floor(angle/temp_set_1)*M_PI+std::floor(angle/temp_set_1)*(temp_set_1-(std::floor(temp_set_1/M_PI)*M_PI));

			}
		}else if(temp_set_1!=0){
			if(std::fmod(temp_set_1,(2*M_PI))==0){
				angle=0;
			}else{
				angle=std::floor(angle/temp_set_1)*M_PI;
			}
		}
		std::cout<<"º   to   "<<angle/M_PI*180<<"º\n"<<std::endl;

	 	std::cout<<": ="<< angle/M_PI*180<<" degrees" <<std::endl;
return angle;
}

void set_initial_odometry(void){
	ros::NodeHandle n_;//_private("~");
	n_/*private*/.getParam("postion_x0",quad_odom.pose.position.x);
	n_/*private*/.getParam("postion_y0",quad_odom.pose.position.y);
	n_/*private*/.getParam("postion_z0",quad_odom.pose.position.z);
	n_/*private*/.getParam("orientation_x0",quad_odom.pose.orientation.x);
	n_/*private*/.getParam("orientation_y0",quad_odom.pose.orientation.y);
	n_/*private*/.getParam("theta_0",theta);
	n_/*private*/.getParam("abs_theta_0",absolute_angle);
	rviz_markers.theta=theta;
	rviz_markers.absolute_angle=absolute_angle;
//	quad_odom.pose.position.x=0;
//	quad_odom.pose.position.y=0;
//	quad_odom.pose.position.z=0;
//	quad_odom.pose.orientation.x=0;
//	quad_odom.pose.orientation.y=0;
//	rviz_markers.theta=theta=0;
////	quad_odom.pose.orientation.z=sin(theta/2);
////	quad_odom.pose.orientation.w=cos(theta/2);
	std::cout<<"Registro odometría en inicio:"<< quad_odom <<"\n";
}

void odometry_update(float t){
		quad_odom.pose.position.x+=quad_odom.twist.linear.x*t;
		quad_odom.pose.position.y+=quad_odom.twist.angular.z*t*quad_odom.twist.linear.x;
		quad_odom.pose.position.z+=quad_odom.twist.linear.z*t;//referenceHei.range;/*+=quad_odom.twist.linear.z*t;*///referenceHei.altitude;
		theta+=quad_odom.twist.angular.z*t;
if (std::abs(theta)>M_PI){
		theta=zero_to_2pi(theta);
}
//quaternion orientation its gonna be necessary only to represent goal direction in rviz, meanwhile, in this object we are going to stare in each cartesian component its concerning euler angle
//		quad_odom.pose.orientation.z=sin(theta/2);
//		quad_odom.pose.orientation.w=cos(theta/2);
		quad_odom.pose.orientation.z=theta;
		std::cout<<"Updating odometry:>>>>>>>>>>>>>>>>>\n\n"<<quad_odom<<"\n\n";
		in_x=double(quad_odom.pose.position.x);
		in_y=double(quad_odom.pose.position.y);
		rviz_markers.theta=theta;
		rviz_markers.absolute_angle=absolute_angle;
		if (theta!=absolute_angle){
			counter++;
		}else{
			std::cout<<"COUNTER FINISHED AT: "<<counter<<std::endl;
		}
		
}

void update_twist(float x, float z, float rv){
	quad_odom.twist.linear.x=x;
	quad_odom.twist.linear.z=z;
	quad_odom.twist.angular.z=rv;
	rviz_markers.tv=x;
	rviz_markers.cmd_z=z;
	rviz_markers.rv=rv;
}

void set_absolute_theta(double * theta_ref, double  a_ref){
	
	absolute_angle=*theta_ref+a_ref;
	absolute_angle=zero_to_2pi(absolute_angle);
	std::cout<<"ABSOLUTE ANGLE UPDATE: prociding to "<<absolute_angle<<"rads\n\n";
}



void localNav_ref_callBack(const local_nav::HeightLocalNav reference){
        std::cout<< "getting" << reference << std::endl;
	referencePos = reference;
	ros::NodeHandle n_;
	switch (reference.xyst){
		case 0:
			n_/*private*/.setParam("xystate", 0);
			std::cout<<"XY REACTIVE CONTROL BLOCKED\n";
			break;
		case 1:
			std::cout<<"XY REACTIVE CONTROL ENEABLED\n";
			break;
	}
	
	switch (referencePos.ast){
		case 1:
			double *theta_ref;
			*theta_ref=theta;
			set_absolute_theta(theta_ref,reference.a_ref);
			referencePos.ast=0;
			std::cout<<"theta_ref: "<<*theta_ref<<"\na_ref: "<<reference.a_ref<<"\n\n";
			break;
//		case 0://inicio, H, Z
//			std::cout<<"Unable to modify angle reference due to current navigation mode"<<std:endl;
//			break;
		case 0://default:
			break;
	}
//	referencePos.ast=0;

	 if(reference.fst==0){
		n_/*private*/.setParam("movestate", 0);
		//Case iniciated by having pressed X or x: Stops the current flight state to the height z is already working at that moment
		n_/*private*/.setParam("cmd_eval_z", 0);
		n_/*private*/.setParam("xystate", 0);
		std::cout<<"FLYING STATE:  Stopped"<<std::endl;
	}

	//Receives point's reference from msg configured in wheight or teleop and stablishes it as the reference position for hector
}

//void localNav_altimeter_sensor_callBack(const hector_uav_msgs::Altimeter altimeter){
void localNav_altimeter_sensor_callBack(const sensor_msgs::Range sonar){
//TODO Control por regulación de sensor barométrico

//	referenceHei=altimeter;
	referenceHei=sonar;
	int xy_eval;	
	
	//hector_uav_msgs::Altimeter quad_z=altimeter.altitude;
//	float quad_z=altimeter.altitude;
	float quad_z=sonar.range;
//	float quad_z=quad_odom.pose.position.z;
	ros::NodeHandle n_;//private("~");
	
	std::cout<<"LIVE ALTITUDE MEASURE: "<< quad_z/*altimeter.altitude*/<<" METERS"<<std::endl;
	//referenceHei = altimeter;
		
	int movestate_z,cmd_eval_z;
	double z_ref;//,* a_ref;
	double h_run, h_set, g_run;//Percentages for z velocity control
	z_ref=referencePos.z_ref;
//	*a_ref=referencePos.a_ref;
	std::cout<< "Z REF= "<<z_ref<<" meters\n";//A REF= "<<*a_ref<<" radians\n";
	n_/*private*/.getParam("movestate",movestate_z);
	n_/*private*/.getParam("cmd_eval_z",cmd_eval_z);
//	n_/*private*/.getParam("xystate",xy_eval);
	n_/*private*/.getParam("run_to_pid",h_run);
	n_/*private*/.getParam("reached_z_ref",h_set);
	n_/*private*/.getParam("run_to_ground",g_run);

	//n_/*private*/.getParam("z_ref",z_ref);
		
	switch (referencePos.fst)
	{
	  case 1:
	  //Case iniciated by having pressed H or h: Stablishes working height adapting velocity published into /cmd_vel by reference of point z=+2+/-0.5 m and z=+2.0m
		if ( quad_z<z_ref*(1-h_run)){//z_ref*0.75 1.5
			n_/*private*/.setParam("cmd_eval_z", 2);
			n_/*private*/.setParam("movestate", 0);
			std::cout<<"FLYING STATE NEW!! Going to the working height from behind"<<std::endl;
		}else if (quad_z>z_ref*(1+h_run)){//z_ref*1.25 2.5
			n_/*private*/.setParam("cmd_eval_z", -2);
			n_/*private*/.setParam("movestate", 0);
			std::cout<<"FLYING STATE NEW!! Going to the working height from top"<<std::endl;
		}else if ((quad_z>=z_ref*(1-h_set)) && (quad_z<=z_ref*(1+h_set))){//z_ref*0.95&&z_ref*1.05 1.9 2.1
			n_/*private*/.setParam("cmd_eval_z", 0);
			n_/*private*/.setParam("movestate", 1);
			std::cout<<"FLYING STATE NEW!! WORKING HEIGHT REACHED"<<std::endl;
		}else if (quad_z>=z_ref*(1-h_run)){//z_ref*0.75 1.5
			n_/*private*/.setParam("cmd_eval_z", 1);
			n_/*private*/.setParam("movestate", 0);
			std::cout<<"FLYING STATE NEW!! Reaching working height from behind"<<std::endl;
		}else if(quad_z<=z_ref*(1+h_run)){//z_ref*1.25 2.5
			n_/*private*/.setParam("cmd_eval_z", -1);
			n_/*private*/.setParam("movestate", 0);
			std::cout<<"FLYING STATE NEW!! Reaching working height from top"<<std::endl;
		}
	 break;
	 case 2:
//Case iniciated by having pressed G or g: Sends the quadrator to the ground adapting velocity published in /cmd_vel by reference of oin z=+0.8m
		n_/*private*/.setParam("movestate", 0);
		if (quad_z>z_ref*(1-g_run)){//z_ref*0.6 0.8
			n_/*private*/.setParam("cmd_eval_z", -2);
			std::cout<<"FLYING STATE NEW!! Going to the ground"<<std::endl;
		}else if(quad_z<=z_ref*(1-g_run)){//z_ref*0.6
			n_/*private*/.setParam("cmd_eval_z", -3);
			std::cout<<"FLYING STATE NEW!! Aproaching to the ground"<<std::endl;
		}else if(quad_z<=0.2){
			n_/*private*/.setParam("cmd_eval_z", 0);
			std::cout<<"FLYING STATE: At the ground"<<std::endl;
		}
	 break;
	 case 0:
		n_/*private*/.setParam("movestate", 0);
		//Case iniciated by having pressed X or x: Stops the current flight state to the height z is already working at that moment
		n_/*private*/.setParam("cmd_eval_z", 0);
		n_/*private*/.setParam("xystate", 0);
		std::cout<<"FLYING STATE:  Stopped"<<std::endl;
	default:
		n_/*private*/.setParam("movestate", 0);
	break;
	 }
	 std::cout<<"movestate:"<<movestate_z<<std::endl;
}


void cloud_callBack(const pcl::PCLPointCloud2::Ptr& ptCloud){//ConstPtr& ptCloud){
	/* This is the point cloud callback function. When ahttps://github.com/PointCloudLibrary/pcl2 new cloud is received, this 
	   function will be called automatically by the program. 
	*/
//switch(referencePos.ist){
//	case 1:
//		std::cout<<"REFERENCE_POS "<<referencePos.ist<<std::endl;
//	break;

//	case 0:
//		std::cout<<"REFERENCE_POS "<<referencePos.ist<<std::endl;
//float t_loop;//variable which stores time lapse between kinect cloud last update
ros::NodeHandle n_;//private("~");
float loop_time_secs;

pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_examp (new pcl::PointCloud<pcl::PointXYZ>);
std::vector<int> indices;
int det_st;
float numero_points;

pcl::VoxelGrid<pcl::PointXYZ> vxGrid;
double vox_size;
n_/*private*/.getParam("voxel_grid_size",vox_size);
pcl::PointCloud<pcl::PointXYZ> ptCloudPrXYZ_; // Processed Point CloudXYZ

pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloudPrXYZ_filtered (new pcl::PointCloud<pcl::PointXYZ>);
pcl::PassThrough<pcl::PointXYZ> pass;
double pass_size;
n_/*private*/.getParam("pass_filter_x",pass_size);

float z_min, z_max, x_min, x_max, y_min, y_max;
int check1,check2,check3,check4,check5,check6;
float z_range, x_range, y_range;
float z_length, x_length, y_length;

float r_bound,l_bound;//left boundary for x cloud analysis
double num_point;//max number of points for the vector
n_/*private*/.getParam("x_segm_number",num_point);
float step=roundf((r_bound*2/num_point)*100)/100;//roundf((x_range/num_point)*100)/100;//cvm_cell*2/180	

float temp_ref_p=2.0,temp_ref_n=-2.0;//y_max;
float temp_z,abcd,temp_x,temp_y;
local_nav::CloudVector obst_list;
int indicador_1=0,indicador_2=0;

pcl::PointCloud<pcl::PointXYZ> ptCloud_cvm;
int w=0;
local_nav::CloudVector cloud_stored;
int obs=0;

//Kinect data stores by const reference to a const ptr type p2 called ptCloud
//1- We start to control sistem time while running the node and storage time between clouds received from kinect control
	std::cout<<"Número de puntos en el mensaje:	"<<ptCloud->row_step*ptCloud->width<<std::endl;
	std::cout<<"SIZE OBST_LIST: "<<obst_list.z.size()<<std::endl;
	if (t.size()>1){
		t.push_back(ros::Time::now().toSec());
		t_loop=t.at((t.size())-1)-t.at((t.size())-2);
		std::cout<<"TIME BETWEEN CLOUD UPDATE: "<< t_loop <<" seconds"<<std::endl;
		//Update of position odometry
		odometry_update(t_loop);
			
	}else{
//		t_loop=ros::Time::now().toSec();
		t.push_back(ros::Time::now().toSec());
		std::cout<<"FIRST CLOUD RECEIVED: "<< 0 <<" seconds"<<std::endl;
		set_initial_odometry();		
	}
//2- We publish markers to rviz in function of the odometry data and kinematics state of the hector
//	ros::NodeHandle n_private("~");
	marker2_publisher.publish(rviz_markers);
	odom_publisher.publish(quad_odom);	

	// Print some information, to see if this code works
	ROS_INFO("SANJEEV -> NEW CLOUD RECEIVED");


//	pcl::PointCloud<pcl::PointXYZ>::Ptr cloud_examp (new pcl::PointCloud<pcl::PointXYZ>);
  	pcl::fromPCLPointCloud2(*ptCloud, *cloud_examp);
//	std::vector<int> indices;

//	 pcl::io::savePCDFileASCII ("Cloud_examp_NaN.pcd", *cloud_examp);
// pcl::io::savePCDFileASCII ("Cloud_examp_NaN.txt", *cloud_examp);
//  std::cerr << "Saved " << cloud_examp->points.size () << " data points to Cloud_examp_NaN.pcd." << std::endl;

//  for (size_t i = 0; i < cloud_examp->points.size (); ++i){
//    std::cerr << "    " << cloud_examp->points[i].x << " " << cloud_examp->points[i].y << " " << cloud_examp->points[i].z << std::endl;}

	pcl::removeNaNFromPointCloud (*cloud_examp, *cloud_examp,indices); 
//	float numero_points=cloud_examp->points.size ();
//pcl::io::savePCDFileASCII ("Cloud_examp.pcd", *cloud_examp);
 std::cerr << "Saved " << cloud_examp->points.size () << " data points to Cloud_examp.pcd." << std::endl;
if ((cloud_examp->points.size ()<300)&&(cloud_examp->points.size ()>0)){
	std::cout<<"YUHU"<<std::endl;
	det_st=2;
pcl::io::savePCDFileASCII ("Cloud_examp.txt", *cloud_examp);
}else if(cloud_examp->empty()){
	det_st=0;
}else{
	det_st=1;
}
std::cout<<"DETECTION STATE:";
switch(det_st){
	case 0:
		std::cout<<"0 puntos"<<std::endl;
		break;
	case 1:
	case 2:
		std::cout<<"Más que 0 puntos"<<std::endl;
		pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloudGrid (new pcl::PointCloud<pcl::PointXYZ>);
      
		// Print some information, to see if this code works
		ROS_INFO("SANJEEV -> NEW CLOUD RECEIVED");

	
		    // jlf todo HCM implementation here
	//	std::cout << "ANTES DE FILTRAR!!!!!!!!"<< std::endl;
	//	std::cout << "Received cloud msg : "<< "header=" << ptCloud->header << "width="<<ptCloud->width <<", height="<<ptCloud->height<<".\n";
	        
		// Define the grids, to downsample. Probably 5cm would be okay, FOV = 5m
	//	 pcl::VoxelGrid<pcl::PCLPointCloud2> vxGrid;

	//	pcl::VoxelGrid<pcl::PointXYZ> vxGrid;

	//	 vxGrid.setInputCloud(ptCloud);
		vxGrid.setInputCloud(cloud_examp);
		 vxGrid.setLeafSize (vox_size, vox_size, vox_size); //1x1x1 cm^3
		 vxGrid.filter(*ptCloudGrid);//Almacenada info en pointer pcl2
	
	//std::cout << "DESPUÉS DE REDUCIR!!!!!!!!!"<< std::endl;
	//	std::cout << "Filtered cloud msg : "<< "header=" << ptCloudGrid->header << "width="<<ptCloudGrid->width <<", height="<<ptCloudGrid->height<<".\n";
	// Declare the variable storing the processed cloud
	//	 pcl::PointCloud<pcl::PointXYZ> ptCloudPrXYZ_; // Processed Point CloudXYZ
	//	 pcl::fromPCLPointCloud2(*ptCloudGrid, ptCloudPrXYZ_);  // Convert Downsampled CLoud2 to XYZ
		ptCloudPrXYZ_=*ptCloudGrid;

	//	// Publish the filtered cloud to the WMA Kinect Checker
	//	 pub.publish(ptCloudPrXYZ_.makeShared()); // Make Shared to make it a pointer (for XYZ)
	
	//	pcl::io::savePCDFileASCII ("ptCloudPrXYZ_.pcd", ptCloudPrXYZ_);
	//	std::cerr << "Saved " << ptCloudPrXYZ_.points.size () << " data points to ptCloudPrXYZ_.pcd." << std::endl;
	//	pcl::io::savePCDFileASCII ("/home/miguel/catkin_ws/ptCloudPrXYZ_.txt", ptCloudPrXYZ_);

	//Shows reescaled cloud values
	//printing_cloudXYZ (ptCloudPrXYZ_.makeShared());
	
	//	pcl::PointCloud<pcl::PointXYZ>::Ptr ptCloudPrXYZ_filtered (new pcl::PointCloud<pcl::PointXYZ>);//Creamos puntero donde se asignará el espacio filtrado
	// Create the filtering object
	//  	pcl::PassThrough<pcl::PointXYZ> pass;
	  	pass.setInputCloud (ptCloudPrXYZ_.makeShared());
	  	//pass.setFilterFieldName ("z");
	  	//pass.setFilterLimits (0.0, 1.0);
	//	pass.setFilterFieldName ("y");
	//	pass.setFilterLimits (-0.3, 0.3);
		pass.setFilterFieldName ("x");
		pass.setFilterLimits (-pass_size, pass_size);
	  	//pass.setFilterLimitsNegative (true);
	  	pass.filter (*ptCloudPrXYZ_filtered);
	
		std::cout << "DESPUÉS DE FILTRAR!!!!!!!!!"<< std::endl;
		std::cout << "Filtered cloud msg : "<< "header=" << ptCloudPrXYZ_filtered->header << "width="<<ptCloudPrXYZ_filtered->width <<", height="<<ptCloudPrXYZ_filtered->height<<".\n";
		if (ptCloudPrXYZ_filtered->empty()){
			det_st=0;
			break;
		}
	// //Shows the data points of the cloud after filtering
	//printing_cloudXYZ (ptCloudPrXYZ_filtered);
	
	//Publish the filtered XYZ cloud
		pub_fil.publish(ptCloudPrXYZ_filtered);
	
	//Saves filtered cloud data in pcd and txt files
	//	  pcl::io::savePCDFileASCII ("ptCloudPrXYZ_FILTERED.pcd", *ptCloudPrXYZ_filtered);//cd .ros
	//	  pcl::io::savePCDFileASCII ("ptCloudPrXYZ_FILTERED.txt", *ptCloudPrXYZ_filtered);//cd .ros
	//	  std::cout<<"Organized ptcloudPrXYZ_FILTERED?"<< ptCloudPrXYZ_filtered->isOrganized()<<std::endl;
	//	  float z_min, z_max, x_min, x_max, y_min, y_max;
	//	  int check1,check2,check3,check4,check5,check6;
			z_min=ptCloudPrXYZ_filtered->points[0].z;
			z_max=ptCloudPrXYZ_filtered->points[0].z;
			x_min=ptCloudPrXYZ_filtered->points[0].x;
			x_max=ptCloudPrXYZ_filtered->points[0].x;
			y_min=ptCloudPrXYZ_filtered->points[0].y;
			y_max=ptCloudPrXYZ_filtered->points[0].y;
	
		for (size_t i = 0; i < ptCloudPrXYZ_filtered->points.size (); ++i)
		  {   	if (ptCloudPrXYZ_filtered->points[i].z>z_max){
		    		z_max=ptCloudPrXYZ_filtered->points[i].z;
				++check6;
			}
			if (ptCloudPrXYZ_filtered->points[i].z<z_min){
				z_min=ptCloudPrXYZ_filtered->points[i].z;
				//std::cout<<"zmin :iteracion numero: "<< i <<std::endl;
				++check5;
		    	}
			if (ptCloudPrXYZ_filtered->points[i].x>x_max){
		    		x_max=ptCloudPrXYZ_filtered->points[i].x;
				++check2;
		    	}
			if (ptCloudPrXYZ_filtered->points[i].x<x_min){
				x_min=ptCloudPrXYZ_filtered->points[i].x;
				++check1;
			}
			if (ptCloudPrXYZ_filtered->points[i].y>y_max){
		    		y_max=ptCloudPrXYZ_filtered->points[i].y;
				++check4;
		    	}
			if (ptCloudPrXYZ_filtered->points[i].y<y_min){
				y_min=ptCloudPrXYZ_filtered->points[i].y;
				++check3;
			}	  
		  }
		//std::cout<<"mínimos (x, y, z): ("<<x_min<<", "<<y_min<<", "<<z_min<<")\n Se encontraron en los índices: ("<<check1<<", "<<check3<<", "<<check5<<")\n";
		//std::cout<<"máximos (x, y, z): ("<<x_max<<", "<<y_max<<", "<<z_max<<")\nSe encontraron en los índices: ("<<check2<<", "<<check4<<", "<<check6<<")\n";
	
	
	//	float z_range, x_range, y_range;
//		x_range=range_funct(x_max,x_min);
//		y_range=range_funct(y_max,y_min);
//		z_range=range_funct(z_max,z_min);
		//std::cout<<"rangos (x, y, z): ("<<x_range<<", "<<y_range<<", "<<z_range<<")\n";
		
	//	float z_length, x_length, y_length;
//		x_length=length_funct(x_max,x_min);
//		y_length=length_funct(y_max,y_min);
//		z_length=length_funct(z_max,z_min);
	//	std::cout<<"longitudes (x, y, z): ("<<x_length<<", "<<y_length<<", "<<z_length<<")\n"<<std::endl;
		
	//	float r_bound;//left boundary for x cloud analysis
	//	float num_point=180;//number of points for the vector	
		r_bound=pass_size;//roundf((x_range/2)*100)/100;//.9  roundf((cvm_cell-step) * 100) / 100;
		step=roundf((r_bound*2/num_point)*100)/100;//roundf((x_range/num_point)*100)/100;//cvm_cell*2/180
		std::cout<<"r_bound: "<< r_bound<<"\nnumeros:   "<<num_point<<"\nstep:   "<<step<<"\n";
		
	//	float temp_ref_p=2.0,temp_ref_n=-2.0;//y_max;
	//	float temp_z,abcd,temp_x,temp_y;
			
	//	local_nav::CloudVector obst_list;
		float loop_for=ptCloudPrXYZ_filtered->points.size ();
		if ((ptCloudPrXYZ_filtered->points.size ()<num_point)&&(ptCloudPrXYZ_filtered->points.size ()>1)){
			num_point=ptCloudPrXYZ_filtered->width;
		}else if(ptCloudPrXYZ_filtered->points.size ()==1){
			num_point=2;
	//		loop_for=1;
		}
		for (size_t j = 1; j < num_point; ++j)
		{   	
			temp_ref_p=20.0;
			temp_ref_n=-20.0;
			temp_x=temp_y=20.0;
			/*float*/l_bound=roundf((r_bound-step) * 100) / 100;
			//std::cout<<"left boundary:   "<<r_bound<<"\nright boundary:   "<<l_bound<<std::endl;
			/*int*/ indicador_1=0,indicador_2=0;	
			for(int i = 0; i < ptCloudPrXYZ_filtered->points.size (); ++i)
			{ 	
			 if ((ptCloudPrXYZ_filtered->points[i].x>l_bound) && (ptCloudPrXYZ_filtered->points[i].x<=r_bound) )
				{	
					temp_z=ptCloudPrXYZ_filtered->points[i].z;
					//std::cout<<"temp_z:   "<<temp_z<<std::endl;
					++indicador_1;
					
				if(temp_z>0){
					if (temp_z<temp_ref_p)//es menor que la referencia z
					{	//std::cout<< "Se almacena i: "<< i<<"\n\n";					
						//obst_list[j-1]=ptCloudPrXYZ_filtered->points[i];
						//std::cout<<"obst_list: "<<obst_list.z[j-1]<<"\n\n";		
						temp_ref_p=temp_z;
						temp_x=ptCloudPrXYZ_filtered->points[i].x;
						temp_y=ptCloudPrXYZ_filtered->points[i].y;
						++indicador_2;
					}else{//es mayor que la referencia z
					}
				}
                	       	}else{//es mayor que el limite por la derecha y menor que por la izquierda
				}				   		  
	  		}//fin ciclo data. Pasa a actualizar los datos del limite izquierdo
			//cvm_[j-1]=temp_ref_p;
			ptCloudPrXYZ_filtered->points[j-1].x=temp_x;
			ptCloudPrXYZ_filtered->points[j-1].y=temp_y;
			ptCloudPrXYZ_filtered->points[j-1].z=temp_ref_p;
			if (temp_ref_p<20){
				obst_list.x.push_back(temp_x);
				obst_list.y.push_back(temp_y);
				obst_list.z.push_back(temp_ref_p);
			}
			//std::cout<<"obst_list: "<<obst_list.z[j-1]<<"\n\n";
			r_bound=l_bound;
			//std::cout<<"PASA POR X: "<< indicador_1<<" VECES\nPASA POR Z: "<<indicador_2<<" VECES\n";
		}//fin ciclo pantalla
	 	// ptCloudPrXYZ_filtered->points.resize(num_point);
		if (ptCloudPrXYZ_filtered->empty()){
			det_st=0;
			break;
		}
//	  	pcl::io::savePCDFileASCII ("ptCloudPrXYZ_PRECVM.pcd", *ptCloudPrXYZ_filtered);//cd .ros	
//		  pcl::io::savePCDFileASCII ("ptCloudPrXYZ_PRECVM.txt", *ptCloudPrXYZ_filtered);//cd .ros
		
		std::cout<<"VECTOR ENTRANTE:----------\n\n";
		std::cout<<"	Número de puntos totales: "<< obst_list.z.size() <<"\n";
		std::cout<<"	Puntos del vector (x, y, z): " <<"\n\n";
	
		for(size_t i=0; i<obst_list.z.size(); ++i){
			std::cout<<"	punto "<<i+1<<":  "<< obst_list.x.at(i) << ", " << obst_list.y.at(i) <<", "<< obst_list.z.at(i) << std::endl;
		
		}

		break;
	}
	marker_publisher.publish(obst_list);
	std::cout<<"Publishing marker vector"<<std::endl;

}

void method_callBack(const local_nav::CloudVector method_vector){
	ros::NodeHandle n_;//private("~");
	float y_ref=1.00;
	float dif_top;
	float dif_under;
	double tv_max;
	n_/*private*/.getParam("max_forward_vel",tv_max);
	double rv_max;
	n_/*private*/.getParam("max_ang_vel",rv_max);
	rv_max=rv_max*M_PI/180;
	double absolute_angle_read=absolute_angle;//to avoid change angle calculation break

//localNav_update_laser(int n_obs, float *range, double start_angle, double resolution, double offset);
//IMPLEMENTACION PUBLICACION CMD

geometry_msgs::Twist cmd;
double walk_vel_z, run_vel_z, walk_vel_xy, run_vel_xy, z_ref;
int movestate_kinect,cmd_res_z,xy_kinect;
	
	n_/*private*/.getParam("walk_vel",walk_vel_z);
	n_/*private*/.getParam("run_vel",run_vel_z);
	n_/*private*/.getParam("movestate",movestate_kinect);
//	n_/*private*/.getParam("xy_state",xy_kinect);
	n_/*private*/.getParam("cmd_eval_z",cmd_res_z);
	n_/*private*/.getParam("walk_vel",walk_vel_xy);
	n_/*private*/.getParam("run_vel",run_vel_xy);
	n_/*private*/.getParam("z_ref",z_ref);
	
	double diference=/*z_ref-quad_odom.pose.position.z;*/-referenceHei.range+z_ref;
	double k_lin;
	n_/*private*/.getParam("k_P",k_lin);
//	double ground_dif=referenceHei.altitude;
	if (diference>0){std::cout<< "Hay una diferencia de +"<<diference <<std::endl;}else if(diference<0){std::cout<< "Hay una diferencia de "<<diference <<std::endl;}else{std::cout<< "Hay una diferencia de "<<diference <<std::endl;};
	double inte_error=diference-diference_ant;
	double k_inte;
	n_/*private*/.getParam("k_I",k_inte);
	double diff_error=inte_error/t_loop;
	double k_diff;
	n_/*private*/.getParam("k_D",k_diff);
	double pid=k_lin*diference+k_inte*inte_error*t_loop+k_diff*diff_error;
	std::cout<<"pid: "<<pid<<std::endl;
	std::cout<<"Componente lineal: "<<k_lin*diference<<"\nComponente integral: "<<k_inte*inte_error<<"\nComponente diferencial"<<k_diff*diff_error<<std::endl;
	switch (cmd_res_z)
	{
	  case 1:
		cmd.linear.z=pid;//walk_vel_z;
		break;
	  case 2:
		cmd.linear.z=std::min(pid,run_vel_z);
		break;
	  case 0:
		cmd.linear.z=0;
		break;
	  case -1:
		cmd.linear.z=pid;//-walk_vel_z;
		break;
	  case -2:
		cmd.linear.z=std::min(pid,-run_vel_z);
		break;
	  case -3:
		cmd.linear.z=pid;//-ground_dif;
		break;
	}
	diference_ant=diference;
	std::cout<<"movestate vale:   "<<movestate_kinect<<"\n-------------------------------------------------------------\n"<<std::endl;
	
	
	std::cout<<"Goal direction obtained: prociding to: "<< absolute_angle_read <<"radians\n\n";
	if (movestate_kinect=1){
		n_/*private*/.setParam("xy_state",1);
	}else{
		n_/*private*/.setParam("xy_state",0);	
	}
	n_/*private*/.getParam("xy_state",xy_kinect);
xy_kinect=0;
	if (xy_kinect==1)//z_ref
	{	std::cout<<"AVANZAMOS"<<std::endl;
		float avan_ref=20,ang_candidate,ang_eval=999;
		float ang_point,dest_x,dest_z,ang_goal=999,eval_up,eval_down;
		float dist_to_goal,dist_eval,dist_crit=99;
		double tv_cal,rv_cal;
		geometry_msgs::Point goal,nearest;//Points of the point marker
//     	 	float tv_max=0.5;
//		float rv_max=45*M_PI/180;
		double ref_ang=absolute_angle_read-theta;
		ref_ang=zero_to_2pi(ref_ang);
		std::vector<int> vect_ind;
		std::cout<<"REF_ANG"<<ref_ang*180/M_PI<<"º"<<std::endl;
		
		if (method_vector.z.size()!=0)
		{	
			for (size_t i=0; i<method_vector.z.size(); ++i)
			{	
				dist_eval=(std::sqrt(std::pow(method_vector.x.at(i),2)+std::pow(method_vector.z.at(i),2)));
				if (dist_eval<dist_crit){
					dist_crit=dist_eval;
					nearest.x=method_vector.x.at(i);
					nearest.y=0;
					nearest.z=method_vector.z.at(i);
				}	
				
				ang_point=atan(method_vector.x.at(i)/method_vector.z.at(i));
				std::cout<<"ang_point:	"<< ang_point*180/M_PI<<"º"<<std::endl;
					if (ref_ang>0)
					{
						eval_up=ref_ang*1.2;
						eval_down=ref_ang*0.8;
						if ((ang_point>=eval_down)&&(ang_point<=eval_up))
						{   std::cout<<"++++++++++++++"<<std::endl;
							ang_candidate=std::abs(ref_ang-ang_point);
							if (ang_candidate<ang_eval)
							{	ang_goal=ang_point;
								ang_eval=ang_point;
								dest_x=method_vector.x.at(i);
								dest_z=method_vector.z.at(i);
								//pet		
							}else
							{std::cout<<"MAYOR QUE REFERENCIA"<<std::endl;
							}
						}
					}else if(ref_ang<0)
					{
						eval_up=absolute_angle_read*1.2;
						eval_down=absolute_angle_read*0.8;
						if ((ang_point<=eval_down)&&(ang_point>=eval_up))
						{	std::cout<<"-------------------"<<std::endl;
							ang_candidate=std::abs(ref_ang-ang_point);
							if (ang_candidate<ang_eval)
							{	ang_goal=ang_point;
								ang_eval=ang_point;
								dest_x=method_vector.x.at(i);
								dest_z=method_vector.z.at(i);
								//pet		
							}else
							{std::cout<<"MAYOR QUE REFERENCIA"<<std::endl;
							}
						}
					}else if(ref_ang==0)
					{
					if(std::abs(ang_point)<=M_PI/9)
						{	std::cout<<"Cumple dentro de +/- 20º"<<std::endl;
						ang_candidate=std::abs(ref_ang-ang_point);
							if (ang_candidate<ang_eval)
							{	std::cout<<"000000000000000000"<<std::endl;
								ang_goal=ang_point;
								ang_eval=ang_point;
								dest_x=method_vector.x.at(i);
								dest_z=method_vector.z.at(i);
								//pet		
							}else
							{std::cout<<"MAYOR QUE REFERENCIA"<<std::endl;
							}
						}else{std::cout<<"NO DENTRO DE +-20º"<<std::endl;
						}
					}
			}//for
			std::cout<<"PUNTO CRITICO:	"<<nearest.x<<" , "<<nearest.z<<std::endl;
			nearest_publisher.publish(nearest);
			std::cout<<"ang_goal"<< ang_goal<<std::endl;
			if (ang_goal==999)
			{ std::cout<<"9999999999999999999999999999999999999"<<std::endl;
				for (size_t i=0; i<method_vector.z.size(); ++i)
				{	ang_point=atan(method_vector.x.at(i)/method_vector.z.at(i));
				std::cout<<"ang_point:	"<< ang_point*180/M_PI<<"º"<<std::endl;
					if (ref_ang!=0)
					{
						ang_candidate=std::abs(ref_ang-ang_point);
						if (ang_candidate<ang_eval)
						{	ang_goal=ang_point;
							ang_eval=ang_point;
							dest_x=method_vector.x.at(i);
							dest_z=method_vector.z.at(i);
							//pet		
						}else
						{std::cout<<"MAYOR QUE REFERENCIA"<<std::endl;
						}
					}else if(ref_ang==0)
					{
						ang_candidate=std::abs(ang_point);
						if (ang_candidate<ang_eval)
						{	ang_goal=ang_point;
							ang_eval=ang_point;
							dest_x=method_vector.x.at(i);
							dest_z=method_vector.z.at(i);
							//pet		
						}else
						{std::cout<<"MAYOR QUE REFERENCIA"<<std::endl;
						}
					}
				}
			}//999
			goal.x=dest_x;
			goal.y=0;
			goal.z=dest_z;
			std::cout<<"PUNTO OBJETIVO:	"<<goal.x<<" , "<<goal.z<<"  |  "<<ang_goal*180/M_PI<<"º"<<std::endl;
			goal_publisher.publish(goal);
		
			dist_to_goal=std::sqrt(std::pow(goal.x,2)+std::pow(goal.z,2));//-std::sqrt(std::pow(quad_odom.pose.position.x,2)+std::pow(quad_odom.pose.position.y,2)));
			float radio_giro=std::sqrt(std::pow(tv_max,2)+std::pow((rv_max*tv_max),2));
			std::cout<<"RADIO DE GIRO: "<<radio_giro<<"\n\nDISTANCIA OBJETIVO: "<<dist_crit<<std::endl;
			if(dist_crit>=(radio_giro-0.2)){
			
				if(dist_to_goal>=0.4)
				{std::cout<<"Goal a menos de 0'5"<<std::endl;
					if(std::abs(ref_ang)<=M_PI*3/4){
						std::cout<<"Determinando velocidades"<<std::endl;
						tv_cal=std::abs(dist_to_goal*cos(ang_goal)/(2*t_loop));
						tv=std::min(tv_cal,tv_max);
					}else{
						tv=0;
					}//angulo menor a 135	//90
				}else if((dist_to_goal<=0.5)&&(dist_to_goal>=0))/*((quad_odom.pose.position.x>=0.95*goal.z) && (quad_odom.pose.position.y>=0.95*goal.x))*/
					{
						tv=0;
						std::cout<<"Se alcanza el punto"<<std::endl;
					
				}else{//caso en el que no detecta obstaculo
						std::cout<<"Continuamos avanzando"<<std::endl;
						tv=tv_max;
				}//dist_to_goal solo afecta a velocidad lineal
				if (std::abs(ref_ang)>0.0001)
				{
					rv_cal=std::abs(ref_ang/(2*t_loop));
					rv=std::min(rv_cal,double(rv_max));
					rv=std::floor(ref_ang/std::abs(ref_ang))*rv;//se encarga de asignar el desfase correcto
				}else{
					rv=0;
				}
			}else{
				tv=0;
				if (std::abs(ref_ang)>0.0001)
				{
					rv_cal=std::abs(ref_ang/(2*t_loop));
					rv=std::min(rv_cal,double(rv_max));
					rv=std::floor(ref_ang/std::abs(ref_ang))*rv;//se encarga de asignar el desfase correcto
				}else{
				rv=0;
				}
			}//dist_crit
		}else{//method==0
			std::cout<<"No se detectan obstáculos"<<std::endl;
			tv=std::max(quad_odom.twist.linear.x,tv_max);
			if (std::abs(ref_ang)>0.0001)
			{
				rv_cal=std::abs(ref_ang/(2*t_loop));
				rv=std::min(rv_cal,double(rv_max));
				rv=std::floor(ref_ang/std::abs(ref_ang))*rv;//se encarga de asignar el desfase correcto
			}else{
				rv=0;
			}	
		}
	}else if(xy_kinect==0){
		std::cout<<"NO AVANZAMOS"<<std::endl;
			tv=0;
			rv=0;
	}

		cmd.linear.x=float(tv);
		cmd.angular.z=float(rv);
		std::cout<<"STEP OUTPUT:\n	GOAL DIRECTION- "<<absolute_angle_read*180/M_PI<<"º\n	ACTUAL DIRECTION- "<<theta*180/M_PI<<"º\n	tv- "<<tv<<"    rv- "<<rv<<std::endl;	

//		std::cout<<"STEP LOCALNAV:\n	tv- "<<*t_v<<"    rv- "<<*r_v<<std::endl;		
		update_twist(cmd.linear.x,cmd.linear.z, cmd.angular.z);

	 vel_publisher.publish(cmd);

//t.push_back(ros::Time::now().toSec());

};

int kfd = 0;
struct termios cooked, raw;

void quit(int sig)
{
  //tcsetattr(kfd, TCSANOW, &cooked);
  exit(0);
}


int main (int argc, char** argv){
	// Initialize ROS
	ros::init (argc, argv, "local_nav"); 

	set_initial_odometry();

	// NodeHandle is the main access point to communications with the ROS system.
        ros::NodeHandle localNavNH;

        mcs_newLocalNavMain lnMain;
        lnMain.init();
        
        signal(SIGINT,quit);

	// For receiving and publishing the clouds, the buffer size is currently specified as 1
	// Subscribe to the kinect's input point cloud
	ros::Subscriber kinectSub_ = localNavNH.subscribe ("/camera/depth/points", 1, cloud_callBack);
	std::cout<<"points subscribed"<<std::endl;
	
	// Subscribe to the reference local nav point
	ros::Subscriber RefSub_ = localNavNH.subscribe ("localNav_cmd_ref", 1, localNav_ref_callBack);
	std::cout<<"teleop subscribed"<<std::endl;

	// Subscribe to the z status model
	ros::Subscriber gazehSub_ = localNavNH.subscribe("/sonar_height", 1, localNav_altimeter_sensor_callBack);
//	ros::Subscriber gazehSub_ = localNavNH.subscribe("/altimeter", 1, localNav_altimeter_sensor_callBack);
	//std::cout<<"Altimeter subscribed"<<std::endl;

	ros::Subscriber MethodSub_ = localNavNH.subscribe ("/local_nav/mcs_new_method", 1, method_callBack);
	
//	ros::Subscriber MovSub_ = localNavNH.subscribe("/move_check",1,move_callBack);

	// advertise publish robot speed control
       vel_publisher = localNavNH.advertise<geometry_msgs::Twist>("cmd_vel", 1);
	//pub_or = localNavNH.advertise<pcl::PointCloud<pcl::PointXYZ> >("local_nav/originalCloud", 1);
	pub_fil = localNavNH.advertise<pcl::PointCloud<pcl::PointXYZ> >("local_nav/filteredCloud", 1);
	//pub_cvm = localNavNH.advertise<pcl::PointCloud<pcl::PointXYZ> >("local_nav/cvmCloud", 1);
	marker_publisher = localNavNH.advertise<local_nav::CloudVector>("/local_nav/cvm_vector", 1);
	marker2_publisher = localNavNH.advertise<local_nav::Other_markers>("/local_nav/other_markers", 1);
	odom_publisher = localNavNH.advertise<local_nav::Odometry>("/local_nav/quadrotor_odometry",1);
	nearest_publisher=localNavNH.advertise<geometry_msgs::Point>("/local_nav/nearest",1);
	goal_publisher = localNavNH.advertise<geometry_msgs::Point>("/local_nav/goal",1);
	
	// ros::spin() will enter a loop, pumping callbacks.  
	// ros::spin();

	// Instead of spin() we will use spinOnce to make as a safety measure to stop the robot
	// if messages from kinect do not arrive for a "long" period of time.
	ros::Rate r(20); //Callbacks at 20Hz
	
	while (ros::ok()){
		ros::spinOnce();
		// jlf to do obtain the time since the last kinect msg arrived 
		r.sleep();
	}
}

