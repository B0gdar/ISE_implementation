 /*****************************************************************************
 * PROJECT: ISE_INGENIA1516
 * MODULE: ise_localnav: Implements local navigation control.		     *
 *
 * (c) Copyright 2016 Miguel Cordero Sánchez and Francisco Butragueño Martín.*
 * All rights reserved.							     *
 * based on 2D Work by Reid Simmons, Joaquín López, Javier Albertos and      *
 * Miguel Cordero.                                                           *
 *                                                                           *
 * FILE: error_update.cpp                                               *
 *                                                                           *
 * ABSTRACT: "main function": Calls method according to the configuration.   *
 * 
 *
 *          Subscribes to sensor_msgs/PointCloud2 and localNav/moveDir msgs.
 *          Publishes 
 *
 *****************************************************************************/

#include <string>
#include <vector>
#include <algorithm>
#include "nav_msgs/Odometry.h"
#include <stdio.h>
#include <std_msgs/String.h>
#include <sstream>
#include <bebop_msgs/Ardrone3PilotingStateFlyingStateChanged.h>
#include <tf/transform_datatypes.h>
#include <ise_dchallenge/ise_ref.h>
//#include <bebop_msgs::Ardrone3PilotingStateFlyingStateChanged>

#include <ros/ros.h>
//#include <nodelet/loader.h>

//******************************************************************************************************************************************************
//Global variables definition and ROS publishers
//******************************************************************************************************************************************************
static geometry_msgs::Twist  odom_rv;//stores velocity from odom topic
static bool flag_fst;// Flag to execute cmd_vel control on state 2 and 3 only
std::vector<float>t;//vector which stores every clock time when /bebop/odom is updated
static float t_loop;static double t_prev_loop;//variable which stores time lapse between /bebop/odom last update
ros::Publisher pub_cmdvel;//publisher of cmd_vel
static bool flag_init=true;//flag to execute set_initial_odometry() 

static float ez_prev, ex_prev, ey_prev, e_yaw_prev;//Variables which store the last linear error on odom_callback

static std::vector<ise_dchallenge::ise_ref> vec_ref;//vector which stores the high level references which configure the drone's route
static int ref_pos=0;//intitial position

static float speed_min,speed_max,speed_z_min,speed_z_max,k_lin_x, k_lin_y, k_lin_z, k_int_x, k_int_y, k_int_z, k_diff_x, k_diff_y, k_diff_z, perc_x
, perc_y, perc_z;
//variables which store the parameters set on parameters.yaml file. The data related is: limit on velocity on XY, on Z, Proportional const. of PID, Integral const. of PID, Differential const. of PID, tolerance on linear error

//******************************************************************************************************************************************************
//Class: ise_nLNMain
//------------------------------------------------------------------------------------------------------------------------------------------------------
//Class in which the parameters loading is done and asigned into the variables speed_min...perc_z. It is called once when executing the main loop for the first time
//******************************************************************************************************************************************************
class ise_nLNMain
{ 
  public:
  
  void init()
  { 
    ros::NodeHandle n_;
    n_.getParam("speed_min", speed_min);
    n_.getParam("speed_max", speed_max);
    n_.getParam("speed_z_min", speed_z_min);
    n_.getParam("speed_z_max", speed_z_max);
    n_.getParam("k_lin_x", k_lin_x);
    n_.getParam("k_lin_y", k_lin_y);
    n_.getParam("k_lin_z", k_lin_z);
    n_.getParam("k_diff_x", k_diff_x);
    n_.getParam("k_diff_y", k_diff_y);
    n_.getParam("k_diff_z", k_diff_z);
    n_.getParam("k_int_x", k_int_x);
    n_.getParam("k_int_y", k_int_y);
    n_.getParam("k_int_z", k_int_z);
    n_.getParam("perc_x", perc_x);
    n_.getParam("perc_y", perc_y);
    n_.getParam("perc_z", perc_z);
  
  }
  
  ~ise_nLNMain()  {}
  
};

//*******************************************************************************************************************************************************
//Function #1: set_initial_odometry()
//-------------------------------------------------------------------------------------------------------------------------------------------------------
//Function in which is defined the differents points which defines the drone's route. Each point after being defined is included in the vector vec_ref.
//Point definition is given by ise_ref.msg : Point (x,y,z) [meters]  and yaw [radians] 
//******************************************************************************************************************************************************
void set_initial_odometry(){
	
	//IN THIS EXAMPLE THE DRONE PERFORMS A FORWARD ZIGZAG AND REVERSE ZIGZAG IN THE XY PLANE AT Z=1 THEN GOES UP ONE METER
	ise_dchallenge::ise_ref A;A.position.x=0; A.position.y=0;A.position.z=1;A.yaw=0;//A((0,0,1),0)
	vec_ref.push_back(A);//vec_ref(A)
	ise_dchallenge::ise_ref B;B.position.x=1;B.position.y=1;B.position.z=1;B.yaw=0;//B((1,1,1),0)
	vec_ref.push_back(B);//vec_ref(A,B)
	ise_dchallenge::ise_ref C;C.position.x=2;C.position.y=-1;C.position.z=1;C.yaw=M_PI;//C((2,-1,1),PI)
	vec_ref.push_back(C);//vec_ref(A,B,C)
ise_dchallenge::ise_ref D;D.position.x=1;D.position.y=1;D.position.z=1;D.yaw=M_PI;//D((1,1,1),PI)
	vec_ref.push_back(D);//vec_ref(A,B,C,D)
ise_dchallenge::ise_ref E;E.position.x=1;E.position.y=0;E.position.z=1;E.yaw=M_PI;//E((1,0,1),PI)
	vec_ref.push_back(E);//vec_ref(A,B,C,D,E)
ise_dchallenge::ise_ref F;F.position.x=1;F.position.y=0;F.position.z=2;F.yaw=M_PI;//F((1,0,2),PI)
	vec_ref.push_back(F);//vec_ref(A,B,C,D,E,F)
	
	//Display of the route set. Show number of points and data stored.
std::cout<<"tamaño de vector: "<< vec_ref.size()<<std::endl;
	for (int i=0;i<vec_ref.size();i++){	
		std::cout<<"Elemento "<<i<<" = "<< vec_ref.at(i)<<std::endl;
	}
}
//TODO
/*void std_position(const nav_msgs::Odometry *a){
	nav_msgs::Odometry * b = a; 
	std::cout<<"ODOM POSITION"<<std::endl;
	std::cout<<b->pose->pose->position<<std::endl;
	std::cout<<"ODOM X"<<std::endl;
std::cout<<b->pose->pose->position->x<<std::endl;
std::cout<<"------------------------------"<<std::endl;
}*/

//*******************************************************************************************************************************************************
//Function #2: speed_control(proportional error in x[meters], p.e. in y[meters], p.e. in z[meters], p.e. in yaw [radians], time between odom_updates [seconds])
//-------------------------------------------------------------------------------------------------------------------------------------------------------
//Function in which is defined the PID velocity control on x,y,z and yaw. Loads the error of the position within the present reference and the odometry 
// update time. Then is defined the integral and differential error within the previous proportional error stored and the time between odometry updated.
//The speed definition is defined according the velocity constrains set on parameters.yaml and published on cmd_vel topic
//It is only executed when the odometry is being updated and the drone is on hovering or flying state.
//
//******************************************************************************************************************************************************
void speed_control(float a,float b,float c,float d, float t)//parameters passed are error in x, y, z, yaw and time gap between odometry updated
{
	geometry_msgs::Twist twistter;
	//float k_lin,k_inte,k_diff;
	//k_lin=1;
	//k_inte=0;
	//k_diff=0;
	
	//Integral error definition
	float ez_int=c-ez_prev;
	float ey_int=b-ey_prev;
	float ex_int=a-ex_prev;
	
	//Differential error definition
	float ez_diff=ez_int/t;
	float ey_diff=ey_int/t;
	float ex_diff=ex_int/t;
	
	//float speed_max, speed_min;
	//speed_max=0.2;speed_min=-0.2;
	//float pid_x=k_lin*a+0.8*ex_int*t+0.5*ex_diff;;
	//float pid_y=k_lin*b+0.8*ey_int*t+0.5*ey_diff;;
	//float pid_z=k_lin*c+k_inte*ez_int*t+k_diff*ez_diff;
	
	//PID definition
	float pid_x=k_lin_x*a+k_int_x*ex_int*t+k_diff_x*ex_diff;;
	float pid_y=k_lin_y*b+k_int_y*ey_int*t+k_diff_y*ey_diff;;
	float pid_z=k_lin_z*c+k_int_z*ez_int*t+k_diff_z*ez_diff;
	
	//PID definition displayed on screen
	std::cout<<"pid_x: "<<pid_x<<std::endl;
std::cout<<"Componente lineal: "<<k_lin_x*a<<"\nComponente integral: "<<k_int_x*ex_int*t<<"\nComponente diferencial"<<k_diff_x*ex_diff<<std::endl;
std::cout<<"pid_y: "<<pid_y<<std::endl;
std::cout<<"Componente lineal: "<<k_lin_y*b<<"\nComponente integral: "<<k_int_y*ey_int*t<<"\nComponente diferencial"<<k_diff_y*ey_diff<<std::endl;
std::cout<<"pid_z: "<<pid_z<<std::endl;
	std::cout<<"Componente lineal: "<<k_lin_z*c<<"\nComponente integral: "<<k_int_z*ez_int*t<<"\nComponente diferencial"<<k_diff_z*ez_diff<<std::endl;
	
	//Z velocity definition
	if(c>perc_z/*0.1*/){//if the drone is more than 0.1 meters below the z_ref
		twistter.linear.z=std::min(pid_z,speed_z_max);
	}else if(c<perc_z/*0.1*/){//Else
		twistter.linear.z=std::max(pid_z,speed_z_min);
	}else{//just needed for the previous else when c would be stated as absolute value
		/*twistter.linear.z=0;*/twistter.linear.z=pid_z;
		
	}
	
	//Y velocity definition
	if(b>perc_y/*0.1*/){//if the drone is more than 0.1 meters on the left of the y_ref
		twistter.linear.y=std::min(pid_y,speed_max);
	}else if(b<perc_y/*0.1*/){//else
		twistter.linear.y=std::max(pid_y,speed_min);
	}else{//just needed for the previous else when b would be stated as absolute value
		twistter.linear.y=pid_y;
	}
	
	//X velocity definition
	if(a>perc_x/*0.1*/){//if the drone is more than 0.1 meters behind the x_ref
		twistter.linear.x=std::min(pid_x,speed_max);
	}else if(a<perc_x/*0.1*/){//else
		twistter.linear.x=std::max(pid_x,speed_min);
	}else{//just needed for the previous else when a would be stated as absolute value
		twistter.linear.x=pid_x;
	}
	
	//YAW velocity definition: TODO
	if(d<=1/18*M_PI){
		twistter.angular.z=0.2;
	}
	
	//Displayes velocity definition given to the drone
	std::cout<<"TWISTTER"<<std::endl;
	std::cout<<"linear should be: "<<twistter<<std::endl;
	pub_cmdvel.publish(twistter);//publishes on cmd_vel
	
	//stores the present values for next update
	ex_prev=a;
	ey_prev=b;
	ez_prev=c;
	e_yaw_prev=d;
}

//*******************************************************************************************************************************************************
//Function #3: quat_to_RPY(position.orientation(x,y,z,w))
//-------------------------------------------------------------------------------------------------------------------------------------------------------
//Function which transforms the orientation from quaternion form to Euler angles form (ROLL, PITCH, YAW).
//Loads the orientation in quaternion form and returns the yaw angle in radians. 
//
//******************************************************************************************************************************************************
/*geometry_msgs::Vector3*/float quat_to_RPY(geometry_msgs::Quaternion q){
	tf::Quaternion quat(q.x, q.y, q.z, q.w);
	tf::Matrix3x3 m(quat);
	double roll, pitch, yaw;
	m.getRPY(roll, pitch, yaw);
	/*geometry_msgs::Vector3 result;
	result.x=roll;
	result.y=pitch;
	result.z=yaw;*/
	
	//Displays the result Euler angles in degrees
	std::cout<<"Ángulos: "<<std::endl;
	std::cout<<"	Roll:"<<roll/M_PI*180<<"	Pitch:"<<pitch/M_PI*180<<"	Yaw:"<<yaw/M_PI*180<<std::endl;
	return (float)yaw;
}

//*******************************************************************************************************************************************************
//Function #4: zero_to_2pi(angle[radians]) ##### NOT USED!!!!!!!!!!!!!
//-------------------------------------------------------------------------------------------------------------------------------------------------------
//Function which transforms an angle in radians into the nearest angle between [0,PI] and [0,-PI].
//Intended to be used for incremental definition on angles which may incur into inappropriate yaw definition (such 10PI that will force the drone to define 5 360� twists. 
//
//******************************************************************************************************************************************************

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

//***********************************************************************************************************************************************************
//callback #1: fst_callback(drone's flying state)
//-------------------------------------------------------------------------------------------------------------------------------------------------------
//Function performed when the topic /bebop/states/ARDrone3/PilotingState/FlyingStateChanged is updated.
//Enables the speed control function call by the bool definition of the flag flag_fst (flag flying state).
//The control is enabled when the drone is on hovering or flying (states 2 and 3).
//Else the control is not enabled due to safety constrains (states: 0 (ground), 1 (taking off), 4 (landing) and 5 (emergency)). 
//
//**********************************************************************************************************************************************************

void fst_callback(const bebop_msgs::Ardrone3PilotingStateFlyingStateChanged fst){
	std::cout<<"CAMBIO DE ESTADO!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!"<<std::endl;
	switch(fst.state){
		case 0:
			std::cout<<"En el suelo"<<std::endl;
			flag_fst =false;
			break;
		case 1:
			std::cout<<"Subiendo"<<std::endl;
			flag_fst =false;
			break;
		case 2:
			std::cout<<"Listo: Aquí debería empezar a actuar"<<std::endl;
			flag_fst =true;
			break;
		case 3:
			std::cout<<"Volando"<<std::endl;
			flag_fst =true;
			break;
		case 4:
			std::cout<<"Aterrizando"<<std::endl;
			flag_fst =false;
			break;
		case 5:
			std::cout<<"EMERGENCIA!!!"<<std::endl;
			flag_fst =false;
			break;
	}
	//displays on screen if the speed control is activated
	std::cout<<"Flying state:"<<fst.state<<"   Cmd_activated?:"<<flag_fst<<std::endl;
}
//bebop_msgs/Ardrone3PilotingStateFlyingStateChanged

//***********************************************************************************************************************************************************
//callback #2: odom_callback(drone's odometry)
//-------------------------------------------------------------------------------------------------------------------------------------------------------
//Function performed when the topic /bebop/odom is updated.
//Stablishes the time passed from last update and defines the proportional error on x,y,z and yaw comparing the present route reference.
//It is called the quat_to_RPY() to transform drone's odometry yaw from quaternion to Euler angle.
//Checks if the flying state is appropiate to perform the speed control, determine if there is still any route point to reach and if the present reference 
//has being reached to proceed to the next one in the next update. This is done if the drone is in the cube defined by the edges x=-0.1m, y=-0.1m, z=-0.1m from the reference point and error yaw of 10�
//Calls the speed_control() passing the proportional errors on XYZYaw and time between odometry updates
//Finishes by storing the rosmaster clock data attached to that odometry update to use in the next odometry update to set the time between updates. 
//
//**********************************************************************************************************************************************************

void odom_callback(const nav_msgs::Odometry odom_ref){
	
	std::cout<<"segundos:  "<<odom_ref.header.stamp.sec<<"    nanosegundos:  "<<odom_ref.header.stamp.nsec<<"--> en segundos:  "<<(float)odom_ref.header.stamp.nsec/1000000000<<std::endl;
	double tiempo = (double)odom_ref.header.stamp.sec+(double)odom_ref.header.stamp.nsec/1000000000;
	double error_temp = tiempo-t_prev_loop;
	std::cout<<"Tiempo entre actualizaciones: "<<error_temp<<std::endl;
	odom_rv = odom_ref.twist.twist;
	std::cout<<"------------------------------"<<std::endl;
	std::cout<<"ODOM UPDATE"<<std::endl;
	//std::cout<<odom_ref<<std::endl;
	std::cout<<"ODOM POSITION"<<std::endl;
	std::cout<<odom_ref.pose.pose.position<<std::endl;
	std::cout<<"ODOM VEL"<<std::endl;
	std::cout<<"("<<odom_rv.linear<<", "<<odom_rv.angular.z<<" )"<<std::endl;
	std::cout<<"elemento vector"<<ref_pos+1<<std::endl;
	//TODO
	//std_position(odom_ref);	
	//	std::cout<<"FLYING STATE"<<std::endl;
	//	std::cout<<bebop_fst<<std::endl;
	
	//Present reference definition
	float e_z,e_x,e_y,e_yaw, z_ref,x_ref, y_ref,yaw_ref;
	x_ref=vec_ref.at(ref_pos).position.x;
	y_ref=vec_ref.at(ref_pos).position.y;	
	z_ref=vec_ref.at(ref_pos).position.z;
	yaw_ref=vec_ref.at(ref_pos).yaw;
	
	//Displays the references on screen
	std::cout<<"REFERENCIA: ("<<x_ref<<", "<<y_ref<<", "<<z_ref<<" )"<<std::endl;
	
	//Defines the proportional errors on XYZ and yaw
	e_x=x_ref-odom_ref.pose.pose.position.x;
	e_y=y_ref-odom_ref.pose.pose.position.y;
	//geometry_msgs::Twist twistter;
	e_z=z_ref-odom_ref.pose.pose.position.z;
	e_yaw=yaw_ref-quat_to_RPY(odom_ref.pose.pose.orientation);
	if (flag_fst==true){
		
		std::cout<<"SPEED_CONTROL ACTIVATED"<<std::endl;
		//Checks if present reference has being reached for next odometry update
		if(ref_pos<(vec_ref.size()-1)){		
			if((e_y<=perc_y)&&(e_x<=perc_x/*0.1*/)&&(e_z<=perc_z)){
				ref_pos++;		
			}
		}
		//calls the speed control function	
		speed_control(e_x,e_y,e_z,e_yaw,(float)error_temp);
		
	}
	//stores the time stamp corresponding to this odometry update
	t_prev_loop=tiempo;
}

//***********************************************************************************************************************************************************
//callback #3: cmd_callback(drone's cmd_vel)
//-------------------------------------------------------------------------------------------------------------------------------------------------------
//Function performed when the topic /bebop/cmd_vel is updated.
//Displays on screen the command velocity received by the drone set by manual or autonomous control.
//
//**********************************************************************************************************************************************************
void cmdvel_callback(const geometry_msgs::Twist cmd_ref){
	std::cout<<"-----------------------------"<<std::endl;
	std::cout<<"CMD UPDATE"<<std::endl;
	//std::cout<<odom_ref<<std::endl;
	std::cout<<"CMD_VEL"<<std::endl;
	std::cout<<cmd_ref.linear<<std::endl;
	//TODO
	//std_position(odom_ref);
	
}

//***********************************************************************************************************************************************************
//callback #4: vision_callback(vision_ref) #####JUST READS THE OFFSETS PASSED TO TEST NODE INTEGRATION, NOT DEVELOPED
//-------------------------------------------------------------------------------------------------------------------------------------------------------
//Function performed when the topic /ise_vision/offset is updated.
//Displays on screen the Y devition from the center of the video within the vanishing point and the distance between an ARUCO  par and the vanishin point.
//Following work would be focused on integrating this references on the odometry decission making to set the speed control and it's relation within the route definition
//and following control.
//It should perform yaw until having VPO(Vanishing Point Offset)=0 and then perform roll to correct VArO(Vanishing-ARUCO Offset)
//**********************************************************************************************************************************************************
void vision_callback(const geometry_msgs::Point vision_ref){
	std::cout<<"Y deviation from vannishing point: "<<vision_ref.y<<" meters"<<std::endl;
	std::cout<<" Poderated distance point: "<<vision_ref.z<<" meters"<<std::endl;
}

//***********************************************************************************************************************************************************
//main loop:
//-------------------------------------------------------------------------------------------------------------------------------------------------------
// Calls once the set_initial_odometry() function for routes definition and the class ise_nLNMAIN to perform parameters reading and storage.
// Stablishes and perform the subscribing and advertising procedures
//Subscriptions: 
//	-"/bebop/states/ARDrone3/PilotingState/FlyingStateChanged": From bebop_driver-> Current Flying state
//	-"/bebop/odom: From bebop_driver"-> Current bebop's odometry
//	-"/bebop/cmd_vel: From bebop_driver"-> Command velocity received by the bebop
//	-"/ise_vision/offset": From ise_vision-> Offset products from the Raw video streaming treatment of the Vision algorithms.
//Publications: 
//	-"/bebop/cmd_vel": Advertise the velocity definition done by speed_control()
//**********************************************************************************************************************************************************

int main(int argc, char* argv[])
{
	ros::init(argc, argv, "error_test");
	if (flag_init==true){
		set_initial_odometry();
		flag_init=false;
	}
	
	ros::NodeHandle n;
	ise_nLNMain lnMain;
        lnMain.init();
	ros::Subscriber sub_fst = n.subscribe("/bebop/states/ARDrone3/PilotingState/FlyingStateChanged",1,fst_callback);
	ros::Subscriber sub_odom = n.subscribe("/bebop/odom",1,odom_callback);
	ros::Subscriber sub_cmdvel = n.subscribe("/bebop/cmd_vel",1,cmdvel_callback);
	ros::Subscriber sub_isevision = n.subscribe("/ise_vision/offset",1,vision_callback);

//ros::Publisher pub_chatter = n.advertise<std_msgs::String>("/chatter", 1);
pub_cmdvel = n.advertise<geometry_msgs::Twist>("/bebop/cmd_vel",1);

//std_msgs::String msg;

//std::stringstream ss;

//ss<<"Hello world";
//msg.data =ss.str();
//pub_chatter.publish(msg);
	ros::spin();
	return 0;
}
