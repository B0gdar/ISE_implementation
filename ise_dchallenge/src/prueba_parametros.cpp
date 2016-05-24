/*****************************************************************************
 * PROJECT: ISE_INGENIA1516
 * MODULE: ise_localnav: Implements local navigation control.		     *
 *
 * (c) Copyright 2016 Miguel Cordero Sánchez and Francisco Butragueño Martín.*
 * All rights reserved.							     *
 * based on 2D Work by Reid Simmons, Joaquín López, Javier Albertos and      *
 * Miguel Cordero.                                                           *
 *                                                                           *
 * FILE: test_ise_localnav.cpp                                               *
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
//#include <bebop_msgs::Ardrone3PilotingStateFlyingStateChanged>

#include <ros/ros.h>
//#include <nodelet/loader.h>
static geometry_msgs::Twist  odom_rv;
static bool flag_fst;// Flag to execute cmd_vel control on state 2 and 3 only
std::vector<float>t;//vector which stores every clock time when /bebop/odom is updated
static float t_loop;static double t_prev_loop;//variable which stores time lapse between /bebop/odom last update
ros::Publisher pub_cmdvel;
static bool flag_init=true;//flag to execute set_initial_odometry() 

static float ez_prev, ex_prev, ey_prev;

static std::vector<geometry_msgs::Point> vec_ref;
static int ref_pos=0;

class ise_nLNMain
{
  private:
  double walk_vel, run_vel, yaw_rate, yaw_rate_run;//,z_ref;
  int movestate,cmd_eval_z;
	//walk_vel=0.2;
	//z_ref=1;
  
  public:
  
  void init()
  { 
    double absolute_angle;

  //  std::cout<<"!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!";
    
    //ros::NodeHandle n_/*private("~")*/;
//    n_/*private*/.param("walk_vel", walk_vel, walk_vel);
//    n_/*private*/.param("run_vel", run_vel, run_vel);
//    n_/*private*/.param("yaw_rate", yaw_rate, yaw_rate);
//    n_/*private*/.param("yaw_rate_run", yaw_rate_run, yaw_rate_run);
//    n_/*private*/.param("movestate", movestate, movestate);
//    n_/*private*/.param("cmd_eval_z", cmd_eval_z, cmd_eval_z);
  }
  
  ~ise_nLNMain()  {}
  
};

void set_initial_odometry(){
	geometry_msgs::Point A;A.x=0;A.y=0;A.z=2;
	vec_ref.push_back(A);
	geometry_msgs::Point B;B.x=1;B.y=0;B.z=1;
	vec_ref.push_back(B);
	//geometry_msgs::Point C;C.x=1;C.y=2;C.z=1;
	//vec_ref.push_back(C);
	//vec_ref.push_back(A);
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
void speed_control(float a,float b,float c, float t)
{
	geometry_msgs::Twist twistter;
	float k_lin,k_inte,k_diff;
	k_lin=1;
	k_inte=0;
	k_diff=0;
	float ez_int=c-ez_prev;
	float ey_int=b-ey_prev;
	float ex_int=a-ex_prev;
	float ez_diff=ez_int/t;
	float ey_diff=ey_int/t;
	float ex_diff=ex_int/t;
	float speed_max, speed_min;
	speed_max=0.2;speed_min=-0.2;
	float pid_x=k_lin*a+0.8*ex_int*t+0.5*ex_diff;;
	float pid_y=0;//k_lin*b+0.8*ey_int*t+0.5*ey_diff;;
	float pid_z=k_lin*c+k_inte*ez_int*t+k_diff*ez_diff;
	std::cout<<"pid_z: "<<pid_z<<std::endl;
	std::cout<<"Componente lineal: "<<k_lin*c<<"\nComponente integral: "<<k_inte*ez_int*t<<"\nComponente diferencial"<<k_diff*ez_diff<<std::endl;	

	if(c>0.1){
		twistter.linear.z=std::min(pid_z,speed_max);
	}else if(c<0.1){
		twistter.linear.z=std::max(pid_z,speed_min);
	}else{
		/*twistter.linear.z=0;*/twistter.linear.z=pid_z;
		
	}
	/*if(b>0.1){
		twistter.linear.y=std::min(pid_y,speed_max);
	}else if(b<0.1){
		twistter.linear.y=std::max(pid_y,speed_min);
	}else{
		twistter.linear.y=pid_y;
	}*/
twistter.linear.y=pid_y;
	if(a>0.1){
		twistter.linear.x=std::min(pid_x,speed_max);
	}else if(a<0.1){
		twistter.linear.x=std::max(pid_x,speed_min);
	}else{
		twistter.linear.x=pid_x;
	}
	std::cout<<"TWISTTER"<<std::endl;
	std::cout<<"linear should be: "<<twistter<<std::endl;
	pub_cmdvel.publish(twistter);
	ex_prev=a;
	ey_prev=b;
	ez_prev=c;
}
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
	std::cout<<"Flying state:"<<fst.state<<"   Cmd_activated?:"<<flag_fst<<std::endl;
}
//bebop_msgs/Ardrone3PilotingStateFlyingStateChanged

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
	float e_z,e_x,e_y, z_ref,x_ref, y_ref;
	x_ref=vec_ref.at(ref_pos).x;
	y_ref=vec_ref.at(ref_pos).y;	
	z_ref=vec_ref.at(ref_pos).z;
	std::cout<<"REFERENCIA: ("<<x_ref<<", "<<y_ref<<", "<<z_ref<<" )"<<std::endl;
	e_x=x_ref-odom_ref.pose.pose.position.x;
	e_y=y_ref-odom_ref.pose.pose.position.y;
	//geometry_msgs::Twist twistter;
	e_z=z_ref-odom_ref.pose.pose.position.z;
	if (flag_fst==true){
		
		std::cout<<"SPEED_CONTROL ACTIVATED"<<std::endl;
		if(ref_pos<vec_ref.size()-1){		
			if(/*(e_y<=0.1)&&*/(e_x<=0.1)){
				ref_pos++;		
			}
		}	
		speed_control(e_x,e_y,e_z,(float)error_temp);
		
	}
	t_prev_loop=tiempo;
}

void cmdvel_callback(const geometry_msgs::Twist cmd_ref){
	std::cout<<"------------------------------"<<std::endl;
	std::cout<<"CMD UPDATE"<<std::endl;
	//std::cout<<odom_ref<<std::endl;
	std::cout<<"CMD_VEL"<<std::endl;
	std::cout<<cmd_ref.linear<<std::endl;
	//TODO
	//std_position(odom_ref);
	
}

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

ros::Publisher pub_chatter = n.advertise<std_msgs::String>("/chatter", 1);
pub_cmdvel = n.advertise<geometry_msgs::Twist>("/bebop/cmd_vel",1);

std_msgs::String msg;

std::stringstream ss;

ss<<"Yokse no soy 100tifko";
msg.data =ss.str();
pub_chatter.publish(msg);
	ros::spin();
	return 0;
}
