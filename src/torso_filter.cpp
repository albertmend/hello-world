//rosrun rosserial_python serial_node.py _port:=tcp _/rosserial_embeddedlinux/tcp_port:=11416 __name:=rosserial_python_laser2
 
#include <stdio.h>
#include <stdlib.h>
#include <vector>
#include <iostream>
#include <math.h>

#include "ros/ros.h"

#include "std_msgs/Float32MultiArray.h"
#include "std_msgs/Float64.h"
#include "std_msgs/Bool.h"

#include "people_msgs/PositionMeasurementArray.h"
#include "sensor_msgs/LaserScan.h"
#define PI (3.141592653589793)

ros::Publisher torso_pub;
float sum=0;
int cont=0;
float sum_p[682];
float prom_p[682];
bool last[682];
int maxC=20;
float tol=0.1;
//float hold_measure[682];
//int hold;
bool last_follow=false;
bool start_filter=false;




void callback(const std_msgs::Float32MultiArray::ConstPtr& array);
void callback_follow(const std_msgs::Bool follow);

int main(int argc, char **argv)
{
	
	for(int n=0;n<682;n++){
		sum_p[n]=0.0;
		prom_p[n]=0.0;
		last[n]=false;
		
	}
	cont=0;
	ros::init(argc, argv, "torso_filter");
	ros::NodeHandle n;	
	
	torso_pub= n.advertise<sensor_msgs::LaserScan>("torso_filter", 0);
	ros::Subscriber sub3 = n.subscribe("kinect_2_laser", 100, callback);//laser2_msg
	ros::Subscriber sub_follow = n.subscribe("follow_status", 100, callback_follow);//laser2_msg
	
	ros::spin();
	
	return 0;
}

void callback_follow(const std_msgs::Bool follow){
	if(last_follow==true && follow.data==false){
		start_filter=true;
	}
	last_follow=follow.data;
	return;

}


void callback(const std_msgs::Float32MultiArray::ConstPtr& array)
{
	
	sensor_msgs::LaserScan filter_data;
	ros::Time time = ros::Time::now();
	
	filter_data.header.stamp=time;
	filter_data.header.frame_id="laser";
	filter_data.angle_max=1.57079637051+0.523599;
	filter_data.angle_min=-1.57079637051-0.523599;
	filter_data.angle_increment=0.0020453 ;
	filter_data.time_increment=7.50750751e-7;
	filter_data.scan_time=0.1;
	filter_data.range_min=0.020;
	filter_data.range_max=5.6;
	
	printf("%d \n",array->data.size());
	//if (hold<1){
	filter_data.ranges=array->data;
	//}
	sum=0;
	
	
	
	
	
	
	for(int i=0;i<array->data.size();i++){
			filter_data.ranges[i]/=1000; //en metros
	}
	
	torso_pub.publish(filter_data);
	
	
	return;
}
