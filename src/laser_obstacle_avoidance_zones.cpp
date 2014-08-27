/*
Copyright (c) 2005, Brad Kratochvil, Toby Collett, Brian Gerkey, Andrew Howard, ...
All rights reserved.

Redistribution and use in source and binary forms, with or without
modification, are permitted provided that the following conditions are met:

    * Redistributions of source code must retain the above copyright notice,
      this list of conditions and the following disclaimer.
    * Redistributions in binary form must reproduce the above copyright notice,
      this list of conditions and the following disclaimer in the documentation
      and/or other materials provided with the distribution.
    * Neither the name of the Player Project nor the names of its contributors
      may be used to endorse or promote products derived from this software
      without specific prior written permission.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS" AND
ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE IMPLIED
WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR
ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON
ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS
SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
*/

/*
 * laser_obstacle_avoidance_zones.cpp
 *
 * a simple obstacle avoidance demo
 *
 * @todo: implement dinamic factors
 */

#include "ros/ros.h"
#include <stdio.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>

#define PI 3.1415
#define INF 100000000
#define min(a,b) a<b?a:b

using namespace std;

float min_left_dist;
float min_right_dist;
double l,r;
double newspeed = 0;
double newturnrate = 0;
geometry_msgs::Twist cmd_vel;
ros::Publisher cmd_vel_pub;

float offset_degrees = 45; // degrees to ignore at the sides
float offset_samples = offset_degrees * 4; // 4 samples per degree

//float max_distance = 100.0;
float max_distance = 1.0; // before 0.7

float force_turning_right_difference = 20.0;

float max_speed = 0.1; // m/s
float speed_ratio = max_distance*2/max_speed;

float max_turning_speed_rad = 0.5; // rad/sec
float turning_ratio_rad = (100+max_distance)/max_turning_speed_rad;


double limit(double n, double min, double max)
{
	if(n < min)
		return min;
	else if(n > max)
		return max;
	else return n;
}

// degrees to radians
float dtor(double degrees)
{
	return degrees * PI / 180;
}

float get_min_distance_left(sensor_msgs::LaserScan laser)
{
	float min_left_dist = 1000.0;

	for(int i=laser.ranges.size()/2; i<laser.ranges.size()-offset_samples; i++)
	{
		if(laser.ranges[i] < min_left_dist)
			min_left_dist = laser.ranges[i];
	}
	return min_left_dist;
}

float get_min_distance_right(sensor_msgs::LaserScan laser)
{
	float min_right_dist = 1000.0;

	for(int i=offset_samples; i<laser.ranges.size()/2; i++)
	{
		if(laser.ranges[i] < min_right_dist)
			min_right_dist = laser.ranges[i];
	}
	return min_right_dist;
}

float get_min_distance_in_range(sensor_msgs::LaserScan laser, float min_degree, float max_degree)
{
	
	float limit_degrees = abs(laser.angle_min * 180 / PI);
	
	if(min_degree > max_degree)
	{
		float aux = max_degree;
		max_degree = min_degree;
		min_degree = aux;
	}
	
	//cout << "limit_degrees: " << limit_degrees << endl;
	
	// to start from 0
	min_degree += limit_degrees;
	max_degree += limit_degrees;
	
	//cout << "min_degree: " << min_degree << " max_degree: " << max_degree << endl;
	
	float min_rad = min_degree / 180 * PI;
	float max_rad = max_degree / 180 * PI;
	
	int min_offset = (int)(min_rad / laser.angle_increment);
	int max_offset = (int)(max_rad / laser.angle_increment);
	//cout << "min_rad: " << min_rad << " max_rad: " << max_rad << " angle_increment: " << laser.angle_increment << endl;
	//cout << "laser_size: " << laser.ranges.size() << " min_offset: " << min_offset << " max_offset: " << max_offset << endl;
	
	float min = INF;
	
	for(int i=min_offset; i<max_offset; i++)
	{
			if(laser.ranges[i] < min)
				min = laser.ranges[i];
	}
	return min;
	
}


// detect obstacle with rectangle of length: y_region and width: x_region * 2
bool check_front_obstacle(sensor_msgs::LaserScan laser, float x_region, float y_region)
{
	// only use readings from -90º to 90º
	int offset = dtor(30) / laser.angle_increment;
	
	
	float min = INF;
	float x_min,y_min = 0.0;
	float angle, angle_min = 0.0;
	for(int i=offset; i<laser.ranges.size()-offset; i++)
	{
		// limit readings
		if (laser.ranges[i] > 0.05 && laser.ranges[i] < 1.0) // 1.0
		{
			angle = laser.angle_min + i * laser.angle_increment;
			double x = laser.ranges[i] * sin(angle);
			double y = laser.ranges[i] * cos(angle);
			//cout << "x: " << x << " ,y: " << y << " ,dist: " << laser.ranges[i] << endl;
			//if(laser.ranges[i] < min)
			//{
				//min = laser.ranges[i];
				//x_min = x;
				//y_min = y;
				//angle_min = angle;
			//}
			if ((fabs(x) < x_region) && (y < y_region)) 
			{
				//cout << "Point in x: " << x_min << " ,y: " << y_min << " ,angle: " << angle << " ,dist: " << min << endl;
				return true;
			}
		}
	}
	//cout << "Point in x: " << x_min << " ,y: " << y_min << " ,angle: " << angle << " ,dist: " << min << endl;
	return false;
}

// TODO: doesn´t work
bool check_front_obstacle_semicircle(sensor_msgs::LaserScan laser, float radius)
{
	int offset = 90 * 4; // 4 samples per degree
	float x_center = 0.0;
	float y_center = 0.5;
	float dist_hokuyo_to_center = 0.25;
	
	for(int i=offset; i<laser.ranges.size()-offset; i++)
	{
		// limit readings
		if (laser.ranges[i] > 0.05 && laser.ranges[i] < 1.0) 
		{
			double x = laser.ranges[i] * sin(laser.angle_min + i * laser.angle_increment);
			double y = laser.ranges[i] * cos(laser.angle_min + i * laser.angle_increment);
			
			y = y - dist_hokuyo_to_center;
			cout << "x: " << x << " ,y: " << y << " ,dist: " << laser.ranges[i] << endl;
			return true;
			float dist = sqrt(x*x + y*y);
			if(dist < radius)
			{
				cout << "distance: " << dist << endl;
				return true;
			}
			
		}
	}
	return false;
}

void autonomous_behave(const sensor_msgs::LaserScan &laser)
{
	newspeed = newturnrate = 0.0;
	
	min_left_dist = get_min_distance_left(laser);
	min_right_dist = get_min_distance_right(laser);
	
	float l1 = get_min_distance_in_range(laser, 0, 30);
	float l2 = get_min_distance_in_range(laser, 30, 60);
	float l3 = get_min_distance_in_range(laser, 60, 90);
	
	float r1 = get_min_distance_in_range(laser, -30, 0);
	float r2 = get_min_distance_in_range(laser, -60, -30);
	float r3 = get_min_distance_in_range(laser, -90, -60);
	
	cout << "----------------------------" << endl;

	//cout << "l3: " << l3 << " l2: " << l2 << " l1: " << l1 << " r1: " << r1 << " r2: " << r2 << " r3: " << r3 << endl;
	
	l = min_left_dist;
	r = min_right_dist;
    
  //cout << "l: " << l << endl;
  //cout << "r: " << r << endl;
	
	//if(abs(l - r) < 1000)
	//	cout << "Same distance" << endl;
	
	// limit max distance
	if (l > max_distance)
        l = max_distance;
  if (r > max_distance)
        r = max_distance;
        
  l1 = limit(l1,0,max_distance);
  l2 = limit(l2,0,max_distance);
  l3 = limit(l3,0,max_distance);
  r1 = limit(r1,0,max_distance);
  r2 = limit(r2,0,max_distance);
  r3 = limit(r3,0,max_distance);
  


  
  float v1 = l1 + r1;
  float v2 = l2 + r2;
  float v3 = l3 + r3;
  
  // assign weights dinamically
  float p1 = 0.5;
  float p2 = 0.3;
  float p3 = 0.2;
  float z1 = min(l1,r1);
  float z2 = min(l2,r2);
  float z3 = min(l3,r3);
  float f1,f2,f3 = 0.0;
  
  //// @todo: horrible implementation, use std::sort instead
  //if(z1 < z2 && z1 < z3)
	//{
	  //f1 = p1;
	  //if(z2 < z3)
	  //{
			//f2 = p2;
			//f3 = p3;
		//}
		//else
		//{
			//f2 = p3;
			//f3 = p2;
		//}
	//}
	//else if(z2 < z1 && z2 < z3)
	//{
		//f2 = p1;
		//if(z1 < z3)
	  //{
			//f1 = p2;
			//f3 = p3;
		//}
		//else
		//{
			//f1 = p3;
			//f3 = p2;
		//}
	//}
	//else
	//{
		//f3 = p1;
		//if(z1 < z2)
	  //{
			//f1 = p2;
			//f2 = p3;
		//}
		//else
		//{
			//f1 = p3;
			//f2 = p2;
		//}
	//}
  f1 = 0.2;
  f2 = 0.3;
  f3 = 0.5;
  
  //cout << "l1: " << l1 << " r1: " << r1 << " ,weight: " << f1 << endl;
  //cout << "l2: " << l2 << " r2: " << r2 << " ,weight: " << f2 << endl;
  //cout << "l3: " << l3 << " r3: " << r3 << " ,weight: " << f3 << endl;
  
	
	if(!check_front_obstacle(laser,0.35,0.55))
		newspeed = (v1*f1 + v2*f2 + v3*f3)/speed_ratio;
	else 
	{
		cout << "colision detected!!" << endl;
		newspeed = 0.0;
	}
	newspeed = limit(newspeed, -max_speed, max_speed);


	float w1 = l1 - r1;
	float w2 = l2 - r2;
	float w3 = l3 - r3;
	
	newturnrate = w1*f1 + w2*f2 + w3*f3;
	//cout << "turn rate: " << newturnrate << endl;
		 
	// degrees to radians
	//newturnrate = dtor(newturnrate);
	
	//newturnrate = newturnrate/(2.15/max_turning_speed_rad); // 2.15/max_turn_rate_rad
	
	//cout << "turn rate (rad): " << newturnrate << endl;
	
	// limit angular speed
	newturnrate = limit(newturnrate, -max_turning_speed_rad, max_turning_speed_rad);
	
	
	
	float limit_robot_stuck = 0.25;
	
	// check if robot is stuck
	if(newspeed == 0.0 && newturnrate < limit_robot_stuck)
	{
		if(newturnrate > 0)
			newturnrate = limit_robot_stuck;
		else if(newturnrate < 0)
			newturnrate = -limit_robot_stuck;
	}
	
	
	cout << "turn rate (rad limited): " << newturnrate << endl;
		
	//cout << "speed: " << newspeed << "turn: " << newturnrate << endl;
	
	
	
	cmd_vel.linear.x = newspeed;
	cmd_vel.linear.y = 0.0;
	cmd_vel.linear.z = 0.0;
	cmd_vel.angular.x = 0.0;
	cmd_vel.angular.y = 0.0;
	cmd_vel.angular.z = newturnrate;
	
	cmd_vel_pub.publish(cmd_vel);
    
	
}



int main(int argc, char **argv)
{
		ros::init(argc, argv, "laser_obstacle_avoidance");
    ros::NodeHandle nh;
    cmd_vel_pub = nh.advertise<geometry_msgs::Twist>("/summit_xl_controller/command", 1);
    ros::Subscriber laser_sub = nh.subscribe("scan", 10, autonomous_behave);
    
    ros::spin();
}
