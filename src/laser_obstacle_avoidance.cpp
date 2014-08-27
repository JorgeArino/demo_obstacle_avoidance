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
 * laserobstacleavoid.cc
 *
 * a simple obstacle avoidance demo
 *
 * @todo: this has been ported to libplayerc++, but not tested
 */

#include "ros/ros.h"
#include <stdio.h>
#include <iostream>
#include <geometry_msgs/Twist.h>
#include <sensor_msgs/LaserScan.h>
#include <math.h>

#define RAYS 1081
#define PI 3.1415

using namespace std;

float min_left_dist;
float min_right_dist;
double l,r;
double newspeed = 0;
double newturnrate = 0;
geometry_msgs::Twist cmd_vel;
ros::Publisher cmd_vel_pub;

float offset_degrees = 45; // degrees to ignore at the sides
float offset_samples = offset_degrees*4; // 4 samples per degree

//float max_distance = 100.0;
float max_distance = 0.7;

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

bool check_front_obstacle(sensor_msgs::LaserScan laser, float x_region, float y_region)
{
	int offset = 90 * 4; // 4 samples per degree
	
	for(int i=offset; i<laser.ranges.size()-offset; i++)
	{
		// limit readings
		if (laser.ranges[i] > 0.05 && laser.ranges[i] < 1.0) 
		{
			double x = laser.ranges[i] * sin(laser.angle_min + i * laser.angle_increment);
			double y = laser.ranges[i] * cos(laser.angle_min + i * laser.angle_increment);
			//cout << "x: " << x << " ,y: " << y << " ,dist: " << laser.ranges[i] << endl;
			if ((fabs(x) < x_region) && (y < y_region)) 
			{
				return true;
			}
		}
	}
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
	
	cout << "----------------------------" << endl;
	

	
	cout << "Minimum distance to the left: " << min_left_dist << endl;
	cout << "Minimum distance to the right: " << min_right_dist << endl;
	
	
	//l = (1e5*min_right_dist)/500-max_distance; 
  //r = (1e5*min_left_dist)/500-max_distance;

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
        
   
  cout << "l (limited): " << l << endl;
  cout << "r(limited): " << r << endl;

	/*
	if(l<max_distance || r<max_distance)
	{
		// TODO: force turning to one or another side depending on the sign of the difference
		if(abs(l-r) < force_turning_right_difference) 
		{
			cout << "difference: " << abs(l - r) << endl;
			cout << "Force turning" << endl;
			r = r-100; //turn to the other side
			
		}
	}
	*/
	
		if(!check_front_obstacle(laser,0.6,0.5))
			newspeed = (r+l)/speed_ratio;
		else 
		{
			cout << "colision detected!!" << endl;
			newspeed = 0.0;
		}
		newspeed = limit(newspeed, -max_speed, max_speed);
	

    
    
    //newturnrate = (r-l);
    newturnrate = l - r;
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
