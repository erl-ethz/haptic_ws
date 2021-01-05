/*
 * Emanuele Aucone, ERL, ETH Zurich, Switzerland
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0

 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 *
 * Last modified on: 05.01.2021
 */

#include <thread>
#include <chrono>
#include <math.h>
#include <iostream>
#include <fstream>

#include <Eigen/Core>
#include <mav_msgs/conversions.h>
#include <mav_msgs/default_topics.h>
#include <ros/ros.h>
#include <std_srvs/Empty.h>
#include <trajectory_msgs/MultiDOFJointTrajectory.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/WrenchStamped.h>
#include <mav_msgs/Actuators.h>
#include <std_msgs/Float32.h>
#include <Eigen/Geometry>

using namespace std;

//////////////////// Global variables ////////////////////

ofstream logFile;				  	// Log file to save test parameters

// Pubs and subs
ros::Subscriber pos_sub;				// Subscriber to odometry
ros::Subscriber ft_sensor_sub;				// Subscriber to F/T sensor
ros::Subscriber rotor_speed_sub;			// Subscriber to rotor angular velocity		
ros::Publisher  pos_pub;				// Publisher for position controller
ros::Publisher  ft_filtered_pub;			// Publisher for filtered F/T data
ros::Publisher  thrust_pub;				// Publisher for current Thrust value
const int PUB_TIME = 1;				  	// New waypoint is published every 'PUB_TIME' seconds

// Force/torque variables
Eigen::Vector3f raw_forces;
Eigen::Vector3f raw_torques;
double force_magnitude;
const double MIN_LATERAL_FORCE_THRESHOLD  = 0.002; 	// [N]
const double MAX_LATERAL_FORCE_THRESHOLD  = 2.6;   	// [N]
const double CAGE_WEIGHT_OFFSET           = 0.147;  	// [N]
const double CAGE_RADIUS                  = 0.255;  	// [m]

// UAV variables
Eigen::Vector3f pos;				  	// UAV global position [m]
Eigen::Vector4f rotor_speed;				// Angular velocity of the rotors [rad/s]
double thrust;						// Current Thrust force value
const double coeff_thrust = 0.00000854858;		// Thrust coefficient for our drone (from Parrot Ardrone in RotorS)
double hovering_thrust;					// Thrust value while hovering

// Moving Average Filter variables
geometry_msgs::Wrench ft_filtered_msg;
const int N = 50;				  	// Window's size
double forces_cum[3][N];
double torques_cum[3][N];
Eigen::Vector3f sum_forces;
Eigen::Vector3f sum_torques;
bool first_flag = true;

// Position controller variables
geometry_msgs::PoseStamped pos_msg;
std_msgs::Float32 thrust_msg;
double starting_time;
double desired_heading, commanded_heading;
double first_interaction_lateral_position;
double lateral_slide = 0.0;
double MAX_LATERAL_SLIDING = 0.01;	  		// [m]
double pushing_effort       = 1;
const double DELTA_PUSH	    = 0.2;
const double MAX_EFFORT     = 500;
const double CONTROL_GAIN_Y = 0.01; 	 	 	// Controller Gain for Y component
const double CONTROL_GAIN_Z = 0.001;		 	// Controller Gain for Z component
const double MAX_HEADING    = M_PI/2;		 	// Max desired heading angle [rad], equal to 90 deg
bool sliding_flag           = false;
bool contact_flag	    = false;

//////////////////// Functions ////////////////////

// Unpause Gazebo simulation environment
void UnpauseGazebo()
{
	std_srvs::Empty srv;
	bool unpaused = ros::service::call("/gazebo/unpause_physics", srv);
	unsigned int i = 0;

	// Trying to unpause Gazebo for 10 seconds.
	while (i <= 10 && !unpaused)
	{
		ROS_INFO("Wait for 1 second before trying to unpause Gazebo again.");
		std::this_thread::sleep_for(std::chrono::seconds(1));
		unpaused = ros::service::call("/gazebo/unpause_physics", srv);
		++i;
	}

	if (!unpaused)
		ROS_FATAL("Could not wake up Gazebo.");
	else
		ROS_INFO("Unpaused the Gazebo simulation.");

	ros::spinOnce();

	pos_msg.header.stamp = ros::Time::now();
	pos_msg.pose.position.x = 0.25;
	pos_msg.pose.position.y = 0.0;
	pos_msg.pose.position.z = 0.45;
	pos_pub.publish(pos_msg);

	// Wait for 5 seconds to let the Gazebo GUI show up.
  	ros::Duration(5.0).sleep();
}

// Callback for drone position data acquisition from odometry sensor
void positionCallback(const geometry_msgs::PointStamped::ConstPtr& position_msg)
{
	// Store position data for control strategy
	pos[0] = position_msg->point.x;
	pos[1] = position_msg->point.y;
	pos[2] = position_msg->point.z;
}

// Callback for rotor/propellers angular velocity
void rotorSpeedCallback(const mav_msgs::Actuators::ConstPtr& omega_msg)
{
	// Store rotors speed data
	rotor_speed[0] = omega_msg->angular_velocities[0];
	rotor_speed[1] = omega_msg->angular_velocities[1];
	rotor_speed[2] = omega_msg->angular_velocities[2];
	rotor_speed[3] = omega_msg->angular_velocities[3];

	thrust = coeff_thrust * ( rotor_speed[0]*rotor_speed[0] +
				  rotor_speed[1]*rotor_speed[1] +
				  rotor_speed[2]*rotor_speed[2] +
				  rotor_speed[3]*rotor_speed[3] );
}

// Callback for force/torque measurement acquisition from FT sensor
void forceTorqueSensorCallback(const geometry_msgs::WrenchStamped::ConstPtr& ft_msg)
{
	// Store raw forces/torques data
	raw_forces[0]  = ft_msg->wrench.force.x;
	raw_forces[1]  = ft_msg->wrench.force.y;
	raw_forces[2]  = ft_msg->wrench.force.z;
	raw_torques[0] = ft_msg->wrench.torque.x;
	raw_torques[1] = ft_msg->wrench.torque.y;
	raw_torques[2] = ft_msg->wrench.torque.z;
}

// Filtering force/torque sensor measurements
void FilterFTSensor()
{
	int i, j;		
	// FIR: Moving Average Filtering
	if(first_flag)
	{
		for(i = 0; i < N; i++)
		{
			ros::spinOnce();
			// Fill the first N elements for the 3 forces and 3 torques
			for(j = 0; j < 3; j++)
			{
				forces_cum[j][i]  = raw_forces[j];
				torques_cum[j][i] = raw_torques[j];
				sum_forces[j]    += forces_cum[j][i];
				sum_torques[j]   += torques_cum[j][i];
			}
		}
		first_flag = false;
	}
	else
	{
		// Shift the moving average window
		for(i = 0; i < N-1; i++)
		{
			for(j = 0; j < 3; j++)
			{
				forces_cum[j][i]  = forces_cum[j][i+1];
				torques_cum[j][i] = torques_cum[j][i+1];
			}
		}
			
		ros::spinOnce();
		for(j = 0; j < 3; j++)
		{
			// Add new samples
			forces_cum[j][N-1]  = raw_forces[j];
			torques_cum[j][N-1] = raw_torques[j];
			// Update the sum
			sum_forces[j]      += forces_cum[j][N-1];
			sum_torques[j]     += torques_cum[j][N-1];
		}
	}
	// Publish converted measurements for visualization
	ft_filtered_msg.force.x  = sum_forces[0] / N;
	ft_filtered_msg.force.y  = sum_forces[1] / N; 
	ft_filtered_msg.force.z  = sum_forces[2] / N; 
	ft_filtered_msg.torque.x = sum_torques[0] / N;
	ft_filtered_msg.torque.y = sum_torques[1] / N;
	ft_filtered_msg.torque.z = sum_torques[2] / N;
	ft_filtered_pub.publish(ft_filtered_msg);

	// As the window will be shifted, remove the first elements
	for(j = 0; j < 3; j++)
	{
		sum_forces[j]  -= forces_cum[j][0];
		sum_torques[j] -= torques_cum[j][0];
	}

	// Compute force magnitude
	force_magnitude = sqrt(ft_filtered_msg.force.x*ft_filtered_msg.force.x + 
			       ft_filtered_msg.force.y*ft_filtered_msg.force.y + 
			       ft_filtered_msg.force.z*ft_filtered_msg.force.z);
}

// Compute the desired heading angle
void ComputeDesiredHeading()
{
	// Calculate desired angle from the components of the external force
	desired_heading = atan2(ft_filtered_msg.force.y, ft_filtered_msg.force.z);

	cout << "Desired heading angle: " << (180/M_PI)*desired_heading << " [deg], " 
                                          <<            desired_heading << " [rad]" 
				          				<< endl;
}

// New waypoint computed with respect to the ellipsoid formulation (upper waypoint in case of sliding)
void ControlStrategy(double effort, double heading_d)
{
	cout << "-------" << endl;

	// Saturation of the commanded heading angle
	if(heading_d > MAX_HEADING)
		heading_d = MAX_HEADING;
	else if(heading_d < -MAX_HEADING)
		heading_d = -MAX_HEADING;

	// Staturation of the pushing effort
	if(effort >= MAX_EFFORT)
		effort = MAX_EFFORT;

	// Compose controller message
	pos_msg.header.stamp = ros::Time::now();
	// X position is always fixed (because the drone is constraint to the structure)
	pos_msg.pose.position.x = pos[0];
	// Change Y,Z coordinate depending on the current state of the system
	pos_msg.pose.position.y = pos[1] - (effort * CONTROL_GAIN_Y * sin(heading_d));
	pos_msg.pose.position.z = pos[2] - (effort * CONTROL_GAIN_Z * cos(heading_d));
	pos_pub.publish(pos_msg);

	// Update control parameters
	commanded_heading = heading_d;
	pushing_effort = effort;

	cout << "COMMANDED ANGLE: " << (180/M_PI)*commanded_heading << " [deg], " 
                                    <<            commanded_heading << " [rad]" 
				                                    << endl;
	cout << "EFFORT: "          << effort                       << endl;
}

// State Machine handler
void StateMachine()
{
	cout << "\r\n\n\033[32m\033[1m--------------------------------------------------------------\033[0m" << endl;
	
	// Print force filtered data
	cout << "Lateral Force (x) [N]: "  << ft_filtered_msg.force.x   << endl;
	cout << "Lateral Force (y) [N]: "  << ft_filtered_msg.force.y   << endl;
	cout << "Vertical Force (z) [N]: " << ft_filtered_msg.force.z   << endl;
	cout << "Force magnitude [N]: "    << force_magnitude           << endl;
	cout << "Rolling moment (x) [Nm] " << ft_filtered_msg.torque.x  << endl;
	cout << "Pitching moment (y) [Nm] " << ft_filtered_msg.torque.y << endl;
	cout << "-------" 		                                << endl;

	if(!sliding_flag)
	{
		// No contact detected: drone in landing phase (flying down)
		if(fabs(ft_filtered_msg.force.y) < MIN_LATERAL_FORCE_THRESHOLD && !contact_flag)
		{
			cout << "\033[32m\033[0mLANDING PHASE! \033[0m" << endl;

			// Reset variables	
			pushing_effort = 1;	
			sliding_flag   = false;
			contact_flag = false;
			starting_time = ros::Time::now().toSec();

			// Height reference decreased of a fixed amount
			ControlStrategy(pushing_effort, 0);
		}
		// Contact detected: lay on the osbtacle to minimize the energy until a threshold in lateral or vertical force is overcome
		else if((fabs(ft_filtered_msg.force.y) > MIN_LATERAL_FORCE_THRESHOLD && fabs(ft_filtered_msg.force.y) < MAX_LATERAL_FORCE_THRESHOLD))
		{
			cout << "\033[32m\033[0mCONTACT DETECTED: MINIMIZING ENERGY! \033[0m" << endl;
			
			// First time in contact
			if(!contact_flag)
			{
				// Save current lateral position
				first_interaction_lateral_position = pos[1];			
				contact_flag = true;
			}

			// Lower down the reference every PUB_TIME seconds
			if((ros::Time::now().toSec() - starting_time) > PUB_TIME) 
			{
				pushing_effort += DELTA_PUSH;
				starting_time = ros::Time::now().toSec();
			}
		
			// Decrease the height reference to lay more and more on the detected obstacle
			ControlStrategy(pushing_effort, 0);

			sliding_flag = false;
			// Compute lateral slide
			lateral_slide = fabs(first_interaction_lateral_position - pos[1]);
		}
	}

	// Threshold overcome: obstacle too compliant, the drone is slipping, no need to push more
	//else if(fabs(ft_filtered_msg.force.y) > MAX_LATERAL_FORCE_THRESHOLD)
	/*if(lateral_slide > MAX_LATERAL_SLIDING)
	{
		cout << "\033[32m\033[0mRESTING PHASE! \033[0m" << endl;

		// Compute desired heading only before the drone starts to slide
		if(!sliding_flag)
			ComputeDesiredHeading();

		// Update variable
		sliding_flag = true;

		// Adaptive adjustment of the desired heading angle
		int sign = (ft_filtered_msg.torque.x > 0) ? -1 : 1;		
		ControlStrategy(pushing_effort*lateral_slide/MAX_LATERAL_SLIDING, sign*desired_heading); 
		// Compute lateral slide
		lateral_slide = fabs(first_interaction_lateral_position - pos[1]);
	}
	*/
	// After Resting the demo is finished. Any attempts or lower again the thrust leads to slip
			
	cout << "Lateral slide: " << 100*(fabs(first_interaction_lateral_position - pos[1])) << " [cm]" << endl;
	// Print current Thrust force value
	cout << "THRUST: " << thrust << " [N]. Energy minimized: " << 100*(1-thrust/hovering_thrust) << "%" << endl;

	// Print current position and commanded waypoint
	cout << "-------"                      << endl;	
	cout << "Current position = X: "       << pos[0] 
             <<                  ", Y: "       << pos[1] 
             <<                  ", Z: "       << pos[2] 
	                                       << endl;
	cout << "Publishing new waypoint! X: " << pos_msg.pose.position.x 
             <<                        ", Y: " << pos_msg.pose.position.y 
             <<                        ", Z: " << pos_msg.pose.position.z 
	                                       << endl;
}

// Reach home position above the resting spot and be ready for resting
void HomePosition()
{
	pos_msg.header.stamp = ros::Time::now();
	pos_msg.pose.position.x = 0.75;
	pos_msg.pose.position.y = 0.0;
	pos_msg.pose.position.z = 0.75;
	pos_pub.publish(pos_msg);

	sleep(1);
	ros::spinOnce();

	pos_msg.header.stamp = ros::Time::now();
	pos_msg.pose.position.x = 0.75;
	pos_msg.pose.position.y = 0.0;
	pos_msg.pose.position.z = 1.5;
	pos_pub.publish(pos_msg);

	sleep(1);
	ros::spinOnce();

	pos_msg.header.stamp = ros::Time::now();
	pos_msg.pose.position.x = 0.75;
	pos_msg.pose.position.y = 0.2;
	pos_msg.pose.position.z = 2.0;
	pos_pub.publish(pos_msg);

	sleep(1);
	ros::spinOnce();

	pos_msg.header.stamp = ros::Time::now();
	pos_msg.pose.position.x = 0.75;
	pos_msg.pose.position.y = 0.6;
	pos_msg.pose.position.z = 2.25;
	pos_pub.publish(pos_msg);

	sleep(1);
	ros::spinOnce();

	pos_msg.header.stamp = ros::Time::now();
	pos_msg.pose.position.x = 0.75;
	pos_msg.pose.position.y = 1.2;
	pos_msg.pose.position.z = 2.5;
	pos_pub.publish(pos_msg);

	sleep(10);
	ros::spinOnce();

	hovering_thrust = thrust;
	cout << "Home position reached. Hovering Thrust: " << hovering_thrust << " [N] " << endl;
}

// Main function
int main(int argc, char** argv)
{
	ros::init(argc, argv, "high_level_3D_resting_waypoint_controller");
	ros::NodeHandle nh;
	ros::Rate loop(100);

	// Subscribers and Publishers 
	pos_sub         = nh.subscribe("/haptic_drone/odometry_sensor1/position", 10, &positionCallback);
	ft_sensor_sub   = nh.subscribe("/haptic_drone/ft_sensor_topic", 10, &forceTorqueSensorCallback);
	rotor_speed_sub	= nh.subscribe("/haptic_drone/motor_speed", 10, &rotorSpeedCallback);
	pos_pub         = nh.advertise<geometry_msgs::PoseStamped>(mav_msgs::default_topics::COMMAND_POSE, 10);
	ft_filtered_pub = nh.advertise<geometry_msgs::Wrench>("/haptic_drone/ft_filtered_measurements", 10);
	thrust_pub	= nh.advertise<std_msgs::Float32>("/haptic_drone/thrust", 10);
	
	sleep(5);

	ros::spinOnce();
	
	// Initialization of the demo
	cout << "\r\n\n\n\033[32m\033[1mCLICK TO START GAZEBO\033[0m" << endl; 
	while ((getchar()) != '\n');
	UnpauseGazebo();	
	cout << "\r\n\n\n\033[32m\033[1mGAZEBO STARTED! \033[0m" << endl; 

	// Reach home position
	HomePosition();

	// Initialization of the demo
	cout << "\r\n\n\n\033[32m\033[1mCLICK TO START LANDING\033[0m" << endl; 
	while ((getchar()) != '\n');	
	cout << "\r\n\n\n\033[32m\033[1mLANDING STARTED! \033[0m" << endl; 
	
	//double starting_time = ros::Time::now().toSec();
	
	while(ros::ok())         
	{
		ros::spinOnce();

		// Filter force/torque measurements
		FilterFTSensor();
			
		// Publish thrust value
		thrust_msg.data = thrust;
		thrust_pub.publish(thrust_msg);
		cout << "THRUST: " << thrust << " [N]. Energy minimized: " << 100*(1-thrust/hovering_thrust) << "%" << endl;
	 		
		// State Machine running!
		//StateMachine();
		
		// Open log file, save data and close it
		/*
		logFile.open("/home/emanuele/catkin_ws/haptic_drone_log_file.txt", std::ofstream::app);
		logFile << pos[1] 		     	   << ", " 
			<< pos[2] 		     	   << ", " 
			<< pos_msg.pose.position.y   	   << ", " 
			<< pos_msg.pose.position.z   	   << ", " 
			<< (180/M_PI)*desired_heading  	   << ", "
			<< (180/M_PI)*commanded_heading	   << ", "
			<< pushing_effort	     	   << ", "
			<< ft_filtered_msg.torque.x  	   << ", " 
			<< ft_filtered_msg.force.y   	   << ", " 
			<< ft_filtered_msg.force.z  	   << ", " 
			<< thrust			   << ";" << endl;
		logFile.close();
		*/
		loop.sleep();          
	}
	return 0;
}
