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
 * Last modified on: 22.12.2020
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

///////////////////////////////////////////// Global variables /////////////////////////////////////////////

ofstream logFile;				  	// Log file to save test parameters

// Pubs and subs
ros::Subscriber pos_sub;				// Subscriber to odometry
ros::Subscriber ft_sensor_sub;				// Subscriber to F/T sensor
ros::Subscriber rotor_speed_sub;			// Subscriber to rotor angular velocity		
ros::Publisher  pos_pub;				// Publisher for position controller
ros::Publisher  ft_filtered_pub;			// Publisher for filtered F/T data
ros::Publisher  thrust_pub;				// Publisher for current Thrust value
const int PUB_TIME = 1;				  	// New waypoint is published every 'PUB_TIME' seconds during mid-phase

// Force/torque variables
Eigen::Vector3f raw_forces;
Eigen::Vector3f raw_torques;
double force_magnitude;
const double MIN_LATERAL_FORCE_THRESHOLD  = 0.005; 	// [N]
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
Eigen::Vector3f forces;
Eigen::Vector3f torques;
double force_y[N];
double force_z[N];
double torque_x[N];
double sum_force_y  = 0.0;
double sum_force_z  = 0.0;
double sum_torque_x = 0.0;
bool first_flag     = true;

// Position controller variables
geometry_msgs::PoseStamped pos_msg;
std_msgs::Float32 thrust_msg;
double starting_time;
double desired_heading, commanded_heading;
double first_interaction_lateral_position;
double lateral_slide = 0.0;
double MAX_LATERAL_SLIDING = 0.005;	  		// [m]
double pushing_effort       = 1;
double last_effort;
const double DELTA_PUSH	    = 0.2;
const double MAX_EFFORT     = 500;
const double CONTROL_GAIN_Y = 0.1; 	 	 	// Controller Gain for Y component
const double CONTROL_GAIN_Z = 0.01;		 	// Controller Gain for Z component
const double MAX_HEADING    = M_PI/2;		 	// Max desired heading angle [rad], equal to 90 deg
bool sliding_flag           = false;
bool contact_flag	    = false;

// Estimated parameters
double obstacle_inclination, friction_force, normal_force, mu;

bool obstacle_pos = false;

///////////////////////////////////////////// Callbacks /////////////////////////////////////////////

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

///////////////////////////////////////////// Functions /////////////////////////////////////////////

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

	// Wait for 5 seconds to let the Gazebo GUI show up.
  	ros::Duration(5.0).sleep();
}

// Filtering force/torque sensor measurements
void FilterFTSensor()
{
	int i;		
	// FIR: Moving Average Filtering
	if(first_flag)
	{
		for(i = 0; i < N; i++)
		{
			ros::spinOnce();
			// Fill the first N elements
			force_y[i]    = raw_forces[1];
			force_z[i]    = raw_forces[2];
			torque_x[i]   = raw_torques[0];
			sum_force_y  += force_y[i];
			sum_force_z  += force_z[i];
			sum_torque_x += torque_x[i];
		}
		first_flag = false;
	}
	else
	{
		// Shift the moving average window
		for(i = 0; i < N-1; i++)
		{
			force_y[i]  = force_y[i+1];
			force_z[i]  = force_z[i+1];
			torque_x[i] = torque_x[i+1];
		}
		// Add a new sample	
		ros::spinOnce();
		force_y[N-1]  = raw_forces[1];
		force_z[N-1]  = raw_forces[2];
		torque_x[N-1] = raw_torques[0];
		// Update the sum
		sum_force_y  += force_y[N-1];
		sum_force_z  += force_z[N-1];
		sum_torque_x += torque_x[N-1];

	}
	// Publish converted measurements for visualization
	ft_filtered_msg.force.y  = sum_force_y / N; 
	ft_filtered_msg.force.z  = sum_force_z / N; 
	ft_filtered_msg.torque.x = sum_torque_x / N;
	ft_filtered_pub.publish(ft_filtered_msg);

	// As the window will be shifted, remove the first element
	sum_force_y  -= force_y[0];
	sum_force_z  -= force_z[0];
	sum_torque_x -= torque_x[0];

	force_magnitude = sqrt(ft_filtered_msg.force.x*ft_filtered_msg.force.x + 
			       ft_filtered_msg.force.y*ft_filtered_msg.force.y + 
			       ft_filtered_msg.force.z*ft_filtered_msg.force.z);
}

// Compute the desired heading angle
void ComputeDesiredHeading()
{
	// Calculate desired angle from the components of the external force
	desired_heading = atan2(fabs(ft_filtered_msg.force.y), fabs(ft_filtered_msg.force.z));

	cout << "Desired heading angle: " << (180/M_PI)*desired_heading << " [deg], " 
                                          <<            desired_heading << " [rad]" 
				          				<< endl;
}

// Compute obstacle inclination angle, friction force, normal force and friction coefficient
void ComputeObstacleInformation()
{
	double A = ft_filtered_msg.force.z + CAGE_WEIGHT_OFFSET;
	double B = ft_filtered_msg.force.y;	
	double C = -ft_filtered_msg.torque.x/CAGE_RADIUS;

	// Inclination angle of the obstacle
	obstacle_inclination = 2 * atan((A + sqrt(A*A + B*B - C*C))/(B + C));
	cout << "OBSTACLE INCLINATION: " << (180/M_PI)*obstacle_inclination << " [deg], " 
                                         <<            obstacle_inclination << " [rad]" 
					 << endl;

	// Friction Force
	if(obstacle_inclination <= -M_PI/2) 
	{
		friction_force = C;
		cout << "FRICTION FORCE: " << friction_force << " [N] (positive direction)" << endl;
	}
	else
	{
		friction_force = -C;
		cout << "FRICTION FORCE: " << friction_force << " [N] (negative direction)" << endl;
	}

	// Normal Force
	normal_force = (ft_filtered_msg.force.y - C*cos(obstacle_inclination))/sin(obstacle_inclination);
	if(ft_filtered_msg.force.z <= 0)
		cout << "NORMAL FORCE: " << normal_force << " [N] (pointing down)" << endl;
	else
		cout << "NORMAL FORCE: " << normal_force << " [N] (pointing up)"   << endl;

	// Friction coefficient
	mu = fabs(friction_force/normal_force);
	cout << "FRICTION COEFFICIENT: " << mu << endl;
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
}

// State Machine handler
void StateMachine()
{
	cout << "\r\n\n\033[32m\033[1m--------------------------------------------------------------\033[0m" << endl;
	
	// Print force filtered data
	cout << "Rolling moment (x) [Nm] " << ft_filtered_msg.torque.x << endl;
	cout << "Lateral Force (y) [N]: "  << ft_filtered_msg.force.y  << endl;
	cout << "Vertical Force (z) [N]: " << ft_filtered_msg.force.z  << endl;
	cout << "Force magnitude [N]: "    << force_magnitude          << endl;
	cout << "-------" 		                               << endl;
	
	// Calculate contact parameters
	ComputeObstacleInformation();
	cout << "-------" << endl;

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
		last_effort = pushing_effort;
	}

	// Threshold overcome: obstacle too compliant, the drone is slipping, no need to push more
	//else if(fabs(ft_filtered_msg.force.y) > MAX_LATERAL_FORCE_THRESHOLD)
	if(lateral_slide > MAX_LATERAL_SLIDING)
	{
		cout << "\033[32m\033[0mRESTING PHASE! \033[0m" << endl;

		// Compute desired heading only before the drone starts to slide
		if(!sliding_flag)
			ComputeDesiredHeading();

		// Update variable
		sliding_flag = true;

		// Adaptive adjustment of the desired heading angle and the controller gain
		int sign = (ft_filtered_msg.torque.x > 0) ? -1 : 1;		
		ControlStrategy(last_effort*lateral_slide/MAX_LATERAL_SLIDING, sign*desired_heading);
		// Compute lateral slide
		lateral_slide = fabs(first_interaction_lateral_position - pos[1]);
	}
	
	// After Resting the demo is finished. Any attempts or lower again the thrust leads to slip

	// Print commanded heading angle and effort/gain
	cout << "COMMANDED ANGLE: " << (180/M_PI)*commanded_heading << " [deg], " 
                                    <<            commanded_heading << " [rad]" 
				                                    << endl;
	cout << "LAST EFFORT: "     << last_effort		    << endl;
	cout << "EFFORT: "          << pushing_effort               << endl;
	
	// Print lateral slide when in contact
	if(contact_flag)
		cout << "Lateral slide: " << 100*(fabs(first_interaction_lateral_position - pos[1])) << " [cm]" << endl;

	// Publiush and print current Thrust force value
	thrust_msg.data = thrust;
	thrust_pub.publish(thrust_msg);
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
	if(obstacle_pos)
	{
		cout << "Obstacle on the right" << endl;

		pos_msg.header.stamp = ros::Time::now();
		pos_msg.pose.position.x = pos[0];
		pos_msg.pose.position.y = 0.0;
		pos_msg.pose.position.z = 1.2;
		pos_pub.publish(pos_msg);

		sleep(2);
		ros::spinOnce();

		pos_msg.header.stamp = ros::Time::now();
		pos_msg.pose.position.x = pos[0];
		pos_msg.pose.position.y = 0.0;
		pos_msg.pose.position.z = 1.5;
		pos_pub.publish(pos_msg);

		sleep(1);
		ros::spinOnce();

		pos_msg.header.stamp = ros::Time::now();
		pos_msg.pose.position.x = pos[0];
		pos_msg.pose.position.y = 0.0;
		pos_msg.pose.position.z = 2.0;
		pos_pub.publish(pos_msg);

		sleep(2);
		ros::spinOnce();

		pos_msg.header.stamp = ros::Time::now();
		pos_msg.pose.position.x = pos[0];
		pos_msg.pose.position.y = 1.2;
		pos_msg.pose.position.z = 2.5;
		pos_pub.publish(pos_msg);
	}
	else
	{
		cout << "Obstacle on the left" << endl;

		pos_msg.header.stamp = ros::Time::now();
		pos_msg.pose.position.x = pos[0];
		pos_msg.pose.position.y = 1.2;
		pos_msg.pose.position.z = 1.2;
		pos_pub.publish(pos_msg);

		sleep(2);
		ros::spinOnce();

		pos_msg.header.stamp = ros::Time::now();
		pos_msg.pose.position.x = pos[0];
		pos_msg.pose.position.y = 1.2;
		pos_msg.pose.position.z = 1.5;
		pos_pub.publish(pos_msg);

		sleep(1);
		ros::spinOnce();

		pos_msg.header.stamp = ros::Time::now();
		pos_msg.pose.position.x = pos[0];
		pos_msg.pose.position.y = 1.2;
		pos_msg.pose.position.z = 2.0;
		pos_pub.publish(pos_msg);

		sleep(2);
		ros::spinOnce();

		pos_msg.header.stamp = ros::Time::now();
		pos_msg.pose.position.x = pos[0];
		pos_msg.pose.position.y = 0.4;
		pos_msg.pose.position.z = 2.5;
		pos_pub.publish(pos_msg);
	}

	sleep(7);
	ros::spinOnce();

	hovering_thrust = thrust;
	cout << "Home position reached. Hovering Thrust: " << hovering_thrust << " [N] " << endl;
}

///////////////////////////////////////////// Main function /////////////////////////////////////////////
int main(int argc, char** argv)
{
	ros::init(argc, argv, "high_level_2D_resting_waypoint_controller");
	ros::NodeHandle nh;
	ros::Rate loop(100);

	//obstacle_pos = true; 	// On the right
	obstacle_pos = false;	// On the left

	// Subscribers and Publishers 
	pos_sub         = nh.subscribe("/haptic_drone_with_structure/odometry_sensor1/position", 10, &positionCallback);
	ft_sensor_sub   = nh.subscribe("/haptic_drone_with_structure/ft_sensor_topic", 10, &forceTorqueSensorCallback);
	rotor_speed_sub	= nh.subscribe("/haptic_drone_with_structure/motor_speed", 10, &rotorSpeedCallback);
	pos_pub         = nh.advertise<geometry_msgs::PoseStamped>(mav_msgs::default_topics::COMMAND_POSE, 10);
	ft_filtered_pub = nh.advertise<geometry_msgs::Wrench>("/haptic_drone_with_structure/ft_filtered_measurements", 10);
	thrust_pub	= nh.advertise<std_msgs::Float32>("/haptic_drone_with_structure/thrust", 10);
	
	sleep(5);

	ros::spinOnce();
	
	// Initialization of the demo
	cout << "\r\n\n\n\033[32m\033[1mCLICK TO START GAZEBO\033[0m" << endl; 
	while ((getchar()) != '\n');
	UnpauseGazebo();	
	cout << "\r\n\n\n\033[32m\033[1mGAZEBO STARTED! \033[0m" << endl; 

	ros::spinOnce();
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
				 		
		// State Machine running!
		StateMachine();
		
		// Open log file, save data and close it
		/*
		logFile.open("/home/emanuele/catkin_ws/haptic_drone_with_structure_log_file.txt", std::ofstream::app);
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
