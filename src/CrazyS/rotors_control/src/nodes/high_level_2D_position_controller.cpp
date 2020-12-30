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
 * Last modified on: 11.11.2020
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
#include <Eigen/Geometry>

using namespace std;

//////////////////// Global variables ////////////////////

ofstream logFile;				  	// Log file to save test parameters

// Pubs and subs
ros::Subscriber pos_sub;
ros::Publisher pos_pub;
ros::Subscriber ft_sensor_sub;
ros::Publisher ft_filtered_pub;
const int PUB_TIME = 1;				  	// New waypoint is published every 'PUB_TIME' seconds

// Force/torque variables
Eigen::Vector3f raw_forces;
Eigen::Vector3f raw_torques;
double force_magnitude;
const double MIN_LATERAL_FORCE_THRESHOLD  = 0.002; 	// [N]
const double MAX_LATERAL_FORCE_THRESHOLD  = 0.2;   	// [N]
const double MAX_VERTICAL_FORCE_THRESHOLD = 1;  	// [N]
const double CAGE_WEIGHT_OFFSET           = 0.147;  	// [N]
const double CAGE_RADIUS                  = 0.255;  	// [m]

// Position variables
Eigen::Vector3f pos;				  	// UAV global position

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
double desired_roll, commanded_roll;
double pushing_effort       = 1;
const double DELTA_PUSH	    = 0.1;
const double MAX_EFFORT     = 10;
const double LIMIT_HEIGHT   = 3.9;		 	// Max height waypoint [m]
const double CONTROL_GAIN_Y = 1; 	 	 	// Controller Gain for Y component
const double CONTROL_GAIN_Z = 0.1;		 	// Controller Gain for Z component
const double MIN_ROLL       = 0.08727;			// Min desired roll angle [rad], equal to 5 deg
const double MAX_ROLL       = 0.261799;		 	// Max desired roll angle [rad], equal to 15 deg
bool sliding_flag           = false;

// Estimated parameters
double obstacle_inclination, friction_force, normal_force, mu;

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

// Compute the desired roll angle
void ComputeDesiredRoll()
{
	// Calculate roll angle from the components of the external force
	// desired_roll = -atan2(ft_filtered_msg.force.y, fabs(ft_filtered_msg.force.z));
	desired_roll = atan2(fabs(ft_filtered_msg.force.y), fabs(ft_filtered_msg.force.z));

	cout << "Desired roll angle: " << (180/M_PI)*desired_roll << " [deg], " 
                                       <<            desired_roll << " [rad]" 
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
	if(obstacle_inclination <= 0) 
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
void ControlStrategy(double effort, double roll_d)
{
	cout << "-------" << endl;

	// Saturation of the commanded roll angle
	if(roll_d > MAX_ROLL)
		roll_d = MAX_ROLL;
	else if(roll_d < -MAX_ROLL)
		roll_d = -MAX_ROLL;

	// Staturation of the pushing effort
	if(effort >= MAX_EFFORT)
		effort = MAX_EFFORT;

	// Compose controller message
	pos_msg.header.stamp = ros::Time::now();
	// X position is always fixed (because the drone is constraint to the structure)
	pos_msg.pose.position.x = pos[0];
	// Change Y,Z coordinate depending on the current state of the system
	pos_msg.pose.position.y = pos[1] - (effort * CONTROL_GAIN_Y * sin(roll_d));
	pos_msg.pose.position.z = pos[2] + (effort * CONTROL_GAIN_Z * cos(roll_d));
	pos_pub.publish(pos_msg);

	// Update control parameters
	commanded_roll = roll_d;
	pushing_effort = effort;

	cout << "COMMANDED ANGLE: " << (180/M_PI)*commanded_roll << " [deg], " 
                                    <<            commanded_roll << " [rad]" 
				                                 << endl;

	cout << "EFFORT: "          << effort                    << endl;

}

// New waypoint computed as the lower waypoint for horizontal translation
void AlternativeControlStrategy(double effort, double roll_d)
{
	cout << "-------" << endl;

	// Staturation of the pushing effort
	if(effort >= MAX_EFFORT)
		effort = MAX_EFFORT;

	// Compose controller message
	pos_msg.header.stamp = ros::Time::now();
	// X position is always fixed (because the drone is constraint to the structure)
	pos_msg.pose.position.x = pos[0];
	// Change Y,Z coordinate depending on the current state of the system
	if(roll_d > 0)
		pos_msg.pose.position.y = pos[1] - (effort * CONTROL_GAIN_Y * cos(roll_d));
	else	
		pos_msg.pose.position.y = pos[1] + (effort * CONTROL_GAIN_Y * cos(roll_d));
	pos_msg.pose.position.z = pos[2] + (effort * CONTROL_GAIN_Z * sin(fabs(roll_d)));
	pos_pub.publish(pos_msg);

	// Update control parameters
	commanded_roll = roll_d;
	pushing_effort = effort;

	// Update control parameters
	commanded_roll = roll_d;
	pushing_effort = effort;

	cout << "COMMANDED ANGLE: " << (180/M_PI)*commanded_roll << " [deg], " 
                                    <<            commanded_roll << " [rad]" 
				                                 << endl;

	cout << "EFFORT: "          << effort                    << endl;
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

	// No contact detected: drone free to fly up
	if(fabs(ft_filtered_msg.force.y) < MIN_LATERAL_FORCE_THRESHOLD)
	{
		cout << "\033[32m\033[0mFREE FLIGHT MODE! \033[0m" << endl;

		// Reset variables	
		pushing_effort = 1;	
		sliding_flag   = false;

		// Free flight: height reference increased of a fixed amount
		ControlStrategy(pushing_effort, 0);
	}
	// Contact detected: push until a threshold in the lateral or vertical force is overcome
	else if(((fabs(ft_filtered_msg.force.y) > MIN_LATERAL_FORCE_THRESHOLD && fabs(ft_filtered_msg.force.y) < MAX_LATERAL_FORCE_THRESHOLD) && fabs(ft_filtered_msg.force.z) < MAX_VERTICAL_FORCE_THRESHOLD) && !sliding_flag)
	{
		cout << "\033[32m\033[0mCONTACT DETECTED: PUSH MODE! \033[0m" << endl;
		
		// Reset variable
		sliding_flag = false;

		// Increase the height reference to push against the detected obstacle 
		pushing_effort += DELTA_PUSH;
		ControlStrategy(pushing_effort, 0);
	}
	// Threshold overcome: obstacle not traversable
	else if(fabs(ft_filtered_msg.force.y) > MAX_LATERAL_FORCE_THRESHOLD || fabs(ft_filtered_msg.force.z) > MAX_VERTICAL_FORCE_THRESHOLD || pushing_effort == MAX_EFFORT)
	{
		cout << "\033[32m\033[0mOBSTACLE NOT TRAVERSABLE: SLIDE MODE! \033[0m" << endl;

		// Slide along the obstacle adjusting new position waypoint according to the desired roll
		if(!sliding_flag)
			ComputeDesiredRoll();

		// Update variable
		sliding_flag = true;

		int sign = (ft_filtered_msg.torque.x > 0) ? -1 : 1;
		if(fabs(desired_roll) > MIN_ROLL)
		{
			// Pure sliding: adaptive adjustment of the desired sliding heading angle		
			ControlStrategy(pushing_effort, sign*desired_roll*fabs(ft_filtered_msg.force.z)/MAX_VERTICAL_FORCE_THRESHOLD);
		}
		else
		{
			// Horizontal translation
			AlternativeControlStrategy(pushing_effort, sign*desired_roll);
			// Reset variables	
			pushing_effort = 1;	
			sliding_flag   = false;
		}
	}
	// Fault and failure handling
	else
	{
		// The drone was sliding
		if(sliding_flag)
		{
			cout << "\033[32m\033[0mPUSH MORE AND SLIDE AGAIN! \033[0m" << endl;

			// Update with last sliding roll command  and increased push
			pushing_effort += DELTA_PUSH;
			ControlStrategy(pushing_effort, commanded_roll);
		}
	}		
	
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

// Main function
int main(int argc, char** argv)
{
	ros::init(argc, argv, "high_level_2D_position_controller");
	ros::NodeHandle nh;
	ros::Rate loop(100);

	// Subscribers and Publishers 
	ft_sensor_sub   = nh.subscribe("/haptic_drone_with_structure/ft_sensor_topic", 10, &forceTorqueSensorCallback);
	pos_sub         = nh.subscribe("/haptic_drone_with_structure/odometry_sensor1/position", 10, &positionCallback);
	pos_pub         = nh.advertise<geometry_msgs::PoseStamped>(mav_msgs::default_topics::COMMAND_POSE, 10);
	ft_filtered_pub = nh.advertise<geometry_msgs::Wrench>("/haptic_drone_with_structure/ft_filtered_measurements", 10);
	
	sleep(5);

	ros::spinOnce();
	
	// Initialization of the demo
	cout << "\r\n\n\n\033[32m\033[1mCLICK TO START \033[0m" << endl; 
	while ((getchar()) != '\n');
	UnpauseGazebo();	
	cout << "\r\n\n\n\033[32m\033[1mSTARTED! \033[0m" << endl; 
	
	double starting_time = ros::Time::now().toSec();
	bool stop = false;
	
	while(ros::ok())         
	{
		ros::spinOnce();

		// Filter force/torque measurements
		FilterFTSensor();
				 		
		if(!stop)
		{
			// Check the status and control the drone every PUB_TIME seconds
			if((ros::Time::now().toSec() - starting_time) > PUB_TIME) 
			{
				// State Machine running!
				StateMachine();
				// Update starting time for the next cycle	
				starting_time = ros::Time::now().toSec();
			}	

			// Open log file, save data and close it
			logFile.open("/home/emanuele/ethz_ws/haptic_drone_with_structure_log_file.txt", std::ofstream::app);
			logFile << pos[1] 		     	   << ", " 
				<< pos[2] 		     	   << ", " 
				<< pos_msg.pose.position.y   	   << ", " 
				<< pos_msg.pose.position.z   	   << ", " 
				<< (180/M_PI)*desired_roll   	   << ", "
				<< (180/M_PI)*commanded_roll 	   << ", "
				<< pushing_effort	     	   << ", "
				<< ft_filtered_msg.torque.x  	   << ", " 
				<< ft_filtered_msg.force.y   	   << ", " 
				<< ft_filtered_msg.force.z  	   << ", " 
				<< (180/M_PI)*obstacle_inclination << ", "
				<< friction_force		   << ", "
				<< normal_force			   << ", "
				<< mu 	     			   << ";" 
								   << endl;
			logFile.close();

			//Condition verified when the new height waypoint is ouside the structure limits
			if(pos[2] >= LIMIT_HEIGHT)
			{
				cout << "\r\n\n\n\033[32m\033[1mMAXIMUM HEIGHT REACHED! \033[0m" << endl; 
				stop = true;
			}
		}

		loop.sleep();          
	}
	return 0;
}
