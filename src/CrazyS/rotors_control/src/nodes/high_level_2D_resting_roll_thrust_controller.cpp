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
#include <mav_msgs/RollPitchYawrateThrust.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/WrenchStamped.h>
#include <mav_msgs/Actuators.h>
#include <std_msgs/Float32.h>
#include <Eigen/Geometry>

using namespace std;

//////////////////// Global variables ////////////////////

ofstream logFile;				  	// Log file to save test parameters

// Pubs and subs
ros::Subscriber orientation_sub;			// Subscriber to odometry
ros::Subscriber ft_sensor_sub;				// Subscriber to F/T sensor
ros::Subscriber rotor_speed_sub;			// Subscriber to rotor angular velocity		
ros::Publisher  cmd_pub;				// Publisher for thrust/roll controller
ros::Publisher  ft_filtered_pub;			// Publisher for filtered F/T data
ros::Publisher  thrust_pub;				// Publisher for current Thrust value
const int PUB_TIME = 3;				  	// New waypoint is published every 'PUB_TIME' seconds

// Force/torque variables
Eigen::Vector3f raw_forces;
Eigen::Vector3f raw_torques;
double force_magnitude;
const double MIN_VERTICAL_FORCE_THRESHOLD  = 0.1; 	// [N]
const double MAX_VERTICAL_FORCE_THRESHOLD  = 2.6;   	// [N]
const double CAGE_WEIGHT_OFFSET           = 0.147;  	// [N]
const double CAGE_RADIUS                  = 0.255;  	// [m]

// UAV variables
Eigen::Vector3f pos;				  	// UAV global position [m]
Eigen::Vector4f rotor_speed;				// Angular velocity of the rotors [rad/s]
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
mav_msgs::RollPitchYawrateThrust cmd_msg;
std_msgs::Float32 thrust_msg;
double starting_time;
double current_roll, desired_roll, commanded_roll;
double current_thrust, desired_thrust, commanded_thrust;
double first_interaction_lateral_position;
const double BASE_THRUST    = 15.30;			// A bit more than Hovering Thrust
const double DELTA_PUSH	    = 0.05;
const double MAX_EFFORT     = 10000;
const double CONTROL_GAIN_Y = 0.01; 	 	 	// Controller Gain for Y component
const double CONTROL_GAIN_Z = 0.001;		 	// Controller Gain for Z component
const double MAX_HEADING    = M_PI/2;		 	// Max desired heading angle [rad], equal to 90 deg
const double MAX_ROLL       = 0.0523599;			// Max desired roll angle [rad], equal to 3 deg
bool sliding_flag           = false;
bool contact_flag	    = false;

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

// Callback for drone orientation data acquisition from odometry sensor
void orientationCallback(const geometry_msgs::Pose::ConstPtr& pose_msg)
{
	// Store roll data for control strategy
	Eigen::Quaterniond q(pose_msg->orientation.w,
			     pose_msg->orientation.x, 
			     pose_msg->orientation.y,
			     pose_msg->orientation.z);
	auto euler = q.toRotationMatrix().eulerAngles(2, 1, 0);
	current_roll = M_PI + euler[2];
	if(current_roll > M_PI)
		current_roll -= 2*M_PI;
}

// Callback for rotor/propellers angular velocity
void rotorSpeedCallback(const mav_msgs::Actuators::ConstPtr& omega_msg)
{
	// Store rotors speed data
	rotor_speed[0] = omega_msg->angular_velocities[0];
	rotor_speed[1] = omega_msg->angular_velocities[1];
	rotor_speed[2] = omega_msg->angular_velocities[2];
	rotor_speed[3] = omega_msg->angular_velocities[3];

	current_thrust = coeff_thrust * ( rotor_speed[0]*rotor_speed[0] +
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
void  ComputeDesiredRoll()
{
	// Calculate desired angle from the components of the external force
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

	commanded_roll = roll_d;
	cout << "CURRENT ROLL ANGLE: "   << (180/M_PI)*current_roll   << " [deg], " 
                                         <<            current_roll   << " [rad]" 
				         << endl;
	cout << "COMMANDED ROLL ANGLE: " << (180/M_PI)*commanded_roll << " [deg], " 
                                         <<            commanded_roll << " [rad]" 
				         << endl;
	commanded_thrust = effort;
	cout << "CURRENT THRUST: "   << current_thrust   << endl;
	cout << "DESIRED THRUST: "   << desired_thrust   << endl;
	cout << "COMMANDED THRUST: " << commanded_thrust << endl;

	// Publish Thrust and Roll
	cmd_msg.header.stamp = ros::Time::now();
	cmd_msg.roll         = commanded_roll;
	cmd_msg.thrust.z     = commanded_thrust;
	cmd_pub.publish(cmd_msg);

	// Publish thrust value
	thrust_msg.data = current_thrust;
	thrust_pub.publish(thrust_msg);
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

	// No contact detected: drone in landing phase (flying down)
	if(ft_filtered_msg.force.z < (MIN_VERTICAL_FORCE_THRESHOLD+CAGE_WEIGHT_OFFSET))
	{
		cout << "\033[32m\033[0mLANDING PHASE! \033[0m" << endl;

		// Reset variables	
		desired_thrust = BASE_THRUST-2*DELTA_PUSH;	
		sliding_flag   = false;
		starting_time = ros::Time::now().toSec();

		ControlStrategy(desired_thrust, 0);
	}
	// Contact detected: lay on the osbtacle to minimize the energy until a threshold in lateral or vertical force is overcome
	else if((ft_filtered_msg.force.z > (MIN_VERTICAL_FORCE_THRESHOLD+CAGE_WEIGHT_OFFSET)) && (ft_filtered_msg.force.z < (MAX_VERTICAL_FORCE_THRESHOLD+CAGE_WEIGHT_OFFSET)))
	{
		cout << "\033[32m\033[0mCONTACT DETECTED: MINIMIZING ENERGY! \033[0m" << endl;

		// Lower down the reference every PUB_TIME seconds
		if((ros::Time::now().toSec() - starting_time) > PUB_TIME) 
		{
			commanded_thrust -= DELTA_PUSH;
			starting_time = ros::Time::now().toSec();
		}
	
		ControlStrategy(commanded_thrust, 0);

		sliding_flag = false;
	}
	// Threshold overcome: obstacle too compliant, the drone is slipping, no need to push more
	else
	{
		cout << "\033[32m\033[0mRESTING PHASE! \033[0m" << endl;

		// Compute desired heading only before the drone starts to slide
		if(!sliding_flag)
			ComputeDesiredRoll();

		// Update variable
		sliding_flag = true;

		// Adaptive adjustment of the desired heading angle
		int sign = (ft_filtered_msg.torque.x > 0) ? 1 : -1;
		commanded_thrust -= (ft_filtered_msg.force.z);		
		ControlStrategy(commanded_thrust, sign*desired_roll); 
	}
	
	// After Resting the demo is finished. Any attempts or lower again the thrust leads to slip
	
	// Print current Thrust force value
	cout << "THRUST: " << current_thrust << " [N]. Energy minimized: " << 100*(1-current_thrust/hovering_thrust) << "%" << endl;
}

// Reach home position above the resting spot and be ready for resting
void HomePosition()
{
	cmd_msg.header.stamp = ros::Time::now();
	cmd_msg.roll         = 0.05;
	cmd_msg.thrust.z     = BASE_THRUST;
	cmd_pub.publish(cmd_msg);

	sleep(2.5);
	ros::spinOnce();

	cmd_msg.header.stamp = ros::Time::now();
	cmd_msg.roll         = -0.05;
	cmd_msg.thrust.z     = BASE_THRUST-2*DELTA_PUSH;
	cmd_pub.publish(cmd_msg);

	sleep(1.5);
	ros::spinOnce();

	cmd_msg.header.stamp = ros::Time::now();
	cmd_msg.roll         = 0.0;
	cmd_msg.thrust.z     = BASE_THRUST-DELTA_PUSH;
	cmd_pub.publish(cmd_msg);

	sleep(5);
	ros::spinOnce();


	cmd_msg.header.stamp = ros::Time::now();
	cmd_msg.roll         = -0.05;
	cmd_msg.thrust.z     = BASE_THRUST;//DELTA_PUSH;
	cmd_pub.publish(cmd_msg);

	sleep(2.5);
	ros::spinOnce();

	cmd_msg.header.stamp = ros::Time::now();
	cmd_msg.roll         = 0.05;
	cmd_msg.thrust.z     = BASE_THRUST-2*DELTA_PUSH;
	cmd_pub.publish(cmd_msg);

	sleep(1.5);
	ros::spinOnce();

	cmd_msg.header.stamp = ros::Time::now();
	cmd_msg.roll         = 0.0;
	cmd_msg.thrust.z     = BASE_THRUST-DELTA_PUSH;
	cmd_pub.publish(cmd_msg);

	sleep(1);
	ros::spinOnce();

	hovering_thrust = current_thrust;
	cout << "Home position reached. Hovering Thrust: " << hovering_thrust << " [N] " << endl;
}

// Main function
int main(int argc, char** argv)
{
	ros::init(argc, argv, "high_level_2D_resting_roll_thrust_controller");
	ros::NodeHandle nh;
	ros::Rate loop(100);

	// Subscribers and Publishers 
	orientation_sub = nh.subscribe("/haptic_drone_with_structure/odometry_sensor1/pose", 10, &orientationCallback);
	ft_sensor_sub   = nh.subscribe("/haptic_drone_with_structure/ft_sensor_topic", 10, &forceTorqueSensorCallback);
	rotor_speed_sub	= nh.subscribe("/haptic_drone_with_structure/motor_speed", 10, &rotorSpeedCallback);
	cmd_pub 	= nh.advertise<mav_msgs::RollPitchYawrateThrust>(mav_msgs::default_topics::COMMAND_ROLL_PITCH_YAWRATE_THRUST, 10);
	ft_filtered_pub = nh.advertise<geometry_msgs::Wrench>("/haptic_drone_with_structure/ft_filtered_measurements", 10);
	thrust_pub	= nh.advertise<std_msgs::Float32>("/haptic_drone_with_structure/thrust", 10);
	
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
				 		
		// Check the status and control the drone every PUB_TIME seconds
		//if((ros::Time::now().toSec() - starting_time) > PUB_TIME) 
		//{
			// State Machine running!
			StateMachine();
			// Update starting time for the next cycle	
			//starting_time = ros::Time::now().toSec();
		//}	

		// Open log file, save data and close it
		/*
		logFile.open("/home/emanuele/catkin_ws/haptic_drone_with_structure_log_file_RPYT.txt", std::ofstream::app);
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
