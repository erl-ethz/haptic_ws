#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
/*
#include <boost/bind.hpp>
#include <gazebo/msgs/msgs.hh>
#include <gazebo/transport/transport.hh>

#include <thread>
#include "ros/ros.h"
#include "ros/callback_queue.h"
#include "ros/subscribe_options.h"
#include "std_msgs/Float32.h"
*/
namespace gazebo
{
	class TorsionalSpringPlugin : public ModelPlugin
	{
		public: TorsionalSpringPlugin() {}

		private:
			physics::ModelPtr model;
			sdf::ElementPtr sdf;
			
			physics::JointPtr joint;

			/*
			common::PID pid;
			int forceIteration;
			*/

			// Set point
			double setPoint;

			// Spring constant
			double kx;

			// Pointer to update event connection
			event::ConnectionPtr updateConnection;

		public: void Load(physics::ModelPtr _model, sdf::ElementPtr _sdf)
		{
			// Safety check
			if (_model->GetJointCount() == 0)
			{
				std::cerr << "You have zero joints! Something is wrong! Not loading plugin." << std::endl;
				return;
			}

			// Store model pointer
			this->model = _model;

			// Store the SDF pointer
			this->sdf = _sdf;

			if (_sdf->HasElement("joint"))
				this->joint = _model->GetJoint(_sdf->Get<std::string>("joint"));
			else
				gzerr << "Must specify joint to apply a torsional spring at!\n";

			this->kx = 0.0;
			if (_sdf->HasElement("kx"))
				this->kx = _sdf->Get<double>("kx");
			else
				printf("Torsional spring coefficient not specified! Defaulting to: %f\n", this->kx);

			this->setPoint = -0.01;

			if (_sdf->HasElement("set_point"))
				this->setPoint = _sdf->Get<double>("set_point");
			else
				printf("Set point not specified! Defaulting to: %f\n", this->setPoint);
			/*
			// Create the node
			this->node = transport::NodePtr(new transport::Node());
			this->node->Init(this->model->GetWorld()->Name());

			// Create a topic name
			std::string topicName = "~/" + this->model->GetName() + "/spring_joint/vel_cmd";

			//std::cout << "topicName: " << topicName << std::endl;

			// Subscribe to the topic, and register a callback
			this->sub = this->node->Subscribe(topicName, &TorsionalSpringPlugin::OnMsg, this);
			//std::cout << "Subscribed to " << this->sub << std::endl;

			this->forceIteration = 0;

			// Initialize ros, if it has not already bee initialized.
			if (!ros::isInitialized())
			{
				int argc = 0;
				char **argv = NULL;
				ros::init(argc, argv, "gazebo_client", ros::init_options::NoSigintHandler);
			}

			// Create our ROS node. This acts in a similar manner to the Gazebo node
			this->rosNode.reset(new ros::NodeHandle("gazebo_client"));

			// Create a named topic, and subscribe to it.
			ros::SubscribeOptions so = ros::SubscribeOptions::create<std_msgs::Float32>(
			      "/" + this->model->GetName() + "/spring_joint/vel_cmd",
			      1,
			      boost::bind(&TorsionalSpringPlugin::OnRosMsg, this, _1),
			      ros::VoidPtr(), &this->rosQueue);
			this->rosSub = this->rosNode->subscribe(so);

			// Spin up the queue helper thread.
			this->rosQueueThread =
			  std::thread(std::bind(&TorsionalSpringPlugin::QueueThread, this));
			*/
			gzerr << "Loaded gazebo_joint_torsional_spring" << std::endl;

		}
		/*
		/// \brief A node use for ROS transport
		private: std::unique_ptr<ros::NodeHandle> rosNode;

		/// \brief A ROS subscriber
		private: ros::Subscriber rosSub;

		/// \brief A ROS callbackqueue that helps process messages
		private: ros::CallbackQueue rosQueue;

		/// \brief A thread the keeps running the rosQueue
		private: std::thread rosQueueThread;

		/// \brief A node used for transport
	   	 private: transport::NodePtr node;
	
	    	/// \brief A subscriber to a named topic.
	    	private: transport::SubscriberPtr sub;

		/// \brief Handle an incoming message from ROS
		public: void OnRosMsg(const std_msgs::Float32ConstPtr &_msg)
		{
		  std::cout << "_msg->data: " << _msg->data << std::endl;
		 
		  for (int i = 0; i < 10; i++)
		  	this->joint->SetForce(0, 1000 * float(_msg->data));
		}

		/// \brief ROS helper function that processes messages
		private: void QueueThread()
		{
			static const double timeout = 0.01;
			while (this->rosNode->ok())
			{
				this->rosQueue.callAvailable(ros::WallDuration(timeout));
			}
		}

		public: void SetVelocity(const double &_vel)
		{
			// Set the joint's target velocity.
			this->model->GetJointController()->SetVelocityTarget(this->joint->GetScopedName(), _vel);
		}

		private: void OnMsg(ConstVector3dPtr &_msg)
		{
			this->SetVelocity(_msg->x());
		}
		*/
		public: void Init()
		{
			// Listen to update event
			this->updateConnection = event::Events::ConnectWorldUpdateBegin(
				std::bind (&TorsionalSpringPlugin::OnUpdate, this) );
		}

		protected: void OnUpdate()
		{
			double current_angle = this->joint->Position(0);
			//for (int i = 0; i < 5; i++)
				this->joint->SetForce(0, this->kx*(this->setPoint-current_angle));
			//std::cout << "Current angle (branch): " << current_angle << std::endl;			
		}
		
	};

	GZ_REGISTER_MODEL_PLUGIN(TorsionalSpringPlugin)
}
