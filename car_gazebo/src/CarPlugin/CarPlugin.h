#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <vector>
#include <ros/ros.h>
#include "ros/callback_queue.h"
#include <car_msgs/MotorsControl.h>
#include <thread>

namespace gazebo
{
	class CarPlugin : public ModelPlugin
	{
	public:
		void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
	private:
		physics::ModelPtr _model;
		event::ConnectionPtr _updateConnection;

		std::vector<physics::JointPtr> _leftWheelsJoints;
		std::vector<physics::JointPtr> _rightWheelsJoints;
		common::PID _leftPID, _rightPID;
		float _maxVelocity;

		std::unique_ptr<ros::NodeHandle> _nh;
		std::unique_ptr<ros::AsyncSpinner> _spinner;		
		ros::Subscriber _cmdSub;

		const std::string COMMANDS_TOPIC = "/motors_commands";

		void init_joints(sdf::ElementPtr sdf);
		void init_ros();
		void add_wheel(std::vector<physics::JointPtr>& joints, std::string joint_name);
		void setup_joints(const std::vector<physics::JointPtr>& joints, const common::PID& pid);
		void set_speed(const std::vector<physics::JointPtr>& joints, float speed);
		void velocity_callback(const car_msgs::MotorsControl::ConstPtr& msg);
		void on_update();		

	};

	// Register this plugin with the simulator
  	GZ_REGISTER_MODEL_PLUGIN(CarPlugin)
}