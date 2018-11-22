#include "CarPlugin.h"

void gazebo::CarPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)
{
	// Init Model and joints	
	_model = parent;
	_updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&CarPlugin::on_update, this));
	_maxVelocity = sdf->Get<float>("max_velocity");

	init_joints(sdf);
	init_ros();

}

void gazebo::CarPlugin::init_joints(sdf::ElementPtr sdf)
{
	add_wheel(_leftWheelsJoints,  "wheel_1_left_joint");
	add_wheel(_leftWheelsJoints,  "wheel_2_left_joint");
    add_wheel(_leftWheelsJoints,  "wheel_3_left_joint");
	add_wheel(_rightWheelsJoints, "wheel_1_right_joint");
	add_wheel(_rightWheelsJoints, "wheel_2_right_joint");
	add_wheel(_rightWheelsJoints, "wheel_3_right_joint");

	float p = sdf->Get<float>("p");
	float i = sdf->Get<float>("i");
	float d = sdf->Get<float>("d");

	_leftPID = common::PID(p, i, d); // wtf? Is this correct???
	_rightPID = common::PID(p, i, d);

	setup_joints(_leftWheelsJoints, _leftPID);
	setup_joints(_rightWheelsJoints, _rightPID);

}

void gazebo::CarPlugin::init_ros()
{	
	if(!ros::isInitialized)
	{
		int argc=0;
		char **argv = NULL;
		ros::init(argc, argv, "car_plugin_node", ros::init_options::NoSigintHandler);
	}


	_nh = std::make_unique<ros::NodeHandle>();
	_cmdSub = _nh->subscribe(COMMANDS_TOPIC, 100, &CarPlugin::velocity_callback, this);
	_spinner = std::make_unique<ros::AsyncSpinner>(0);
	_spinner->start();
	ROS_INFO_STREAM("car_plugin_node started");
}


void gazebo::CarPlugin::add_wheel(std::vector<physics::JointPtr>& joints, std::string joint_name)
{	
	auto ptr = _model->GetJoint(joint_name);
	if(ptr != nullptr)
		joints.push_back(ptr);
	else
		ROS_ERROR_STREAM("Joint " << joint_name << " not found in model");

}

void gazebo::CarPlugin::setup_joints(const std::vector<physics::JointPtr>& joints, const common::PID& pid)
{
	for(auto& joint: joints)	
		_model->GetJointController()->SetVelocityPID(joint->GetScopedName(), pid);	
}

void gazebo::CarPlugin::set_speed(const std::vector<physics::JointPtr>& joints, float speed)
{
	for(auto& joint: joints)
		_model->GetJointController()->SetVelocityTarget(joint->GetScopedName(), speed);
}

void gazebo::CarPlugin::velocity_callback(const car_msgs::MotorsControl::ConstPtr& msg)
{
	set_speed(_leftWheelsJoints, msg->left / 255.0f * _maxVelocity);
	set_speed(_rightWheelsJoints, msg->right / 255.0f * _maxVelocity);
}


void gazebo::CarPlugin::on_update()
{
}