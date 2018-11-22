#include <functional>
#include <gazebo/gazebo.hh>
#include <gazebo/physics/physics.hh>
#include <gazebo/common/common.hh>
#include <ignition/math/Vector3.hh>
#include <vector>
#include <ros/ros.h>
#include <ros/callback_queue.h>
#include <car_msgs/MotorsControl.h>
#include <thread>

namespace gazebo
{
    class TestRobotPlugin : public ModelPlugin
    {
    public:        
        void Load(physics::ModelPtr parent, sdf::ElementPtr sdf);
    private:
        physics::ModelPtr _model;
        event::ConnectionPtr _updateConnection;        

        std::unique_ptr<ros::NodeHandle> _nh;
        ros::CallbackQueue _rosQueue;
        ros::Subscriber _cmdSub;
        std::thread _spin_thread;

        physics::JointPtr _leftJoint, _rightJoint;
        common::PID _leftPID, _rightPID;
        
        const std::string COMMANDS_TOPIC = "/motors_commands";
        const float MAX_ANGULAR_VELOCITY = 20;

        void init_ros();
        void velocity_callback(const car_msgs::MotorsControl::ConstPtr& msg);
        void spin_thread_func();
        void on_update();       

    };

    // Register this plugin with the simulator
    GZ_REGISTER_MODEL_PLUGIN(TestRobotPlugin)
}