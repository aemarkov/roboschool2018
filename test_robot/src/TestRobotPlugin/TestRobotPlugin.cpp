#include "TestRobotPlugin.h"

void gazebo::TestRobotPlugin::Load(physics::ModelPtr parent, sdf::ElementPtr sdf)    
{
    // Init Model and joints
    _model = parent;
    _updateConnection = event::Events::ConnectWorldUpdateBegin(std::bind(&TestRobotPlugin::on_update, this));
    init_ros();

    // Find and setup joints
    _leftJoint = _model->GetJoint("wheel_l_joint");    
    _rightJoint = _model->GetJoint("wheel_r_joint");
    _leftPID = common::PID(0.2, 0, 0);
    _rightPID = common::PID(0.2, 0, 0);
    _model->GetJointController()->SetVelocityPID(_leftJoint->GetScopedName(), _leftPID);
    _model->GetJointController()->SetVelocityPID(_rightJoint->GetScopedName(), _rightPID);
}

void gazebo::TestRobotPlugin::init_ros()
{   
    // Init ros node
    if(!ros::isInitialized)
    {
        int argc=0;
        char **argv = NULL;
        ros::init(argc, argv, "test_robot_plugin_node", ros::init_options::NoSigintHandler);
    }

    // Создаем хэндлер ноды
    _nh = std::make_unique<ros::NodeHandle>();

    // Подписываемся на топик команд
    // Используем свою очередь сообщений, потому что в Gazebo мы не можем
    // использовать ros::spin()
    auto so = ros::SubscribeOptions::create<car_msgs::MotorsControl>(
                COMMANDS_TOPIC,
                100,
                boost::bind(&TestRobotPlugin::velocity_callback, this, _1),
                ros::VoidPtr(), &_rosQueue);

    _cmdSub = _nh->subscribe(so);

    // Запускаем отдельный поток, в котором будет обрабатываться очередь сообщений ROS
    _spin_thread = std::thread(&TestRobotPlugin::spin_thread_func, this);

    ROS_INFO_STREAM("test_robot_plugin_node started");
}



void gazebo::TestRobotPlugin::velocity_callback(const car_msgs::MotorsControl::ConstPtr& msg)
{
    // Устанавливаем угловую скорость джоинтов
    _model->GetJointController()->SetVelocityTarget(_leftJoint->GetScopedName(), msg->left / 255.0f * MAX_ANGULAR_VELOCITY);
    _model->GetJointController()->SetVelocityTarget(_rightJoint->GetScopedName(), msg->right / 255.0f * MAX_ANGULAR_VELOCITY);
}

// Цикл обработки сообщений ROS и вызова коллбеков
void gazebo::TestRobotPlugin::spin_thread_func()
{
    static const double timeout = 0.01;
    while (_nh->ok())   
        _rosQueue.callAvailable(ros::WallDuration(timeout));    
}

void gazebo::TestRobotPlugin::on_update()
{    
}