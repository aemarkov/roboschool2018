#include "LightPlugin.h"

using namespace gazebo;

gazebo::LightPlugin::LightPlugin() :
    _offDiffuseColor(0.6, 0.6, 0.6, 1.0),
    _offEmissiveColor(0.0, 0.0, 0.0, 0.0)
{
}

void gazebo::LightPlugin::Load(rendering::VisualPtr parent, sdf::ElementPtr sdf)
{        
    ROS_WARN("++++++++++++++++++++++++++++++++++++++++++");

    _model = parent;    
    _updateConnection = event::Events::ConnectPreRender(boost::bind(&LightPlugin::OnUpdate, this));    

    /* Для visual плагина не удается получить параметры:
    http://answers.gazebosim.org/question/13494/warning-and-fail-to-read-an-sdf-element-converting-a-deprecated-sdf-sourcedata-string/
    Исправлено в версии 7.5:
    https://bitbucket.org/osrf/gazebo/issues/2202/gazebo7-visualplugins-are-deprecated
    */

    auto name = parent->GetName();
    if(name == "traffic_light::green::green")
    {
        _onColor = common::Color(0.0, 1.0, 0.0, 1.0);
        _topic = "/light/green";
    }
    else if(name == "traffic_light::red::red")
    {
        _onColor = common::Color(1.0, 0.0, 0.0, 1.0);
        _topic = "/light/red";
    }
    else
    {
        ROS_ERROR_STREAM("Invalid visual: " << name);
        return;
    }

    init_ros();
}

void gazebo::LightPlugin::OnUpdate()
{
}

void gazebo::LightPlugin::init_ros()
{
    if(!ros::isInitialized)
    {
        int argc=0;
        char **argv = NULL;
        ros::init(argc, argv, "traffic_light_plugin", ros::init_options::NoSigintHandler);
    }

    _nh = std::make_unique<ros::NodeHandle>();
    _spinner = std::make_unique<ros::AsyncSpinner>(0);
    _cmdSub = _nh->subscribe(_topic, 1, &LightPlugin::cmd_callback, this);
    _spinner->start();
}

void gazebo::LightPlugin::cmd_callback(const std_msgs::Bool::ConstPtr& ptr)
{
    if(ptr->data)
        set_color(_onColor, _onColor);
    else
        set_color(_offDiffuseColor, _offEmissiveColor);
}

void gazebo::LightPlugin::set_color(common::Color& diffuseColor, common::Color& emissiveColor)
{
    _model->SetEmissive(emissiveColor);
    _model->SetDiffuse(diffuseColor);
}