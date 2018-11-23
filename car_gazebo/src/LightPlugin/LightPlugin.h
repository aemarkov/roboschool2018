#ifndef __LIGHTPLUGIN_H__
#define __LIGHTPLUGIN_H__

#include <boost/bind.hpp>
#include <gazebo/gazebo.hh>
#include <gazebo/rendering/Visual.hh>
#include <gazebo/common/common.hh>
#include <ros/ros.h>
#include <std_msgs/Bool.h>
#include <string>

namespace gazebo
{
    class LightPlugin : public VisualPlugin
    {
    public:
        LightPlugin();
        void Load(rendering::VisualPtr parent, sdf::ElementPtr sdf);        
        void OnUpdate();

        void init_ros();
        void cmd_callback(const std_msgs::Bool::ConstPtr& ptr);
        void set_color(common::Color&, common::Color&);

    private:
        std::unique_ptr<ros::NodeHandle> _nh;
        std::unique_ptr<ros::AsyncSpinner> _spinner;
        ros::Subscriber _cmdSub;

        std::string _topic;
        common::Color _onColor;
        common::Color _offDiffuseColor, _offEmissiveColor;

        rendering::VisualPtr _model;
        event::ConnectionPtr _updateConnection;
    };

    GZ_REGISTER_VISUAL_PLUGIN(LightPlugin)
}

#endif