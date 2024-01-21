#ifndef TURTLEBOT_BT_INCLUDE_EXEC_HPP
#define TURTLEBOT_BT_INCLUDE_EXEC_HPP

#include <memory>
#include <string>
#include <vector>
#include <chrono>
#include <functional>
#include "behaviortree_cpp/behavior_tree.h"
#include "behaviortree_cpp/bt_factory.h"
#include "behaviortree_cpp/action_node.h"
#include "behaviortree_cpp/tree_node.h"
#include "rclcpp/rclcpp.hpp"
#include "rclcpp_action/rclcpp_action.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "nav2_msgs/action/navigate_to_pose.hpp"
#include "std_msgs/msg/string.hpp"

namespace turtle_behavior
{
    class ActionSubscriber: public rclcpp::Node
    {
        public:
            ActionSubscriber();
            void status_callback(action_msgs::msg::GoalStatusArray::SharedPtr msg); 
           
    };

    class ActionPublisher: public rclcpp::Node
    {
        public:
            ActionPublisher();

            void twist_callback();
    };


    class Use_nav2 : public BT::StatefulActionNode
    {
     public:
        Use_nav2(const std::string& name, const BT::NodeConfiguration& config)
            : StatefulActionNode(name, config)
        {
        }
        
        
        
        static BT::PortsList providedPorts()
        {
            return{ BT::InputPort<double>("goal_x"),
                    BT::InputPort<double>("goal_y"),
                    BT::InputPort<double>("goal_z")
             };
        }

        BT::NodeStatus onStart() override;

        BT::NodeStatus onRunning() override;

        void onHalted() override;
        
    };

    class Rotate : public BT::StatefulActionNode
    {
        public:
        Rotate(const std::string& name, const BT::NodeConfiguration& config)
            : StatefulActionNode(name, config)
        {}

        static BT::PortsList providedPorts()
        {
            return{ BT::InputPort<double>("inp_vel")};
        }

        BT::NodeStatus onStart() override;

        BT::NodeStatus onRunning() override;

        void onHalted() override;
    };

}
#endif