#include "exec.hpp"
#include <iostream>
#include <ament_index_cpp/get_package_share_directory.hpp>

#include <behaviortree_cpp_v3/loggers/bt_zmq_publisher.h>
namespace turtle_behavior
{
    
    std::string status = "/navigate_to_pose/_action/status";
    rclcpp::Subscription<action_msgs::msg::GoalStatusArray>::SharedPtr subscription_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr publisher_;
    rclcpp::TimerBase::SharedPtr timer_;
    using action = nav2_msgs::action::NavigateToPose;
    std::string action_name = "/navigate_to_pose";
    int status_msg;
    std::chrono::system_clock::time_point _completion_time;
    
    
    int inp_vel;
    double goal_x, goal_y, goal_z;
    using namespace std::chrono_literals;
    ActionSubscriber::ActionSubscriber():Node("actionsubscriber")
        {
            status_msg = 0;
            subscription_ = this->create_subscription<action_msgs::msg::GoalStatusArray>(
            "/navigate_to_pose/_action/status",  // Change this to the topic you want to subscribe to
            10,         // Queue size
            std::bind(&ActionSubscriber::status_callback, this, std::placeholders::_1)
            );

        }   
    
    void ActionSubscriber::status_callback(action_msgs::msg::GoalStatusArray::SharedPtr msg)
        {   
            auto status_list = msg->status_list;
            status_msg = status_list.back().status;
            
            RCLCPP_INFO(this->get_logger(), "Received Action Status: %d", status_msg); 
        }

  
  
    BT::NodeStatus Use_nav2::onRunning()
        {
            std::cout<<"in_running"<<std::endl;
            // auto node_s = std::make_shared<ActionSubscriber>();
            // rclcpp::spin_some(node_s);
            
            //rclcpp_action::ClientGoalHandle<action>::SharedPtr goal_handle = onStart().goal_handle_future.get();
            //auto result_future = action_client->async_get_result(goal_handle);
            if (status_msg > 4 || status_msg <= 2)
            {
                std::cout<<status_msg<<std::endl;
                return BT::NodeStatus::RUNNING;
            }
            else{
                std::cout<<status_msg<<std::endl;
                return BT::NodeStatus::SUCCESS;
            }
            
        }

      BT::NodeStatus Use_nav2::onStart()
        {
            using action = nav2_msgs::action::NavigateToPose;
            std::string action_name = "/navigate_to_pose";
            rclcpp::Node::SharedPtr node_=rclcpp::Node::make_shared("navigate_to_mybt");
            auto action_client = rclcpp_action::create_client<action>(node_, action_name);
            std::cout<<"instart"<<std::endl;
            if(!action_client->wait_for_action_server(std::chrono::seconds(20)))
            {
                printf("not available");
                return BT::NodeStatus::FAILURE;
            }

            
            auto goal_msg = action::Goal();
            
            geometry_msgs::msg::PoseStamped p;
            getInput<double>("goal_x", goal_x);
            getInput<double>("goal_y", goal_y);
            getInput<double>("goal_z", goal_z);
            goal_msg.pose.header.frame_id = "map";
            goal_msg.pose.pose.position.x = goal_x;
            goal_msg.pose.pose.position.y = goal_y;
            goal_msg.pose.pose.position.z = goal_z;
            auto goal_handle_future = action_client->async_send_goal(goal_msg);
            
                
            return BT::NodeStatus::RUNNING;
        } 

    void Use_nav2::onHalted()
    {
        printf("HERE");
    }

    ActionPublisher::ActionPublisher():Node("actionpublisher")
    {
       publisher_ = this->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
       timer_ = this->create_wall_timer(500ms, std::bind(&ActionPublisher::twist_callback, this)); 
    }
    
    void ActionPublisher::twist_callback()
    {
        RCLCPP_INFO(this->get_logger(), "Time to ratate");
        auto twist_msg = geometry_msgs::msg::Twist();
        twist_msg.linear.x = 0.0;
        twist_msg.linear.y = 0.0;
        twist_msg.linear.z = 0.0;

        twist_msg.angular.x = 0.0;
        twist_msg.angular.y = 0.0;
        twist_msg.angular.z = 0.0;
        publisher_->publish(twist_msg);
    }
    
    BT::NodeStatus Rotate::onStart()
    {
        //rclcpp::Node::SharedPtr node_=rclcpp::Node::make_shared("rotate_bot");
        std::cout<<"in rotate start"<<std::endl;
        getInput<int>("inp_vel", inp_vel);
        //timer_ = this->create_wall_timer(std::chrono::milliseconds(static_cast<int>(1000.0 / publish_rate_)), std::bind(&RotateTurtlebotPublisher::publishVelocity, this));
        _completion_time=std::chrono::system_clock::now() + std::chrono::milliseconds(3000);
        return BT::NodeStatus::RUNNING;
        
    }

    BT::NodeStatus Rotate::onRunning()
    {
        
        if(std::chrono::system_clock::now() >= _completion_time)
        {
            std::cout<<"pub"<<std::endl;
            auto node = std::make_shared<rclcpp::Node>("single_publisher_node");
            publisher_ = node->create_publisher<geometry_msgs::msg::Twist>("cmd_vel", 10);
            auto twist_msg = geometry_msgs::msg::Twist();
            twist_msg.linear.x = 0.0;
            twist_msg.linear.y = 0.0;
            twist_msg.linear.z = 0.0;

            twist_msg.angular.x = 0.0;
            twist_msg.angular.y = 0.0;
            twist_msg.angular.z = inp_vel;
            publisher_->publish(twist_msg);
            return BT::NodeStatus::SUCCESS;
        }
        else
        {
            std::cout<<"in rotate running"<<std::endl;
            return BT::NodeStatus::RUNNING;
        }
        
    }

    void Rotate::onHalted()
    {
        printf("STOPPED");
    }
}

int main(int argc, char* argv[])
{
    rclcpp::init(argc, argv);
    auto node = rclcpp::Node::make_shared("behavior_tree_node"); 
    BT::BehaviorTreeFactory factory;
    std::cout<<"hey"<<std::endl;
    factory.registerNodeType<turtle_behavior::Use_nav2>("Use_nav2");
    factory.registerNodeType<turtle_behavior::Rotate>("Rotate");
    std::string package_path = ament_index_cpp::get_package_share_directory("turtlebot_bt");
    auto tree = factory.createTreeFromFile(package_path + "/bt_xml/bt_tree_groot.xml");
    std::thread subscriber_thread([node]() {
        rclcpp::spin(std::make_shared<turtle_behavior::ActionSubscriber>());
    });
    BT::PublisherZMQ publisher_zmq(tree);
    tree.tickRootWhileRunning();
     
    subscriber_thread.join();
    std::cout<<"yo"<<std::endl;
    rclcpp::shutdown();
    

    return 0;

}   

