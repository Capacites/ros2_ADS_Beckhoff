#ifndef HEADER_H_ADS_NODE
#define HEADER_H_ADS_NODE

//YAML include

#include <rclcpp/rclcpp.hpp>
#include "Ads_Interface.h"
#include <ros_ads_decode/ADSDecode.hpp>

#include <ros_ads_msgs/msg/ads.hpp>
#include <ros_ads_msgs/msg/state.hpp>

#include <yaml-cpp/yaml.h>


//Standard includes
#include <memory>
#include <string>
#include <vector>
#include <map>
#include <mutex>
#include <set>
#include <variant>

#include <boost/thread.hpp>

using namespace std::chrono_literals;
using namespace std;

namespace ads_node {

class ADSNode: public rclcpp::Node
{

public:

    /**
     * @brief Node constructor.
     *
     * @param options The node options
     */
    ADSNode(rclcpp::NodeOptions options = rclcpp::NodeOptions());

     /**
     * @brief ADSNode::Subscriber send the ADS device the received commands
     * @param msg the command message
     * @return true if the command was successfully sent
     */
    bool subscriber_callback(ros_ads_msgs::msg::ADS::SharedPtr msg);


    int configure();
 /**
     * @brief ADSNode::publishState publish state periodically
     * 
     */
    void publish_state();
    void update_timer_callback();



    /**
     * @brief ADSNode::check_timer_callback publish variables if an event occured
     * @param timer_rate rate to verify if an event occured at
     */
    void check_timer_callback();




/**
 * @brief ADSNode::publish_timer_callback publish variables periodically
 * @param timer_rate rate to publish at
 */
    void publish_timer_callback();
private:




///OLD VARIABEKL


     /**
     * @brief ADSNode::Subscriber send the ADS device the received commands
     * @param msg the command message
     * @return true if the command was successfully sent
     */
   // bool Subscriber(ros_ads_msgs::msg::ADS::SharedPtr msg);

    /**
     * @brief ADSNode::initialize initialize the parameters of the node and it's variables
     * @return 1 if the initialization failed
     * @return 0 otherwise
     */
    //bool initialize();

    /**
     * @brief ADSNode::update update internal variable values memory
     *
     * Starts by verifying and updating connection state
     *
     * @param timer_rate rate to update at
     */
   // void update(int timer_rate);

    /**
     * @brief ADSNode::publishState publish state periodically
     * @param timer_rate rate to publish at
     */
    //void publishState(int timer_rate);

    /**
     * @brief ADSNode::publishOnEvent publish variables if an event occured
     * @param timer_rate rate to verify if an event occured at
     */
   // void publishOnEvent(int timer_rate);

    /**
     * @brief ADSNode::publishOnTimer publish variables periodically
     * @param timer_rate rate to publish at
     */
    //void publishOnTimer(int timer_rate);



private:



//ROS parameters
    int m_timeout = declare_parameter<int>("timeout", 30);
    int m_sub_queue_size = declare_parameter<int>("sub_queue_size",10);
    int m_pub_queue_size = declare_parameter<int>("pub_queue_size", 5);
    std::string m_name = declare_parameter<std::string>("name", "test_device");
    std::string m_YAML_config_file = declare_parameter<std::string>("YAML_config_file", "ros2/ros_ads/map.yaml");
    bool m_debug = declare_parameter<bool>("debug", false);



//Node member variables

    bool m_publish{false};                                     /*!< Control to publish event message                          */
    bool m_configOK{false};                                    /*!< Configuration state to the modbus device                  */



    //ADS specific
    map<string, pair<string, double>> m_publish_on_timer;      /*!< caracteristics of variables to publish periodically       */
    map<string, pair<string, double>> m_publish_on_event;      /*!< caracteristics of variables to publish on event           */
    map<string, pair<string, double>> m_variables;             /*!< caracteristics of declared variables                      */
    mutex m_publish_on_timer_guard;                            /*!< m_publish_on_timer map guard                              */
    mutex m_publish_on_event_guard;                            /*!< m_publish_on_event map guard                              */
    mutex m_variables_guard;                                   /*!< m_variables map guard                                     */

    pair<string, double> m_checker_temp_value;                 /*!< temporary value for event detection                       */

    RosAds_Interface m_ADS = RosAds_Interface();               /*!< ADS device interface                                      */



    /// Ros Msg
    ros_ads_msgs::msg::ADS m_on_event_msg;                          /*!< message to publish on event                               */
    ros_ads_msgs::msg::ADS m_on_timer_msg;                          /*!< message to publish on timer                               */
    ros_ads_msgs::msg::State m_state_msg;                           /*!< state message                                             */

    //ROS components
    
    rclcpp::Publisher<ros_ads_msgs::msg::State>::SharedPtr m_state_publisher;                          /*!< state publisher                                           */
    rclcpp::Publisher<ros_ads_msgs::msg::ADS>::SharedPtr m_event_publisher;                       /*!< on event publisher                                        */
    rclcpp::Publisher<ros_ads_msgs::msg::ADS>::SharedPtr m_timer_publisher;                       /*!< periodical publisher                                      */
    rclcpp::Subscription<ros_ads_msgs::msg::ADS>::SharedPtr m_subscriber;                              /*!< command subscriber                                        */
  
    rclcpp::TimerBase::SharedPtr m_publisher_timer;
    rclcpp::TimerBase::SharedPtr m_checker_timer;
    rclcpp::TimerBase::SharedPtr m_update_timer;              /*!< state publisher thread                                    */
};

}

#endif
