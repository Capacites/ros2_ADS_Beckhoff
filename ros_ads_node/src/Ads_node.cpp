#include "../include/ros_ads_node/Ads_node.h"

#include <iostream>
#include <unistd.h>

using namespace ads_node;

ADSNode::ADSNode(rclcpp::NodeOptions options)
     : Node("ads_node", options)
{


    m_checker_timer = this->create_wall_timer(1s, [this](){check_timer_callback();});
    m_checker_timer->cancel();

    m_timer_publisher = this->create_publisher<ros_ads_msgs::msg::ADS>("/ros_ads/report_timer", rclcpp::QoS(m_pub_queue_size));
    m_event_publisher = this->create_publisher<ros_ads_msgs::msg::ADS>("/ros_ads/report_event", rclcpp::QoS(m_pub_queue_size));
    m_state_publisher = this->create_publisher<ros_ads_msgs::msg::State>("/ros_ads/state", rclcpp::QoS(m_pub_queue_size));
    m_subscriber = this->create_subscription<ros_ads_msgs::msg::ADS>("/ros_ads/command", rclcpp::QoS(m_pub_queue_size), [this](ros_ads_msgs::msg::ADS::SharedPtr msg){std::cout << "1" << std::endl;subscriber_callback(msg);});

    try {
        publish_state();
        configure();
    }
    catch (...) {
        RCLCPP_ERROR(get_logger(), "Configuration file not valid, please provide a valid configuration file");
        publish_state();
    }

    m_checker_timer = this->create_wall_timer(0s, [this](){check_timer_callback();}, mp_callback_group_checker);
    m_checker_timer->cancel();


}

void ADSNode::publish_state()
{

        m_state_msg.header.set__stamp(now());
        m_state_msg.state = m_ADS.getState();
        m_state_msg.error = m_ADS.getADSState();
        m_state_publisher->publish(m_state_msg);
}

int ADSNode::configure()
{
        RCLCPP_ERROR(get_logger(), "configure");
    try 
    { //get parameters from YAML file
        YAML::Node config = YAML::LoadFile(m_YAML_config_file);
        if(config[m_name])
        {
    



            m_ADS.setLocalNetID(config[m_name]["localNetID"].as<string>());
            m_ADS.setRemoteNetID(config[m_name]["remoteNetID"].as<string>());
            m_ADS.setRemoteIPV4(config[m_name]["remoteIP"].as<string>());


            if(config[m_name]["publish_on_timer"].size() != 0)
            {
                auto onTimer = config[m_name]["publish_on_timer"].as<vector<string>>();
                for(auto & var: onTimer)
                {
                    m_publish_on_timer_guard.lock();
                    m_publish_on_timer[var] = pair<string, double>();
                    m_publish_on_timer_guard.unlock();
                    m_variables_guard.lock();
                    m_variables[var] = pair<string, double>();
                    m_variables_guard.unlock();
                }
            }

            if(config[m_name]["publish_on_event"].size() != 0)
            {
                auto onEvent = config[m_name]["publish_on_event"].as<vector<string>>();
                for(auto & var: onEvent)
                {
                    m_publish_on_event_guard.lock();
                    m_publish_on_event[var] = pair<string, double>();
                    m_publish_on_event_guard.unlock();
                    m_variables_guard.lock();
                    m_variables[var] = pair<string, double>();
                    m_variables_guard.unlock();
                }
            }
        }
            // Configure timers with desired rate
        m_publisher_timer = this->create_wall_timer(std::chrono::milliseconds(int(1000./config[m_name]["publish_rate"].as<int>())),
                                                     [this](){publish_timer_callback();},
                                                     mp_callback_group_publisher);

        m_update_timer = this->create_wall_timer(std::chrono::milliseconds(int(1000./config[m_name]["refresh_rate"].as<int>())),
                                                  [this](){update_timer_callback();},
                                                  mp_callback_group_update);

        m_state_timer = this->create_wall_timer(std::chrono::milliseconds(int(1000./config[m_name]["state_rate"].as<int>())),
                                                  [this](){publish_state();},
                                                  mp_callback_group_state);


         
    }catch (...)
    {
        RCLCPP_ERROR(get_logger(), "Invalid configuration file");
        return 0;
    }

    try
    { 
        RCLCPP_INFO(get_logger(),"GO FOR Init Route" );
 
        m_ADS.initRoute();
        RCLCPP_INFO(get_logger(), "Init Route done");

    }
    catch(...)
    {
        RCLCPP_ERROR(get_logger(), "ERROR while connecting with ADS");

        return 0;
    }

    try
    {
      RCLCPP_INFO(this->get_logger(), "Acquiring ADS variables");
      m_ADS.acquireVariables();

      RCLCPP_INFO(this->get_logger(), "ADS variables acquired");
      RCLCPP_INFO(this->get_logger(), "Aliasing ADS variables");
      m_ADS.setName(m_name);
      m_ADS.setFile(m_YAML_config_file);
      m_ADS.bindPLcVar();
      RCLCPP_INFO(get_logger(), "Ready to communicate with the remote PLC via ADS.");
      
    }
    catch(AdsException e)
    {
      RCLCPP_ERROR(this->get_logger(),e.what());
      RCLCPP_ERROR(this->get_logger(), "ERROR in mapping alias with ADS");

      return 0;
    }

    
    return 1;



}

/**
 * @brief ADSNode::Subscriber send the ADS device the received commands
 * @param msg the command message
 * @return true if the command was successfully sent
 */
bool ADSNode::subscriber_callback(ros_ads_msgs::msg::ADS::SharedPtr msg)
{
    m_state_publisher->publish(m_state_msg);
    bool result = true;
    auto command = ros_ads_msgs::decode(msg);

    try {
        if(m_ADS.getState())
        {
            for (auto &[name, value]: command)
            {
                m_ADS.adsWriteValue(name, value);
            }
        }
    }
    catch (...)
    {
        result = false;
    }

    return result;
}


/**
 * @brief ADSNode::publish_timer_callback publish variables periodically
 * @param timer_rate rate to publish at
 */
void ADSNode::publish_timer_callback()

{
    try
    {
        m_on_timer_msg.header.set__stamp(now());
        m_on_timer_msg.var_names = vector<string>();
        m_on_timer_msg.var_types = vector<string>();
        m_on_timer_msg.var_values = vector<double>();

        m_publish_on_timer_guard.lock();
        auto publish_on_timer = m_publish_on_timer;
        m_publish_on_timer_guard.unlock();
        
        for(auto &[name, pair]: publish_on_timer)
        {
            m_on_timer_msg.var_names.push_back(name);
            m_on_timer_msg.var_types.push_back(pair.first);
            m_on_timer_msg.var_values.push_back(pair.second);
        }


        if(m_on_timer_msg.var_names.size() != 0)
        {
            m_timer_publisher->publish(m_on_timer_msg);
        }
        
    }
    catch(...)
    {
        RCLCPP_ERROR(get_logger(), "%s" , "fail publishOnTimer");

    }
}


/**
 * @brief ADSNode::check_timer_callback publish variables if an event occured
 * @param timer_rate rate to verify if an event occured at
 */
void ADSNode::check_timer_callback()
{
    try
    {
        if(m_ADS.getState())
        {
            m_publish_on_event_guard.lock();
            auto publish_on_event = m_publish_on_event;
            m_publish_on_event_guard.unlock();
            m_variables_guard.lock();
            auto variables = m_variables;
            m_variables_guard.unlock();

            m_publish = false;
            for(auto &[key, pair_] : publish_on_event)
            {
                m_checker_temp_value = variables[key];
                if(pair_ != m_checker_temp_value) // A change has occured
                {
                    m_publish_on_event_guard.lock();
                    m_publish_on_event[key] = m_checker_temp_value; // Update value
                    m_publish_on_event_guard.unlock();

                    if(!m_publish)
                    {
                        m_publish = true;
                    }
                }
            }
            if (m_publish) // Publish if a change occured
            {
                m_on_event_msg.header.set__stamp(now());
                m_on_event_msg.var_names = vector<string>();
                m_on_event_msg.var_types = vector<string>();
                m_on_event_msg.var_values = vector<double>();

                for (auto &[key, pair] : publish_on_event)
                {
                    m_on_event_msg.var_names.push_back(key);
                    auto type = pair.first;
                    m_on_event_msg.var_types.push_back(type);
                    auto value = pair.second;
                    m_on_event_msg.var_values.push_back(value);
                }
                if(m_on_event_msg.var_names.size() !=0)
                {
                    m_event_publisher->publish(m_on_event_msg);
                }
            }
        }
    }
    catch(...)
    {
        RCLCPP_ERROR(get_logger(),"fail checker CB");
    }
    

}

/**
 * @brief ADSNode::update_timer_callback update internal variable values memory
 *
 * Starts by verifying and updating connection state
 *
 */
void ADSNode::update_timer_callback()
{
    m_update_timer->cancel();
    try
    {
        m_ADS.connectionCheck();
        if(m_ADS.getState())
        {
            m_ADS.updateMemory();
            auto variables_map = m_ADS.getVariablesMap();
            for(auto &[name, pair]: variables_map)
            { //cast as double for the message
                m_variables[name].first = pair.first;
                switch (pair.second.index())
                {
                case ros_ads_msgs::BOOL:
                {
                    m_variables[name].second = static_cast<double>(get<bool>(pair.second));
                    break;
                }
                case ros_ads_msgs::UINT8_T:
                {
                    m_variables[name].second = static_cast<double>(get<uint8_t>(pair.second));
                    break;
                }
                case ros_ads_msgs::INT8_T:
                {
                    m_variables[name].second = static_cast<double>(get<int8_t>(pair.second));
                    break;
                }
                case ros_ads_msgs::UINT16_T:
                {
                    m_variables[name].second = static_cast<double>(get<uint16_t>(pair.second));
                    break;
                }
                case ros_ads_msgs::INT16_T:
                {
                    m_variables[name].second = static_cast<double>(get<int16_t>(pair.second));
                    break;
                }
                case ros_ads_msgs::UINT32_T:
                {
                    m_variables[name].second = static_cast<double>(get<uint32_t>(pair.second));
                    break;
                }
                case ros_ads_msgs::INT32_T:
                {
                    m_variables[name].second = static_cast<double>(get<int32_t>(pair.second));
                    break;
                }
                case ros_ads_msgs::INT64_T:
                {
                    m_variables[name].second = static_cast<double>(get<int64_t>(pair.second));
                    break;
                }
                case ros_ads_msgs::FLOAT:
                {
                    m_variables[name].second = static_cast<double>(get<float>(pair.second));
                    break;
                }
                case ros_ads_msgs::DOUBLE:
                {
                    m_variables[name].second = static_cast<double>(get<double>(pair.second));
                    break;
                }
                case ros_ads_msgs::DATE:
                {
                    tm temp = get<tm>(pair.second);
                    m_variables[name].second = static_cast<double>(mktime(&temp));
                    break;
                }
                default:
                {

                }
                }
                m_publish_on_timer_guard.lock();
                if(m_publish_on_timer.find(name) != m_publish_on_timer.end() && m_ADS.getState())
                {
                    m_publish_on_timer[name] = m_variables[name];
                }
                m_publish_on_timer_guard.unlock();
            }

            m_checker_timer->execute_callback();
        }
    }
    catch (...)
    {
        RCLCPP_ERROR(get_logger(),"fail checker CB");
    }

    m_update_timer->reset();
    
}



int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ads_node::ADSNode>());
    rclcpp::shutdown();
    
    return 0;
}
