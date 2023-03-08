#include <ros_ads_decode/ADSDecode.hpp>

std::map<std::string, ros_ads_decode::variant_t> ros_ads_decode::decode(ros_ads_msgs::msg::ADS::SharedPtr p_msg)
{
    std::map<std::string, std::variant<bool, uint8_t, int8_t, uint16_t, int16_t, uint32_t, int32_t, int64_t, float, double, tm>> result;
    std::string name;
    std::string type;
    double value;
    bool cresult;
    std::variant<bool, uint8_t, int8_t, uint16_t, int16_t, uint32_t, int32_t, int64_t, float, double, tm> converted_value;

    int minSize = std::min(std::min(p_msg->var_names.size(), p_msg->var_types.size()), p_msg->var_values.size());

    for(int index = 0; index < minSize; index ++)
    {
        cresult = false;
        name = p_msg->var_names[index];
        type = p_msg->var_types[index];
        value = p_msg->var_values[index];
        do
        {
            if(type == "BOOL")
            {
                converted_value = static_cast<bool>(value);
                cresult = true;
                break;
            }
            if(type == "BYTE" || type == "USINT")
            {
                converted_value = static_cast<uint8_t>(value);
                cresult = true;
                break;
            }
            if(type == "SINT")
            {
                converted_value = static_cast<int8_t>(value);
                cresult = true;
                break;
            }
            if(type == "WORD" || type == "UINT")
            {
                converted_value = static_cast<uint16_t>(value);
                cresult = true;
                break;
            }
            if(type == "INT")
            {
                converted_value = static_cast<int16_t>(value);
                cresult = true;
                break;
            }
            if(type == "DWORD" || type == "UDINT")
            {
                converted_value = static_cast<uint32_t>(value);
                cresult = true;
                break;
            }
            if(type == "DINT")
            {
                converted_value = static_cast<int32_t>(value);
                cresult = true;
                break;
            }
            if(type == "LINT")
            {
                converted_value = static_cast<int64_t>(value);
                cresult = true;
                break;
            }
            if(type == "REAL")
            {
                converted_value = static_cast<float>(value);
                cresult = true;
                break;
            }
            if(type == "LREAL")
            {
                converted_value = static_cast<double>(value);
                cresult = true;
                break;
            }
            /*if(type == "DATE")
            {
                auto temp = static_cast<uint32_t>(value);
                ros::Time currentDate(temp);
                time_t tDate(currentDate.toSec());
                tm tmDate;
                gmtime_r(&tDate,&tmDate);
                converted_value = tmDate;
                cresult = true;
                break;
            }*/
        }
        while(false);
        if(cresult)
        {
            result[name] = converted_value;
        }
    }
    return result;
}

