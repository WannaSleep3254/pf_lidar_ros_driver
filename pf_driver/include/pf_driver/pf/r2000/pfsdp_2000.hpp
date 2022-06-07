#pragma once

#include "pf_driver/pf/pfsdp_protocol.hpp"

class PFSDP_2000 : public PFSDPBase
{
public:
  PFSDP_2000(std::shared_ptr<PFSDPBase> base) : PFSDPBase(base)
  {
    declare_specific_parameters();
  }

  virtual std::string get_product()
  {
    return get_parameter_str("product");
  }

  virtual void read_scan_parameters()
  {
    auto resp = get_parameter("angular_fov", "radial_range_min", "radial_range_max", "scan_frequency");
    params_->angular_fov = to_float(resp["angular_fov"]) * M_PI / 180.0;
    params_->radial_range_max = to_float(resp["radial_range_max"]);
    params_->radial_range_min = to_float(resp["radial_range_min"]);

    params_->angle_min = config_->start_angle / 10000.0f * M_PI / 180.0;
    params_->angle_max = params_->angle_min + params_->angular_fov;
    params_->scan_freq = to_float(resp["scan_frequency"]);
  }

  void declare_specific_parameters() override
  {
    rcl_interfaces::msg::ParameterDescriptor descriptorPacketType;
    descriptorPacketType.name = "Packet type";
    descriptorPacketType.description = "Packet type for scan data output";
    descriptorPacketType.read_only = true;
    node_->declare_parameter<std::string>("packet_type", "C", descriptorPacketType);

    rcl_interfaces::msg::ParameterDescriptor descriptorWatchdog;
    descriptorWatchdog.name = "Watchdog";
    descriptorWatchdog.description = "Cease scan data output if watchdog isn't fed in time";
    descriptorWatchdog.read_only = true;
    node_->declare_parameter<bool>("watchdog", true, descriptorWatchdog);

    rcl_interfaces::msg::ParameterDescriptor descriptorWatchdogTimeout;
    descriptorWatchdogTimeout.name = "Watchdog timeout (ms)";
    descriptorWatchdogTimeout.description = "Maximum time for client to feed watchdog";
    descriptorWatchdogTimeout.read_only = true;
    node_->declare_parameter<int>("watchdog_timeout", 10000, descriptorWatchdogTimeout);

    rcl_interfaces::msg::ParameterDescriptor descriptorSamplesPerScan;
    descriptorSamplesPerScan.name = "Samples per scan";
    descriptorSamplesPerScan.description = "Defines the number of samples recorded during one revolution of the sensor "
                                           "head (for details please refer to section 3.1 in the R2000 Ethernet "
                                           "communication protocol).";
    rcl_interfaces::msg::IntegerRange rangeSamplesPerScan;
    rangeSamplesPerScan.from_value = 72;
    rangeSamplesPerScan.to_value = 25200;
    rangeSamplesPerScan.step = 1;
    descriptorSamplesPerScan.integer_range.push_back(rangeSamplesPerScan);
    node_->declare_parameter<int>("samples_per_scan", 2400, descriptorSamplesPerScan);

    rcl_interfaces::msg::ParameterDescriptor descriptorSkipScans;
    descriptorSkipScans.name = "Skip scans";
    descriptorSkipScans.description = "Reduce scan output rate to every (n+1)th scan";
    node_->declare_parameter<int>("skip_scans", 0, descriptorSkipScans);

    rcl_interfaces::msg::ParameterDescriptor descriptorDisplayMode;
    descriptorDisplayMode.name = "HMI display mode";
    descriptorDisplayMode.description = "Controls the content of the HMI LED display during normal operation, i.e. "
                                        "while neither warnings nor errors are displayed and the user did not activate "
                                        "the HMI menu. Supported values are:\n"
                                        "- hmi_display_off\n"
                                        "- static_logo\n"
                                        "- static_text\n"
                                        "- bargraph_distance\n"
                                        "- bargraph_echo\n"
                                        "- bargraph_reflector\n"
                                        "- application_bitmap\n"
                                        "- application_text\n";
    node_->declare_parameter<std::string>("hmi_display_mode", "static_logo", descriptorDisplayMode);
  }

  virtual bool reconfig_callback_impl(const std::vector<rclcpp::Parameter>& parameters) override
  {
    bool successful = PFSDPBase::reconfig_callback_impl(parameters);

    for (const auto& parameter : parameters)
    {
      if (parameter.get_name() == "samples_per_scan" || parameter.get_name() == "hmi_display_mode" ||
          parameter.get_name() == "hmi_language" || parameter.get_name() == "hmi_button_lock" ||
          parameter.get_name() == "hmi_parameter_lock" || parameter.get_name() == "hmi_static_text_1" ||
          parameter.get_name() == "hmi_static_text_2" || parameter.get_name() == "user_notes")
      {
        set_parameter({ KV(parameter.get_name(), parameter.value_to_string()) });
      }
      else if (parameter.get_name() == "watchdog")
      {
        config_->watchdog = parameter.as_bool();
      }
      else if (parameter.get_name() == "watchdog_timeout")
      {
        config_->watchdogtimeout = parameter.as_int();
      }
      else if (parameter.get_name() == "packet_type")
      {
        std::string packet_type = parameter.as_string();
        if (packet_type == "A" || packet_type == "B" || packet_type == "C")
        {
          config_->packet_type = packet_type;
        }
        else
        {
          successful = false;
        }
      }
      else if (parameter.get_name() == "skip_scans")
      {
        config_->skip_scans = parameter.as_int();
      }
    }

    return successful;
  }
};
