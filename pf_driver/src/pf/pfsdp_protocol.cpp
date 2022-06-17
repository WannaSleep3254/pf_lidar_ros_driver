#include <rcl_interfaces/msg/parameter_descriptor.hpp>
#include <rcl_interfaces/msg/integer_range.hpp>
#include "pf_driver/pf/pfsdp_protocol.hpp"

rcl_interfaces::msg::SetParametersResult PFSDPBase::reconfig_callback(const std::vector<rclcpp::Parameter>& parameters)
{
  rcl_interfaces::msg::SetParametersResult result;

  config_mutex_->lock();

  result.successful = reconfig_callback_impl(parameters);

  /*if (result.successful)
  {
    update_scanoutput_config();
  }*/

  config_mutex_->unlock();

  return result;
}

bool PFSDPBase::reconfig_callback_impl(const std::vector<rclcpp::Parameter>& parameters)
{
  bool successful = true;

  for (const auto& parameter : parameters)
  {
    if (parameter.get_name() == "ip_mode" || parameter.get_name() == "scan_frequency" ||
        parameter.get_name() == "subnet_mask" || parameter.get_name() == "gateway" ||
        parameter.get_name() == "scan_direction" || parameter.get_name() == "locator_indication" ||
        parameter.get_name() == "user_tag" || parameter.get_name() == "operating_mode")
    {
      set_parameter({ KV(parameter.get_name(), parameter.value_to_string()) });
    }
    else if (parameter.get_name() == "ip_address")
    {
      info_->hostname = parameter.as_string();
      set_parameter({ KV(parameter.get_name(), parameter.value_to_string()) });
    }
    else if (parameter.get_name() == "port")
    {
      info_->port = parameter.value_to_string();
    }
    else if (parameter.get_name() == "transport")
    {
      std::string transport_str = parameter.as_string();
      info_->handle_type = transport_str == "udp" ? HandleInfo::HANDLE_TYPE_UDP : HandleInfo::HANDLE_TYPE_TCP;
    }
    else if (parameter.get_name() == "scan_topic")
    {
      topic_ = parameter.as_string();
    }
    else if (parameter.get_name() == "frame_id")
    {
      frame_id_ = parameter.as_string();
    }
    else if (parameter.get_name() == "start_angle")
    {
      config_->start_angle = ((parameter.as_double() / M_PI) * 180.0) * 10000;
      set_scanoutput_parameters({ KV(parameter.get_name(), config_->start_angle) });
    }
    else if (parameter.get_name() == "max_num_points_scan")
    {
      config_->max_num_points_scan = parameter.as_int();
      set_scanoutput_parameters({ KV(parameter.get_name(), parameter.value_to_string()) });
    }
  }

  return successful;
}

bool PFSDPBase::init()
{
  // Set callback for dynamic parameters changes
  setup_parameters_callback();

  // Load the critical parameters that are required for everything else
  declare_critical_parameters();

  // Do some config checks
  if (info_->hostname.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "Please provide the scanner IP address on parameter 'ip_address'");
    return false;
  }

  if (info_->port.empty())
  {
    RCLCPP_ERROR(node_->get_logger(), "Please provide the scanner IP port on parameter 'port'");
    return false;
  }

  http_interface = std::make_unique<HTTPInterface>(info_->hostname, "cmd");

  // Declare other common parameters
  declare_common_parameters();

  //update_scanoutput_config();

  return true;
}

bool PFSDPBase::check_error(std::map<std::string, std::string>& mp, const std::string& err_code,
                            const std::string& err_text, const std::string& err_http)
{
  const std::string http_error = mp[err_http];
  const std::string code = mp[err_code];
  const std::string text = mp[err_text];

  // remove error related key-value pairs
  mp.erase(err_http);
  mp.erase(err_code);
  mp.erase(err_text);

  // check if HTTP has an error
  if (http_error.compare(std::string("OK")))
  {
    std::cerr << "HTTP ERROR: " << http_error << std::endl;
    return false;
  }

  // check if 'error_code' and 'error_text' does not exist in the response
  // this happens in case of invalid command
  if (!code.compare("--COULD NOT RETRIEVE VALUE--") || !text.compare("--COULD NOT RETRIEVE VALUE--"))
  {
    std::cout << "Invalid command or parameter requested." << std::endl;
    return false;
  }
  // check for error messages in protocol response
  if (code.compare("0") || text.compare("success"))
  {
    std::cout << "protocol error: " << code << " " << text << std::endl;
    return false;
  }
  return true;
}

void PFSDPBase::declare_common_parameters()
{
  /*rcl_interfaces::msg::ParameterDescriptor descriptorInitialIpAddress;
  descriptorInitialIpAddress.name = "Initial IP address";
  descriptorInitialIpAddress.description = "When initiating scan data output, request_handle_udp must be given an IPv4
  address and port in order to know where to send scandata to."; node_->declare_parameter<std::string>("address", "",
  descriptorInitialIpAddress);*/

  rcl_interfaces::msg::ParameterDescriptor descriptorSubnetMask;
  descriptorSubnetMask.name = "IP netmask";
  node_->declare_parameter<std::string>("subnet_mask", "255.0.0.0", descriptorSubnetMask);

  rcl_interfaces::msg::ParameterDescriptor descriptorGateway;
  descriptorGateway.name = "IP gateway";
  node_->declare_parameter<std::string>("gateway", "0.0.0.0", descriptorGateway);

  rcl_interfaces::msg::ParameterDescriptor descriptorScanFreqency;
  descriptorScanFreqency.name = "Scan frequency";
  descriptorScanFreqency.description = "The parameter scan_frequency defines the set point for the rotational speed of "
                                       "the sensor head and therefore the number of scans recorded per second. For the "
                                       "R2000 valid values range from 10 Hz to 50 Hz with steps of 1 Hz.";
  rcl_interfaces::msg::IntegerRange rangeScanFrequency;
  rangeScanFrequency.from_value = 10;
  rangeScanFrequency.to_value = 50;
  rangeScanFrequency.step = 1;
  descriptorScanFreqency.integer_range.push_back(rangeScanFrequency);
  node_->declare_parameter<int>("scan_frequency", 30, descriptorScanFreqency);

  rcl_interfaces::msg::ParameterDescriptor descriptorScanTopic;
  descriptorScanTopic.name = "Scan topic";
  descriptorScanTopic.description = "Topic on which the LaserScan messages will be published";
  descriptorScanTopic.read_only = true;
  node_->declare_parameter<std::string>("scan_topic", "/scan", descriptorScanTopic);

  rcl_interfaces::msg::ParameterDescriptor descriptorFrameId;
  descriptorFrameId.name = "Scan frame ID";
  descriptorFrameId.description = "Frame ID in which the LaserScan messages will be published";
  descriptorFrameId.read_only = true;
  node_->declare_parameter<std::string>("frame_id", "scanner", descriptorScanTopic);

  rcl_interfaces::msg::ParameterDescriptor descriptorStartAngle;
  descriptorFrameId.name = "Start angle";
  descriptorFrameId.description = "Angle of first scan point for scan data output, in radians";
  node_->declare_parameter<double>("start_angle", -M_PI, descriptorStartAngle);

  rcl_interfaces::msg::ParameterDescriptor descriptorMaxNumPoints;
  descriptorFrameId.name = "Maximum number of points per scan";
  descriptorFrameId.description = "Limit number of points in scan data output";
  node_->declare_parameter<int>("max_num_points_scan", 0, descriptorMaxNumPoints);
}

void PFSDPBase::setup_parameters_callback()
{
  parameters_handle_ =
      node_->add_on_set_parameters_callback(std::bind(&PFSDPBase::reconfig_callback, this, std::placeholders::_1));
}

void PFSDPBase::declare_critical_parameters()
{
  rcl_interfaces::msg::ParameterDescriptor descriptorIpAddress;
  descriptorIpAddress.name = "IP address";
  node_->declare_parameter<std::string>("ip_address", "10.0.10.9", descriptorIpAddress);

  rcl_interfaces::msg::ParameterDescriptor descriptorPort;
  descriptorPort.name = "Initial IP port";
  descriptorPort.description = "See address";
  node_->declare_parameter<int>("port", 0, descriptorPort);

  rcl_interfaces::msg::ParameterDescriptor descriptorTransport;
  descriptorTransport.name = "IP transport port";
  descriptorTransport.description = "Can be either tcp or udp";
  descriptorTransport.read_only = true;
  node_->declare_parameter<std::string>("transport", "tcp", descriptorTransport);
}

void PFSDPBase::update_scanoutput_config()
{
  if (http_interface)
  {
    param_map_type query = { KV("handle", info_->handle),
                             KV("start_angle", config_->start_angle),
                             KV("packet_type", config_->packet_type),
                             KV("max_num_points_scan", config_->max_num_points_scan),
                             KV("watchdogtimeout", config_->watchdogtimeout),
                             KV("skip_scans", config_->skip_scans),
                             KV("watchdog", config_->watchdog ? "on" : "off") };
    auto resp = get_request("set_scanoutput_config", { "" }, query);

    // recalculate scan params
    read_scan_parameters();
  }
}

bool PFSDPBase::set_scanoutput_config(std::string handle, ScanConfig config)
{
  param_map_type query = { KV("handle", handle),
                           KV("start_angle", config.start_angle),
                           KV("packet_type", config.packet_type),
                           KV("max_num_points_scan", config.max_num_points_scan),
                           KV("watchdogtimeout", config.watchdogtimeout),
                           KV("skip_scans", config.skip_scans),
                           KV("watchdog", config.watchdog ? "on" : "off") };
  auto resp = get_request("set_scanoutput_config", { "" }, query);

  // update global config_
  get_scanoutput_config(handle);
  read_scan_parameters();
  return true;
}

void PFSDPBase::get_scanoutput_config(std::string handle)
{
  auto resp = get_request(
                "get_scanoutput_config",
  { "start_angle", "packet_type", "watchdogtimeout", "skip_scans", "watchdog", "max_num_points_scan" },
  { KV("handle", handle) });
  config_->packet_type = resp["packet_type"];
  config_->start_angle = to_long(resp["start_angle"]);
  config_->watchdogtimeout = to_long(resp["watchdogtimeout"]);
  config_->watchdog = (resp["watchdog"] == "off") ? false : true;
  config_->skip_scans = to_long(resp["skip_scans"]);
  config_->max_num_points_scan = to_long(resp["max_num_points_scan"]);
}

void PFSDPBase::set_scanoutput_parameters(const param_map_type &params)
{
  param_map_type query{ {"handle", info_->handle} };
  query.insert(params.begin(), params.end());
  get_request("set_scanoutput_config", { "" }, query);
  read_scan_parameters();
}

void PFSDPBase::request_handle_udp(const std::string packet_type)
{
  param_map_type query = { KV("address", info_->endpoint), KV("port", info_->port) };
  if (!packet_type.empty())
  {
    query["packet_type"] = packet_type;
  }
  else
  {
    query["packet_type"] = config_->packet_type;
  }
  auto resp = get_request("request_handle_udp", { "handle", "port" }, query);
  info_->handle = resp["handle"];
}

void PFSDPBase::request_handle_tcp(const std::string port, const std::string packet_type)
{
  param_map_type query;
  if (!port.empty())
  {
    query["port"] = port;
  }
  else if (info_->port != "0")
  {
    query["port"] = info_->port;
  }
  if (!packet_type.empty())
  {
    query["packet_type"] = packet_type;
  }
  else
  {
    query["packet_type"] = config_->packet_type;
  }
  auto resp = get_request("request_handle_tcp", { "handle", "port" }, query);

  info_->handle = resp["handle"];
  info_->port = resp["port"];

  // TODO: port and pkt_type should be updated in config_
}

std::string PFSDPBase::get_parameter_str(const std::string param)
{
  std::map<std::string, std::string> resp = get_parameter(param);
  if (resp.empty())
  {
    return std::string("");
  }
  return resp[param];
}

float PFSDPBase::get_parameter_float(const std::string param)
{
  std::map<std::string, std::string> resp = get_parameter(param);
  if (resp.empty())
  {
    return std::numeric_limits<float>::quiet_NaN();
  }
  return to_float(resp[param]);
}

int64_t PFSDPBase::get_parameter_int(const std::string param)
{
  std::map<std::string, std::string> resp = get_parameter(param);
  if (resp.empty())
  {
    return std::numeric_limits<std::int64_t>::quiet_NaN();
  }
  return to_long(resp[param]);
}

ProtocolInfo PFSDPBase::get_protocol_info()
{
  ProtocolInfo opi;
  auto resp = get_request("get_protocol_info", { "protocol_name", "version_major", "version_minor", "commands" });
  if (resp.empty())
  {
    opi.isError = true;
    return opi;
  }

  opi.version_major = atoi(resp["version_major"].c_str());
  opi.version_minor = atoi(resp["version_minor"].c_str());
  opi.protocol_name = resp["protocol_name"];

  return opi;
}

const std::map<std::string, std::string> PFSDPBase::get_request(const std::string command, std::vector<std::string> json_keys, const param_map_type query)
{
  const std::string err_code = "error_code";
  const std::string err_text = "error_text";
  const std::string err_http = "error_http";
  std::vector<std::string> keys = { err_code, err_text };
  keys.insert(keys.end(), json_keys.begin(), json_keys.end());
  std::map<std::string, std::string> json_resp = http_interface->get(keys, command, query);

  if (!check_error(json_resp, err_code, err_text, err_http))
  {
    return std::map<std::string, std::string>();
  }

  return json_resp;
}

bool PFSDPBase::get_request_bool(const std::string command, std::vector<std::string> json_keys, std::initializer_list<param_type> query)
{
  std::map<std::string, std::string> resp = get_request(command, json_keys, query);
  if (resp.empty())
  {
    return false;
  }
  return true;
}
