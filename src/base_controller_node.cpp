#include <unistd.h>
#include <iostream>
#include <sstream>
#include <chrono>
#include <signal.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <serial/serial.h>
#include <std_msgs/msg/float32.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <sensor_msgs/msg/battery_state.hpp>

using namespace std;

#define TWO_PI (2*M_PI)
#define TO_RAD_PER_SEC ((2*M_PI)/60)

using namespace std::chrono_literals;

namespace roboteqBridge {
  class baseController: public rclcpp::Node {
  public:

    baseController():Node("base_controller_node"), count_(0){

      this->declare_parameter("port_name", this->param_port_name);
      this->param_port_name = this->get_parameter("port_name").as_string();
      this->serial_port.setPort(param_port_name);
      RCLCPP_INFO_STREAM(this->get_logger(), "Port name\t\t: " << param_port_name);

      this->declare_parameter("port_timeout", this->param_port_timeout);
      this->param_port_timeout = this->get_parameter("port_timeout").as_int();
      serial::Timeout timeout = serial::Timeout::simpleTimeout(param_port_timeout);
  		this->serial_port.setTimeout(timeout);
      RCLCPP_INFO_STREAM(this->get_logger(), "Port timeout\t: " << param_port_timeout);

      this->declare_parameter("baud_rate", this->param_baud_rate);
      this->param_baud_rate = this->get_parameter("baud_rate").as_int();
      this->serial_port.setBaudrate(param_baud_rate);
      RCLCPP_INFO_STREAM(this->get_logger(), "Port baudrate\t: " << param_baud_rate);

      this->declare_parameter("encoder_gear_ratio", this->param_gear_ratio);
      this->param_gear_ratio = this->get_parameter("encoder_gear_ratio").as_double();
      RCLCPP_INFO_STREAM(this->get_logger(), "Gear ratio\t: " << param_gear_ratio);

      this->declare_parameter("wheel_radius", this->param_wheel_radius);
      this->param_wheel_radius = this->get_parameter("wheel_radius").as_double();
      RCLCPP_INFO_STREAM(this->get_logger(), "Wheel radius\t: " << param_wheel_radius);

      this->declare_parameter("wheel_separation", this->param_wheel_separation);
      this->param_wheel_separation = this->get_parameter("wheel_separation").as_double();
      RCLCPP_INFO_STREAM(this->get_logger(), "Wheel separation\t: " << param_wheel_separation);

      this->declare_parameter("channel_order_RL", this->param_channel_order_RL);
      this->param_channel_order_RL = this->get_parameter("channel_order_RL").as_bool();
      RCLCPP_INFO_STREAM(this->get_logger(), "Channel order\t: " << param_channel_order_RL);

      this->declare_parameter("battery_frame_id", this->param_battery_frame_id);
      this->param_battery_frame_id = this->get_parameter("battery_frame_id").as_string();
      battery_msg.header.frame_id = param_battery_frame_id;
      RCLCPP_INFO_STREAM(this->get_logger(), "Battery frame\t: " << param_battery_frame_id);

      this->declare_parameter("power_supply_technology", this->param_battery_tech);
      this->param_battery_frame_id = this->get_parameter("power_supply_technology").as_int();
      battery_msg.power_supply_technology = param_battery_tech;
      battery_msg.present = true;

      this->declare_parameter("watchdog_timeout", this->param_watchdog_timeout);
      this->param_watchdog_timeout = this->get_parameter("watchdog_timeout").as_int();

      this->declare_parameter("current_limit", this->param_current_limit);
      this->param_current_limit = this->get_parameter("current_limit").as_int();

      this->declare_parameter("max_rpm", this->param_max_rpm);
      this->param_max_rpm = this->get_parameter("max_rpm").as_int();

      this->declare_parameter("acceleration_rate", this->param_acceleration_rate);
      this->param_acceleration_rate = this->get_parameter("acceleration_rate").as_int();

      this->declare_parameter("deceleration_rate", this->param_deceleration_rate);
      this->param_deceleration_rate = this->get_parameter("deceleration_rate").as_int();

      this->declare_parameter("control_loop_mode", this->param_control_loop);
      this->param_control_loop = this->get_parameter("control_loop_mode").as_int();

      this->declare_parameter("encoder_ppr", this->param_encoder_ppr);
      this->param_encoder_ppr = this->get_parameter("encoder_ppr").as_int();

      RCLCPP_INFO(this->get_logger(), "-------------------");
      try {
          serial_port.open();
          if ( serial_port.isOpen() )
          {
              RCLCPP_INFO_STREAM(this->get_logger(), "Connection\t: OPEN");
          }
      }
      catch (serial::IOException e) {
        RCLCPP_ERROR(this->get_logger(),"Failed to open serial port. Exiting...");
        try {
          rclcpp::shutdown();
        }
        catch (rclcpp::exceptions::RCLError e){}
      }

      this->set_current_limit(param_current_limit);
      this->set_max_rpm(param_max_rpm);
      this->set_acceleration_rate(param_acceleration_rate);
      this->set_deceleration_rate(param_deceleration_rate);
      this->set_encoder_ppr(param_encoder_ppr);
      this->set_pid_parameters(1, 1.5, 1.0, 0.0); // TODO: dyn. reconf
      this->set_pid_parameters(2, 1.5, 1.0, 0.0);
      this->set_watchdog(param_watchdog_timeout);
      this->set_control_loop(param_control_loop);
      this->disable_echo();
      this->query_motor_feedback_stream();
      this->query_battery_state_stream();
      this->serial_port.flush();

      this->timer_ = this->create_wall_timer(1ms, std::bind(&baseController::read_feedback_stream, this));
      this->battery_pub = create_publisher<sensor_msgs::msg::BatteryState>("/base/battery", 10);
      this->left_current_pub   = create_publisher<std_msgs::msg::Float32>("/left_motor/measured/current", 10);
      this->right_current_pub  = create_publisher<std_msgs::msg::Float32>("/right_motor/measured/current", 10);
      this->left_velocity_pub  = create_publisher<std_msgs::msg::Float32>("/left_motor/measured/angular_velocity", 10);
      this->right_velocity_pub = create_publisher<std_msgs::msg::Float32>("/right_motor/measured/angular_velocity", 10);

      this->twist_sub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&baseController::twist_callback, this, std::placeholders::_1));
      this->left_velocity_sub = this->create_subscription<std_msgs::msg::Float32>("left_motor/command/angular_velocity", 1, std::bind(&baseController::left_motor_callback, this, std::placeholders::_1));
      this->right_velocity_sub = this->create_subscription<std_msgs::msg::Float32>("right_motor/command/angular_velocity", 1, std::bind(&baseController::right_motor_callback, this, std::placeholders::_1));
    }

  private:

    struct motor_state {
      int32_t decaamps;
      int32_t encoder;
      double velocity;
      double current;
    };

    size_t  count_;
    rclcpp::TimerBase::SharedPtr timer_;

    serial::Serial serial_port;
    std::string param_port_name = "/dev/roboteq";
    uint16_t param_port_timeout = 500; // ms
    long     param_baud_rate = 234000;

    uint16_t param_watchdog_timeout = 2000; // ms
    uint16_t param_current_limit = 60; // amps
    uint16_t param_acceleration_rate = 8000; // rpm/sec
    uint16_t param_deceleration_rate = 8000; // rpm/sec
    uint16_t param_max_rpm = 2000; // rpm
    uint16_t param_control_loop = 1; // closed-loop speed
    uint16_t param_encoder_ppr = 2048;

    double   param_gear_ratio = 18.6868;
    double   param_wheel_radius = 0.167;
    double   param_wheel_separation = 0.560;
    bool     param_channel_order_RL = true;

    motor_state left_motor;
    motor_state right_motor;

    sensor_msgs::msg::BatteryState battery_msg;
    rclcpp::Publisher<sensor_msgs::msg::BatteryState>::SharedPtr battery_pub;
    std::string param_battery_frame_id = "base_link";
    uint8_t param_battery_tech = 1;

    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_current_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_current_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr left_velocity_pub;
    rclcpp::Publisher<std_msgs::msg::Float32>::SharedPtr right_velocity_pub;
    std_msgs::msg::Float32 left_motor_current_msg;
    std_msgs::msg::Float32 right_motor_current_msg;
    std_msgs::msg::Float32 left_motor_velocity_msg;
    std_msgs::msg::Float32 right_motor_velocity_msg;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr left_velocity_sub;
    rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr right_velocity_sub;

    float calculate_wheel_angular_velocity(int32_t motor_rpm) {
        return ( ( (float)motor_rpm * TO_RAD_PER_SEC ) / param_gear_ratio );
    }

    void twist_callback(const geometry_msgs::msg::Twist &twist){
      double left_wheel_angular_velocity  = ((2.0f * twist.linear.x - param_wheel_separation * twist.angular.z)/2.0f) / param_wheel_radius;
      double right_wheel_angular_velocity = ((2.0f * twist.linear.x + param_wheel_separation * twist.angular.z)/2.0f) / param_wheel_radius;
      int32_t right_wheel_rpm = (int32_t)((floor(60.0 * right_wheel_angular_velocity)/TWO_PI)*param_gear_ratio);
      int32_t left_wheel_rpm  = (int32_t)((floor(60.0 * left_wheel_angular_velocity)/TWO_PI)*param_gear_ratio);
      send_motor_rpm_left(left_wheel_rpm);
      send_motor_rpm_right(right_wheel_rpm);
    }

    void left_motor_callback(const std_msgs::msg::Float32 &command){
      int32_t left_wheel_rpm  = (int32_t)((floor(60.0 * command.data)/TWO_PI)*param_gear_ratio);
      send_motor_rpm_left(left_wheel_rpm);
    }

    void right_motor_callback(const std_msgs::msg::Float32 &command){
      int32_t right_wheel_rpm = (int32_t)((floor(60.0 * command.data)/TWO_PI)*param_gear_ratio);
      send_motor_rpm_right(right_wheel_rpm);
    }

    void reset_controller()
    {
      // reset controller
      serial_port.write("^RESET\r");
      // stop motors
      serial_port.write("!G 1 0\r");
      serial_port.write("!G 2 0\r");
      serial_port.write("!S 1 0\r");
      serial_port.write("!S 2 0\r");
      serial_port.flush();
      RCLCPP_INFO(this->get_logger(), "Controller reset. Full stop.");
    }

    void disable_echo()
    {
      // disable command echo
      serial_port.write("^ECHOF 1\r");
      serial_port.flush();
      RCLCPP_INFO(this->get_logger(), "Command echo \t: OFF");
    }

    void enable_echo()
    {
      // enable command echo
      serial_port.write("^ECHOF 0\r");
      serial_port.flush();
      RCLCPP_INFO(this->get_logger(), "Command echo \t: ON");
    }

    void set_watchdog(uint16_t watchdog_timeout)
    {
      // set watchdog timeout
      std::stringstream wdt_cmd;
      wdt_cmd << "^RWD " << watchdog_timeout << "\r";
      serial_port.write(wdt_cmd.str());
      RCLCPP_INFO(this->get_logger(), "Watchdog timeout\t: %d", watchdog_timeout);
    }

    void set_current_limit(uint16_t motor_amp_limit)
    {
      // set motor amps limit
      std::stringstream alim_cmd1;
      alim_cmd1 << "^ALIM 1 " << motor_amp_limit*10 << "\r";
      serial_port.write(alim_cmd1.str());
      std::stringstream alim_cmd2;
      alim_cmd2 << "^ALIM 2 " << motor_amp_limit*10 << "\r";
      serial_port.write(alim_cmd2.str());
      RCLCPP_INFO(this->get_logger(), "Max. current\t: %d Amps", motor_amp_limit);
    }

    void set_max_rpm(uint16_t motor_max_speed)
    {
      // set max motor speed (rpm) for relative speed commands
      std::stringstream mxrpm_cmd1;
      mxrpm_cmd1 << "^MXRPM 1 " << motor_max_speed << "\r";
      serial_port.write(mxrpm_cmd1.str());
      std::stringstream mxrpm_cmd2;
      mxrpm_cmd2 << "^MXRPM 2 " << motor_max_speed << "\r";
      serial_port.write(mxrpm_cmd2.str());
      RCLCPP_INFO(this->get_logger(), "Max. velocity\t: %d RPM", motor_max_speed);
    }

    void set_acceleration_rate(uint16_t motor_max_acceleration)
    {
      // set max motor acceleration rate (rpm/s * 10)
      std::stringstream mac_cmd1;
      mac_cmd1 << "^MAC 1 " << motor_max_acceleration*10 << "\r";
      serial_port.write(mac_cmd1.str());
      std::stringstream mac_cmd2;
      mac_cmd2 << "^MAC 2 " << motor_max_acceleration*10 << "\r";
      serial_port.write(mac_cmd2.str());
      RCLCPP_INFO(this->get_logger(), "Acceleration \t: %d RPM/sec", int32_t(motor_max_acceleration));
    }

    void set_deceleration_rate(uint16_t motor_max_deceleration)
    {
      // set max motor deceleration rate (rpm/s * 10)
      std::stringstream mdec_cmd1;
      mdec_cmd1 << "^MDEC 1 " << motor_max_deceleration*10 << "\r";
      serial_port.write(mdec_cmd1.str());
      std::stringstream mdec_cmd2;
      mdec_cmd2 << "^MDEC 2 " << motor_max_deceleration*10 << "\r";
      serial_port.write(mdec_cmd2.str());
      RCLCPP_INFO(this->get_logger(), "Deceleration \t: %d RPM/sec", int32_t(motor_max_deceleration));
    }

    void set_digital_output(const int32_t number, bool state)
    {
      std::stringstream cmd;
      if (state) { cmd << "!D1 " << number << "\r"; }
      else { cmd << "!D0 " << number << "\r"; }
      serial_port.write(cmd.str());
      serial_port.flush();
    }

    void disable_output_actions(uint8_t io_channel)
    {
      // disable automatic digital output actions
      std::stringstream set_doa_cmd;
      set_doa_cmd << "^DOA " << io_channel << " 0\r";
      serial_port.write(set_doa_cmd.str());
      RCLCPP_INFO(this->get_logger(), "Disabled output actions on: %i", io_channel);
    }

    void set_control_loop(uint8_t mode){

      /*
      0: Open-loop
      1: Closed-loop speed
      2: Closed-loop position relative
      3: Closed-loop count position
      4: Closed-loop position tracking
      5: Closed-loop torque
      6: Closed-loop speed position
      */

      switch (mode){
        case 0:
          serial_port.write("^MMOD 1 0\r");
          serial_port.write("^MMOD 2 0\r");
          RCLCPP_INFO(this->get_logger(), "Control loop\t: open");
        break;
        case 1:
          // set motor operating mode to closed-loop speed
          serial_port.write("^MMOD 1 1\r");
          serial_port.write("^MMOD 2 1\r");
          // set encoder mode (18 for feedback on motor1, 34 for feedback on motor2)
          serial_port.write("^EMOD 1 18\r");
          serial_port.write("^EMOD 2 34\r");
          RCLCPP_INFO(this->get_logger(), "Control loop\t: closed-loop speed");
        break;
        default:
          RCLCPP_INFO(this->get_logger(), "Control loop\t: unsupported, reverting to open loop");
          serial_port.write("^MMOD 1 0\r");
          serial_port.write("^MMOD 2 0\r");
        break;
      }
    }

    void set_encoder_ppr(uint16_t encoder_ppr)
    {
      // set encoder counts (ppr)
      std::stringstream right_enccmd;
      right_enccmd << "^EPPR 1 " << encoder_ppr << "\r";
      std::stringstream left_enccmd;
      left_enccmd << "^EPPR 2 " << encoder_ppr << "\r";
      serial_port.write(right_enccmd.str());
      serial_port.write(left_enccmd.str());
      RCLCPP_INFO(this->get_logger(), "Encoder resolution: %d", encoder_ppr);
    }

    void set_pid_parameters(uint8_t motor_id, double P, double I, double D)
    {
      std::stringstream cmd;
      cmd << "^KP "<< motor_id << " " << uint16_t(P*10) << "\r";
      serial_port.write(cmd.str());
      cmd.str(std::string());
      cmd << "^KI "<< motor_id << " " << uint16_t(I*10) << "\r";
      serial_port.write(cmd.str());
      cmd.str(std::string());
      cmd << "^KD "<< motor_id << " " << uint16_t(D*10) << "\r";
      serial_port.write(cmd.str());
      RCLCPP_INFO(this->get_logger(), "Motor-%d PID\t: %f \t%f \t%f", motor_id, P, I, D);
    }

    void clear_buffer_history()
    {
      // clear buffer history
      std::stringstream stream_command;
      stream_command << "# C\r";
      serial_port.write(stream_command.str());
    }

    void query_motor_feedback_stream()
    {
      // query motor speed and current streams at 20Hz
      std::stringstream stream_command;
      if (param_channel_order_RL) {
          stream_command.str(std::string());
          stream_command << R"(/"RM="," "?a 1_?s 1_# 20)" << "\r";
          serial_port.write(stream_command.str());
          stream_command.str(std::string());
          stream_command << R"(/"LM="," "?a 2_?s 2_# 20)" << "\r";
          serial_port.write(stream_command.str());
      }
      else {
          stream_command.str(std::string());
          stream_command << R"(/"RM="," "?a 2_?s 2_# 20)" << "\r";
          serial_port.write(stream_command.str());
          stream_command.str(std::string());
          stream_command << R"(/"LM="," "?a 1_?s 1_# 20)" << "\r";
          serial_port.write(stream_command.str());
      }
      RCLCPP_INFO(this->get_logger(), "Feedback streams  : ON");
    }

    void query_battery_state_stream(){
      std::stringstream stream_command;
      stream_command.str(std::string());
      stream_command << R"(/"BA="," "?v 2_?t 1_# 100)" << "\r";
      serial_port.write(stream_command.str());
      RCLCPP_INFO(this->get_logger(), "Battery stream    : ON");
    }

    void send_motor_power_right(const int32_t motor_power)
    {
        std::stringstream cmd;
        if (param_channel_order_RL) {
            cmd << "!G 1 " << motor_power << "\r"; // Ch1: R, Ch2: L
        } else {
            cmd << "!G 2 " << motor_power << "\r";
        }
        serial_port.write(cmd.str());
        serial_port.flush();
    }

    void send_motor_power_left(const int32_t motor_power)
    {
        std::stringstream cmd;
        if (param_channel_order_RL) {
            cmd << "!G 1 " << motor_power << "\r"; // Ch1: R, Ch2: L
        } else {
            cmd << "!G 2 " << motor_power << "\r";
        }
        serial_port.write(cmd.str());
        serial_port.flush();
    }

    void send_motor_rpm_right(const int32_t motor_rpm)
    {
        std::stringstream cmd;
        if (param_channel_order_RL) {
            cmd << "!S 1 " << motor_rpm << "\r"; // Ch1: R, Ch2: L
        } else {
            cmd << "!S 2 " << motor_rpm << "\r";
        }
        serial_port.write(cmd.str());
        serial_port.flush();
    }

    void send_motor_rpm_left(const int32_t motor_rpm)
    {
        std::stringstream cmd;
        if (param_channel_order_RL) {
            cmd << "!S 2 " << motor_rpm << "\r"; // Ch1: R, Ch2: L
        } else {
            cmd << "!S 1 " << motor_rpm << "\r";
        }
        serial_port.write(cmd.str());
        serial_port.flush();
    }

    void emergency_stop(){
      std::stringstream cmd;
      cmd << "!EX\r";
      serial_port.write(cmd.str());
      serial_port.flush();
    }

    void reset_emergency_stop(){
      std::stringstream cmd;
      cmd << "!MG\r";
      serial_port.write(cmd.str());
      serial_port.flush();
    }

    void read_feedback_stream()
    {
        std::string buffer = "";
        std::string lkey = "LM=";
        std::string rkey = "RM=";
        std::string bkey = "BA=";

        if (serial_port.available()>0)
        {
            serial_port.readline(buffer, 65536, "\r");
            if (
                  (buffer.length() > 4) &&
                  ((buffer.substr(0,3) == lkey) ||
                   (buffer.substr(0,3) == rkey) ||
                   (buffer.substr(0,3) == bkey))
                  )
            {
                // Valid header keyword
                std::string data = buffer.substr(3);
                std::stringstream ss(data);
                if (buffer.substr(0,3) == rkey) { // RIGHT MOTOR
                    ss >> right_motor.decaamps >> right_motor.encoder;
                    if (ss.fail()) { // Invalid message
                        RCLCPP_WARN(this->get_logger(),"Right motor channel: unexpected data");
                        return;
                    }
                    right_motor.velocity = calculate_wheel_angular_velocity(right_motor.encoder);
                    right_motor.current  = right_motor.decaamps/10.0f;
                    right_motor_current_msg.data  = right_motor.current;
                    right_motor_velocity_msg.data = right_motor.velocity;
                    right_current_pub->publish(right_motor_current_msg);
                    right_velocity_pub->publish(right_motor_velocity_msg);
                }
                else if (buffer.substr(0,3) == lkey) { // LEFT MOTOR
                    ss >> left_motor.decaamps >> left_motor.encoder;
                    if (ss.fail()) { // Invalid message
                        RCLCPP_WARN(this->get_logger(),"Left motor channel: unexpected data");
                        return;
                    }
                    left_motor.velocity = calculate_wheel_angular_velocity(left_motor.encoder);
                    left_motor.current  = left_motor.decaamps/10.0f;
                    left_motor_current_msg.data = left_motor.current;
                    left_motor_velocity_msg.data = left_motor.velocity;
                    left_current_pub->publish(left_motor_current_msg);
                    left_velocity_pub->publish(left_motor_velocity_msg);
                }
                else if (buffer.substr(0,3) == bkey) { // BATTERY
                    int32_t battery_decavolts;
                    int32_t controller_temp;
                    ss >> battery_decavolts >> controller_temp;
                    if (ss.fail()) {
                        // Invalid message
                        RCLCPP_WARN(this->get_logger(),"Battery channel: unexpected data");
                        return;
                    }
                    battery_msg.header.stamp = this->get_clock()->now();
                    battery_msg.temperature = controller_temp;
                    battery_msg.current = -(fabs(left_motor.current) + fabs(right_motor.current));
                    battery_msg.voltage = battery_decavolts/10.0;
                    battery_pub->publish(battery_msg);
                }
            } // eof valid header
            else return; // Return if reading failed
        } // eof bytes available
    }

  };
}


int main(int argc, char** argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<roboteqBridge::baseController>());
  rclcpp::shutdown();
  return 0;
};
