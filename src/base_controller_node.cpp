#include <unistd.h>
#include <iostream>
#include <sstream>
#include <signal.h>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp/node.hpp>
#include <serial/serial.h>
#include <geometry_msgs/msg/twist.hpp>

using namespace std;

#define TWO_PI (2*M_PI)
#define TO_RAD_PER_SEC ((2*M_PI)/60)

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

      twist_sub = this->create_subscription<geometry_msgs::msg::Twist>("cmd_vel", 1, std::bind(&baseController::twist_callback, this, std::placeholders::_1));

      RCLCPP_INFO_STREAM(this->get_logger(), "Tyring to open serial port " << param_port_name << "..." );
      try {
          serial_port.open();
          if ( serial_port.isOpen() )
          {
              RCLCPP_INFO(this->get_logger(), "Successfully opened serial port.");
          }
      }
      catch (serial::IOException e) {
        RCLCPP_ERROR(this->get_logger(),"Failed to open serial port");
        try {
          rclcpp::shutdown();
        }
        catch (rclcpp::exceptions::RCLError e){}
      }

      this->reset_controller();
      this->disable_echo();
      this->query_motor_feedback_stream();
      this->query_battery_state_stream();
      this->serial_port.flush();

      while (rclcpp::ok()) {
        read_feedback_stream();
      }
    }

  private:

    size_t  count_;
    serial::Serial serial_port;
    std::string param_port_name = "/dev/roboteq";
    uint16_t param_port_timeout = 500; // ms
    long   param_baud_rate = 115200;
    double param_gear_ratio = 55.0;
    double param_wheel_radius = 0.167;
    double param_wheel_separation = 0.560;
    bool   param_channel_order_RL = true;

    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr twist_sub;

    void twist_callback(const geometry_msgs::msg::Twist &twist){
      double left_track_linear_velocity  = (2.0f * twist.linear.x - param_wheel_separation * twist.angular.z)/2.0f;
      double right_track_linear_velocity = (2.0f * twist.linear.x + param_wheel_separation * twist.angular.z)/2.0f;
      send_motor_rpm_left( linear_velocity_to_rpm(left_track_linear_velocity));
      send_motor_rpm_right(linear_velocity_to_rpm(right_track_linear_velocity));
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
      // disable echo
      serial_port.write("^ECHOF 1\r");
      serial_port.flush();
      RCLCPP_INFO(this->get_logger(), "Command echo off.");
    }

    void set_watchdog(uint16_t watchdog_timeout)
    {
      // set watchdog timeout
      std::stringstream wdt_cmd;
      wdt_cmd << "^RWD " << watchdog_timeout << "\r";
      serial_port.write(wdt_cmd.str());
      RCLCPP_INFO(this->get_logger(), "Watchdog configured.");
    }

    void set_current_limit(uint16_t motor_amp_limit){
      // set motor amps limit (A * 10)
      std::stringstream alim_cmd1;
      alim_cmd1 << "^ALIM 1 " << motor_amp_limit*10 << "\r";
      serial_port.write(alim_cmd1.str());
      std::stringstream alim_cmd2;
      alim_cmd2 << "^ALIM 2 " << motor_amp_limit*10 << "\r";
      serial_port.write(alim_cmd2.str());
      RCLCPP_INFO(this->get_logger(), "Maximum motor current set: %f Amps", motor_amp_limit*10.0f);
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
      RCLCPP_INFO(this->get_logger(), "Maximum motor speed set: %d RPM", motor_max_speed);
    }

    void set_acceleration_rate(uint16_t motor_max_acceleration){
      // set max motor acceleration rate (rpm/s * 10)
      std::stringstream mac_cmd1;
      mac_cmd1 << "^MAC 1 " << motor_max_acceleration*10 << "\r";
      serial_port.write(mac_cmd1.str());
      std::stringstream mac_cmd2;
      mac_cmd2 << "^MAC 2 " << motor_max_acceleration*10 << "\r";
      serial_port.write(mac_cmd2.str());
      RCLCPP_INFO(this->get_logger(), "Maximum motor acceleration set: %d RPM/sec", int32_t(motor_max_acceleration));
    }

    void set_deceleration_rate(uint16_t motor_max_deceleration){
      // set max motor deceleration rate (rpm/s * 10)
      std::stringstream mdec_cmd1;
      mdec_cmd1 << "^MDEC 1 " << motor_max_deceleration*10 << "\r";
      serial_port.write(mdec_cmd1.str());
      std::stringstream mdec_cmd2;
      mdec_cmd2 << "^MDEC 2 " << motor_max_deceleration*10 << "\r";
      serial_port.write(mdec_cmd2.str());
      RCLCPP_INFO(this->get_logger(), "Maximum motor deceleration set: %d RPM/sec", int32_t(motor_max_deceleration));
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
          RCLCPP_INFO(this->get_logger(), "Open loop control.");
        break;
        case 1:
          // set motor operating mode to closed-loop speed
          serial_port.write("^MMOD 1 1\r");
          serial_port.write("^MMOD 2 1\r");
          // set encoder mode (18 for feedback on motor1, 34 for feedback on motor2)
          serial_port.write("^EMOD 1 18\r");
          serial_port.write("^EMOD 2 34\r");
          RCLCPP_INFO(this->get_logger(), "Closed loop speed control.");
        break;
        default:
          RCLCPP_INFO(this->get_logger(), "This mode is not implemented.");
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
      RCLCPP_INFO(this->get_logger(), "Encoder resolution set.");
    }

    void clear_buffer_history()
    {
      // clear buffer history
      std::stringstream stream_command;
      stream_command << "# C\r";
      serial_port.write(stream_command.str());
    }

    void query_motor_feedback_stream(){
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
      RCLCPP_INFO(this->get_logger(), "Feedback streams queried.");
    }

    void query_battery_state_stream(){
      std::stringstream stream_command;
      stream_command.str(std::string());
      stream_command << R"(/"BA="," "?v 2_?t 1_# 100)" << "\r";
      serial_port.write(stream_command.str());
      RCLCPP_INFO(this->get_logger(), "Battery state stream queried.");
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
                    int32_t current_decaamp_right;
                    int32_t odom_encoder_right;
                    ss >> current_decaamp_right >> odom_encoder_right;
                    if (ss.fail()) {
                        // Invalid message
                        RCLCPP_WARN(this->get_logger(),"Right motor channel: unexpected data");
                        return;
                    }
                }
                else if (buffer.substr(0,3) == lkey) { // LEFT MOTOR
                    int32_t odom_encoder_left;
                    int32_t current_decaamp_left;
                    ss >> current_decaamp_left >> odom_encoder_left;
                    if (ss.fail()) {
                        // Invalid message
                        RCLCPP_WARN(this->get_logger(),"Left motor channel: unexpected data");
                        return;
                    }
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
                }
            } // eof valid header
            else return; // Return if reading failed
        } // eof bytes available
    }

    double map(double x, double in_min, double in_max, double out_min, double out_max) {
      return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
    };

    int32_t linear_velocity_to_rpm(double lin_vel)
    {
        return int32_t( (60.0f * lin_vel) / (param_wheel_radius * TWO_PI * param_gear_ratio) );
    }

    int32_t angular_velocity_to_rpm(double ang_vel)
    {
        return int32_t((ang_vel / TWO_PI) * 60.0 * param_gear_ratio );
    }

    int32_t norm_to_power_cmd(double norm)
    {
      return int32_t(map(norm, -1.0, 1.0, -2000, 2000));
    }

    double calculate_wheel_angular_velocity(int32_t motor_rpm)
    {
        return ( ( (double)motor_rpm * TO_RAD_PER_SEC ) / param_gear_ratio );
    }
  };
}


int main(int argc, char** argv){
  rclcpp::init(argc,argv);
  rclcpp::spin(std::make_shared<roboteqBridge::baseController>());
  rclcpp::shutdown();
  return 0;
};
