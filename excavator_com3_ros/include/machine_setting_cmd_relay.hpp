#pragma once
#include <iostream>
#include <memory>
#include <rclcpp/rclcpp.hpp>
#include "com3_msgs/msg/excavator_com3_machine_setting.hpp"

#include <com3/excavator_com3_dbc.hpp>

#define initial_interval 100
#define cmd_timeout 500

namespace excavator_com3_can
{
    class machine_setting_cmd_relay : public excavator_com3_dbc, public rclcpp::Node
    {
    public:
        machine_setting_cmd_relay(boost::asio::io_context &io, std::string can_port, std::string dbc_path)
            : excavator_com3_dbc(dbc_path), rclcpp::Node("machine_setting_cmd_relay"),
              sock(io), send_timer(io, boost::asio::chrono::milliseconds(initial_interval))
        {
          this->declare_parameter("can_port", "can");
          this->declare_parameter("dbc_path", "excavator_com3.dbc");

          const auto idx = canary::get_interface_index(can_port);
          auto const ep = canary::raw::endpoint{idx};
          sock.open();
          sock.bind(ep);

          setting_cmd = std::make_shared<excavator_com3::machine_setting_cmd>();

          alive_cnt = 0;
          start_receive();
          send_timer.async_wait(boost::bind(&machine_setting_cmd_relay::send_machine_setting_cmd, this));

          sub_js_cmd = this->create_subscription<com3_msgs::msg::ExcavatorCom3MachineSetting>(
              "machine_setting", 10, [this](const com3_msgs::msg::ExcavatorCom3MachineSetting::SharedPtr msg)
              { this->machine_setting_cmd_callback(msg); });
        }

        ~machine_setting_cmd_relay()
        {
            send_timer.cancel();
        }

    private:
        void send_machine_setting_cmd()
        {
            frame f = {};
            setting_cmd->alive_counter++;

            excavator_com3_dbc::encode(*setting_cmd, f);

            sock.async_send(canary::net::buffer(&f, sizeof(f)), boost::bind(&machine_setting_cmd_relay::send_handle, this));
            send_timer.expires_at(send_timer.expiry() + boost::asio::chrono::milliseconds(setting_cmd->Cycle_time()));
            send_timer.async_wait(boost::bind(&machine_setting_cmd_relay::send_machine_setting_cmd, this));
        }

        void send_handle() {}

        void start_receive()
        {
            sock.async_receive(canary::net::buffer(&recv_f, sizeof(recv_f)), boost::bind(&machine_setting_cmd_relay::handle_receive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

            if (recv_f.header.payload_length())
            {
                excavator_com3_dbc::decode(recv_f);
            }
        }

        void handle_receive(const boost::system::error_code &error,
                            std::size_t /*bytes_transferred*/)
        {
            if (!error)
            {
                start_receive();
            }
        }

        void machine_setting_cmd_callback(const com3_msgs::msg::ExcavatorCom3MachineSetting::SharedPtr &msg)
        {
            setting_cmd->engine_rpm = (uint8_t)msg->engine_rpm;
            setting_cmd->engine_onoff = msg->engine_off;
            setting_cmd->hydraulic_onoff = msg->hydraulic_off;
            setting_cmd->power_eco_mode = msg->power_eco_mode;
            setting_cmd->travel_mode = msg->travel_mode;
            last_cmd_time = this->get_clock()->now();
        }

        canary::raw::socket sock;
        boost::asio::steady_timer send_timer;
        frame recv_f;
        std::shared_ptr<excavator_com3::machine_setting_cmd> setting_cmd;
        rclcpp::Subscription<com3_msgs::msg::ExcavatorCom3MachineSetting>::SharedPtr sub_js_cmd;
        rclcpp::Time last_cmd_time;
        std::uint8_t alive_cnt;
    };
}