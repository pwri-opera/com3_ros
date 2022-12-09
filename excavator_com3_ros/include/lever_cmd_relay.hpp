#pragma once

#include <iostream>
#include <rclcpp/rclcpp.hpp>
#include <memory>
#include "com3_msgs/msg/excavator_com3_lever1.hpp"
#include "com3_msgs/msg/excavator_com3_lever2.hpp"

#define initial_interval 100
namespace excavator_com3_can
{
    class lever_cmd_relay : public excavator_com3_dbc
    {
    public:
        lever_cmd_relay(boost::asio::io_context &io, std::string can_port, std::string dbc_path)
            : excavator_com3_dbc(dbc_path),
              send_timer(io, boost::asio::chrono::milliseconds(initial_interval)),
              send_timer1(io, boost::asio::chrono::milliseconds(initial_interval)),
              sock(io)
        {
            node_ = rclcpp::Node::make_shared("lever_cmd_relay");

            const auto idx = canary::get_interface_index(can_port);
            auto const ep = canary::raw::endpoint{idx};
            sock.open();
            sock.bind(ep);

            lever_cmd_1 = boost::make_shared<excavator_com3::lever_cmd_1>();
            lever_cmd_2 = boost::make_shared<excavator_com3::lever_cmd_2>();

            alive_cnt = 0;
            start_receive();
            send_timer.async_wait(boost::bind(&lever_cmd_relay::send_lever_cmd_1, this));
            send_timer1.async_wait(boost::bind(&lever_cmd_relay::send_lever_cmd_2, this));
            sub_js_cmd_ = node_->create_subscription<com3_msgs::msg::ExcavatorCom3Lever1>("lever1", 10,
                                                                                          [this](const com3_msgs::msg::ExcavatorCom3Lever1::SharedPtr msg)
                                                                                          { this->lever1_cmd_callback(msg); });
            sub_travel_cmd_ = node_->create_subscription<com3_msgs::msg::ExcavatorCom3Lever2>("lever2", 10,
                                                                                              [this](const com3_msgs::msg::ExcavatorCom3Lever2::SharedPtr msg)
                                                                                              { this->lever2_cmd_callback(msg); });
            rclcpp::spin(node_);
        };
        ~lever_cmd_relay()
        {
            send_timer.cancel();
            send_timer1.cancel();
        }

    private:
        void send_lever_cmd_1()
        {
            frame f = {};
            excavator_com3_dbc::encode(*lever_cmd_1, f);

            sock.async_send(canary::net::buffer(&f, sizeof(f)), boost::bind(&lever_cmd_relay::send_handle, this));
            send_timer.expires_at(send_timer.expiry() + boost::asio::chrono::milliseconds(lever_cmd_1->Cycle_time()));
            send_timer.async_wait(boost::bind(&lever_cmd_relay::send_lever_cmd_1, this));
        }

        void send_lever_cmd_2()
        {
            frame f = {};
            excavator_com3_dbc::encode(*lever_cmd_2, f);

            sock.async_send(canary::net::buffer(&f, sizeof(f)), boost::bind(&lever_cmd_relay::send_handle, this));
            send_timer1.expires_at(send_timer1.expiry() + boost::asio::chrono::milliseconds(lever_cmd_2->Cycle_time()));
            send_timer1.async_wait(boost::bind(&lever_cmd_relay::send_lever_cmd_2, this));
        }

        void send_handle() {}

        void start_receive()
        {
            sock.async_receive(canary::net::buffer(&recv_f, sizeof(recv_f)), boost::bind(&lever_cmd_relay::handle_receive, this, boost::asio::placeholders::error, boost::asio::placeholders::bytes_transferred));

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

        void lever1_cmd_callback(const com3_msgs::msg::ExcavatorCom3Lever1::SharedPtr &msg)
        {
            lever_cmd_1->swing_cmd = std::abs(msg->swing_cmd);
            lever_cmd_1->swing_status_cmd = 0;
            lever_cmd_1->swing_direction_cmd = std::signbit(msg->swing_cmd);

            lever_cmd_1->boom_cmd = std::abs(msg->boom_cmd);
            lever_cmd_1->boom_status_cmd = 0;
            lever_cmd_1->boom_direction_cmd = std::signbit(msg->boom_cmd);

            lever_cmd_1->arm_cmd = std::abs(msg->arm_cmd);
            lever_cmd_1->arm_status_cmd = 0;
            lever_cmd_1->arm_direction_cmd = std::signbit(msg->arm_cmd);

            lever_cmd_1->bucket_cmd = std::abs(msg->bucket_cmd);
            lever_cmd_1->bucket_status_cmd = 0;
            lever_cmd_1->bucket_direction_cmd = std::signbit(msg->bucket_cmd);

            // last_cmd_time = ros::Time::now();
            last_cmd_time = node_->get_clock()->now();
        }
        void lever2_cmd_callback(const com3_msgs::msg::ExcavatorCom3Lever2::SharedPtr &msg)
        {
            lever_cmd_2->left_track_cmd = std::abs(msg->left_track_cmd);
            lever_cmd_2->left_track_status_cmd = 0;
            lever_cmd_2->left_track_direction_cmd = std::signbit(msg->left_track_cmd);
            lever_cmd_2->right_track_cmd = std::abs(msg->right_track_cmd);
            lever_cmd_2->right_track_status_cmd = 0;
            lever_cmd_2->right_track_direction_cmd = std::signbit(msg->right_track_cmd);

            // last_cmd_time = ros::Time::now();
            last_cmd_time = node_->get_clock()->now();
        }

        boost::asio::steady_timer send_timer, send_timer1;
        canary::raw::socket sock;
        frame recv_f;
        boost::shared_ptr<excavator_com3::lever_cmd_1> lever_cmd_1;
        boost::shared_ptr<excavator_com3::lever_cmd_2> lever_cmd_2;

        // ros::NodeHandle nh;
        // ros::Subscriber sub_js_cmd, sub_travel_cmd;
        // ros::Time last_cmd_time;
        rclcpp::Node::SharedPtr node_;
        rclcpp::Subscription<com3_msgs::msg::ExcavatorCom3Lever1>::SharedPtr sub_js_cmd_;
        rclcpp::Subscription<com3_msgs::msg::ExcavatorCom3Lever2>::SharedPtr sub_travel_cmd_;
        rclcpp::Time last_cmd_time;

        // std::string com3_can_port;
        // std::string dbc_path;
        std::uint8_t alive_cnt;
    };
}