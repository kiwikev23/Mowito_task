#include <chrono>
#include <memory>
#include "rclcpp/rclcpp.hpp"
#include "mowito_task/srv/mode_select.hpp"

using namespace std::chrono_literals;

//Client node to test the service call and mode changing, Toggles between modes to show the change in output msg in the topic

class ModeSelectClient : public rclcpp::Node
{
public:
    ModeSelectClient() : Node("mode_select_client")
    {
        client_ = this->create_client<mowito_task::srv::ModeSelect>("/set_mode");

        while (!client_->wait_for_service(1s))
        {
            if (!rclcpp::ok())
            {
                RCLCPP_ERROR(this->get_logger(), "Interrupted while waiting for the service. Exiting.");
                return;
            }
            RCLCPP_INFO(this->get_logger(), "Waiting for service to be available...");
        }
    }

    void send_request(bool mode)
    {
        auto request = std::make_shared<mowito_task::srv::ModeSelect::Request>();
        request->mode = mode;

        auto response = std::make_shared<mowito_task::srv::ModeSelect::Response>();

        auto future = client_->async_send_request(request);

        if (future.wait_for(3s) == std::future_status::ready)
        {
            response = future.get();
            std::cout<<response->response<<std::endl;
        }
        else
        {
            std::cout<<response->response<<std::endl;
        }
    }

private:
    rclcpp::Client<mowito_task::srv::ModeSelect>::SharedPtr client_;
};

int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto client_node = std::make_shared<ModeSelectClient>();

    bool current_mode = true; 

    rclcpp::Rate loop_rate(2);
    while (rclcpp::ok())
    {
        RCLCPP_INFO(client_node->get_logger(), "Sending mode: %s", current_mode ? "true" : "false");
        client_node->send_request(current_mode);

      
        current_mode = !current_mode;

       
        loop_rate.sleep();
    }

    rclcpp::shutdown();
    return 0;
}
