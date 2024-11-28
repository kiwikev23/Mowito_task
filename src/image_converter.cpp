#include <memory>
#include "sensor_msgs/msg/image.hpp"
#include "rclcpp/rclcpp.hpp"
#include <cv_bridge/cv_bridge.h>
#include <opencv2/opencv.hpp>
#include "mowito_task/srv/mode_select.hpp"


class ImageConverter : public rclcpp::Node 
{
public: 
    ImageConverter() : Node("image_conversion")
    {

        //parameter declaration
        this->declare_parameter<std::string>("input_topic", "/image_raw");
        this->declare_parameter<std::string>("output_topic", "/converted_image");

        auto input_topic = this->get_parameter("input_topic").as_string();
        auto output_topic = this->get_parameter("output_topic").as_string();

        //publish the changed video stream
        publisher_ = this->create_publisher<sensor_msgs::msg::Image>(output_topic, 10);
        
        //subscribe from the usb_cam topic
        subscriber_ = this->create_subscription<sensor_msgs::msg::Image>(
            input_topic, 
            10, 
            std::bind(&ImageConverter::callback, this, std::placeholders::_1)
        );

        //intialise the service to change modes
        service_ = this->create_service<mowito_task::srv::ModeSelect>("/set_mode",std::bind(&ImageConverter::setting_mode, this, std::placeholders::_1, std::placeholders::_2));

    }


private:
    rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscriber_;
    rclcpp::Service<mowito_task::srv::ModeSelect>::SharedPtr service_;

    
    //boolean intialised the mode
    bool mode;

    //to track if the mode is changed
    bool change = true;

    void callback(const sensor_msgs::msg::Image::SharedPtr msg)
    {
        cv_bridge::CvImage mode_image;

        try
        {
            //cv bridge for working with opencv functions
            cv_bridge::CvImagePtr cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);

            //Mode 1
            if (mode)
            {
                if (!change){
                    cv::destroyAllWindows();
                }

                cv::Mat gray_image;

                //converting image to grayscale
                cv::cvtColor(cv_ptr->image, gray_image, cv::COLOR_BGR2GRAY);
                
                mode_image.header = msg->header;
                mode_image.encoding = sensor_msgs::image_encodings::MONO8;
                mode_image.image = gray_image;

                //publish back from cv_image to a ros image
                publisher_->publish(*mode_image.toImageMsg());
                std::cout<<"Mode 1"<<std::endl;

                cv::imshow("Mode 1",gray_image);
                cv::waitKey(1);
                change = true;

            
            }
            
            //Mode 2
            else if (!mode)
            {
                if (change){
                    cv::destroyAllWindows();
                }

                mode_image.header = msg->header;
                mode_image.encoding = sensor_msgs::image_encodings::BGR8;
                mode_image.image = cv_ptr->image;

                //publish back from cv_image to a ros image
                publisher_->publish(*mode_image.toImageMsg());
                std::cout<<"Mode 2"<<std::endl;


                cv::imshow("Mode 2",cv_ptr->image);
                cv::waitKey(1);
                change = false;

            }
        }

        
        catch (cv_bridge::Exception &e)
        {
            std::cout<<"Error"<<std::endl;
        }

    }

    //service callback
    void setting_mode(const std::shared_ptr<mowito_task::srv::ModeSelect::Request> request, std::shared_ptr<mowito_task::srv::ModeSelect::Response> response)
    {
        mode = request->mode;
        response->response = "Mode is changed to " + std::string(mode ? "true" : "false");
        std::cout<<"Mode changed"<<std::endl;

    } 


};


int main(int argc, char *argv[])
{
    rclcpp::init(argc, argv);
    auto node = std::make_shared<ImageConverter>();
    rclcpp::spin(node);
    rclcpp::shutdown();
    return 0;
}