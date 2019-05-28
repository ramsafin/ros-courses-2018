#pragma once

#include <string>
#include <functional>

#include <ros/ros.h>
#include <image_transport/image_transport.h>

class ImageSubscriber final {
  public:
    using Callback = std::function<void(const sensor_msgs::ImageConstPtr&)>;

    ImageSubscriber(ros::NodeHandle& nh, const std::string& topic, const Callback& handler): img_transport_{nh} {
      img_subscriber_ = img_transport_.subscribe(topic, 1, handler);
    }

    virtual ~ImageSubscriber() = default;
  
  private:
    image_transport::ImageTransport img_transport_;
    image_transport::Subscriber img_subscriber_;
};
