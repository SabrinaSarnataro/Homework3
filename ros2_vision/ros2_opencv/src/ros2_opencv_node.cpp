#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/image.hpp"
#include "std_msgs/msg/header.hpp"
#include <chrono>
#include <cv_bridge/cv_bridge.h> // cv_bridge converts between ROS 2 image messages and OpenCV image representations.
#include <image_transport/image_transport.hpp> // Using image_transport allows us to publish and subscribe to compressed image streams in ROS2
#include <opencv2/opencv.hpp> // We include everything about OpenCV as we don't care much about compilation time at the moment.

#include <iostream>
 
using namespace std::chrono_literals;
using namespace cv;
 
class MinimalImagePublisherSubscriber : public rclcpp::Node {
public:
  MinimalImagePublisherSubscriber() : Node("opencv_image_publisher") {
  
    // Pubblicazione sul nuovo topic "/videocamera_processata"
    publisher_ = this->create_publisher<sensor_msgs::msg::Image>("/videocamera_processata", 10);

  
    // Sottoscrizione al topic "/videocamera"
    subscription_ = this->create_subscription<sensor_msgs::msg::Image>("/videocamera", 10,
        std::bind(&MinimalImagePublisherSubscriber::image_callback, this, std::placeholders::_1));

    
    
  
  }
 
private:
   void image_callback(const sensor_msgs::msg::Image::SharedPtr msg) {
    try {
    
      // Conversione dell'immagine ROS in formato OpenCV
      Mat input_image = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8)->image;
      

      Mat grey_image ;
      
      cvtColor(input_image, grey_image, cv::COLOR_BGR2GRAY);
      
      
       // Setup SimpleBlobDetector parameters.
      SimpleBlobDetector::Params params;
      
      // Change thresholds
      params.minThreshold = 10;
      params.maxThreshold = 200;
      
      // Filter by Area.
      params.filterByArea = true;
      params.minArea = 0;
      params.maxArea = 8000000;
 
      // Filter by Circularity
      params.filterByCircularity = true;
      params.minCircularity = 0.8;
      
      // Filter by Convexity
      params.filterByConvexity = true;
      params.minConvexity = 0.9;
      
      // Filter by Inertia
      params.filterByInertia = true;
      params.minInertiaRatio = 0.7;
      
      // Set up detector with params
      Ptr<SimpleBlobDetector> detector = SimpleBlobDetector::create(params);
   
      //Detect blobs
      std::vector<KeyPoint> keypoints;  
      detector->detect(grey_image, keypoints);
      
      //Draw detected blobs as circles
      // DrawMatchesFlags::DRAW_RICH_KEYPOINTS flag ensures the size of the circle corresponds to the size of blob
      Mat im_with_keypoints;  //Mat è una matrice
      drawKeypoints(input_image, keypoints, im_with_keypoints , Scalar(0,0,255), DrawMatchesFlags::DRAW_RICH_KEYPOINTS );
      
      /* DEBUG
      std::cout << "keypoint size " << keypoints.size() << std::endl;   
      if(keypoints.size()){
      	std::cout << "Circle detected" << std::endl;
      }*/
                
      // Conversione dell'immagine processata in formato ROS
      auto processed_msg = cv_bridge::CvImage(msg->header, sensor_msgs::image_encodings::BGR8,  im_with_keypoints).toImageMsg();

      // Pubblicazione dell'immagine processata su "/videocamera_processata"
      publisher_->publish(*processed_msg);
      RCLCPP_INFO(this->get_logger(), "Immagine processata pubblicata su /videocamera_processata " );
    } 
    catch (cv_bridge::Exception &e) {
      RCLCPP_ERROR(this->get_logger(), "Eccezione cv_bridge: %s", e.what());
    }
  }

  rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
  rclcpp::Publisher<sensor_msgs::msg::Image>::SharedPtr publisher_;
};
 
 
 
 
int main(int argc, char *argv[]) {
  rclcpp::init(argc, argv);
  // create a ros2 node
  auto node = std::make_shared<MinimalImagePublisherSubscriber>();
 
  // process ros2 callbacks until receiving a SIGINT (ctrl-c)
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
  
  
}
