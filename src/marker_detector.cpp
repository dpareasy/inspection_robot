#include <iostream>
#include <aruco/aruco.h>
#include <aruco/cvdrawingutils.h>
#include <algorithm>
#include <set>
#include <inspection_robot/MarkerList.h>
#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>



class ArucoMarkerPublisher
{
private:
  // ArUco stuff
  aruco::MarkerDetector mDetector_;
  std::vector<aruco::Marker> markers_;
  aruco::CameraParameters camParam_;

  // define the number of markers to be detected
  std::vector<int> detected_markers;
  inspection_robot::MarkerList markers_msg;

  // node params
  double marker_size_;
  bool useCamInfo_;

  // ROS pub-sub
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;

  ros::Publisher marker_publisher;

  image_transport::Publisher image_pub_;
  image_transport::Publisher debug_pub_;

  cv::Mat inImage_;

public:
  ArucoMarkerPublisher() : nh_("~"), it_(nh_), useCamInfo_(true)
  {
    image_sub_ = it_.subscribe("/camera1/image_raw", 1, &ArucoMarkerPublisher::image_callback, this);
    image_pub_ = it_.advertise("result", 1);
    debug_pub_ = it_.advertise("debug", 1);
    marker_publisher = nh_.advertise<inspection_robot::MarkerList>("MarkerList", 10);
    nh_.param<bool>("use_camera_info", useCamInfo_, false);
    camParam_ = aruco::CameraParameters();
  }

  void image_callback(const sensor_msgs::ImageConstPtr &msg)
  {
    bool publishImage = image_pub_.getNumSubscribers() > 0;
    bool publishDebug = debug_pub_.getNumSubscribers() > 0;

    ros::Time curr_stamp = msg->header.stamp;
    cv_bridge::CvImagePtr cv_ptr;

    try
    {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
      inImage_ = cv_ptr->image;

      // clear out previous detection results
      markers_.clear();

      // ok, let's detect
      mDetector_.detect(inImage_, markers_, camParam_, marker_size_, false);

      for (std::size_t i = 0; i < markers_.size(); ++i)
      {
        if (std::find(detected_markers.begin(), detected_markers.end(), markers_[i].id) == detected_markers.end())
        {
          detected_markers.push_back(markers_[i].id);
          markers_msg.markers = markers_[i].id;
          std::cout << "Marker " << markers_[i].id << " have been detected" << std::endl;
          marker_publisher.publish(markers_msg);
        }
        else
        {
          break;
        }
      }

      // draw detected markers on the image for visualization
      for (std::size_t i = 0; i < markers_.size(); ++i)
      {
        markers_[i].draw(inImage_, cv::Scalar(0, 0, 255), 2);
      }
      // publish input image with markers drawn on it
      if (publishImage)
      {
        // show input with augmented information
        cv_bridge::CvImage out_msg;
        out_msg.header.stamp = curr_stamp;
        out_msg.encoding = sensor_msgs::image_encodings::RGB8;
        out_msg.image = inImage_;
        image_pub_.publish(out_msg.toImageMsg());
      }

      // publish image after internal image processing
      if (publishDebug)
      {
        // show also the internal image resulting from the threshold operation
        cv_bridge::CvImage debug_msg;
        debug_msg.header.stamp = curr_stamp;
        debug_msg.encoding = sensor_msgs::image_encodings::MONO8;
        debug_msg.image = mDetector_.getThresholdedImage();
        debug_pub_.publish(debug_msg.toImageMsg());
      }
    }
    catch (cv_bridge::Exception &e)
    {
      ROS_ERROR("cv_bridge exception: %s", e.what());
    }
  }
};

int main(int argc, char **argv)
{
  ros::init(argc, argv, "aruco_marker_publisher");
  

  ArucoMarkerPublisher node;

  ros::spin();
}