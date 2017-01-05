#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <sys/time.h>

#define FRAME_HEIGHT 360
#define FRAME_WIDTH  640

class ImageConverter {
  ros::NodeHandle nh_;
  image_transport::ImageTransport it_;
  image_transport::Subscriber image_sub_;
  image_transport::Publisher image_pub_;
  double distance;
  double delta_heading;
  long long time_micros;
  int count;

public:
  ImageConverter() : it_(nh_) {
    time_micros = 0;
    count = 0;
    // Subscrive to input video feed and publish output video feed
    image_sub_ = it_.subscribe("/camera/rgb/image_raw", 1, 
      &ImageConverter::imageCb, this);
    image_pub_ = it_.advertise("/image_converter/output_video", 1);
  }

  ~ImageConverter() {
  }

  void imageCb(const sensor_msgs::ImageConstPtr& msg) {
    cv_bridge::CvImagePtr cv_ptr;
    cv_bridge::CvImagePtr processed_ptr;

    count++;

    if (count % 100 == 0) {
      long long current_micros;
      struct timeval tv;

      gettimeofday(&tv, NULL);
      current_micros = (long long)tv.tv_sec * 1000000 + tv.tv_usec;
      printf("time: %f\n", (double)(current_micros  - time_micros) / 100);
      time_micros = current_micros;
    }

    // Use cv_bridge() to convert the ROS image to OpenCV format
    try {
      cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
    }
    catch (cv_bridge::Exception& e) {
      ROS_ERROR("cv_bridge exception: %s", e.what());
      return;
    }

    // Process the frame using the process_image() function
    processed_ptr = process_image(cv_ptr);
                       
    // Output modified video stream
    image_pub_.publish(processed_ptr->toImageMsg());
  }

  cv_bridge::CvImagePtr process_image(cv_bridge::CvImagePtr cv_ptr) {
    cv::Mat blured;
    cv::Mat hsv;

    cv::blur(cv_ptr->image, blured, cv::Size(3, 3));
    cv::cvtColor(blured, hsv, CV_RGB2HSV);

    // works kind of thresholded = cv2.inRange(hsv, cv.Scalar(0, 190, 125, 0), cv.Scalar(15, 255, 230, 0))
    //               thresholded2 = cv2.inRange(hsv, cv.Scalar(165, 190, 125, 0), cv.Scalar(180, 255, 230, 0))
    // thresholded = cv2.inRange(hsv, (0, 210, 125, 0), (10, 255, 255, 0))
    // thresholded2 = cv2.inRange(hsv, (170, 210, 125, 0), (180, 255, 255, 0))

    cv::Mat thresholded, thresholded2;

    cv::inRange(hsv, cv::Scalar(0, 210, 125, 0), cv::Scalar(25, 255, 255, 0), thresholded);
    cv::inRange(hsv, cv::Scalar(155, 210, 125, 0), cv::Scalar(180, 255, 255, 0), thresholded2);
    cv::bitwise_or(thresholded, thresholded2, thresholded);

    std::vector<std::vector<cv::Point> > contours;
    std::vector<cv::Vec4i> hierarchy;
    int max_index = -1;
    double a_max = -1.0;

    cv::findContours(thresholded, contours, hierarchy, CV_RETR_TREE, CV_CHAIN_APPROX_SIMPLE, cv::Point(0, 0));

    for (int i = 0; i < contours.size(); i++ ) {
      double a = cv::contourArea(cv::Mat(contours[i]));

      if (a > a_max) {
        a_max = a;
        max_index = i;
      }
    }

    if (max_index >= 0) {
      std::vector<cv::Point> cnt = contours[max_index];
      std::vector<cv::Point> poly;
      cv::Rect r;

      cv::approxPolyDP(cv::Mat(cnt), poly, 3, true);
      r = cv::boundingRect(cv::Mat(poly));

      if (r.width * r.height > 160 && r.width / r.height < 1.0) {
        std::vector<std::vector<cv::Point> > poly_vec;

        poly_vec.push_back(poly);
        distance += 0.24 * FRAME_HEIGHT / (0.3969 * r.height);
        delta_heading += (70.42 / FRAME_WIDTH) * ((r.x + r.width / 2) - (FRAME_WIDTH / 2));
        cv::drawContours(cv_ptr->image, poly_vec, 0, cv::Scalar(12, 236, 160), 1);
        cv::rectangle(cv_ptr->image, r.tl(), r.br(), cv::Scalar(12, 236, 160), 2);
      }
    }

    return cv_ptr;
  }
};

int main(int argc, char** argv) {
  ros::init(argc, argv, "image_converter");
  ImageConverter ic;
  ros::spin();
  return 0;
}
