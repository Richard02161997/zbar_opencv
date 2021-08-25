#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/core/core.hpp>
#include <opencv2/highgui/highgui.hpp>
#include <opencv2/imgproc/imgproc.hpp>
#include <zbar.h>
#include <iostream>
#include <iomanip>
#include "std_msgs/String.h"
#include <sstream>
#include <cstdio>
#include "ros/time.h"

using namespace std;
using namespace cv;
using namespace zbar;

bool forready = true;
//static const std::string OPENCV_WINDOW="Image window";

void zbarscanner(cv_bridge::CvImagePtr cv_ptr)
{
    ros::NodeHandle nh;
    ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("zbar_opencv_code", 1000);
    if(forready){
        ROS_INFO("zbar_opencv ready");
        forready = false;
    }
    // Create a zbar reader
    ImageScanner scanner;

    // Configure the reader
    scanner.set_config(ZBAR_NONE, ZBAR_CFG_ENABLE, 1);

    // Capture an OpenCV frame
    cv::Mat frame,frame_grayscale;
    frame=cv_ptr->image;
    // Convert to grayscale
    cvtColor(frame, frame_grayscale, CV_BGR2GRAY);

    // Obtain image data
    int width = frame_grayscale.cols;
    int height = frame_grayscale.rows;
    uchar *raw = (uchar *)(frame_grayscale.data);

    // Wrap image data
    Image image(width, height, "Y800", raw, width * height);

    // Scan the image for barcodes
    scanner.scan(image);

    // Extract results
    int counter = 0;
    for (Image::SymbolIterator symbol = image.symbol_begin(); symbol != image.symbol_end(); ++symbol) {
	  	std_msgs::String msg;  // 建立暫存的message，先將資料存入此變數，再進行publish
	        std::stringstream ss;
	        ss << symbol->get_data();
	        msg.data = ss.str();   // 寫入msg message中的data欄位 
	  
	        ROS_INFO("%s", msg.data.c_str());   
            chatter_pub.publish(msg);  //使用前面建立的Publisher物件chatter_pub的publish()方法，將暫存的msg送出 (publish)
            //Delay
            //ROS_INFO("Take a break");   
            //sleep(10);
            //ROS_INFO("Lets Go");

            ros::spinOnce();   // 呼叫一次 callback function，在subscriber才有用
            
            // Draw location of the symbols found
            if (symbol->get_location_size() == 4) {

                line(frame, Point(symbol->get_location_x(0), symbol->get_location_y(0)), Point(symbol->get_location_x(1), symbol->get_location_y(1)), Scalar(0, 255, 0), 2, 8, 0);
                line(frame, Point(symbol->get_location_x(1), symbol->get_location_y(1)), Point(symbol->get_location_x(2), symbol->get_location_y(2)), Scalar(0, 255, 0), 2, 8, 0);
                line(frame, Point(symbol->get_location_x(2), symbol->get_location_y(2)), Point(symbol->get_location_x(3), symbol->get_location_y(3)), Scalar(0, 255, 0), 2, 8, 0);
                line(frame, Point(symbol->get_location_x(3), symbol->get_location_y(3)), Point(symbol->get_location_x(0), symbol->get_location_y(0)), Scalar(0, 255, 0), 2, 8, 0);
            }
            counter++;
    }
}


class ImageConverter
{
    ros::NodeHandle nh;
    //ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("zbar_opencv", 1000);

    image_transport::ImageTransport it;
    image_transport::Subscriber image_sub;
    image_transport::Publisher image_pub;

  public:
    ImageConverter():it(nh)
    {
        //使用image_transport订阅图像话题“in” 和 发布图像话题“out”
        image_sub=it.subscribe("/usb_cam/image_raw",1,&ImageConverter::imageCb,this);
        image_pub=it.advertise("zbar_opencv",1);
        //ROS_INFO("ready2");

    }
    ~ImageConverter(){}

    //订阅回调函数
    void imageCb(const sensor_msgs::ImageConstPtr& msg)
    {
        cv_bridge::CvImagePtr cv_ptr;
        try
        {
            //将ROS图像消息转化为适合Opencv的CvImage
            cv_ptr=cv_bridge::toCvCopy(msg,sensor_msgs::image_encodings::BGR8);
        }
        catch(cv_bridge::Exception& e)
        {
            ROS_ERROR("cv_bridge exception: %s",e.what());
            return;
        }

        zbarscanner(cv_ptr);
        image_pub.publish(cv_ptr->toImageMsg());
    }
};


int main(int argc, char **argv) {
    ros::init(argc,argv,"zbar_opencv");
    //ros::NodeHandle nh;
    //ros::Publisher chatter_pub = nh.advertise<std_msgs::String>("qr_code", 1000);
    ImageConverter ic;
    ros::spin();
    return 0;
}
