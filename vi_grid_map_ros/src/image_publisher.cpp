// ROS
#include <ros/ros.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

// OpenCV
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <sys/stat.h>
#include "unistd.h"
#include <string.h>

class ImagePublisher
{
    //ros::NodeHandle nh_;
    ros::NodeHandle pnh_;
    ros::Publisher image_pub_;

    std::string imageTopic_;
    std::string imagePath_;

public:
    ImagePublisher()
        :pnh_("~")
    {
        pnh_.getParam("image_path", imagePath_);
        pnh_.getParam("image_topic", imageTopic_);
        image_pub_ = pnh_.advertise<sensor_msgs::Image>(imageTopic_, 10);

        ros::Rate loop_rate(1);
        
        //ファイル存在チェック
        struct stat st;
        const char* file = imagePath_.c_str();

        //error回避
        sleep(10);
        
        while(ros::ok()){
            int ret = stat(file, &st);
            if (0 == ret){
                cv::Mat	image;
                image = cv::imread(imagePath_);

                std::vector<int> param = std::vector<int>(2);
                param[0] = cv::IMWRITE_PNG_COMPRESSION;
                param[1] = 9;//default(3)  0-9　最高設定

                std::vector<uchar>buf;
                cv::imencode(".png", image, buf, param);
                cv::Mat im_decode = cv::imdecode(cv::Mat(buf), cv::IMREAD_COLOR);

                cv::Mat source = im_decode;
                cv::Mat alpha_image = cv::Mat(source.size(), CV_8UC3);
                cv::cvtColor(source, alpha_image, CV_RGB2RGBA);
                
                for (int y = 0; y < alpha_image.rows; ++y) {
                    for (int x = 0; x < alpha_image.cols; ++x) {
                        cv::Vec4b px = alpha_image.at<cv::Vec4b>(x, y);
                        if (px[0] + px[1] + px[2] == 0) {
                            px[3] = 0;
                            alpha_image.at<cv::Vec4b>(x, y) = px;
                        }
                    }
                }
                
                std_msgs::Header header; // empty header
                header.frame_id = "map_grid";
                header.stamp = ros::Time::now();
                
                cv_bridge::CvImage img_bridge;
                img_bridge = cv_bridge::CvImage(header, "rgba8", alpha_image);
                
                sensor_msgs::Image msg;
                img_bridge.toImageMsg(msg);
                image_pub_.publish(msg);
            }   else    {    
                std::string ros_err = imagePath_ + " Not Exist!";
                ROS_ERROR("%s", ros_err.c_str());
            }

            ros::spinOnce();
            loop_rate.sleep();
        }
    }
    ~ImagePublisher(){}
};

int main(int argc, char** argv)
{
    ros::init(argc, argv, "image_publisher");
    ImagePublisher ip;

    return 0;
}
