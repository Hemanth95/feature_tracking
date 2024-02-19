#include <opencv2/opencv.hpp>
#include "sensor_msgs/msg/image.hpp"
#include <cv_bridge/cv_bridge.h>
#include <rclcpp/rclcpp.hpp>

class ImageProcessor : public rclcpp::Node
{
public:
    ImageProcessor() : Node("image_processor")
    {
        subscription_ = this->create_subscription<sensor_msgs::msg::Image>(
            "camera/image_raw", 10, std::bind(&ImageProcessor::imageCallback, this, std::placeholders::_1));
    }

    std::pair<std::vector<cv::KeyPoint>, cv::Mat> getOrbFeatures(cv::Mat img){
        cv::Ptr<cv::ORB> orb = cv::ORB::create(500, 1.2f, 8, 31, 0, 2, cv::ORB::HARRIS_SCORE, 31, 20);
        std::vector<cv::KeyPoint> keypoints;
        cv::Mat descriptors;
        orb->detectAndCompute(img, cv::noArray(), keypoints, descriptors);
        return {keypoints, descriptors};
    }

    void matchHammingFeatures(cv::Mat img1, cv::Mat img2){
        auto [keypoints1, descriptors1] = getOrbFeatures(img1);
        auto [keypoints2, descriptors2] = getOrbFeatures(img2);
        cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
        std::vector<cv::DMatch> matches;
        matcher->match(descriptors1, descriptors2, matches);
        cv::Mat img_matches;
        cv::drawMatches(img1, keypoints1, img2, keypoints2, matches, img_matches);
        cv::imshow("ORB Matches", img_matches);
        cv::waitKey(1);
    }
    
void matchOrbFeatures(cv::Mat img1, cv::Mat img2) {
    auto [keypoints1, descriptors1] = getOrbFeatures(img1);
    auto [keypoints2, descriptors2] = getOrbFeatures(img2);
    
    cv::Ptr<cv::DescriptorMatcher> matcher = cv::DescriptorMatcher::create(cv::DescriptorMatcher::BRUTEFORCE_HAMMING);
    std::vector<cv::DMatch> matches;
    matcher->match(descriptors1, descriptors2, matches);
    
    std::vector<cv::Point2f> points1, points2;
    for (const auto& match : matches) {
        points1.push_back(keypoints1[match.queryIdx].pt);
        points2.push_back(keypoints2[match.trainIdx].pt);
    }
    
    cv::Mat inliers;
    cv::findHomography(points1, points2, cv::RANSAC, 3.0, inliers);
    
    std::vector<cv::DMatch> inlierMatches;
    for (int i = 0; i < inliers.rows; i++) {
        if (inliers.at<uchar>(i)) {
            inlierMatches.push_back(matches[i]);
        }
    }
    
    cv::Mat img_matches;
    cv::drawMatches(img1, keypoints1, img2, keypoints2, inlierMatches, img_matches);
    cv::imshow("RANSAC Matches", img_matches);
    cv::waitKey(1);
}
    

private:
    void imageCallback(const sensor_msgs::msg::Image::SharedPtr msg)
    {  
        try
        {
            cv::Mat image = cv_bridge::toCvShare(msg, "bgr8")->image;
            if(prev_image.empty()){
                prev_image = image;
            }
            else{
                matchOrbFeatures(prev_image, image);
                prev_image = image;
            }
            // getOrbFeatures(image);
            // Process the image here
        }
        catch (cv_bridge::Exception& e)
        {
            RCLCPP_ERROR(this->get_logger(), "cv_bridge exception: %s", e.what());
            return;
        }
    }
    cv::Mat prev_image;
    rclcpp::Subscription<sensor_msgs::msg::Image>::SharedPtr subscription_;
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);
    rclcpp::spin(std::make_shared<ImageProcessor>());
    rclcpp::shutdown();
    return 0;
}