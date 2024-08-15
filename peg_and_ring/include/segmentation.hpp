#ifndef _SEGMENTATION_HPP_
#define _SEGMENTATION_HPP_

#include <opencv2/opencv.hpp>
#include <iostream>
#include <vector>
#include <image_transport/image_transport.h>
#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>
#include <opencv2/imgproc/imgproc.hpp>
#include <opencv2/highgui/highgui.hpp>

/*
    Hue values of basic colors:
    Orange  0-22
    Yellow 22- 38
    Green 38-75
    Blue 75-130
    Violet 130-160
    Red 160-179
*/

cv::Mat hue_red(cv::Mat HSV)
{
    cv::Mat hueMask_red_upper;
    inRange(HSV, cv::Scalar(160, 90, 90), cv::Scalar(180, 255, 255), hueMask_red_upper);
    cv::Mat hueMask_red_lower;
    inRange(HSV, cv::Scalar(0, 90, 90), cv::Scalar(10, 255, 255), hueMask_red_lower);

    return hueMask_red_lower | hueMask_red_upper;
}

cv::Mat hue_blue(cv::Mat HSV)
{
    cv::Mat hueMask_blue;
    inRange(HSV, cv::Scalar(105, 90, 80), cv::Scalar(130, 255, 255), hueMask_blue);

    return hueMask_blue;
}

cv::Mat hue_green(cv::Mat HSV)
{
    cv::Mat hueMask_green;
    inRange(HSV, cv::Scalar(60, 90, 80), cv::Scalar(90, 255, 255), hueMask_green);

    return hueMask_green;
}

cv::Mat hue_yellow(cv::Mat HSV)
{
    cv::Mat hueMask_yellow;
    inRange(HSV, cv::Scalar(22, 90, 80), cv::Scalar(38, 255, 255), hueMask_yellow);

    return hueMask_yellow;
}


/**
 * @brief mser particular shapes recognition algorithm
 * @param image original image
 * @param dest image where to put the mser result
 * @param minArea minimum area accepted
 * @param maxArea maximum area accepted
 * @return the recognized shape drawn on dest image
 */
cv::Mat mser(cv::Mat image, cv::Mat dest, int minArea, int maxArea)
{
    cv::Mat result = dest;

    // MSER params (delta value for local detection, min acceptable area, max acceptable area)
    cv::Ptr<cv::MSER> ms = cv::MSER::create(5, minArea, maxArea);
    std::vector<std::vector<cv::Point>> regions;
    std::vector<cv::Rect> mser_bbox;
    ms->detectRegions(image, regions, mser_bbox);

    for (int i = 0; i < regions.size(); i++)
    {
        // rectangle -> rectangular contour | ellipse -> circular contour
        // rectangle params (img, Rect rec, color, thickness, lineType=8, shift=0)
        //rectangle(image, mser_bbox[i], CV_RGB(0, 255, 0));
        // ellipse params (img, RotatedRect& box, color, thickness, int lineType=8)
        ellipse(result, fitEllipse(regions[i]), cv::Scalar(255, 255, 0), 5); // TODO maybe 4
    }
    return result;
}

/**
 * @brief mser2 the mser variant who saves the found circles in a vector
 * @param image original image
 * @param minArea minimum area
 * @param maxArea maximum area
 * @param mser_bbox vector where to save the found circles
 */
void mser2(cv::Mat image, int minArea, int maxArea, std::vector<cv::Rect> *mser_bbox)
{
    // MSER params (delta value for local detection, min acceptable area, max acceptable area)
    cv::Ptr<cv::MSER> ms = cv::MSER::create(5, minArea, maxArea);
    //mser_parameter* temp = new mser_parameters();
    std::vector<std::vector<cv::Point>> regions;
    ms->detectRegions(image, regions, *mser_bbox);
}

/**
 * @brief hough round shapes recognition algorithm
 * @param image original image
 * @param dest destination image
 * @param minRadius minRadius of circle
 * @param maxRadius maxRadius of circle
 * @return the recognized shape drawn on dest image
 */
cv::Mat hough(cv::Mat image, cv::Mat dest, int minRadius, int maxRadius)
{
    cv::Mat result = dest;

    std::vector<cv::Vec3f> circles;
    HoughCircles(image, circles, cv::HOUGH_GRADIENT, 1,
                 image.rows / 16,              // Change this value to detect circles with different distances to each other
                 100, 30, minRadius, maxRadius // Change the last two parameters - 10, 50
                 // (min_radius & max_radius) to detect larger circles
    );
    for (size_t i = 0; i < circles.size(); i++)
    {
        cv::Vec3i c = circles[i];
        // circle params (img, center, radius, color, thickness, lineType)
        circle(result, cv::Point(c[0], c[1]), c[2], cv::Scalar(255), 3, cv::LINE_AA);

        //hough can miss the radius length... we can fill the area (non optimal solution)
        //circle(hough, cv::Point(c[0], c[1]), 1, cv::Scalar(255), 60, LINE_AA);
    }
    return result;
}

/**
 * @brief findMainArea find the main area (white circle) in the image
 * @param image
 * @param HSV
 * @return main area mask
 */
cv::Mat findMainArea(cv::Mat image)
{
    cv::Mat gray, img2;

    bilateralFilter(image, img2, 15, 80, 80);
    cvtColor(img2, gray, CV_BGR2GRAY);
    Canny(gray, gray, 40, 45);

    // Basic b/n image to use
    cv::Mat bn(image.rows, image.cols, CV_8UC1, cv::Scalar(0));

    std::vector<cv::Vec3f> circles;
    HoughCircles(gray, circles, cv::HOUGH_GRADIENT, 1,
                 image.rows / 16,  // Change this value to detect circles with different distances to each other
                 100, 30, 200, 500 // Change the last two parameters
                 // (min_radius & max_radius) to detect larger circles
    );
    for (size_t i = 0; i < circles.size(); i++)
    {
        cv::Vec3i c = circles[i];
        // circle params (img, center, radius, color, thickness, lineType)
        //circle(image, cv::Point(c[0], c[1]), c[2], cv::Scalar(255), 3, LINE_AA);
        circle(bn, cv::Point(c[0], c[1]), 1, cv::Scalar(255), c[2] * 2, cv::LINE_AA); // *2
    }

    return bn;
}

#endif // SEGMENTATION_H