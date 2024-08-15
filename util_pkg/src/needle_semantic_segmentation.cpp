#include <needle_semantic_segmentation.h>

#include <std_srvs/Empty.h>

std::vector<cv::Point> calib_points;
void cvImageCallback(int event, int x, int y, int flags, void *userdata)
{
    if (event == cv::EVENT_LBUTTONDOWN)
    {
        std::cout << "Left button of the mouse is clicked - position (" << x << ", " << y << ")" << std::endl;
        calib_points.push_back(cv::Point(x, y));
    }
    else if (event == cv::EVENT_RBUTTONDOWN)
    {
        // cv::Mat &xyz = *((cv::Mat *)userdata);
        // cv::Mat HSV;
        // cv::cvtColor(xyz, HSV, cv::COLOR_BGR2HSV);
        // cv::Vec3b val = HSV.at<cv::Vec3b>(y, x);
        // std::cout << " Point " << x << ", " << y << " val " << val << std::endl;
    }
}

NeedleSemanticSeg::NeedleSemanticSeg(ros::NodeHandle &nh) : nh_(nh), private_nh_("~"),
                                                            it_(nh), height(480), width(640), is_initialized(false), image_calib(false),
                                                            start_procedure(true), save_data(false)

{
    private_nh_.param("camera_image_frame", camera_image_frame, std::string(""));
    private_nh_.param("reference_frame", reference_frame, std::string(""));
    private_nh_.param("template_img_str", template_img_str, std::string(""));
    private_nh_.param("camera_config", camera_filename, std::string(""));
    private_nh_.param("log_points", log_points, std::string(""));

    private_nh_.param("rec_threshold", rec_threshold, 0.0);
    private_nh_.param("pixel_value", pixel_value, 15.0);

    readCameraParameters(camera_filename);

    camera_image_sub = private_nh_.subscribe("image", 1, &NeedleSemanticSeg::grabImage, this);
    camera_info_sub = private_nh_.subscribe("image_info", 1, &NeedleSemanticSeg::cameraInfoCallback, this);
    debug_image_publisher_ = it_.advertise("/debug_image", 1);
    rendered_image_publisher_ = it_.advertise("/color_image", 1);
    rendered_image_publisher_two = it_.advertise("/template_image", 1);

    service_start = private_nh_.advertiseService("start", &NeedleSemanticSeg::startProcedure, this);
    service_save = private_nh_.advertiseService("save", &NeedleSemanticSeg::saveData, this);

    track_point = cv::Point(21, 12); // to param small_templateNeedle_light_gt_smooth
    // track_point = cv::Point(43, 12); // to param templateNeedle_light_gt_smooth
    // track_point = cv::Point(58, 14); // to param test_template_gt

    br = new tf::TransformBroadcaster();

    template_img = cv::imread(template_img_str);

    // cv::namedWindow("template_img", cv::WINDOW_FREERATIO);
    // cv::imshow("template_img", template_img);
    // cv::waitKey(0);

    private_nh_.getParam("mask_anatomy", mask_anatomy);

    std::vector<double> camera_R;
    if (private_nh_.getParam("camera_R", camera_R))
    {
        private_nh_.param("pix2mm", pix2mm, 0.0);

        ROS_INFO_STREAM("ROTATION MATRIX found");
        // cv::Mat R
        R_camera = (cv::Mat_<double>(3, 3) << camera_R.at(0), camera_R.at(1), camera_R.at(2),
                    camera_R.at(3), camera_R.at(4), camera_R.at(5),
                    camera_R.at(6), camera_R.at(7), camera_R.at(8));
        std::cout << R_camera << std::endl;
    }

    XmlRpc::XmlRpcValue cp;
    if (private_nh_.getParam("/semantic_scene/colors", cp))
    {
        // idx_color.resize(cp.size(), 0);
        ROS_INFO_STREAM("COLORS parameters found");
        for (int i = 0; i < cp.size(); ++i)
        {
            for (XmlRpc::XmlRpcValue::iterator j = cp[i].begin(); j != cp[i].end(); ++j)
            {
                anatomies.push_back(std::string(j->first));
                colors.push_back({j->second[0], j->second[1], j->second[2]});
            }
        }

        for (int i = 0; i < anatomies.size(); i++)
            for (int j = 0; j < mask_anatomy.size(); j++)
                if (anatomies.at(i) == mask_anatomy.at(j))
                    idx_color.push_back(i);

        camera_semantic_sub = private_nh_.subscribe("semantic", 1, &NeedleSemanticSeg::cameraSemanticCallbackComp, this);
    }
}

NeedleSemanticSeg::~NeedleSemanticSeg()
{
}

void NeedleSemanticSeg::cameraSemanticCallback(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        semantic_img = cv_ptr->image;
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void NeedleSemanticSeg::cameraSemanticCallbackComp(const sensor_msgs::CompressedImageConstPtr &msg)
{
    try
    {
        semantic_img = cv::imdecode(cv::Mat(msg->data), 1);
        // ROS_INFO_STREAM("semantic");
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }
}

void NeedleSemanticSeg::PublishRenderedImage(image_transport::Publisher pub, cv::Mat image, std::string encoding)
{
    std_msgs::Header header;
    header.stamp = ros::Time::now();
    header.frame_id = "camera_color_optical_frame";
    const sensor_msgs::ImagePtr rendered_image_msg = cv_bridge::CvImage(header, encoding, image).toImageMsg();
    pub.publish(rendered_image_msg);
}

bool NeedleSemanticSeg::initNeedleTrack(cv::Mat &templateImg, cv::Mat &img, float recThreshold, std::vector<cv::Point2f> &point2Track, cv::Point &corner_template)
{
    bool isNedInitialized = false;
    cv::Rect region_of_interest;
    cv::Mat outMatch;
    cv::Mat gray, templateImgGray;
    cv::cvtColor(img, gray, cv::COLOR_BGR2GRAY);
    cv::cvtColor(templateImg, templateImgGray, cv::COLOR_BGR2GRAY);

    cv::matchTemplate(gray, templateImgGray, outMatch, CV_TM_CCOEFF_NORMED);

    double minVal, maxVal;
    cv::Point *minloc = new cv::Point();
    cv::Point *maxloc = new cv::Point();
    cv::minMaxLoc(outMatch, &minVal, &maxVal, minloc, maxloc, cv::Mat());

    cv::Point btm_right(maxloc->x + templateImg.size().width, maxloc->y + templateImg.size().height);
    std::cout << "Looking for the needle: " << maxVal << " threshold: " << recThreshold << std::endl;

    if (maxVal > recThreshold)
    {
        std::cout << "Needle found" << std::endl;
        corner_template = *maxloc;
        cv::rectangle(img, cv::Rect(*maxloc, btm_right), cv::Scalar(0, 255, 0), 1, 8);

        region_of_interest = cv::Rect(maxloc->x, maxloc->y, templateImg.size().width, templateImg.size().height);
        cv::Mat img_roi = gray(region_of_interest);

        isNedInitialized = true;
    }
    delete minloc;
    delete maxloc;
    return isNedInitialized;
}

cv::Point NeedleSemanticSeg::findRightmostPoint(const std::vector<cv::Point> &points)
{
    if (points.empty())
    {
        // Handle the case when the vector is empty.
        throw std::invalid_argument("Input vector is empty.");
    }

    cv::Point rightmostPoint = points[0]; // Initialize with the first point.

    // Iterate through the vector and update rightmostPoint if a point with a higher x-coordinate is found.
    for (const auto &point : points)
    {
        if (point.x > rightmostPoint.x)
        {
            rightmostPoint = point;
        }
    }

    return rightmostPoint;
}

bool NeedleSemanticSeg::readCameraParameters(std::string filename)
{
    cv::FileStorage fs;
    fs.open(filename, cv::FileStorage::READ);
    if (!fs.isOpened())
    {
        std::cout << "failed to open " << filename << std::endl;
        return false;
    }
    else
    {
        fs["camera_w"] >> width;
        fs["camera_h"] >> height;
        fs["pattern_width"] >> pattern_width;
        fs["pattern_height"] >> pattern_height;
        fs["square_size"] >> square_size;
        fs["camera_matrix"] >> K;
        std::cout << filename << " LOADED with the following parameters" << std::endl;
        std::cout << "camera_w " << width << "\n"
                  << "camera_h " << height << "\n"
                  << "pattern_width " << pattern_width << "\n"
                  << "pattern_height " << pattern_height << "\n"
                  << "square_size " << square_size << std::endl;
        std::cout << " K " << K << std::endl;
        std::cout << "--------------------------------------" << std::endl;
        return true;
    }
}

cv::Mat NeedleSemanticSeg::poseFromPoints(std::vector<cv::Point3f> objectPoints, std::vector<cv::Point> corners)
{
    cv::Mat T;

    std::vector<cv::Point2f> objectPointsPlanar;

    for (size_t i = 0; i < objectPoints.size(); i++)
    {
        objectPointsPlanar.push_back(cv::Point2f(objectPoints[i].x, objectPoints[i].y));
    }
    std::vector<cv::Point2f> corners_f, imagePoints;

    for (auto &c : corners)
        corners_f.emplace_back(c);

    cv::undistortPoints(corners_f, imagePoints, K, D);

    cv::Mat H_points = cv::findHomography(objectPointsPlanar, imagePoints);

    // Normalization to ensure that ||c1|| = 1
    double norm = std::sqrt(H_points.at<double>(0, 0) * H_points.at<double>(0, 0) +
                            H_points.at<double>(1, 0) * H_points.at<double>(1, 0) +
                            H_points.at<double>(2, 0) * H_points.at<double>(2, 0));
    H_points /= norm;
    cv::Mat c1 = H_points.col(0);
    cv::Mat c2 = H_points.col(1);
    cv::Mat c3 = c1.cross(c2);
    cv::Mat tvec = H_points.col(2);
    cv::Mat R(3, 3, CV_64F);
    for (int i = 0; i < 3; i++)
    {
        R.at<double>(i, 0) = c1.at<double>(i, 0);
        R.at<double>(i, 1) = c2.at<double>(i, 0);
        R.at<double>(i, 2) = c3.at<double>(i, 0);
    }
    cv::Mat W, U, Vt;
    cv::SVDecomp(R, W, U, Vt);
    R = U * Vt;
    cv::Mat rvec;
    // std::cout << " R " << R << std::endl;
    // cv::Rodrigues(R, rvec);
    // T.push_back(tvec);
    // T.push_back(rvec);

    return R;
}

void NeedleSemanticSeg::grabImage(const sensor_msgs::ImageConstPtr &msg)
{
    cv_bridge::CvImageConstPtr cv_ptr;
    cv::Mat image;
    try
    {
        cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
        // ROS_INFO_STREAM("RGB");
        if (!image_calib && R_camera.empty())
        {
            if (!cv_ptr->image.empty())
            {

                cv::namedWindow("calib", cv::WINDOW_FREERATIO);
                cv::imshow("calib", cv_ptr->image);
                // set the callback function for any mouse event
                //  cv::setMouseCallback("calib", CallBackFunc, NULL);
                cv::setMouseCallback("calib", cvImageCallback, NULL);

                cv::waitKey(20000); // 10 sec, need 6 points.
                cv::destroyWindow("calib");

                pix2mm = pixel_value / cv::norm(calib_points.at(0) - calib_points.at(1));
                std::cout << "NEW pix2mm " << pix2mm << " " << cv::norm(calib_points.at(0) - calib_points.at(1)) << std::endl;

                std::vector<cv::Point3f> objs;
                objs.push_back(cv::Point3f(0.0, 0.0, 0.0));
                objs.push_back(cv::Point3f(0.03, 0.0, 0.0));
                objs.push_back(cv::Point3f(0.0, 0.03, 0.0));
                objs.push_back(cv::Point3f(0.03, 0.03, 0.0));

                std::vector<cv::Point> calib_four_points{calib_points.at(0), calib_points.at(1), calib_points.at(2), calib_points.at(3)};
                R_camera = poseFromPoints(objs, calib_four_points);
                std::cout << " NEW R_camera " << R_camera << std::endl;

                image_calib = true;
            }
        }
    }
    catch (cv_bridge::Exception &e)
    {
        ROS_ERROR("cv_bridge exception: %s", e.what());
        return;
    }

    if (!start_procedure && !save_data)
    {

        cv::Mat mask = cv::Mat::ones(height, width, CV_8U);
        // cv::Rect roi = cv::Rect(0, 0, mask.cols / 2, mask.rows);
        // mask(roi).setTo(cv::Scalar::all(255));
        // FilterInstrument(cv_ptr->image, mask);

        // MASK SEMANTIC

        std::vector<cv::Mat> anatomy_section;
        anatomy_section.resize(mask_anatomy.size());

        cv::Mat mask_semantic = cv::Mat::ones(height, width, CV_8U);

        if (!semantic_img.empty())
        {

            if (calib_points.size() > 4)
            {
                std::vector<cv::Point2f> idx_roi;
                idx_roi.push_back(calib_points.at(4));
                idx_roi.push_back(calib_points.at(5));
                idx_roi.push_back(calib_points.at(6));
                idx_roi.push_back(calib_points.at(7));
                std::cout << " ok " << std::endl;

                cv::Mat bb_mask, res;
                bb_mask = createROI(idx_roi, semantic_img);
                semantic_img.copyTo(res, bb_mask);

                semantic_img = res;
            }
            PublishRenderedImage(rendered_image_publisher_, semantic_img, "bgr8");
            for (size_t i = 0; i < mask_anatomy.size(); i++)
            {
                cv::Mat mask_temp;
                // std::cout << colors.at(idx_color.at(i))[2] << " " << colors.at(idx_color.at(i))[1] << " " << colors.at(idx_color.at(i))[0] << std::endl;
                // cv::inRange(semantic_img,
                //             cv::Scalar(colors.at(idx_color.at(i))[2] - 35, colors.at(idx_color.at(i))[1] - 35, colors.at(idx_color.at(i))[0] - 35),
                //             cv::Scalar(colors.at(idx_color.at(i))[2] + 35, colors.at(idx_color.at(i))[1] + 35, colors.at(idx_color.at(i))[0] + 35),
                //             mask_temp); // 50 L

                cv::Mat HSV;
                cv::cvtColor(semantic_img, HSV, cv::COLOR_BGR2HSV);

                cv::Mat mask1, mask2;
                cv::inRange(HSV, cv::Scalar(0, 50, 50), cv::Scalar(15, 255, 255), mask1);
                cv::inRange(HSV, cv::Scalar(160, 50, 50), cv::Scalar(180, 255, 255), mask2);

                mask_temp = mask1 | mask2;

                semantic_img.copyTo(anatomy_section.at(i), mask_temp);
                // std::cout << " size " << mask_semantic.size() << std::endl;
                // std::cout << " mask_temp size " << mask_temp.size() << std::endl;
                mask_semantic = mask_temp;
                // mask_semantic = mask_semantic + mask_temp;
            }

            cv::Mat out_mask_morph, out_mask_morph2;
            morphOperation(mask_semantic, out_mask_morph, 2, 1);
            morphOperation(out_mask_morph, out_mask_morph2, 3, 3);

            std::vector<cv::Point> non_zero_points;
            cv::Mat out_mask = increaseMaskDimension(out_mask_morph2, 10);

            cv::Mat max_mask = cv::Mat::zeros(semantic_img.size(), CV_8U);
            filterContour(out_mask, max_mask);

            cv::findNonZero(max_mask, non_zero_points);

            // Find the top-right pixel
            if (non_zero_points.size() > 0)
            {

                cv::Point top_right_pixel = findRightmostPoint(non_zero_points);
                mask_points3d.push_back(cv::Point3f(top_right_pixel.x * pix2mm, top_right_pixel.y * pix2mm, 0.0));
                /*
                                cv::Mat point_mat = (cv::Mat_<double>(3, 1) << top_right_pixel.x * pix2mm, top_right_pixel.y * pix2mm, 0.0);

                                // Multiply the rotation matrix R by the point_mat to get the rotated point
                                cv::Mat rotated_point_mat = R_camera * point_mat;

                                // Convert the rotated_point_mat back to a cv::Point3f
                                cv::Point3f rotated_point(rotated_point_mat.at<double>(0, 0),
                                                          rotated_point_mat.at<double>(1, 0),
                                                          rotated_point_mat.at<double>(2, 0));

                                mask_points3d.push_back(rotated_point);
                */
                auto img = cv_ptr->image.clone();
                auto img_template = cv_ptr->image.clone();
                cv::circle(img, top_right_pixel, 3, cv::Scalar(0, 0, 255), -1);
                // PublishRenderedImage(rendered_image_publisher_, img, "bgr8");

                cv::Point corner_template;
                cv::Mat fix_template_img;
                // adapt_to_template(template_img, fix_template_img, 1);
                cv::Mat res;
                cv::Mat out_mask_two = increaseMaskDimension(max_mask, 10); // if HOLES 8 -- 3
                // cv::Mat out_mask_two = increaseRightPartOfMask(out_mask, 50);

                int x = top_right_pixel.x;
                int y = top_right_pixel.y - 5;
                int width = 15;
                int height = 20;
                // our rectangle...
                cv::Rect rect(x, y, width, height);
                // essentially do the same thing
                cv::rectangle(out_mask_two, rect, cv::Scalar(255, 255, 255), -1);

                img_template.copyTo(res, out_mask_two);

                cv::Mat fix_res;
                adapt_to_template(res, fix_res, 2);

                // cv::Mat out_template_mask = increaseMaskDimension(template_img, 1);
                PublishRenderedImage(rendered_image_publisher_two, out_mask_two, "mono8");
                PublishRenderedImage(debug_image_publisher_, template_img, "mono8");

                is_initialized = initNeedleTrack(template_img, res, rec_threshold, tracked_features, corner_template);

                if (corner_template.x != 0 && corner_template.y != 0)
                {

                    cv::Point temp_p(track_point.x + corner_template.x, track_point.y + corner_template.y);
                    cv::circle(img, temp_p, 3, cv::Scalar(0, 255, 0), -1);

                    points3d.push_back(cv::Point3f(temp_p.x * pix2mm, temp_p.y * pix2mm, 0.0)); // senza portare indietro
                                                                                                /*
                                                                                                                    cv::Mat point_mat = (cv::Mat_<double>(3, 1) << temp_p.x * pix2mm, temp_p.y * pix2mm, 0.0);
                                                                            
                                                                                                                    // Multiply the rotation matrix R by the point_mat to get the rotated point
                                                                                                                    cv::Mat rotated_point_mat = R_camera * point_mat;
                                                                            
                                                                                                                    // Convert the rotated_point_mat back to a cv::Point3f
                                                                                                                    cv::Point3f rotated_point(rotated_point_mat.at<double>(0, 0),
                                                                                                                                              rotated_point_mat.at<double>(1, 0),
                                                                                                                                              rotated_point_mat.at<double>(2, 0));
                                                                            
                                                                                                                    points3d.push_back(rotated_point); // senza portare indietro
                                                                                                */
                    // PublishRenderedImage(rendered_image_publisher_, img, "bgr8");
                }
                else
                {

                    points3d.push_back(cv::Point3f(0.0, 0.0, 0.0));
                }

                std::string res_dir = log_points;

                auto diff_time = ros::Time::now().toSec() - start_time;

                cv::imwrite(res_dir + "/img_" + std::to_string(diff_time) + ".jpg", img);

                ROS_INFO_STREAM(" ros time " << diff_time);
                act_time.push_back(diff_time);
            }
        }
    }
    // END MASK SEMANTIC
}

cv::Mat NeedleSemanticSeg::createROI(std::vector<cv::Point2f> coord, cv::Mat &img)
{
    std::vector<cv::Point> pt;
    for (int ao = 0; ao < coord.size(); ao++)
    {
        pt.push_back(coord.at(ao));
    }
    cv::polylines(img, pt, true, cv::Scalar(0, 0, 0), 1, 8, 0);

    std::vector<cv::Point> ROI_Poly;
    cv::approxPolyDP(pt, ROI_Poly, 1.0, true);
    cv::Mat mask(img.size(), CV_8UC1);

    for (int i = 0; i < mask.cols; i++)
        for (int j = 0; j < mask.rows; j++)
            mask.at<uchar>(cv::Point(i, j)) = 0;

    cv::fillConvexPoly(mask, &ROI_Poly[0], ROI_Poly.size(), 255, 8, 0);
    return mask;
}

bool NeedleSemanticSeg::startProcedure(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
    try
    {
        start_procedure = false;
        ROS_INFO("START PROCEDURE");

        start_time = ros::Time::now().toSec();
        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
}

bool NeedleSemanticSeg::saveData(std_srvs::Empty::Request &request, std_srvs::Empty::Response &response)
{
    try
    {
        save_data = true;
        ROS_INFO("SAVE  DATA");

        std::ofstream point_cloud_file;
        std::string point_cloud_log = log_points + ".csv"; //"/home/andrea/workspace/src/utils/util_pkg/templates/point_cloud.csv";
        point_cloud_file.open(point_cloud_log);

        for (int i = 0; i < act_time.size(); i++)
        {
            point_cloud_file << act_time.at(i) << ";" << getPoints3d().at(i) << std::endl;
        }
        // for (auto &p : getPoints3d())
        // {
        //     point_cloud_file << p << std::endl;
        // }
        point_cloud_file.close();

        // mask
        std::ofstream point_cloud_file_mask;
        std::string point_cloud_log_mask = log_points + "_mask.csv"; //"/home/andrea/workspace/src/utils/util_pkg/templates/point_cloud.csv";
        point_cloud_file_mask.open(point_cloud_log_mask);
        for (int i = 0; i < act_time.size(); i++)
        {
            point_cloud_file_mask << act_time.at(i) << ";" << getMaskPoints3d().at(i) << std::endl;
        }

        // for (auto &p : getMaskPoints3d())
        // {
        //     point_cloud_file_mask << p << std::endl;
        // }
        point_cloud_file_mask.close();

        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        return false;
    }
}

void NeedleSemanticSeg::cameraInfoCallback(const sensor_msgs::CameraInfoConstPtr &msg)
{
    // camera parameters
    fx = msg->K[0];
    fy = msg->K[4];
    cx = msg->K[2];
    cy = msg->K[5];

    baseline = msg->P[3];

    k1 = msg->D[0];
    k2 = msg->D[1];
    p1 = msg->D[2];
    p2 = msg->D[3];
    k3 = msg->D[4];

    height = msg->height;
    width = msg->width;

    camera_info_sub.shutdown(); // Shutdown the sub we want camera_info once
}

cv::Mat NeedleSemanticSeg::eul2rotm(geometry_msgs::Vector3 eul)
{
    // ZYX --  x roll, y pitch, z yaw
    cv::Mat rotm(3, 3, CV_32F);
    float s_1 = std::sin(eul.x);
    float c_1 = std::cos(eul.x);
    float s_2 = std::sin(eul.y);
    float c_2 = std::cos(eul.y);
    float s_3 = std::sin(eul.z);
    float c_3 = std::cos(eul.z);

    rotm.at<float>(0, 0) = c_1 * c_2;
    rotm.at<float>(0, 1) = c_1 * s_2 * s_3 - s_1 * c_3;
    rotm.at<float>(0, 2) = c_1 * s_2 * c_3 + s_1 * s_3;

    rotm.at<float>(1, 0) = s_1 * c_2;
    rotm.at<float>(1, 1) = s_1 * s_2 * s_3 + c_1 * c_3;
    rotm.at<float>(1, 2) = s_1 * s_2 * c_3 - c_1 * s_3;

    rotm.at<float>(2, 0) = -s_2;
    rotm.at<float>(2, 1) = c_2 * s_3;
    rotm.at<float>(2, 2) = c_2 * c_3;
    return rotm;
}

void NeedleSemanticSeg::adapt_to_template(cv::Mat &in, cv::Mat &out, int morph_op)
{

    cv::Mat gray;
    cv::cvtColor(in, gray, CV_BGR2GRAY);
    cv::Mat binaryIMG, binaryIMG_m, binaryIMG_m2, inverted_binary_image;

    // cv::adaptiveThreshold(gray, binaryIMG, 255, CV_ADAPTIVE_THRESH_GAUSSIAN_C, CV_THRESH_BINARY, 13, 3);
    cv::adaptiveThreshold(gray, binaryIMG, 255, CV_ADAPTIVE_THRESH_MEAN_C, CV_THRESH_BINARY, 13, 1); // old -- 31,3

    morphOperation(binaryIMG, binaryIMG_m, 2, morph_op);
    morphOperation(binaryIMG_m, binaryIMG_m2, 3, 1);

    bitwise_not(binaryIMG_m2, inverted_binary_image); // inverting the binary image and storing it in inverted_binary_image matrix

    cv::Mat image;
    cv::cvtColor(inverted_binary_image, out, CV_GRAY2BGR);
}

// Function to increase the dimension of a mask by 'numPixels'
cv::Mat NeedleSemanticSeg::increaseMaskDimension(const cv::Mat &mask, int numPixels)
{
    if (numPixels <= 0)
    {
        // If numPixels is non-positive, return the original mask.
        return mask.clone();
    }

    // Define a kernel for dilation. You can choose the shape and size as per your requirement.
    cv::Mat kernel = cv::getStructuringElement(cv::MORPH_RECT, cv::Size(numPixels, numPixels));

    // Perform dilation on the mask.
    cv::Mat dilatedMask;
    cv::dilate(mask, dilatedMask, kernel);

    return dilatedMask;
}

cv::Mat NeedleSemanticSeg::increaseRightPartOfMask(const cv::Mat &mask, int numPixels)
{
    return cv::Mat();
}

void NeedleSemanticSeg::morphOperation(cv::Mat img, cv::Mat &morphMask, int op, int morph_size)
{
    int morph_elem = 2; // Element:\n 0: Rect - 1: Cross - 2: Ellipse"
    // int     morph_size = 1;
    cv::Mat element = getStructuringElement(
        morph_elem, cv::Size(2 * morph_size + 1, 2 * morph_size + 1), cv::Point(morph_size, morph_size));

    morphologyEx(img, morphMask, op, element); // Closing: 3,  It is useful in closing small holes
}

std::vector<cv::Point3d> NeedleSemanticSeg::getPoints3d()
{
    return points3d;
}

std::vector<cv::Point3d> NeedleSemanticSeg::getMaskPoints3d()
{
    return mask_points3d;
}

void NeedleSemanticSeg::filterContour(cv::Mat &img, cv::Mat &mask)
{
    int largest_area = 0;
    int largest_contour_index = 0;
    std::vector<std::vector<cv::Point>> contours;
    std::vector<cv::Vec4i> hierarchy;
    cv::findContours(img, contours, hierarchy, CV_RETR_CCOMP, CV_CHAIN_APPROX_SIMPLE);
    std::cout << " contours.size() " << contours.size() << std::endl;
    for (int i = 0; i < contours.size(); i++)
    {
        double a = cv::contourArea(contours[i], false);
        if (a > largest_area)
        {
            largest_area = a;
            largest_contour_index = i;
        }
    }
    cv::Scalar color(255, 255, 255);
    cv::drawContours(mask, contours, largest_contour_index, color, cv::FILLED, 8, hierarchy);
}
/*
cv::Rect NeedleSemanticSeg::detectRegions(cv::Mat& img, int min_area)
{
    cv::Rect roi;
    std::vector<std::vector<cv::Point>> regions;
    std::vector<cv::Rect>               mser_bbox;
    ms->setMinArea(min_area);
    ms->detectRegions(img, regions, mser_bbox);
    int maxArea     = 0;
    int idx_maxArea = 0;
    if (regions.size() > 0) {
        for (int i = 0; i < regions.size(); i++) {
            // cv::rectangle(img, mser_bbox[i], cv::Scalar(255, 255, 255));
            int tempArea = mser_bbox[i].height * mser_bbox[i].width;
            if (maxArea < tempArea) {
                maxArea     = tempArea;
                idx_maxArea = i;
            }
        }
        // cv::rectangle(img, mser_bbox[idx_maxArea], cv::Scalar(255, 255, 255));
        roi = mser_bbox[idx_maxArea];
    }
}
*/
// ------------------------------------------------------------------

int main(int argc, char **argv)
{
    ros::init(argc, argv, "needle_semantic_seg");
    ros::NodeHandle nodeHandler;

    NeedleSemanticSeg igb(nodeHandler);

    // Spin me round
    ros::spin();

    return 0;
}
