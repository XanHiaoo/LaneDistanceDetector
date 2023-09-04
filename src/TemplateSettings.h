#ifndef LANEDISTANCEDETECTOR_TEMPLATESETTINGS_H
#define LANEDISTANCEDETECTOR_TEMPLATESETTINGS_H

#include <opencv2/core.hpp>
enum LOAD_RESULT_CODE : int
{
    LOAD_ERROR = -1,
    LOAD_PENDING = 0,
    LOAD_PLANE_PARAMETERS = 1,
    LOAD_MISSING_PLANE_PARAMETERS = 2,
    LOAD_MISSING_CAMERA_MARTIX = 3,
    LOAD_MISSING_VERTICAL_LINE_POINT_PAIR = 4,
};
struct TemplateSettings
{
    std::string TEMPLATE_SETTINGS_PATH_;
    cv::Mat TEMPLATE_CAMERA_MATRIX_ = (cv::Mat_<double>(3, 3) << 1, 0, 0, 0, 1, 0, 0, 0, 1);
    double TEMPLATE_LANE_WIDTH_ = 1.0;
    std::pair<cv::Point, cv::Point> TEMPLATE_VERTICAL_LINE_POINT_PAIR1_;
    std::pair<cv::Point, cv::Point> TEMPLATE_VERTICAL_LINE_POINT_PAIR2_;
    cv::Vec4d TEMPLATE_PLANE_PARAMETERS_;

    LOAD_RESULT_CODE load(const std::string &path);
    bool savePlaneParameters(cv::Vec4d planeParameters);
};

#endif // LANEDISTANCEDETECTOR_TEMPLATESETTINGS_H
