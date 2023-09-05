#include "LaneDistanceDetector.h"
#include <fmt/core.h>
#include <iostream>
#include <nlohmann/json.hpp>
#include <opencv2/opencv.hpp>

using json = nlohmann::json;

void testLaneDistanceDetector()
{
    LaneDistanceDetector detector;

    std::string templateSettingsPath = "D:\\code\\c++\\MonocularDistanceMeasurement\\LaneData\\Lane1\\";

    cv::Mat template_image = cv::imread(templateSettingsPath + "template.jpg");
    if (template_image.empty())
    {
        std::cerr << "Failed to load image!" << std::endl;
        return;
    }
    detector.setLaneImageTemplate(template_image);

    double d1;
    double d2;
    cv::Vec3d point;
    MODELING_RESULT_CODE modeling_result_code;
    LOAD_RESULT_CODE load_result_code = detector.loadSettings(templateSettingsPath + "template.json");
    switch (load_result_code)
    {
    case LOAD_PLANE_PARAMETERS:
        fmt::print("平面方程已加载\n");
        d1 = detector.calculateDistanceBetweenPoints(cv::Point(2835, 1690), cv::Point(3027, 1408));
        d2 = detector.calculateDistanceBetweenPoints(cv::Point(3219, 1137), cv::Point(3344, 952));
        point = detector.calculatePixel2Coordinate(cv::Point(3027, 1408));
        fmt::print("Pixel({},{}) Coordinate({},{},{})\n", 3027, 1408, point[0], point[1], point[2]);
        fmt::print("Distance\n{}\n{}\n", d1, d2);    
        break;

    case LOAD_PENDING:
        fmt::print("平面方程待计算\n");
        modeling_result_code = detector.modeling();
        fmt::print("平面建模完成\n");
        if (modeling_result_code == MODELING_SUCCESEE)
        {
            d1 = detector.calculateDistanceBetweenPoints(cv::Point(2835, 1690), cv::Point(3027, 1408));
            d2 = detector.calculateDistanceBetweenPoints(cv::Point(3219, 1137), cv::Point(3344, 952));
            fmt::print("{}\n{}", d1, d2);
        }
        break;

    case LOAD_MISSING_PLANE_PARAMETERS:
    case LOAD_MISSING_CAMERA_MARTIX:
    case LOAD_MISSING_VERTICAL_LINE_POINT_PAIR:
    case LOAD_ERROR:
        fmt::print("配置加载失败\n");
        break;

    default:
        fmt::print("配置加载失败\n");
        break;
    }
}

int main()
{
    //system("chcp 65001");
    testLaneDistanceDetector();
    return 0;
}
