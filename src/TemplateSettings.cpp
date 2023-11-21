#include "TemplateSettings.h"
#include <fmt/core.h>
#include <fstream>
#include <iostream>
#include <nlohmann/json.hpp>
using json = nlohmann::json;

LOAD_RESULT_CODE TemplateSettings::load(const std::string &path)
{
    TEMPLATE_SETTINGS_PATH_ = path;
    json jsonObject;
    // 读取template.json文件
    std::ifstream jsonFile(path);
    if (jsonFile.is_open())
    {
        jsonFile >> jsonObject;
        jsonFile.close();

        // 检查是否存在CameraMatrix参数
        if (jsonObject.contains("CameraMatrix"))
        {
            auto cameraMatrixObject = jsonObject["CameraMatrix"];
            if (cameraMatrixObject.is_array() && cameraMatrixObject.size() == 3)
            {
                for (int i = 0; i < 3; ++i)
                {
                    for (int j = 0; j < 3; ++j)
                    {
                        TEMPLATE_CAMERA_MATRIX_.at<double>(i, j) = cameraMatrixObject[i][j];
                    }
                }
                fmt::print("CameraMatrix loaded from template.json.\n");
            }
            else
            {
                fmt::print(stderr, "Invalid CameraMatrix data in template.json!\n");
                return LOAD_MISSING_CAMERA_MARTIX;
            }
        }
        else
        {
            fmt::print(stderr, "CameraMatrix parameter not found in template.json.\n");
            return LOAD_MISSING_CAMERA_MARTIX;
        }

        if (jsonObject.contains("PlaneParameters"))
        {
            auto planeParametersObject = jsonObject["PlaneParameters"];
            if (planeParametersObject.contains("A") && planeParametersObject.contains("B") &&
                planeParametersObject.contains("C") && planeParametersObject.contains("D"))
            {
                TEMPLATE_PLANE_PARAMETERS_ = cv::Vec4d(planeParametersObject["A"], planeParametersObject["B"],
                                                       planeParametersObject["C"], planeParametersObject["D"]);
                fmt::print("PlaneParameters loaded from template.json.\n");
                return LOAD_PLANE_PARAMETERS;
            }
            else
            {
                fmt::print(stderr, "Invalid PlaneParameters data in template.json!\n");
            }
        }
        else
        {
            fmt::print(stderr, "PlaneParameters parameter not found in template.json.\n");
        }

        if (jsonObject.contains("TemplateVerticalLine"))
        {
            auto templateVerticalLineObject = jsonObject["TemplateVerticalLine"];
            if (templateVerticalLineObject.contains("Lane1"))
            {
                auto lane1Object = templateVerticalLineObject["Lane1"];
  

                if (lane1Object.contains("Point1") && lane1Object.contains("Point2"))
                {

                    TEMPLATE_VERTICAL_LINE_POINT_PAIR1_.first.x = lane1Object["Point1"][0].get<int>();
                    TEMPLATE_VERTICAL_LINE_POINT_PAIR1_.first.y = lane1Object["Point1"][1].get<int>();
                    TEMPLATE_VERTICAL_LINE_POINT_PAIR1_.second.x = lane1Object["Point2"][0].get<int>();
                    TEMPLATE_VERTICAL_LINE_POINT_PAIR1_.second.y = lane1Object["Point2"][1].get<int>();

                    fmt::print("TemplateVerticalLine points loaded from template.json.\n");
                }
                else
                {
                    fmt::print(stderr, "Invalid points data in TemplateVerticalLine "
                                       "section of template.json!\n");
                    return LOAD_MISSING_VERTICAL_LINE_POINT_PAIR;
                }
            }
            else
            {
                fmt::print(stderr, "Lane1 not found in TemplateVerticalLine "
                                   "section of template.json!\n");
                return LOAD_MISSING_VERTICAL_LINE_POINT_PAIR;
            }
        }
        else
        {
            fmt::print(stderr, "TemplateVerticalLine parameter not found in template.json.\n");
            return LOAD_MISSING_VERTICAL_LINE_POINT_PAIR;
        }
    }
    else
    {
        fmt::print(stderr, "Failed to open template.json for reading.\n");
        return LOAD_ERROR;
    }
    return LOAD_PENDING;
}

bool TemplateSettings::savePlaneParameters(cv::Vec4d planeParameters)
{
    json jsonObject;

    // 读取原有的JSON内容
    if (!TEMPLATE_SETTINGS_PATH_.empty())
    {
        std::ifstream jsonFile(TEMPLATE_SETTINGS_PATH_);
        if (jsonFile.is_open())
        {
            jsonFile >> jsonObject;
            jsonFile.close();
        }
        else
        {
            fmt::print(stderr, "Failed to open JSON file for reading.\n");
            return false;
        }
    }

    // 创建PlaneParameters对象并存储ABC参数
    json planeParametersObject;
    planeParametersObject["A"] = planeParameters[0];
    planeParametersObject["B"] = planeParameters[1];
    planeParametersObject["C"] = planeParameters[2];
    planeParametersObject["D"] = planeParameters[3];

    jsonObject["PlaneParameters"] = planeParametersObject;

    // 将JSON写入文件
    if (!TEMPLATE_SETTINGS_PATH_.empty())
    {
        std::ofstream outFile(TEMPLATE_SETTINGS_PATH_);
        if (outFile.is_open())
        {
            outFile << jsonObject.dump(4);
            outFile.close();
            fmt::print("PlaneParameters已保存至json文件\n");
            return true;
        }
        else
        {
            fmt::print(stderr, "Failed to save JSON file!\n");
        }
    }
    else
    {
        fmt::print(stderr, "File path is empty!\n");
    }
    return false;
}
