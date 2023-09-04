#ifndef LANEDISTANCEDETECTOR_LANEDISTANCEDETECTORIMPL_H
#define LANEDISTANCEDETECTOR_LANEDISTANCEDETECTORIMPL_H

#include "TemplateSettings.h"
#include <opencv2/opencv.hpp>

class LaneDistanceDetectorImpl
{
  public:
    LaneDistanceDetectorImpl();  // Constructor
    ~LaneDistanceDetectorImpl(); // Destructor

    LOAD_RESULT_CODE settingsLoaded;

    void setLaneImageTemplate(const cv::Mat &image);
    void setDetectImage(const cv::Mat &image);
    void setCameraMatrix(const cv::Mat &cameraMatrix);
    void setVanishingPoint(const cv::Point &vanishingPoint);
    void setPlaneParameters(const cv::Vec4d &planeParameters);
    void setVerticalLinePointPairs(const std::vector<std::pair<cv::Point, cv::Point>> &pointPairs);
    void setVerticalLinePair(const std::pair<cv::Vec3d, cv::Vec3d> &lines);

  public:
    LOAD_RESULT_CODE loadSettings(const std::string &path);
    bool calculateVanishingPoint();
    cv::Vec3d calculatingParallelPlane();
    cv::Vec4d calculatePlaneParameters();
    cv::Vec3d calculatePixel2Coordinate(const cv::Point &point);
    double calculateDistanceBetweenPoints(const cv::Point &point1, const cv::Point &point2);

  private:
    TemplateSettings settings;
    cv::Mat laneTemplateImage_;
    cv::Mat detectImage_;
    cv::Mat cameraMatrix_;
    cv::Point vanishingPoint_;
    double templateLaneWidth_ = 1.0;
    cv::Vec4d planeParameters_;
    std::vector<std::pair<cv::Point, cv::Point>> verticalLinePointPairs_;
    std::pair<cv::Vec3d, cv::Vec3d> verticalLinePair_;

  private:
    void savePlaneParameters(cv::Vec4d planeParameters);
};

#endif // LANEDISTANCEDETECTOR_LANEDISTANCEDETECTORIMPL_H
