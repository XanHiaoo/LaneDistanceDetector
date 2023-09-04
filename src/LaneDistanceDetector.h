#ifndef LANEDISTANCEDETECTOR_LANEDISTANCEDETECTOR_H
#define LANEDISTANCEDETECTOR_LANEDISTANCEDETECTOR_H

#include <opencv2/core.hpp>
#include <vector>

#include "LaneDistanceDetectorImpl.h"
#include "TemplateSettings.h"

enum MODELING_RESULT_CODE : int
{
    MODELING_ERROR = -1,
    MODELING_SUCCESEE = 0
};

class LaneDistanceDetectorImpl;

class LaneDistanceDetector
{
  public:
    LaneDistanceDetector();
    ~LaneDistanceDetector();

    void setLaneImageTemplate(const cv::Mat &image);
    LOAD_RESULT_CODE loadSettings(const std::string &path);
    MODELING_RESULT_CODE modeling();
    double calculateDistanceBetweenPoints(const cv::Point &point1, const cv::Point &point2);
    cv::Vec3d calculatePixel2Coordinate(const cv::Point &point);

  private:
    LaneDistanceDetectorImpl *detector_;
};

#endif // LANEDISTANCEDETECTOR_LANEDISTANCEDETECTOR_H
