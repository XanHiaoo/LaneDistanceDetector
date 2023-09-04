#include "LaneDistanceDetector.h"

LaneDistanceDetector::LaneDistanceDetector()
{
    detector_ = new LaneDistanceDetectorImpl();
}

LaneDistanceDetector::~LaneDistanceDetector()
{
    delete detector_;
}

void LaneDistanceDetector::setLaneImageTemplate(const cv::Mat &image)
{
    detector_->setLaneImageTemplate(image);
}

LOAD_RESULT_CODE LaneDistanceDetector::loadSettings(const std::string &path)
{
    return (detector_->loadSettings(path));
}

MODELING_RESULT_CODE LaneDistanceDetector::modeling()
{
    if (detector_->settingsLoaded == LOAD_PLANE_PARAMETERS)
        return MODELING_SUCCESEE;
    else if (detector_->settingsLoaded == LOAD_PENDING)
    {
        detector_->calculateVanishingPoint();
        detector_->calculatingParallelPlane();
        detector_->calculatePlaneParameters();
        return MODELING_SUCCESEE;
    }
    else
    {
        return MODELING_ERROR;
    }
}

cv::Vec3d LaneDistanceDetector::calculatePixel2Coordinate(const cv::Point &point)
{
    return detector_->calculatePixel2Coordinate(point);
}

double LaneDistanceDetector::calculateDistanceBetweenPoints(const cv::Point &point1, const cv::Point &point2)
{
    return detector_->calculateDistanceBetweenPoints(point1, point2);
}