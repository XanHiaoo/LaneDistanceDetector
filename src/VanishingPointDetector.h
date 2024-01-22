// VanishingPointDetector.h
#pragma once

#include <vector>
#include <opencv2/opencv.hpp>

class VanishingPointDetector {
public:
    VanishingPointDetector(const cv::Mat& inputImage);
    ~VanishingPointDetector();

    std::vector<std::vector<double>> GetLines();
    int* GetVanishingPoint();

private:
    float REJECT_DEGREE_TH;
    cv::Mat image;
    std::vector<std::vector<double>> filteredLines;

private:
    std::vector<std::vector<double>> FilterLines(std::vector<cv::Vec4i> lines);
};

