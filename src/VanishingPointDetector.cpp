// VanishingPointDetector.cpp
#define _USE_MATH_DEFINES

#include "VanishingPointDetector.h"
#include <iostream>
#include <algorithm>
#include <opencv2/core.hpp>

VanishingPointDetector::VanishingPointDetector(const cv::Mat& inputImage) {
    REJECT_DEGREE_TH = 4.0;

    image = inputImage.clone();

    /*if (image.empty()) {
        std::cerr << "Error: Input image is empty." << std::endl;
        exit(-1);
    }*/
}

VanishingPointDetector::~VanishingPointDetector() {
    // Destructor, if needed
}

std::vector<std::vector<double>> VanishingPointDetector::FilterLines(std::vector<cv::Vec4i> lines) {
    std::vector<std::vector<double>> FinalLines;

    for (int i = 0; i < lines.size(); i++)
    {
        cv::Vec4i line = lines[i];
        int x1 = line[0], y1 = line[1];
        int x2 = line[2], y2 = line[3];

        double m, c;

        // Calculating equation of the line : y = mx + c
        if (x1 != x2)
            m = (double)(y2 - y1) / (double)(x2 - x1);
        else
            m = 100000000.0;
        c = y2 - m * x2;

        // theta will contain values between - 90 -> + 90.
        double theta = atan(m) * (180.0 / M_PI);

        // Rejecting lines of slope near to 0 degree or 90 degree and storing others
        if (REJECT_DEGREE_TH <= abs(theta) && abs(theta) <= (90.0 - REJECT_DEGREE_TH) && -75.0 <= theta &&
            theta <= -20.0)
        {
            double l = pow((pow((y2 - y1), 2) + pow((x2 - x1), 2)),
                0.5); // length of the line
            std::vector<double> FinalLine{ (double)x1, (double)y1, (double)x2, (double)y2, m, c, l };
            FinalLines.push_back(FinalLine);
        }
    }

    // Removing extra lines
    // (we might get many lines, so we are going to take only longest 15 lines
    // for further computation because more than this number of lines will only
    // contribute towards slowing down of our algo.)
    if (FinalLines.size() > 15)
    {
        std::sort(FinalLines.begin(), FinalLines.end(),
            [](const std::vector<double>& a, const std::vector<double>& b) { return a[6] > b[6]; });

        FinalLines = std::vector<std::vector<double>>(FinalLines.begin(), FinalLines.begin() + 15);
    }

    return FinalLines;
}

std::vector<std::vector<double>> VanishingPointDetector::GetLines() {
    int height = image.rows;
    int width = image.cols;

    // Select the region of interest (ROI) as the lower half of the image
    cv::Mat roi = image(cv::Rect(0, height / 2, width, height / 2));

    // Converting ROI to grayscale
    cv::Mat Grayimage;
    cv::cvtColor(roi, Grayimage, cv::COLOR_BGR2GRAY);
    // Blurring image to reduce noise.
    cv::Mat BlurGrayimage;
    cv::GaussianBlur(Grayimage, BlurGrayimage, cv::Size(5, 5), 1);
    // Generating Edge image
    cv::Mat Edgeimage;
    cv::Canny(BlurGrayimage, Edgeimage, 40, 255);

    // Finding lines in the ROI
    std::vector<cv::Vec4i> lines;
    cv::HoughLinesP(Edgeimage, lines, 1, CV_PI / 180, 50, 10, 15);

    // Check if lines found and exit if not.
    if (lines.size() == 0)
    {
        std::cout << "Not enough lines found in the image for Vanishing Point detection." << std::endl;
        exit(3);
    }

    // Transform the lines' coordinates to the original image coordinate space
    for (cv::Vec4i& line : lines)
    {
        line[1] += height / 2;
        line[3] += height / 2;
    }

    // Filtering lines wrt angle
    std::vector<std::vector<double>> FilteredLines;
    FilteredLines = FilterLines(lines);

    filteredLines = FilteredLines;
    return FilteredLines;
}

int* VanishingPointDetector::GetVanishingPoint() {
    // We will apply RANSAC inspired algorithm for this. We will take combination
    // of 2 lines one by one, find their intersection point, and calculate the
    // total error(loss) of that point. Error of the point means root of sum of
    // squares of distance of that point from each line.
    int* VanishingPoint = new int[2];
    VanishingPoint[0] = -1;
    VanishingPoint[1] = -1;

    double MinError = 1000000000.0;

    for (int i = 0; i < filteredLines.size(); i++)
    {
        for (int j = i + 1; j < filteredLines.size(); j++)
        {
            double m1 = filteredLines[i][4], c1 = filteredLines[i][5];
            double m2 = filteredLines[j][4], c2 = filteredLines[j][5];

            if (m1 != m2)
            {
                double x0 = (c1 - c2) / (m2 - m1);
                double y0 = m1 * x0 + c1;

                double err = 0;
                for (int k = 0; k < filteredLines.size(); k++)
                {
                    double m = filteredLines[k][4], c = filteredLines[k][5];
                    double m_ = (-1 / m);
                    double c_ = y0 - m_ * x0;

                    double x_ = (c - c_) / (m_ - m);
                    double y_ = m_ * x_ + c_;

                    double l = pow((pow((y_ - y0), 2) + pow((x_ - x0), 2)), 0.5);

                    err += pow(l, 2);
                }

                err = pow(err, 0.5);

                if (MinError > err)
                {
                    MinError = err;
                    VanishingPoint[0] = (int)x0;
                    VanishingPoint[1] = (int)y0;
                }
            }
        }
    }

    return VanishingPoint;
}
