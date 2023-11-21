#define _USE_MATH_DEFINES

#include "LaneDistanceDetectorImpl.h"

float REJECT_DEGREE_TH = 4.0;

std::vector<std::vector<double>> FilterLines(std::vector<cv::Vec4i> Lines)
{
    std::vector<std::vector<double>> FinalLines;

    for (int i = 0; i < Lines.size(); i++)
    {
        cv::Vec4i Line = Lines[i];
        int x1 = Line[0], y1 = Line[1];
        int x2 = Line[2], y2 = Line[3];

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
            std::vector<double> FinalLine{(double)x1, (double)y1, (double)x2, (double)y2, m, c, l};
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
                  [](const std::vector<double> &a, const std::vector<double> &b) { return a[6] > b[6]; });

        FinalLines = std::vector<std::vector<double>>(FinalLines.begin(), FinalLines.begin() + 15);
    }

    return FinalLines;
}

std::vector<std::vector<double>> GetLines(cv::Mat Image)
{
    int height = Image.rows;
    int width = Image.cols;

    // Select the region of interest (ROI) as the lower half of the image
    cv::Mat roi = Image(cv::Rect(0, height / 2, width, height / 2));

    // Converting ROI to grayscale
    cv::Mat GrayImage;
    cv::cvtColor(roi, GrayImage, cv::COLOR_BGR2GRAY);
    // Blurring image to reduce noise.
    cv::Mat BlurGrayImage;
    cv::GaussianBlur(GrayImage, BlurGrayImage, cv::Size(5, 5), 1);
    // Generating Edge image
    cv::Mat EdgeImage;
    cv::Canny(BlurGrayImage, EdgeImage, 40, 255);

    // Finding Lines in the ROI
    std::vector<cv::Vec4i> Lines;
    cv::HoughLinesP(EdgeImage, Lines, 1, CV_PI / 180, 50, 10, 15);

    // Check if lines found and exit if not.
    if (Lines.size() == 0)
    {
        std::cout << "Not enough lines found in the image for Vanishing Point detection." << std::endl;
        exit(3);
    }

    // Transform the lines' coordinates to the original image coordinate space
    for (cv::Vec4i &line : Lines)
    {
        line[1] += height / 2;
        line[3] += height / 2;
    }

    // Filtering Lines wrt angle
    std::vector<std::vector<double>> FilteredLines;
    FilteredLines = FilterLines(Lines);

    return FilteredLines;
}

int *GetVanishingPoint(std::vector<std::vector<double>> Lines)
{
    // We will apply RANSAC inspired algorithm for this. We will take combination
    // of 2 lines one by one, find their intersection point, and calculate the
    // total error(loss) of that point. Error of the point means root of sum of
    // squares of distance of that point from each line.
    int *VanishingPoint = new int[2];
    VanishingPoint[0] = -1;
    VanishingPoint[1] = -1;

    double MinError = 1000000000.0;

    for (int i = 0; i < Lines.size(); i++)
    {
        for (int j = i + 1; j < Lines.size(); j++)
        {
            double m1 = Lines[i][4], c1 = Lines[i][5];
            double m2 = Lines[j][4], c2 = Lines[j][5];

            if (m1 != m2)
            {
                double x0 = (c1 - c2) / (m2 - m1);
                double y0 = m1 * x0 + c1;

                double err = 0;
                for (int k = 0; k < Lines.size(); k++)
                {
                    double m = Lines[k][4], c = Lines[k][5];
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

LaneDistanceDetectorImpl::LaneDistanceDetectorImpl()
{
    settingsLoaded = LOAD_ERROR;
    // Constructor implementation
}

LaneDistanceDetectorImpl::~LaneDistanceDetectorImpl()
{
    // Destructor implementation
}

void LaneDistanceDetectorImpl::setLaneImageTemplate(const cv::Mat &image)
{
    laneTemplateImage_ = image.clone();
}

void LaneDistanceDetectorImpl::setDetectImage(const cv::Mat &image)
{
    laneTemplateImage_ = image.clone();
}

void LaneDistanceDetectorImpl::setCameraMatrix(const cv::Mat &cameraMatrix)
{
    cameraMatrix_ = cameraMatrix.clone();
}

void LaneDistanceDetectorImpl::setVanishingPoint(const cv::Point &vanishingPoint)
{
    vanishingPoint_ = vanishingPoint;
}

void LaneDistanceDetectorImpl::setPlaneParameters(const cv::Vec4d &planeParameters)
{
    planeParameters_ = planeParameters;
}

void LaneDistanceDetectorImpl::setVerticalLinePointPairs(const std::vector<std::pair<cv::Point, cv::Point>> &pointPairs)
{
    verticalLinePointPairs_ = pointPairs;
}

void LaneDistanceDetectorImpl::setVerticalLinePair(const std::pair<cv::Vec3d, cv::Vec3d> &lines)
{
    verticalLinePair_ = lines;
}

bool LaneDistanceDetectorImpl::calculateVanishingPoint()
{
    if (laneTemplateImage_.empty())
    {
        std::cerr << "模板图像未设置!" << std::endl;
        return false;
    }
    cv::Mat Image = laneTemplateImage_.clone();

    // Getting the lines from the image
    std::vector<std::vector<double>> Lines;
    Lines = GetLines(Image);

    // Get vanishing point
    int *VanishingPoint = GetVanishingPoint(Lines);

    // Checking if vanishing point found
    if (VanishingPoint[0] == -1 && VanishingPoint[1] == -1)
    {
        std::cout << "Vanishing Point not found. Possible reason is that not enough "
                     "lines are found in the image for determination of vanishing point."
                  << std::endl;
        return false;
    }
    else
    {
        setVanishingPoint(cv::Point(VanishingPoint[0], VanishingPoint[1]));
    }

#ifdef _DEBUG
    // Check if vanishing point is within image bounds
    if (VanishingPoint[0] < 0 || VanishingPoint[0] >= Image.cols || VanishingPoint[1] < 0 ||
        VanishingPoint[1] >= Image.rows)
    {
        // Calculate the required border extension based on vanishing point position
        int top = std::max(0, -VanishingPoint[1] + 20);
        int bottom = std::max(0, VanishingPoint[1] - Image.rows + 20);
        int left = std::max(0, -VanishingPoint[0] + 20);
        int right = std::max(0, VanishingPoint[0] - Image.cols + 20);

        // Extend image borders and fill with black color
        cv::Mat extendedImage;
        cv::copyMakeBorder(Image, extendedImage, top, bottom, left, right, cv::BORDER_CONSTANT, cv::Scalar(0, 0, 0));

        // Update vanishing point coordinates
        VanishingPoint[0] += left;
        VanishingPoint[1] += top;

        // Update Lines' coordinates
        for (auto &Line : Lines)
        {
            Line[0] += left;
            Line[1] += top;
            Line[2] += left;
            Line[3] += top;
        }

        // Drawing lines and vanishing point on extended image
        for (const auto &Line : Lines)
        {
            cv::line(extendedImage, cv::Point((int)Line[0], (int)Line[1]), cv::Point((int)Line[2], (int)Line[3]),
                     cv::Scalar(0, 255, 0), 2);
        }
        cv::circle(extendedImage, cv::Point(VanishingPoint[0], VanishingPoint[1]), 10, cv::Scalar(0, 0, 255), -1);
        /*cv::putText(extendedImage, "VanishingPoint", cv::Point(VanishingPoint[0] -
           10, VanishingPoint[1] + 10), cv::FONT_HERSHEY_SIMPLEX, 0.5, cv::Scalar(0,
           0, 255), 2);*/

        // Resize the extended image to fit the screen size
        cv::Mat resizedImage;
        double scaleFactor = std::min(1.0,
                                      1000.0 / extendedImage.cols); // Resize factor to fit within a 1000-pixel width
        cv::resize(extendedImage, resizedImage, cv::Size(), scaleFactor, scaleFactor);

        // Showing the final resized image
        cv::imshow("OutputImage", resizedImage);
        cv::waitKey(0);
    }
    else
    {
        // Drawing lines and vanishing point on original image
        for (const auto &Line : Lines)
        {
            cv::line(Image, cv::Point((int)Line[0], (int)Line[1]), cv::Point((int)Line[2], (int)Line[3]),
                     cv::Scalar(0, 255, 0), 2);
        }
        cv::circle(Image, cv::Point(VanishingPoint[0], VanishingPoint[1]), 10, cv::Scalar(0, 0, 255), -1);
        /*cv::putText(Image, "VanishingPoint", cv::Point(VanishingPoint[0]-500,
           VanishingPoint[1]+500), cv::FONT_HERSHEY_SIMPLEX, 5, cv::Scalar(255, 0,
           0), 10);*/
    }
#endif

    return true;
}

cv::Vec3d LaneDistanceDetectorImpl::calculatingParallelPlane()
{
    if (verticalLinePointPairs_.size() < 1)
    {
        // Handle the case when there are not enough point pairs
        return cv::Vec3d(0.0, 0.0, 0.0);
    }

    // Get the camera matrix values
    double fx = cameraMatrix_.at<double>(0, 0);
    double fy = cameraMatrix_.at<double>(1, 1);
    double u = cameraMatrix_.at<double>(0, 2);
    double v = cameraMatrix_.at<double>(1, 2);

    // Calculate the direction vector of the line passing through vanishingPoint_
    cv::Vec3d dirVanishing((vanishingPoint_.x - u) / fx, (vanishingPoint_.y - v) / fy, 1.0);

    // Retrieve the two sets of point pairs
    const std::pair<cv::Point, cv::Point> &pointPair1 = verticalLinePointPairs_[0];

    // Extract points from the point pairs
    const cv::Point &p1 = pointPair1.first;
    const cv::Point &p2 = pointPair1.second;


    // Calculate the direction vectors of the lines for the two point pairs
    cv::Vec3d dir1((p1.x - u) / fx, (p1.y - v) / fy, 1.0);
    cv::Vec3d dir2((p2.x - u) / fx, (p2.y - v) / fy, 1.0);

    std::pair<cv::Vec3d, cv::Vec3d> linePair(dir1, dir2);

    setVerticalLinePair(linePair);

    // Calculate the normal vectors of the planes formed by the point pairs
    cv::Vec3d normal1 = dir1.cross(dir2);
    // Normalize the normal vector
    normal1 /= cv::norm(normal1);

    cv::Vec3d normal2 = dirVanishing;
    normal2 /= cv::norm(normal2);

    // Calculate the coefficients of the plane equations Ax + By + Cz + D = 0
    double A1 = normal1[0];
    double B1 = normal1[1];
    double C1 = normal1[2];

    double A2 = normal2[0];
    double B2 = normal2[1];
    double C2 = normal2[2];

    // Calculate the direction vector of the intersection line of the two planes
    //cv::Vec3d intersectionLineDir = normal1.cross(normal2);
    cv::Vec3d intersectionLineDir(1.0, (-A1 / A2), (-B1 / B2));
    intersectionLineDir /= cv::norm(intersectionLineDir);
    // Calculate the coefficients of the plane equation Ax + By + Cz + D = 0
    double A = intersectionLineDir[0];
    double B = intersectionLineDir[1];
    double C = intersectionLineDir[2];

    // Calculate the coefficients of the plane formed by the intersection line and
    // dirVanishing plane
    cv::Vec3d finalPlaneCoefficients(A, B, C);

    cv::Vec3d normalplane = dirVanishing.cross(finalPlaneCoefficients);
    normalplane /= cv::norm(normalplane);

    // Store the normalplane as a Vec4d with a trailing 0.0 to represent D
    cv::Vec4d planeParameters(normalplane[0], normalplane[1], normalplane[2], 0.0);

    setPlaneParameters(planeParameters);

    return normalplane;
}

cv::Vec4d LaneDistanceDetectorImpl::calculatePlaneParameters()
{
    // Extract lines from the line pair
    cv::Vec3d line1 = verticalLinePair_.first;
    cv::Vec3d line2 = verticalLinePair_.second;

    // Calculate the direction vectors of the lines
    cv::Vec3d dir1(line1[0], line1[1], line1[2]);
    cv::Vec3d dir2(line2[0], line2[1], line2[2]);

    double A = planeParameters_[0];
    double B = planeParameters_[1];
    double C = planeParameters_[2];
    // Calculate the unit normal vector of the plane
    cv::Vec3d normal(A, B, C);
    double normalLength = cv::norm(normal);
    normal /= normalLength; // Normalize the normal vector

    // Calculate the distance between the two lines
    double distance = 5.7; // Desired distance between the two points

    // Calculate the D value of the plane such that the two points are at the desired distance
    double D = (A * dir1[0] + B * dir1[1] + C * dir1[2]) + distance * normalLength;

    // Store the updated D value in planeParameters_
    planeParameters_[3] = D;

    savePlaneParameters(planeParameters_);
    return planeParameters_;
}


cv::Vec3d LaneDistanceDetectorImpl::calculatePixel2Coordinate(const cv::Point& point)
{
    // Get the camera matrix values
    double fx = cameraMatrix_.at<double>(0, 0);
    double fy = cameraMatrix_.at<double>(1, 1);
    double u = cameraMatrix_.at<double>(0, 2);
    double v = cameraMatrix_.at<double>(1, 2);

    // Calculate the direction vectors of the lines for the two point pairs
    cv::Vec3d dir((point.x - u) / fx, (point.y - v) / fy, 1.0);

    // Get plane parameters
    double A = planeParameters_[0];
    double B = planeParameters_[1];
    double C = planeParameters_[2];
    double D = planeParameters_[3];

    // Calculate the intersection points between the lines and the plane
    double t = D / (A* dir[0] + B* dir[1] + C* dir[2]);
    cv::Vec3d intersectionPoint(dir[0] * t, dir[1] * t, dir[2] * t);

    return intersectionPoint;
}


double LaneDistanceDetectorImpl::calculateDistanceBetweenPoints(const cv::Point &point1, const cv::Point &point2)
{


    // Calculate the intersection points between the lines and the plane
    cv::Vec3d intersectionPoint1 = calculatePixel2Coordinate(point1);
    cv::Vec3d intersectionPoint2 = calculatePixel2Coordinate(point2);

    // Calculate the distance between the intersection points
    double distance = cv::norm(intersectionPoint1 - intersectionPoint2);

    return distance;
}

LOAD_RESULT_CODE LaneDistanceDetectorImpl::loadSettings(const std::string &path)
{
    LOAD_RESULT_CODE result = settings.load(path);
    switch (result)
    {
    case LOAD_PLANE_PARAMETERS:
        settingsLoaded = LOAD_PLANE_PARAMETERS;
        cameraMatrix_ = settings.TEMPLATE_CAMERA_MATRIX_;
        planeParameters_ = settings.TEMPLATE_PLANE_PARAMETERS_;
        break;

    case LOAD_PENDING:
        cameraMatrix_ = settings.TEMPLATE_CAMERA_MATRIX_;
        verticalLinePointPairs_.push_back(settings.TEMPLATE_VERTICAL_LINE_POINT_PAIR1_);
        settingsLoaded = LOAD_PENDING;
        break;

    case LOAD_MISSING_PLANE_PARAMETERS:
        settingsLoaded = LOAD_MISSING_PLANE_PARAMETERS;
        break;

    case LOAD_MISSING_CAMERA_MARTIX:
        settingsLoaded = LOAD_MISSING_CAMERA_MARTIX;
        break;

    case LOAD_MISSING_VERTICAL_LINE_POINT_PAIR:
        settingsLoaded = LOAD_MISSING_VERTICAL_LINE_POINT_PAIR;
        break;

    case LOAD_ERROR:
        settingsLoaded = LOAD_ERROR;
        break;

    default:
        settingsLoaded = LOAD_ERROR;
        break;
    }
    return result;
}

void LaneDistanceDetectorImpl::savePlaneParameters(cv::Vec4d planeParameters)
{
    if (settingsLoaded == LOAD_PENDING)
    {
        settings.savePlaneParameters(planeParameters_);
    }
}
