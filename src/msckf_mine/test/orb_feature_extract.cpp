#include "common_include.h"
#include "camera.h"
#include "config.h"
#include "msckf.h"
#include "data_reader.h"
#include "ORBextractor.h"
#include <vector>

using namespace MSCKF_MINE;



cv::Mat showFeatures(const cv::Mat &mImage, const vector<KeyPoint> &mvKeys)
{
    cv::Mat imageWithFeatures = mImage.clone();
    cv::cvtColor(imageWithFeatures, imageWithFeatures, CV_GRAY2BGR);
    for(vector<KeyPoint>::const_iterator iter= mvKeys.begin(); iter!=mvKeys.end(); iter++ )
    {
        KeyPoint point = *iter;
        cv::circle(imageWithFeatures, Point(point.pt.x, point.pt.y), 3, Scalar(0,255,0), 1);
    }

    return imageWithFeatures.clone();

}


int main(int argc, char *argv[])
{
    Config::setParameterFile("../config/config.yaml");

    MSCKF msckf;

    Mat image = cv::imread("/home/m/ws/src/msckf_mine/datasets/MH_01_easy/mav0/cam0/data/1403636579763555584.png", CV_LOAD_IMAGE_GRAYSCALE);
    msckf.mImage = image.clone();
    msckf.extractFeatures();

    cv::Mat imFeature = showFeatures(msckf.mImage, msckf.mvKeys);

    cv::imshow("features", imFeature);

    cv::waitKey(0);

    return 0;
}
