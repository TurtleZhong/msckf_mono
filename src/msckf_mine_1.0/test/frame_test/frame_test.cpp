#include "common_include.h"
#include "camera.h"
#include "config.h"
#include "msckf.h"
#include "data_reader.h"
#include "frame.h"
#include <vector>

using namespace MSCKF_MINE;

Mat ShowFeatures(Frame &frame);

int main(int argc, char *argv[])
{
    Config::setParameterFile("../config/config.yaml");

    MSCKF msckf;

    DataReader data;
    vector<CAMERA> vCameraData = data.mvCameraData;

    for(vector<CAMERA>::iterator iter = vCameraData.begin(); iter!=vCameraData.end();iter++)
    {
        string imagePath = iter->img_name;
        Mat image = cv::imread(imagePath,CV_LOAD_IMAGE_GRAYSCALE);

        msckf.imageComing(image,iter->time_stamp);

        cout <<BOLDRED << "frame id = " << msckf.mLastFrame.mnId << endl;

        cout << msckf.mvFeatureContainer.size() << endl;

        /*the 15th feature information*/
        cout << "Information of 15th feature" << endl;
        cout << "Feature ID: " << msckf.mvFeatureContainer[14].mnId << endl;
        cout <<BOLDGREEN << "Frame ID:"<< msckf.mvFeatureContainer[14].mnFrameId <<WHITE <<  endl;
        Feature &feature = msckf.mvFeatureContainer[14];
        for(int i = 0; i < feature.mvObservation.size(); i++)
        {
            cout << "The " << i << "th observation\n" <<feature.mvObservation[i] << endl;
        }

        Mat imFeature = ShowFeatures(msckf.mLastFrame);

        /*information of msckf and frame*/

        cv::imshow("features", imFeature);
        cv::waitKey(0);

    }

    return 0;
}

Mat ShowFeatures(Frame &frame)
{
    vector<Point2f> &oldcorners = frame.mvOldCorners;
    vector<Point2f> &newcorners = frame.mvNewCorners;
    Mat image = frame.mImgGray;
    cvtColor(image,image,CV_GRAY2BGR);
    for(int i = 0; i < oldcorners.size(); i++)
    {
        cv::circle( image, oldcorners[i], 3, Scalar(0,0,255), -1, 8, 0 );
    }
    for(int i = 0; i < newcorners.size(); i++)
    {
        cv::circle( image, newcorners[i], 3, Scalar(0,255,0), -1, 8, 0 );
    }
    return image;
}
