#include <iostream>
#include <opencv2/opencv.hpp>
#include "data_reader.h"
#include "types.h"
#include "common_include.h"
#include "config.h"


using namespace std;
using namespace cv;
using namespace MSCKF_MINE;


int main(int argc, char *argv[])
{

    Config::setParameterFile("../config/config.yaml");
    DataReader data;
    vector<CAMERA> vCamera = data.mvCameraData;
    int maxCorners = 300;
    double qualityLevel = 0.01;
    double minDistance = 10;
    int blockSize = 10;
    bool useHarrisDetector = false;
    double k = 0.04;

    Mat feedImage, currentImage;
    bool needFeedImage = true;
    vector<Point2f> corners;
    vector<Point2f> corners_after;

    for(vector<CAMERA>::iterator iter_cam = vCamera.begin(); iter_cam!=vCamera.end(); iter_cam++)
    {
        Mat image = imread(iter_cam->img_name,CV_LOAD_IMAGE_GRAYSCALE);

        /// Parameters for Shi-Tomasi algorithm



        if(corners_after.size()< 100)
        {
            corners.clear();

            feedImage = image;
            goodFeaturesToTrack( image,
                         corners,
                         maxCorners,
                         qualityLevel,
                         minDistance,
                         Mat(),
                         blockSize,
                         useHarrisDetector,
                         k );

            /// Draw corners detected
            cout<<"** Number of corners detected: "<<corners.size()<<endl;
            cvtColor(image,image,CV_GRAY2BGR);
            int r = 4;
            for( int i = 0; i < corners.size(); i++ )
               { circle( image, corners[i], r, Scalar(0,255,0), 1, 8, 0 ); }



            imshow("current frame", image);
            waitKey(0);
        }
        else
        {
            vector<uchar> status;
            vector<float> err;
            currentImage = image;
            calcOpticalFlowPyrLK(feedImage,currentImage,corners,corners_after, status, err);

            for(int i=0;i<corners_after.size();i++)
            {
                //状态要是1，并且坐标要移动下的那些点
                if(status[i]&&((abs(corners_after[i].x-corners_after[i].x)+
                    abs(corners_after[i].y-corners_after[i].y))>3))
                {
                    corners_after[k++] = corners_after[i];
                }
            }
            corners_after.resize(k);//截取

        }

    }




    return 0;
}

