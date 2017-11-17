/**
* This file is part of ORB-SLAM2.
*
* Copyright (C) 2014-2016 Raúl Mur-Artal <raulmur at unizar dot es> (University of Zaragoza)
* For more information see <https://github.com/raulmur/ORB_SLAM2>
*
* ORB-SLAM2 is free software: you can redistribute it and/or modify
* it under the terms of the GNU General Public License as published by
* the Free Software Foundation, either version 3 of the License, or
* (at your option) any later version.
*
* ORB-SLAM2 is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE. See the
* GNU General Public License for more details.
*
* You should have received a copy of the GNU General Public License
* along with ORB-SLAM2. If not, see <http://www.gnu.org/licenses/>.
*/

#include "ORBmatcher.h"

#include<limits.h>

#include<opencv2/core/core.hpp>
#include<opencv2/features2d/features2d.hpp>


#include<stdint-gcc.h>

using namespace std;

namespace MSCKF_MINE
{

const int ORBmatcher::TH_HIGH = 100;
const int ORBmatcher::TH_LOW = 50;
const int ORBmatcher::HISTO_LENGTH = 30;

ORBmatcher::ORBmatcher(float nnratio, bool checkOri): mfNNratio(nnratio), mbCheckOrientation(checkOri)
{
}


float ORBmatcher::RadiusByViewingCos(const float &viewCos)
{
    if(viewCos>0.998)
        return 2.5;
    else
        return 4.0;
}



int ORBmatcher::MatcheTwoFrames(Frame &CurrentFrame, const Frame &LastFrame, const bool bMono)
{
    int nmatches = 0;

    // Rotation Histogram (to check rotation consistency)
    vector<int> rotHist[HISTO_LENGTH];
    for(int i=0;i<HISTO_LENGTH;i++)
        rotHist[i].reserve(500);
    const float factor = 1.0f/HISTO_LENGTH;

    for(int i=0; i<LastFrame.N; i++)
    {



        float u = LastFrame.mvKeys[i].pt.x;
        float v = LastFrame.mvKeys[i].pt.y;

        if(u<CurrentFrame.mnMinX || u>CurrentFrame.mnMaxX)
            continue;
        if(v<CurrentFrame.mnMinY || v>CurrentFrame.mnMaxY)
            continue;

        // Search in a window. Size depends on scale
        float radius = 50;

        vector<size_t> vIndices2;

        vIndices2 = CurrentFrame.GetFeaturesInArea(u,v, radius, 0, 7);

        if(vIndices2.empty())
            continue;

        const cv::Mat dMP = LastFrame.mDescriptors.row(i);

        int bestDist = 256;
        int bestIdx2 = -1;

        for(vector<size_t>::const_iterator vit=vIndices2.begin(), vend=vIndices2.end(); vit!=vend; vit++)
        {
            const size_t i2 = *vit;

            const cv::Mat &d = CurrentFrame.mDescriptors.row(i2);

            const int dist = DescriptorDistance(dMP,d);

            if(dist<bestDist)
            {
                bestDist=dist;
                bestIdx2=i2;
            }
        }

        if(bestDist<=TH_HIGH)
        {
            //CurrentFrame.mvpMapPoints[bestIdx2]=pMP;
            /*add by zhong*/
            CurrentFrame.matchesId[bestIdx2] = i;
            nmatches++;
            //cout << "bestDist = " << bestDist << " ";



            if(mbCheckOrientation)
            {
                float rot = LastFrame.mvKeysUn[i].angle-CurrentFrame.mvKeysUn[bestIdx2].angle;
                if(rot<0.0)
                    rot+=360.0f;
                int bin = round(rot*factor);
                if(bin==HISTO_LENGTH)
                    bin=0;
                assert(bin>=0 && bin<HISTO_LENGTH);
                rotHist[bin].push_back(bestIdx2);
            }
        }

    }

    //Apply rotation consistency
        if(mbCheckOrientation)
        {
            int ind1=-1;
            int ind2=-1;
            int ind3=-1;

            ComputeThreeMaxima(rotHist,HISTO_LENGTH,ind1,ind2,ind3);

            for(int i=0; i<HISTO_LENGTH; i++)
            {
                if(i!=ind1 && i!=ind2 && i!=ind3)
                {
                    for(size_t j=0, jend=rotHist[i].size(); j<jend; j++)
                    {
                        /*add by zhong*/
                        CurrentFrame.matchesId.erase(rotHist[i][j]);
                        nmatches--;

                    }
                }
            }
        }

    return nmatches;
}


void ORBmatcher::ComputeThreeMaxima(vector<int>* histo, const int L, int &ind1, int &ind2, int &ind3)
{
    int max1=0;
    int max2=0;
    int max3=0;

    for(int i=0; i<L; i++)
    {
        const int s = histo[i].size();
        if(s>max1)
        {
            max3=max2;
            max2=max1;
            max1=s;
            ind3=ind2;
            ind2=ind1;
            ind1=i;
        }
        else if(s>max2)
        {
            max3=max2;
            max2=s;
            ind3=ind2;
            ind2=i;
        }
        else if(s>max3)
        {
            max3=s;
            ind3=i;
        }
    }

    if(max2<0.1f*(float)max1)
    {
        ind2=-1;
        ind3=-1;
    }
    else if(max3<0.1f*(float)max1)
    {
        ind3=-1;
    }
}


// Bit set count operation from
// http://graphics.stanford.edu/~seander/bithacks.html#CountBitsSetParallel
int ORBmatcher::DescriptorDistance(const cv::Mat &a, const cv::Mat &b)
{
    const int *pa = a.ptr<int32_t>();
    const int *pb = b.ptr<int32_t>();

    int dist=0;

    for(int i=0; i<8; i++, pa++, pb++)
    {
        unsigned  int v = *pa ^ *pb;
        v = v - ((v >> 1) & 0x55555555);
        v = (v & 0x33333333) + ((v >> 2) & 0x33333333);
        dist += (((v + (v >> 4)) & 0xF0F0F0F) * 0x1010101) >> 24;
    }

    return dist;
}

} //namespace ORB_SLAM
