#include "P_FrameViewer.h"

namespace Position
{
    PFrameViewer::PFrameViewer():mStatus(eTrackNoImage),mTrackPts(0)
    {

    }

    void PFrameViewer::drawPts( IKeyFrame *pkf, Mat &im)
    {
        assert(pkf);
        size_t N = pkf->getKeys().size();
        const MapPtVector &pts = pkf->getWorldPoints(); 
        const KeyPtVector keypts = pkf->getKeys();
        assert(N == pts.size());

        const float r           = 8.0;
        const float thickness   = 2.0;
        const int   sz          = 3;
        mTrackPts = 0;
        for(size_t i = 0; i < N ; ++i )
        {
            cv::Point2f pt1,pt2,pt3,pt4;
     
            pt1.x = keypts[i].pt.x;
            pt1.y = keypts[i].pt.y + r;

            pt2.x = keypts[i].pt.x;
            pt2.y = keypts[i].pt.y - r;

            pt3.x = keypts[i].pt.x + r;
            pt3.y = keypts[i].pt.y;

            pt4.x = keypts[i].pt.x - r;
            pt4.y = keypts[i].pt.y;

            if(pts[i])
            {
                cv::line(im,pt1,pt2,cv::Scalar(0,255,0),thickness);
                cv::line(im,pt3,pt4,cv::Scalar(0,255,0),thickness);
                cv::circle(im,keypts[i].pt,sz,cv::Scalar(0,255,0),-1);
                ++mTrackPts;
            }
        }

    }

    //绘制帧
    void PFrameViewer::drawFrame()
    {
        assert(mMap);
        IKeyFrame *scurrent = NULL;
        IKeyFrame *pcurrent = mMap->currentKeyFrame();;
        Mat oimg;
        if(scurrent != pcurrent)
        {
            Mat img = pcurrent->getData()->_img;
            if(img.empty())
                return;
            drawKeypoints(img, pcurrent->getKeys(),oimg,CV_RGB(0,0,255));

            drawPts(pcurrent,oimg);

            resize(oimg, oimg, Size(img.cols >> 2, img.rows >> 2));

            Mat outi;
            drawFrameText(oimg,mStatus,outi);
            
            imshow("Frame",outi);
            cv::waitKey(1);
            scurrent = pcurrent;
        }
    }


    void PFrameViewer::drawFrameText(Mat &img, eTrackStatus status, Mat &imText)
    {
        stringstream s;
        // if (mStatus == eTrackNoImage)
        //     s << "LOAD VOC FILE, WAITING FOR IMAGES";
        // else if (mStatus == eTrackNoReady)
        //     s << " TRYING TO INITIALIZE ";
        // else if (mStatus == eTrackOk)
        {
            int nKFs = mMap->keyFrameInMap(); 
            int nMPs = mMap->mapPointsInMap();
            const IKeyFrame *curfm = mMap->currentKeyFrame();
            s << "Pic: " << curfm->getData()->_name.c_str() << " ";
            s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mTrackPts;
        }
        // else if (mStatus == eTrackLost)
        // {
        //     s << " TRACK LOST. TRYING TO RELOCALIZE ";
        // }
        // else
        // {
        //     ;
        // }

        int baseline = 0;
        cv::Size textSize = cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

        resize(img,imText,Size(img.cols,img.rows + textSize.height));
        imText(Rect2i(0,img.rows,img.cols,textSize.height)).setTo(0);
        cv::putText(imText, s.str(), cv::Point(5, imText.rows ), cv::FONT_HERSHEY_PLAIN, 1, CV_RGB(255,0,0), 1, 8);
    }

} // namespace Position