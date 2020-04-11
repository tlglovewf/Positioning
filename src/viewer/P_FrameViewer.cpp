#include "P_FrameViewer.h"

namespace Position
{
    PFrameViewer::PFrameViewer():mStatus(eTrackNoImage)
    {

    }

    //绘制帧
    void PFrameViewer::drawFrame()
    {
        assert(mMap);
        IKeyFrame *scurrent = NULL;
        IKeyFrame *pcurrent = mMap->currentKeyFrame();;
        if(scurrent != pcurrent)
        {
            Mat img = pcurrent->getData()._img;
            Mat oimg;
            drawKeypoints(img, pcurrent->getKeys(),oimg,CV_RGB(0,0,255));
            
            scurrent = pcurrent;
        }
    }


    void PFrameViewer::drawFrameText(Mat &img, eTrackStatus status, Mat &imText)
    {
        stringstream s;
        if (mStatus == eTrackNoImage)
            s << "LOAD VOC FILE, WAITING FOR IMAGES";
        else if (mStatus == eTrackNoReady)
            s << " TRYING TO INITIALIZE ";
        else if (mStatus == eTrackOk)
        {
            // int nKFs = mMap->keyFrameInMap(); 
            // int nMPs = mMap->mapPointsInMap();
            // const IKeyFrame *curfm = mMap->currentKeyFrame();
            // s << "Pic: " << curfm->getData()._name.c_str() << " ";
            // s << "KFs: " << nKFs << ", MPs: " << nMPs << ", Matches: " << mnTracked;
            // if (mnTrackedVO > 0)
            //     s << ", + VO matches: " << mnTrackedVO;
        }
        else if (mStatus == eTrackLost)
        {
            s << " TRACK LOST. TRYING TO RELOCALIZE ";
        }
        else
        {
            ;
        }

        int baseline = 0;
        cv::Size textSize = cv::getTextSize(s.str(), cv::FONT_HERSHEY_PLAIN, 1, 1, &baseline);

        imText = cv::Mat(img.rows + textSize.height + 10, img.cols, img.type());
        img.copyTo(imText.rowRange(0, img.rows).colRange(0, img.cols));
        imText.rowRange(img.rows, imText.rows) = cv::Mat::zeros(textSize.height + 10, img.cols, img.type());
        cv::putText(imText, s.str(), cv::Point(5, imText.rows - 5), cv::FONT_HERSHEY_PLAIN, 1, cv::Scalar(255, 255, 255), 1, 8);
    }

} // namespace Position