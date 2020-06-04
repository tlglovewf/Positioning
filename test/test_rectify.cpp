#include "P_Interface.h"
#include "P_Utils.h"
#include "P_Factory.h"
#include "P_Frame.h"
#include "project/newhwproject.h"

int main(void)
{
    std::shared_ptr<Position::IConfig>              pcfg(new Position::ImgAutoConfig("../config/config_new.yaml"));
    std::shared_ptr<Position::IData>                pdata(new Position::NewHwProjectData(pcfg));
    std::shared_ptr<Position::IFeature>             pfeature(Position::PFactory::CreateFeature(Position::eFeatureSift,pcfg));
    std::shared_ptr<Position::IFeatureMatcher>      pmatcher(Position::PFactory::CreateFeatureMatcher(Position::eFMKnnMatch,GETCFGVALUE(pcfg,MatchRatio,float)));
    std::shared_ptr<Position::IPoseEstimation>      ppose(Position::PFactory::CreatePoseEstimation(Position::ePoseEstCV));

    SETGLOBALCONFIG(pcfg);
    if(pdata->loadDatas())
    {
        Position::FrameDataPtrVIter first = pdata->begin() + 300;
        Position::FrameDataPtrVIter secnd = pdata->begin() + 302;

        const std::string imgpath = GETCFGVALUE(pcfg,ImgPath,string) + "/";

        (*first)->_img = imread(imgpath + (*first)->_name);
        (*secnd)->_img = imread(imgpath + (*secnd)->_name);

        Position::IFrame *pf1 = new Position::PFrame(*first,pfeature,0);
        Position::IFrame *pf2 = new Position::PFrame(*secnd,pfeature,1);

        Position::MatchVector matches = pmatcher->match(pf1,pf2,10);

        Position::CameraParam cam = pdata->getCamera();
        ppose->setCamera(cam);
        ppose->setFrames(pf1,pf2);
        Mat R;
        Mat t;
        Position::Pt3Vector p3ts;
        
        if(ppose->estimate(R,t,matches,p3ts))
        {
            Mat _R1(3,3,MATCVTYPE);
            Mat _R2(3,3,MATCVTYPE);
            Mat P1,P2,Q;
            
            cv::stereoRectify(cam.K,cam.D,cam.K,cam.D,(*first)->_img.size(),R,t,_R1,_R2,P1,P2,Q,CV_CALIB_ZERO_DISPARITY);

            cout << R   << endl;
            cout << _R1 << endl;
            cout << _R2 << endl;

            Mat x_map1,x_map2;
            Mat y_map1,y_map2;

            cv::initUndistortRectifyMap(cam.K,cam.D,R,cam.K,(*first)->_img.size(),CV_32FC1,x_map1,y_map1);

            Mat oimg1;
            cv::remap((*first)->_img,oimg1,x_map1,y_map1,cv::INTER_LINEAR);

            cv::initUndistortRectifyMap(cam.K,cam.D,_R1,cam.K,(*first)->_img.size(),CV_32FC1,x_map2,y_map2);
            Mat oimg2;
            cv::remap((*secnd)->_img,oimg2,x_map2,y_map2,cv::INTER_LINEAR);

            imwrite("/media/tu/Work/Datas/newdata/" + (*first)->_name,(*first)->_img);
            imwrite("/media/tu/Work/Datas/newdata/" + (*secnd)->_name,(*secnd)->_img);

            imwrite("/media/tu/Work/Datas/newdata/o" + (*first)->_name,oimg1);
            imwrite("/media/tu/Work/Datas/newdata/o" + (*secnd)->_name,oimg2);
        }

    }


    return 0;
}