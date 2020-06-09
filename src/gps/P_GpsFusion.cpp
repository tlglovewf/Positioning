#include "P_GpsFusion.h"
#include "GpsGlobalMap.h"
#include "GpsSim3Optimizer.h"
#include "P_Converter.h"
#include "P_IOHelper.h"
#include "Thirdparty/GeographicLib/include/LocalCartesian.hpp"
#include "P_Utils.h"
namespace  Position
{   


    //gps fuse
    bool  GpsFunsion::fuse(const std::shared_ptr<IMap> &pmap, const CameraParam &cam)
    {
        assert(pmap);
        GeographicLib::LocalCartesian geoConverter;
        gps::GpsGlobalMap map;
        GpsSim3Optimizer sim3opt;

        KeyFrameVector vpkfs = pmap->getAllFrames();
        
        
        LOG_INFO("Gps fuse Begin.");
        Time_Interval time;
        time.start();
        //只为测试
        const std::string opath = GETCFGVALUE(GETGLOBALCONFIG(),OutPath,string);
        const std::string strori = opath + "/ori.txt";
        const std::string strfus = opath + "/est.txt";
        cout << "Gps Save to: " << opath.c_str() << endl;
        std::ofstream fori;
        fori.open(strori);

        std::ofstream ffuse;
        ffuse.open(strfus);

        for(size_t i = 0; i <vpkfs.size(); ++i)
        {
            IKeyFrame *pkf = vpkfs[i];

            if(pkf->isBad())
            {
                continue;
            }
            Mat Trw = pkf->getPose();
            Mat Rwc = Trw.rowRange(0,3).colRange(0,3).t();
            Mat twc = pkf->getCameraCenter();  -Rwc * Trw.rowRange(0,3).col(3);

            Eigen::Vector3d posi(twc.at<MATTYPE>(0),
                                 twc.at<MATTYPE>(1),
                                 twc.at<MATTYPE>(2));

            Eigen::Quaterniond rot(PConverter::toMatrix3d(Rwc));
            std::shared_ptr<gps::GpsFrame> frame_p = std::make_shared<gps::GpsFrame>();
            frame_p->time_stamp = pkf->getData()->_pos._t;

            frame_p->fx = cam.K.at<MATTYPE>(0,0);
            frame_p->fy = cam.K.at<MATTYPE>(1,1);
            frame_p->cx = cam.K.at<MATTYPE>(2,0);
            frame_p->cy = cam.K.at<MATTYPE>(2,1);

            frame_p->position=posi;

            frame_p->direction.w() = rot.w();
            frame_p->direction.x() = rot.x();
            frame_p->direction.y() = rot.y();
            frame_p->direction.z() = rot.z();

            frame_p->kps= pkf->getKeys();

            for(int j=0; j<  pkf->getKeys().size(); j++)
            {
                frame_p->obss.push_back(nullptr);
            }
            frame_p->id= pkf->index();
            
            GPSData gps;
            gps.lat = pkf->getData()->_pos.pos.lat;
            gps.lon = pkf->getData()->_pos.pos.lon;
            gps.alt = pkf->getData()->_pos.pos.alt;
            gps.t   = pkf->getData()->_pos._t;
            gps.conf = 1;

            MATTYPE xyz[3];
            static bool initGps = false;
            if(!initGps)
            {
                geoConverter.Reset(gps.lat,gps.lon,gps.alt);
                sim3opt.geoConverter.Reset(gps.lat,gps.lon,gps.alt);
                initGps = true;
            }
            geoConverter.Forward(gps.lat,gps.lon,gps.alt,xyz[0],xyz[1],xyz[2]);

            Eigen::Vector3d gps_position(xyz[0],xyz[1],xyz[2]);

            frame_p->gps_position = gps_position;
            frame_p->gps_accu     = 1;

            map.frames.push_back(frame_p);
            BLHCoordinate blh;        

            blh.lon = gps.lon;
            blh.lat = gps.lat;
            blh.alt = gps.alt;
            PStaticWriter::WriteRealTrace(fori,blh,pkf->getData()->_name);
        }


        //遍历所有的地图点,将所有地图点存入GpsGlobalMap::mappoints
        MapPtVector vpMPs = pmap->getAllMapPts();
        for(int i=0; i<vpMPs.size(); i++)
        {
            IMapPoint* mp = vpMPs[i];
            if (mp->isBad())
            {
                continue;
            }
            std::shared_ptr<gps::GpsMapPoint> mappoint_p=std::make_shared<gps::GpsMapPoint>();
            cv::Mat mp_posi_cv=vpMPs[i]->getWorldPos();
            Eigen::Vector3d posi(mp_posi_cv.at<MATTYPE>(0),mp_posi_cv.at<MATTYPE>(1),mp_posi_cv.at<MATTYPE>(2));
            mappoint_p->position=posi;//坐标
            mappoint_p->id=vpMPs[i]->index();//id
            map.mappoints.push_back(mappoint_p);
        }

        for(int i=0; i<vpkfs.size(); i++)
        {
            IKeyFrame* pKF = vpkfs[i];
            if(pKF->isBad())
            {
                continue;
            }
            if(pKF->index() != map.frames[i]->id)
            {
                LOG_ERROR("Gps fuse:pKF->mnId!=frame_p->id");
                return false;
            }

            for(int j = 0; j < pKF->getWorldPoints().size(); j++)
            {
                if(pKF->getWorldPoints()[j] != NULL)
                {
                    for(int k=0; k<map.mappoints.size(); k++)
                    {
                        if(map.mappoints[k]->id == pKF->getWorldPoints()[j]->index())
                        {
                            map.frames[i]->obss[j]=map.mappoints[k];
                            break;
                        }
                    }
                }
            }

        }

        sim3opt.setOrbMap(map);

        sim3opt.beginOpt();

        time.prompt("Gps fuse cost");
        LOG_INFO("Gps Fuse End.");


        for(size_t i = 0; i <vpkfs.size(); ++i)
        {
            
            double lon,lat,alt;
            double xyz[3] = {0};
            sim3opt.getFrameXYZ(i,xyz[0],xyz[1],xyz[2]);
            geoConverter.Reverse(xyz[0], xyz[1],xyz[2],lat,lon,alt);
           
            BLHCoordinate blh;
            blh.lon = lon;
            blh.lat = lat;
            blh.alt = alt;
            PStaticWriter::WriteEstTrace(ffuse,blh,Point3d(0,0,0), vpkfs[i]->getData()->_name);
        }


        return true;
    }
}