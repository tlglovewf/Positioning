
#include "project/imgautoproject.h"
#include "P_Writer.h"

#include  "Thirdparty/rapidjson/document.h"

namespace Position
{
    ImgAutoConfig::ImgAutoConfig(const std::string &path):
          PrjPath(""),
          CamMatrixPath("")
    {
        PUSH_MAP(PrjPath);
        PUSH_MAP(CamMatrixPath);

        load(path);   
    }

    //其他信息加载
    void ImgAutoConfig::loadmore() 
    {
        READ_VALUE(PrjPath);
        READ_VALUE(CamMatrixPath);
    }

    void ImgAutoData::loadCameraParams(const std::string &path)
    {
        //read camera matrix
        FileStorage intr(path, FileStorage::READ);
        if (intr.isOpened())
	    {
	    	intr["P1"] >> mCamera.K;
	    	if (mCamera.K.cols > 3)
	    	{
	    		mCamera.K = mCamera.K.colRange(0, 3);
	    	}
            if(mCamera.K.type() != MATCVTYPE)
            {
                mCamera.K.convertTo(mCamera.K,MATCVTYPE);
                mCamera.RCam2Imu.convertTo(mCamera.RCam2Imu,MATCVTYPE);
                mCamera.TCam2Imu.convertTo(mCamera.TCam2Imu,MATCVTYPE);
            }
            mCamera.fps = GETCFGVALUE(mpCfg,ImgFps,int);
            mCamera.rgb = GETCFGVALUE(mpCfg,ImgRgb,int);
            PROMT_S("Camera params >>>>>>>>>>>>>>>>>>>>>>>>>");
            PROMT_V("K ",mCamera.K);
            PROMT_V("Fps ",(int)mCamera.fps);
            PROMT_V("Rgb ",(int)mCamera.rgb);
            PROMT_S("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
            //add
	    }
	    else
	    {
            PROMT_S("Read camera config failed!!!")
	    }

#if 0  //read camera ex paramers
        FileStorage bs  (bspath, FileStorage::READ);
	    if (bs.isOpened())
	    {
	    	bs["RotateMatrixCam2IMU"] >> mCamera.RCam2Imu;
	    	bs["TranslationVectorCam2IMU"] >> mCamera.TCam2Imu;
            PROMT_S("Camera params >>>>>>>>>>>>>>>>>>>>>>>>>");
            PROMT_V("Cam2ImuR",mCamera.RCam2Imu);
            PROMT_V("Cam2ImuT",mCamera.TCam2Imu);
            PROMT_S("<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<<");
	    }
	    else
	    {
            PROMT_S("Read config failed!!!")
	    }
#endif 
    }

     //加载track json
    bool ImgAutoData::loadTrackJson(const std::string &path,FrameDataVector &framedatas)
    {
        if(path.empty() || framedatas.empty())
            return false;
        ifstream json_file;
	    json_file.open(path.c_str());
	    string line, json;
	    if (!json_file.is_open())
	    {
            PROMT_S("Error opening track json file");
	    	return 0;
	    }
	    while (!json_file.eof())
	    {
            getline(json_file, line);
	    	json.append(line);
	    }
	    json_file.close();

        rapidjson::Document doc;
        doc.Parse<0>(json.c_str());

        if (doc.HasMember("errmsg"))
        {
            rapidjson::Value &errmsgValue = doc["errmsg"];
            string errmsg = errmsgValue.GetString();
            if (errmsg != "OK")
            {
                PROMT_S("Json File Error! Check if the file is available!")
                return 0;
            }
            else
            {
                if (doc.HasMember("data"))
                {
                    rapidjson::Value &dataArray = doc["data"];
                    if (dataArray.IsArray())
                    {
                        rapidjson::Value& dataValue = dataArray[0];
                        
                        if (dataValue.HasMember("trackMark"))
                        {
                            rapidjson::Value &trmkArray = dataValue["trackMark"];
                            
                            if (trmkArray.IsArray())
                            {
                                cout << "trmk : " << trmkArray.Size()  << endl;
                                cout << "fmdt : " << framedatas.size() << endl;
                                if(trmkArray.Size() != framedatas.size() )
                                {
                                    PROMT_S("image seq size and trkarray size are not same!");
                                    return false;
                                }
                                for (int j = 0; j < trmkArray.Size(); j++)
                                {
                                    PoseData &item = framedatas[j]._pos;
                                    int mseq;
                                    rapidjson::Value& trmkValue = trmkArray[j];
                                    if (trmkValue.HasMember("latitude"))
                                    {
                                        item.pos.lat = trmkValue["latitude"].GetDouble();
                                    }
                                    if (trmkValue.HasMember("longitude"))
                                    {
                                        item.pos.lon = trmkValue["longitude"].GetDouble();
                                    }
                                    if (trmkValue.HasMember("seqNum"))
                                    {
                                         mseq = atoi(trmkValue["seqNum"].GetString()) - 1;
                                         if(mseq != j)
                                         {
                                             PROMT_V(framedatas[j]._name,"seq numb error. ")
                                         }
                                    }
                                }
                            }
                        }
                        else
                        {
                            PROMT_S("Json File has no trackMark!");
                            return false;
                        }	
                    }
                }
                else
                {
                    PROMT_S("Json File has no data!");
                    return false;
                }
                return true;
            }
        }
        return false;
    }

    //预处理数据
    bool ImgAutoData::loadDatas()
    {
        assert(mpCfg);
        //get path 
        const std::string expath        = GETCFGVALUE(mpCfg,PrjPath,string);

        if(expath.empty())
            return false;

        const std::string raw_data      = expath    + "/raw_data";
        const std::string seqpath       = raw_data  + "/seq.txt";
        const std::string trkjson       = raw_data  + "/track.json";
        const std::string imgpath       = raw_data  + "/image/";
        const std::string trackerpath   = expath    + "tracker";
        const std::string campath       = expath    + "config/extrinsics.xml";

        //load camera paramers
        loadCameraParams(campath);

        try 
        {
            ifstream seqfile;
            seqfile.open(seqpath);
            if(!seqfile.is_open())
            {
                return false;
            }
            //区文件名
            FrameDataVector  framedatas;
            while(!seqfile.eof())
            {
                std::string seqline;
                getline(seqfile,seqline);
                if(seqline.empty())
                    continue;
                FrameData fdata;
                size_t npos = seqline.find_last_of("/");
                fdata._name = seqline.substr(++npos);
                framedatas.push_back(fdata);
            }
            seqfile.close();

            if(!loadTrackJson(trkjson,framedatas))
                return false;

             const std::string strori = "/media/tlg/work/tlgfiles/HDData/result/ori.txt";
            std::ofstream fori;
            fori.open(strori);
            for(int i = 0; i < framedatas.size(); ++i )
            {
                PStaticWriter::WriteRealTrace(fori, framedatas[i]._pos.pos ,framedatas[i]._name);
            }
            

            fori.close();

        }catch(exception e)
        {
            PROMT_V("error",e.what());
        }
      
        //load frame datas



        return true;
    }
}
