#include "P_Converter.h"
#include "P_Utils.h"
namespace Position
{
    std::vector<cv::Mat> PConverter::toDescriptorVector(const cv::Mat &Descriptors)
    {
        std::vector<cv::Mat> vDesc;
        vDesc.reserve(Descriptors.rows);
        for (int j=0;j<Descriptors.rows;j++)
            vDesc.push_back(Descriptors.row(j));

        return vDesc;
    }

    g2o::SE3Quat PConverter::toSE3Quat(const cv::Mat &cvT)
    {
        Eigen::Matrix<double,3,3> R;
        R << cvT.at<MATTYPE>(0,0), cvT.at<MATTYPE>(0,1), cvT.at<MATTYPE>(0,2),
             cvT.at<MATTYPE>(1,0), cvT.at<MATTYPE>(1,1), cvT.at<MATTYPE>(1,2),
             cvT.at<MATTYPE>(2,0), cvT.at<MATTYPE>(2,1), cvT.at<MATTYPE>(2,2);

        Eigen::Matrix<double,3,1> t(cvT.at<MATTYPE>(0,3), cvT.at<MATTYPE>(1,3), cvT.at<MATTYPE>(2,3));

        return g2o::SE3Quat(R,t);
    }

    cv::Mat PConverter::toCvMat(const g2o::SE3Quat &SE3)
    {
        Eigen::Matrix<double,4,4> eigMat = SE3.to_homogeneous_matrix();

        return toCvMat(eigMat);
    }

    cv::Mat PConverter::toCvMat(const g2o::Sim3 &Sim3)
    {
        Eigen::Matrix3d eigR = Sim3.rotation().toRotationMatrix();
        Eigen::Vector3d eigt = Sim3.translation();
        double s = Sim3.scale();
        return toCvSE3(s*eigR,eigt);
    }

    cv::Mat PConverter::toCvMat(const Eigen::Matrix<double,4,4> &m)
    {
        cv::Mat cvMat(4,4,MATCVTYPE);
        for(int i=0;i<4;i++)
            for(int j=0; j<4; j++)
                cvMat.at<MATTYPE>(i,j)=m(i,j);

        return cvMat.clone();
    }

    cv::Mat PConverter::toCvMat(const Eigen::Matrix3d &m)
    {
        cv::Mat cvMat(3,3,MATCVTYPE);
        for(int i=0;i<3;i++)
            for(int j=0; j<3; j++)
                cvMat.at<MATTYPE>(i,j)=m(i,j);

        return cvMat.clone();
    }

    cv::Mat PConverter::toCvMat(const Eigen::Matrix<double,3,1> &m)
    {
        cv::Mat cvMat(3,1,MATCVTYPE);
        for(int i=0;i<3;i++)
                cvMat.at<MATTYPE>(i)=m(i);

        return cvMat.clone();
    }

    cv::Mat PConverter::toCvSE3(const Eigen::Matrix<double,3,3> &R, const Eigen::Matrix<double,3,1> &t)
    {
        cv::Mat cvMat = cv::Mat::eye(4,4,MATCVTYPE);
        for(int i=0;i<3;i++)
        {
            for(int j=0;j<3;j++)
            {
                cvMat.at<MATTYPE>(i,j)=R(i,j);
            }
        }
        for(int i=0;i<3;i++)
        {
            cvMat.at<MATTYPE>(i,3)=t(i);
        }

        return cvMat.clone();
    }

    Eigen::Matrix<double,3,1> PConverter::toVector3d(const cv::Mat &cvVector)
    {
        Eigen::Matrix<double,3,1> v;
        v << cvVector.at<MATTYPE>(0), cvVector.at<MATTYPE>(1), cvVector.at<MATTYPE>(2);

        return v;
    }

    Eigen::Matrix<double,3,1> PConverter::toVector3d(const cv::Point3f &cvPoint)
    {
        Eigen::Matrix<double,3,1> v;
        v << cvPoint.x, cvPoint.y, cvPoint.z;

        return v;
    }

    Eigen::Matrix<double,3,3> PConverter::toMatrix3d(const cv::Mat &cvMat3)
    {
        Eigen::Matrix<double,3,3> M;

        M << cvMat3.at<MATTYPE>(0,0), cvMat3.at<MATTYPE>(0,1), cvMat3.at<MATTYPE>(0,2),
             cvMat3.at<MATTYPE>(1,0), cvMat3.at<MATTYPE>(1,1), cvMat3.at<MATTYPE>(1,2),
             cvMat3.at<MATTYPE>(2,0), cvMat3.at<MATTYPE>(2,1), cvMat3.at<MATTYPE>(2,2);

        return M;
    }

    FloatVector PConverter::toQuaternion(const cv::Mat &M)
    {
        Eigen::Matrix<double,3,3> eigMat = toMatrix3d(M);
        Eigen::Quaterniond q(eigMat);

        FloatVector v(4);
        v[0] = q.x();
        v[1] = q.y();
        v[2] = q.z();
        v[3] = q.w();

        return v;
    }

    std::string PConverter::toString(const cv::Mat &mat)
    {
        const int sz = mat.cols * mat.rows;
        string str;
        str.append("[");
        for(int i = 0; i < mat.rows; ++i)
        {
            for(int j = 0; j < mat.cols; ++j)
            {
                MATTYPE a = mat.at<double>(i,j);
                str.append(std::to_string(a)+",");
            }
        }
        str.pop_back();
        str.append("]");
        return str;
    }


    cv::Mat   PConverter::str2CVMat(const std::string &str,bool ispt /* = false*/)
    {
        if(!str.empty())
        {
            assert(*str.begin()  == '[');
            assert(*str.rbegin() == ']');
            const string sbs = str.substr(1,str.size() - 2);
            Position::StringVector values = Position::PUtils::SplitString(sbs,",");
            if(!ispt)
            {
                int n = sqrt(values.size());
                cv::Mat m = cv::Mat::eye(n,n,MATCVTYPE);
                for(int i = 0 ; i < n; ++i)
                {
                    for(int j = 0; j < n; ++j)
                    {
                        m.at<MATTYPE>(i, j) = atof(values[i * n + j].c_str());
                    }
                }
                return m;
            }
            else
            {
                int n = values.size();
                cv::Mat m = cv::Mat(n,1,MATCVTYPE);
                for(int i = 0; i < n; ++i)
                    m.at<MATTYPE>(i)= atof(values[i].c_str());
                return m;
            }
        }
        return cv::Mat();
    }
}