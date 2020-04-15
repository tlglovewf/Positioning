#pragma once

#include "GpsGlobalMap.h"
#include <eigen3/Eigen/Dense>
#include <eigen3/Eigen/Geometry>
#include <iostream>
#include "Thirdparty/GeographicLib/include/LocalCartesian.hpp"
#include <fstream>
#include "Thirdparty/g2o/g2o/core/block_solver.h"
#include "Thirdparty/g2o/g2o/core/optimization_algorithm_levenberg.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_eigen.h"
#include "Thirdparty/g2o/g2o/types/types_six_dof_expmap.h"
#include "Thirdparty/g2o/g2o/core/robust_kernel_impl.h"
#include "Thirdparty/g2o/g2o/solvers/linear_solver_dense.h"
#include "Thirdparty/g2o/g2o/types/types_seven_dof_expmap.h"


using namespace std;

namespace g2o {
    class EdgePosiPreSim3 : public BaseUnaryEdge<3, Eigen::Vector3d, VertexSim3Expmap>{
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgePosiPreSim3(){};

        bool read(std::istream& is){return true;};

        bool write(std::ostream& os) const{return true;};

        void computeError()  {
        const g2o::VertexSim3Expmap* v1 = static_cast<const VertexSim3Expmap*>(_vertices[0]);
        _error= v1->estimate().inverse().translation()-_measurement;
        //std::cout<<v1->estimate().inverse().translation().transpose()<<std::endl;
        //std::cout<<v1->estimate().scale()<<std::endl;
        //std::cout<<v1->estimate().inverse().translation().transpose()/v1->estimate().scale()<<std::endl;
        }
    };
    class EdgePosiPreSE3 : public BaseUnaryEdge<3, Eigen::Vector3d, VertexSE3Expmap>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgePosiPreSE3(){};

        bool read(std::istream& is){return true;};

        bool write(std::ostream& os) const{return true;};

        void computeError()  
        {
        const g2o::VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
        _error= v1->estimate().inverse().translation()-_measurement;
        }
    };
    
    class EdgeSE3 : public BaseBinaryEdge<6, g2o::SE3Quat, VertexSE3Expmap, VertexSE3Expmap>
    {
    public:
        EIGEN_MAKE_ALIGNED_OPERATOR_NEW;
        EdgeSE3(){};

        bool read(std::istream& is){return true;};

        bool write(std::ostream& os) const{return true;};

        void computeError()  {
            const VertexSE3Expmap* v1 = static_cast<const VertexSE3Expmap*>(_vertices[0]);
            const VertexSE3Expmap* v2 = static_cast<const VertexSE3Expmap*>(_vertices[1]);

            g2o::SE3Quat C(_measurement);
            g2o::SE3Quat error_=C*v1->estimate()*v2->estimate().inverse();
            _error = error_.log();
        }
    };
}

struct GPSData
{
    double t;   //时间
    double lat; //纬度
    double lon; //经度
    double alt; //高程
    double conf;//置信值
};


class GpsSim3Optimizer
{
public:
    GpsSim3Optimizer();
    ~GpsSim3Optimizer();
    void beginOpt();
    void setOrbMap(gps::GpsGlobalMap& globalmap);
    void getGlobalGPS(double time,double& latitude, double& longitude, double& altitude);
    void getFrameXYZ(int index, double &x, double &y, double &z);
private:
    void ComputeSim3();//计算sim3
    void pose_graph_opti_se3();
    void GPS2XYZ(double latitude, double longitude, double altitude, double* xyz);
    void XYZ2GPS(double* xyz , double& latitude, double& longitude, double& altitude);

    gps::GpsGlobalMap mGlobalMap;
    
    GeographicLib::LocalCartesian geoConverter;//gps坐标转换器

    bool initGPS;
};