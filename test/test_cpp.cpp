#include "test.h"
#include "P_Interface.h"
#include "P_IOHelper.h"
#include "P_Checker.h"
#include "P_CoorTrans.h"
#include "P_Utils.h"
#include "project/newhwproject.h"
#include "Thirdparty/sqlite3/sqlite3.h"
#include "P_SemanticGraph.h"
#include "P_Mask.h"
#include "P_ORBFeature.h"
template <typename T>
inline void display(T items)
{
    for (auto i : items)
    {
        cout << i << " ";
    }
    cout << endl;
}

#define _TOSTR(a) #a
#define NMB_STR(a) _TOSTR(a)
#define FILE_LOG() "{" __FILE__ "}"
#define LINE_LOG() "{" NMB_STR(__LINE__) "}"
#define PRINTMESSAGE(X) FILE_LOG() \
LINE_LOG() ":" X

#define FM_T_V "%d" \
               "%s"
#define FM_T_S "%s"

#include <regex>

typedef unsigned int U4;
typedef int L4;


void fit_plane(const Mat &point, Mat &plane)
{
    int rows = point.rows;
    int cols = point.cols;

    Mat center = Mat::zeros(1,cols,MATCVTYPE);

    for(int i = 0; i < cols; ++i)
    {
        for(int j = 0; j < rows; ++j)
        {
            center.at<MATTYPE>(0,i) += point.at<MATTYPE>(j,i);
        }
        center.at<MATTYPE>(0,i) /= rows;
    }

    Mat points2 = Mat::ones(rows,cols,MATCVTYPE);

    for(int i = 0; i < rows; ++i)
    {
        for(int j = 0; j < cols; ++j)
        {
            points2.at<MATTYPE>(i, j) = points2.at<MATTYPE>(i, j) - center.at<MATTYPE>(0,j);
        }
    }

    Mat A,W,U,V;

    cv::gemm(points2,point,1,NULL,0,A,CV_GEMM_A_T);
    
    cv::SVD::compute(A,W,U,V);

    plane = Mat::zeros(cols + 1, 1, MATCVTYPE);
    for( int c = 0; c < cols; ++c)
    {
        plane.at<MATTYPE>(c,0) = V.at<MATTYPE>(cols - 1, c);
        plane.at<MATTYPE>(cols,0) += plane.at<MATTYPE>(c,0) * center.at<MATTYPE>(0,c);
    }
}

/*输入一组坐标值，根据最小二乘法计算直线方程 y = kx + b 
先返回斜率 k ,再返回截距 b*/
Mat OLS_Line(const Position::Pt3Vector  &point)
{
    //Ax = b的形式，将A b 写成矩阵的形式
    Mat A((int)point.size(), 2, MATCVTYPE);
    Mat b((int)point.size(), 1, MATCVTYPE);

    //初始化矩阵A
    for (size_t i = 0; i<point.size(); i++)
        A.at<MATTYPE>((int)i, 1) = 1;
    for (size_t i = 0; i< point.size(); i++)
        A.at<MATTYPE>((int)i, 0) = point[i].x;

    //初始化矩阵b 
    for (size_t i = 0; i< point.size(); i++)
        b.at<MATTYPE>((int)i, 0) = point[i].y;

    //根据线性代数知识，A'* A * x = A' * b 求得的矩阵 x 即为最优解
    //解 x = (A' * A)^-1 * A' * b

    return (A.t()*A).inv()*A.t()*b;
}

/*输入一组坐标值，根据最小二乘法计算平面方程
分别返回 a ,b, c 的值
aX + bY - Z + c = 0 */
Mat OLS_Plane(const Position::Pt3Vector& point)
{
    //Ax = 0的形式，将A, b 写成矩阵的形式
    Mat A((int)point.size(), 3, MATCVTYPE);
    Mat b((int)point.size(), 1, MATCVTYPE);

    //初始化矩阵A
    for (size_t i = 0; i< point.size(); i++)
        A.at<MATTYPE>((int)i, 0) = (point[i].x);

    for (size_t i = 0; i< point.size(); i++)
        A.at<MATTYPE>((int)i, 1) = (point[i].y);

    for (size_t i = 0; i<point.size(); i++)
        A.at<MATTYPE>((int)i, 2) = 1;

    //初始化矩阵b 
    for (size_t i = 0; i< point.size(); i++)
        b.at<MATTYPE>((int)i, 0) = -(point[i].z);

 cout <<"原始点为:"<< A << endl;
    //根据线性代数知识，A'* A * x = A' * b 求得的矩阵 x 即为最优解
    //解 x = (A' * A)^-1 * A' * b

    Mat x = -((A.t()*A).inv()*A.t()*b);

    return x;
}


TESTBEGIN()
   
//add more code
    //  const int rowsz = 5;
    //  Mat points(rowsz,3,MATCVTYPE);
    // Position::Pt3Vector pts;
    //  for( int i = 0; i < rowsz; ++i )
    //  {
    //      points.at<double>(i,0) = i + 1;
    //      points.at<double>(i,1) = i + 1;
    //      points.at<double>(i,2) = 1;

    //      pts.emplace_back(Point3f(i + 1,i + 1,1));
    //  }

    //  Mat plane;

    // fit_plane(points,plane);

    // cout << plane << endl;   

    // Mat plane2 = OLS_Plane(pts);

    // cout << plane2 << endl;

  
TESTEND()