/*M///////////////////////////////////////////////////////////////////////////////////////
//
//  IMPORTANT: READ BEFORE DOWNLOADING, COPYING, INSTALLING OR USING.
//
//  By downloading, copying, installing or using the software you agree to this license.
//  If you do not agree to this license, do not download, install,
//  copy or use the software.
//
//
//                        Intel License Agreement
//                For Open Source Computer Vision Library
//
// Copyright (C) 2000, Intel Corporation, all rights reserved.
// Third party copyrights are property of their respective owners.
//
// Redistribution and use in source and binary forms, with or without modification,
// are permitted provided that the following conditions are met:
//
//   * Redistribution's of source code must retain the above copyright notice,
//     this list of conditions and the following disclaimer.
//
//   * Redistribution's in binary form must reproduce the above copyright notice,
//     this list of conditions and the following disclaimer in the documentation
//     and/or other materials provided with the distribution.
//
//   * The name of Intel Corporation may not be used to endorse or promote products
//     derived from this software without specific prior written permission.
//
// This software is provided by the copyright holders and contributors "as is" and
// any express or implied warranties, including, but not limited to, the implied
// warranties of merchantability and fitness for a particular purpose are disclaimed.
// In no event shall the Intel Corporation or contributors be liable for any direct,
// indirect, incidental, special, exemplary, or consequential damages
// (including, but not limited to, procurement of substitute goods or services;
// loss of use, data, or profits; or business interruption) however caused
// and on any theory of liability, whether in contract, strict liability,
// or tort (including negligence or otherwise) arising in any way out of
// the use of this software, even if advised of the possibility of such damage.
//
//M*/

#ifndef _CVTYPES_H_
#define _CVTYPES_H_

#ifndef SKIP_INCLUDES
  #include <assert.h>
  #include <stdlib.h>
#endif

/* spatial and central moments */
typedef struct CvMoments
{
    double  m00, m10, m01, m20, m11, m02, m30, m21, m12, m03; /* spatial moments */
    double  mu20, mu11, mu02, mu30, mu21, mu12, mu03; /* central moments */
    double  inv_sqrt_m00; /* m00 != 0 ? 1/sqrt(m00) : 0 */
}
CvMoments;

/* Hu invariants */
typedef struct CvHuMoments
{
    double hu1, hu2, hu3, hu4, hu5, hu6, hu7; /* Hu invariants */
}
CvHuMoments;

/**************************** Connected Component  **************************************/

typedef struct CvConnectedComp
{
    double area;    /* area of the connected component  */
    CvScalar value; /* average color of the connected component */
    CvRect rect;    /* ROI of the component  */
    CvSeq* contour; /* optional component boundary
                      (the contour might have child contours corresponding to the holes)*/
}
CvConnectedComp;

/*
Internal structure that is used for sequental retrieving contours from the image.
It supports both hierarchical and plane variants of Suzuki algorithm.
*/
typedef struct _CvContourScanner* CvContourScanner;

/* contour retrieval mode */
#define CV_RETR_EXTERNAL 0
#define CV_RETR_LIST     1
#define CV_RETR_CCOMP    2
#define CV_RETR_TREE     3

/* contour approximation method */
#define CV_CHAIN_CODE               0
#define CV_CHAIN_APPROX_NONE        1
#define CV_CHAIN_APPROX_SIMPLE      2
#define CV_CHAIN_APPROX_TC89_L1     3
#define CV_CHAIN_APPROX_TC89_KCOS   4
#define CV_LINK_RUNS                5

/* Freeman chain reader state */
typedef struct CvChainPtReader
{
    CV_SEQ_READER_FIELDS()
    char      code;
    CvPoint   pt;
    schar     deltas[8][2];
}
CvChainPtReader;

/* initializes 8-element array for fast access to 3x3 neighborhood of a pixel */
#define  CV_INIT_3X3_DELTAS( deltas, step, nch )            \
    ((deltas)[0] =  (nch),  (deltas)[1] = -(step) + (nch),  \
     (deltas)[2] = -(step), (deltas)[3] = -(step) - (nch),  \
     (deltas)[4] = -(nch),  (deltas)[5] =  (step) - (nch),  \
     (deltas)[6] =  (step), (deltas)[7] =  (step) + (nch))

/* Contour tree header */
typedef struct CvContourTree
{
    CV_SEQUENCE_FIELDS()
    CvPoint p1;            /* the first point of the binary tree root segment */
    CvPoint p2;            /* the last point of the binary tree root segment */
}
CvContourTree;

/* Finds a sequence of convexity defects of given contour */
typedef struct CvConvexityDefect
{
    CvPoint* start; /* point of the contour where the defect begins */
    CvPoint* end; /* point of the contour where the defect ends */
    CvPoint* depth_point; /* the farthest from the convex hull point within the defect */
    float depth; /* distance between the farthest point and the convex hull */
}
CvConvexityDefect;

/************ Data structures and related enumerations for Planar Subdivisions ************/

typedef size_t CvSubdiv2DEdge;

#define CV_QUADEDGE2D_FIELDS()     \
    int flags;                     \
    struct CvSubdiv2DPoint* pt[4]; \
    CvSubdiv2DEdge  next[4];

#define CV_SUBDIV2D_POINT_FIELDS()\
    int            flags;      \
    CvSubdiv2DEdge first;      \
    CvPoint2D32f   pt;

#define CV_SUBDIV2D_VIRTUAL_POINT_FLAG (1 << 30)

typedef struct CvQuadEdge2D
{
    CV_QUADEDGE2D_FIELDS()
}
CvQuadEdge2D;

typedef struct CvSubdiv2DPoint
{
    CV_SUBDIV2D_POINT_FIELDS()
}
CvSubdiv2DPoint;

#define CV_SUBDIV2D_FIELDS()    \
    CV_GRAPH_FIELDS()           \
    int  quad_edges;            \
    int  is_geometry_valid;     \
    CvSubdiv2DEdge recent_edge; \
    CvPoint2D32f  topleft;      \
    CvPoint2D32f  bottomright;

typedef struct CvSubdiv2D
{
    CV_SUBDIV2D_FIELDS()
}
CvSubdiv2D;


typedef enum CvSubdiv2DPointLocation
{
    CV_PTLOC_ERROR = -2,
    CV_PTLOC_OUTSIDE_RECT = -1,
    CV_PTLOC_INSIDE = 0,
    CV_PTLOC_VERTEX = 1,
    CV_PTLOC_ON_EDGE = 2
}
CvSubdiv2DPointLocation;

typedef enum CvNextEdgeType
{
    CV_NEXT_AROUND_ORG   = 0x00,
    CV_NEXT_AROUND_DST   = 0x22,
    CV_PREV_AROUND_ORG   = 0x11,
    CV_PREV_AROUND_DST   = 0x33,
    CV_NEXT_AROUND_LEFT  = 0x13,
    CV_NEXT_AROUND_RIGHT = 0x31,
    CV_PREV_AROUND_LEFT  = 0x20,
    CV_PREV_AROUND_RIGHT = 0x02
}
CvNextEdgeType;

/* get the next edge with the same origin point (counterwise) */
#define  CV_SUBDIV2D_NEXT_EDGE( edge )  (((CvQuadEdge2D*)((edge) & ~3))->next[(edge)&3])


/* Defines for Distance Transform */
#define CV_DIST_USER    -1  /* User defined distance */
#define CV_DIST_L1      1   /* distance = |x1-x2| + |y1-y2| */
#define CV_DIST_L2      2   /* the simple euclidean distance */
#define CV_DIST_C       3   /* distance = max(|x1-x2|,|y1-y2|) */
#define CV_DIST_L12     4   /* L1-L2 metric: distance = 2(sqrt(1+x*x/2) - 1)) */
#define CV_DIST_FAIR    5   /* distance = c^2(|x|/c-log(1+|x|/c)), c = 1.3998 */
#define CV_DIST_WELSCH  6   /* distance = c^2/2(1-exp(-(x/c)^2)), c = 2.9846 */
#define CV_DIST_HUBER   7   /* distance = |x|<c ? x^2/2 : c(|x|-c/2), c=1.345 */


/* Filters used in pyramid decomposition */
typedef enum CvFilter
{
    CV_GAUSSIAN_5x5 = 7
}
CvFilter;

/****************************************************************************************/
/*                                    Older definitions                                 */
/****************************************************************************************/

typedef float*   CvVect32f;
typedef float*   CvMatr32f;
typedef double*  CvVect64d;
typedef double*  CvMatr64d;

typedef struct CvMatrix3
{
    float m[3][3];
}
CvMatrix3;


#ifdef __cplusplus
extern "C" {
#endif

typedef float (CV_CDECL * CvDistanceFunction)( const float* a, const float* b, void* user_param );

#ifdef __cplusplus
}
#endif

// 结构 CvConDensation中条件概率密度传播(译者注：粒子滤波的一种特例)（Con-Dens-Aation: 单词 CONditional DENSity propagATION 的缩写）跟踪器的状态。该算法描述可参考 http://www.dai.ed.ac.uk/CVonline/LOCAL_COPIES/ISARD1/condensation.html 
typedef struct CvConDensation
{
    int MP; // 测量向量的维数： Dimension of measurement vector
    int DP; // 状态向量的维数： Dimension of state vector
    float* DynamMatr;       /* 线性动态系统矩阵：Matrix of the linear Dynamics system  */
    float* State;           /* 状态向量：Vector of State                       */
    int SamplesNum;         /* 粒子数：Number of the Samples                 */
    float** flSamples;      /* 粒子向量数组：arr of the Sample Vectors             */
    float** flNewSamples;   /* 粒子向量临时数组：temporary array of the Sample Vectors */
    float* flConfidence;    /* 每个粒子的置信度(译者注：也就是粒子的权值)：Confidence for each Sample            */
    float* flCumulative;    /* 权值的累计：Cumulative confidence                 */
    float* Temp;            /* 临时向量：Temporary vector                      */
    float* RandomSample;    /* 用来更新粒子集的随机向量：RandomVector to update sample set     */
    struct CvRandState* RandS; /* 产生随机向量的结构数组：Array of structures to generate random vectors */
}
CvConDensation;

/*（Hunnish: 害怕有翻译上的不准确，因此在翻译注释时，保留了原来的英文术语，以便大家对照）

结构 CvKalman 用来保存 Kalman 滤波器状态。它由函数 cvCreateKalman 创建，由函数f cvKalmanPredict 和 cvKalmanCorrect 更新，由 cvReleaseKalman 释放. 通常该结构是为标准 Kalman 所使用的 (符号和公式都借自非常优秀的 Kalman 教程 [Welch95]):

xk=A?xk-1+B?uk+wk　　　　　译者注：系统运动方程
zk=H?xk+vk,　　　　　　　　译者注：系统观测方程

其中:

xk (xk-1) - 系统在时刻 k (k-1) 的状态向量 （state of the system at the moment k (k-1)）
zk - 在时刻 k 的系统状态测量向量 （measurement of the system state at the moment k）
uk - 应用于时刻 k 的外部控制 (external control applied at the moment k)

wk 和 vk 分别为正态分布的运动和测量噪声
p(w) ~ N(0,Q)
p(v) ~ N(0,R),

即,
Q - 运动噪声的相关矩阵，常量或变量
R - 测量噪声的相关矩阵，常量或变量

对标准 Kalman 滤波器，所有矩阵: A, B, H, Q 和 R 都是通过 cvCreateKalman 在分配结构 CvKalman 时初始化一次。但是，同样的结构和函数，通过在当前系统状态邻域中线性化扩展 Kalman 滤波器方程，可以用来模拟扩展 Kalman 滤波器，在这种情况下， A, B, H (也许还有 Q 和 R) 在每一步中都被更新。 

*/
/*
standard Kalman filter (in G. Welch' and G. Bishop's notation):

  x(k)=A*x(k-1)+B*u(k)+w(k)  p(w)~N(0,Q)
  z(k)=H*x(k)+v(k),   p(v)~N(0,R)
*/
typedef struct CvKalman
{
    int MP;                     /* 测量向量维数number of measurement vector dimensions */
    int DP;                     /* 状态向量维数number of state vector dimensions */
    int CP;                     /* 控制向量维数number of control vector dimensions */

    /* backward compatibility fields */
#if 1
    float* PosterState;         /* =state_pre->data.fl */
    float* PriorState;          /* =state_post->data.fl */
    float* DynamMatr;           /* =transition_matrix->data.fl */
    float* MeasurementMatr;     /* =measurement_matrix->data.fl */
    float* MNCovariance;        /* =measurement_noise_cov->data.fl */
    float* PNCovariance;        /* =process_noise_cov->data.fl */
    float* KalmGainMatr;        /* =gain->data.fl */
    float* PriorErrorCovariance;/* =error_cov_pre->data.fl */
    float* PosterErrorCovariance;/* =error_cov_post->data.fl */
    float* Temp1;               /* temp1->data.fl */
    float* Temp2;               /* temp2->data.fl */
#endif

    CvMat* state_pre;           /* 预测状态predicted state (x'(k)):
                                    x(k)=A*x(k-1)+B*u(k) */
    CvMat* state_post;          /* 矫正状态corrected state (x(k)):
                                    x(k)=x'(k)+K(k)*(z(k)-H*x'(k)) */
    CvMat* transition_matrix;   /* 状态传递矩阵state transition matrix (A) */
    CvMat* control_matrix;      /* 控制矩阵(如果没有控制，则不使用它)control matrix (B)
                                   (it is not used if there is no control)*/
    CvMat* measurement_matrix;  /* 测量矩阵measurement matrix (H) */
    CvMat* process_noise_cov;   /* 处理噪声相关矩阵process noise covariance matrix (Q) */
    CvMat* measurement_noise_cov; /* 测量噪声相关矩阵measurement noise covariance matrix (R) */
    CvMat* error_cov_pre;       /* 先验错误估计相关矩阵priori error estimate covariance matrix (P'(k)):
                                    P'(k)=A*P(k-1)*At + Q)*/
    CvMat* gain;                /* Kalman 增益矩阵Kalman gain matrix (K(k)):
                                    K(k)=P'(k)*Ht*inv(H*P'(k)*Ht+R)*/
    CvMat* error_cov_post;      /* 后验错误估计相关矩阵posteriori error estimate covariance matrix (P(k)):
                                    P(k)=(I-K(k)*H)*P'(k) */
    CvMat* temp1;               /* 临时矩阵temporary matrices */
    CvMat* temp2;
    CvMat* temp3;
    CvMat* temp4;
    CvMat* temp5;
}
CvKalman;


/*********************** Haar-like Object Detection structures **************************/
#define CV_HAAR_MAGIC_VAL    0x42500000
#define CV_TYPE_NAME_HAAR    "opencv-haar-classifier"

#define CV_IS_HAAR_CLASSIFIER( haar )                                                    \
    ((haar) != NULL &&                                                                   \
    (((const CvHaarClassifierCascade*)(haar))->flags & CV_MAGIC_MASK)==CV_HAAR_MAGIC_VAL)

#define CV_HAAR_FEATURE_MAX  3

/* CvHaarFeature, CvHaarClassifier, CvHaarStageClassifier, 
CvHaarClassifierCascade Boosted Haar 分类器结构的几个结构体是树型结构。

Cascade:
Stage1: 
Classifier11: 
Feature11 
Classifier12: 
Feature12 
... 
Stage2: 
Classifier21: 
Feature21 整个等级可以手工构建，也可以利用函数cvLoadHaarClassifierCascade从已有的
磁盘文件或嵌入式基中导入。 */
//一个 harr 特征由 2－3 个具有相应权重的矩形组成 
//titled ：/* 0 means up-right feature, 1 means 45--rotated feature */ 
//rect[CV_HAAR_FEATURE_MAX]; /* 2-3 rectangles with weights of opposite signs and with absolute values inversely proportional to the areas of the rectangles. if rect[2].weight !=0, then the feature consists of 3 rectangles, otherwise it consists of 2 */ 

typedef struct CvHaarFeature
{
    int  tilted;
    struct
    {
        CvRect r;
        float weight;
    } rect[CV_HAAR_FEATURE_MAX];
}
CvHaarFeature;

typedef struct CvHaarClassifier
{
    int count;
    CvHaarFeature* haar_feature;
    float* threshold;
    int* left;
    int* right;
    float* alpha;
}
CvHaarClassifier;

typedef struct CvHaarStageClassifier
{
    int  count;
    float threshold;
    CvHaarClassifier* classifier;

    int next;
    int child;
    int parent;
}
CvHaarStageClassifier;

typedef struct CvHidHaarClassifierCascade CvHidHaarClassifierCascade;

typedef struct CvHaarClassifierCascade
{
    int  flags;
    int  count;
    CvSize orig_window_size;
    CvSize real_window_size;
    double scale;
    CvHaarStageClassifier* stage_classifier;
    CvHidHaarClassifierCascade* hid_cascade;
}
CvHaarClassifierCascade;

typedef struct CvAvgComp
{
    CvRect rect;
    int neighbors;
}
CvAvgComp;

struct CvFeatureTree;

#endif /*_CVTYPES_H_*/

/* End of file. */
