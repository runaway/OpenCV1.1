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
#include "_cv.h"

/* 2) Mean Shift算法
 

这里来到了CamShift算法，OpenCV实现的第二部分，这一次重点讨论Mean Shift算法。

在讨论Mean Shift算法之前，首先讨论在2D概率分布图像中，如何计算某个区域的重心(Mass Center)的问题，重心可以通过以下公式来计算：

1.计算区域内0阶矩

for(int i=0;i<height;i++)

  for(int j=0;j<width;j++)

     M00+=I(i,j)

2.区域内1阶矩：

for(int i=0;i<height;i++)

  for(int j=0;j<width;j++)

  {

    M10+=i*I(i,j);

    M01+=j*I(i,j);

  }

3.则Mass Center为：

Xc=M10/M00; Yc=M01/M00

接下来，讨论Mean Shift算法的具体步骤，Mean Shift算法可以分为以下4步：

1.选择窗的大小和初始位置.

2.计算此时窗口内的Mass Center.

3.调整窗口的中心到Mass Center.

4.重复2和3，直到窗口中心"会聚"，即每次窗口移动的距离小于一定的阈值。

 

在OpenCV中，提供Mean Shift算法的函数，函数的原型是：

int cvMeanShift(IplImage* imgprob,CvRect windowIn,

                    CvTermCriteria criteria,CvConnectedComp* out);

 

需要的参数为：

1.IplImage* imgprob：2D概率分布图像，传入；

2.CvRect windowIn：初始的窗口，传入；

3.CvTermCriteria criteria：停止迭代的标准，传入；

4.CvConnectedComp* out:查询结果，传出。

(注：构造CvTermCriteria变量需要三个参数，一个是类型，另一个是迭代的最大次数，最后一个表示特定的阈值。例如可以这样构造criteria：criteria=cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,10,0.1)。)

 

返回的参数：

1.int：迭代的次数。

 

实现代码：暂时缺

3) CamShift算法
1.原理

在了解了MeanShift算法以后，我们将MeanShift算法扩展到连续图像序列（一般都是指视频图像序列），这样就形成了CamShift算法。CamShift算法的全称是"Continuously Apaptive Mean-SHIFT"，它的基本思想是视频图像的所有帧作MeanShift运算，并将上一帧的结果（即Search Window的中心和大小）作为下一帧MeanShift算法的Search Window的初始值，如此迭代下去，就可以实现对目标的跟踪。整个算法的具体步骤分5步：

Step 1：将整个图像设为搜寻区域。

Step 2：初始话Search Window的大小和位置。

Step 3：计算Search Window内的彩色概率分布，此区域的大小比Search Window要稍微大一点。

Step 4：运行MeanShift。获得Search Window新的位置和大小。

Step 5：在下一帧视频图像中，用Step 3获得的值初始化Search Window的位置和大小。跳转到Step 3继续运行。

2.实现

在OpenCV中，有实现CamShift算法的函数，此函数的原型是：

  cvCamShift(IplImage* imgprob, CvRect windowIn,

                CvTermCriteria criteria,

                CvConnectedComp* out, CvBox2D* box=0);

其中：

   imgprob：色彩概率分布图像。

   windowIn：Search Window的初始值。

   Criteria：用来判断搜寻是否停止的一个标准。

   out：保存运算结果,包括新的Search Window的位置和面积。

   box：包含被跟踪物体的最小矩形。

 

说明：

1.在OpenCV 4.0 beta的目录中，有CamShift的例子。遗憾的是这个例子目标的跟踪是半自动的，即需要人手工选定一个目标。我正在努力尝试全自动的目标跟踪，希望可以和大家能在这方面与大家交流。 */

/* 在反向投影图中发现目标中心 

int cvMeanShift( const CvArr* prob_image, CvRect window,
                 CvTermCriteria criteria, CvConnectedComp* comp );
prob_image 
目标直方图的反向投影(见 cvCalcBackProject). 
window 
初始搜索窗口 
criteria 
确定窗口搜索停止的准则 
comp 
生成的结构，包含收敛的搜索窗口坐标 (comp->rect 字段) 与窗口内部所有象素点的和 (comp->area 字段). 
函数 cvMeanShift 在给定反向投影和初始搜索窗口位置的情况下，用迭代方法寻找目标中心。当搜索窗口中心的移动小于某个给定值时或者函数已经达到最大迭代次数时停止迭代。 函数返回迭代次数。 
 */
/*F///////////////////////////////////////////////////////////////////////////////////////
//    Name:    cvMeanShift
//    Purpose: MeanShift algorithm
//    Context:
//    Parameters:
//      imgProb     - 2D object probability distribution
//      windowIn    - CvRect of CAMSHIFT Window intial size
//      numIters    - If CAMSHIFT iterates this many times, stop
//      windowOut   - Location, height and width of converged CAMSHIFT window
//      len         - If != NULL, return equivalent len
//      width       - If != NULL, return equivalent width
//      itersUsed   - Returns number of iterations CAMSHIFT took to converge
//    Returns:
//      The function itself returns the area found
//    Notes:
//F*/
CV_IMPL int
cvMeanShift( const void* imgProb, CvRect windowIn,
             CvTermCriteria criteria, CvConnectedComp* comp )
{
    CvMoments moments;
    int    i = 0, eps;
    CvMat  stub, *mat = (CvMat*)imgProb;
    CvMat  cur_win;
    CvRect cur_rect = windowIn;

    CV_FUNCNAME( "cvMeanShift" );

    if( comp )
        comp->rect = windowIn;

    moments.m00 = moments.m10 = moments.m01 = 0;

    __BEGIN__;

    CV_CALL( mat = cvGetMat( mat, &stub ));

    if( CV_MAT_CN( mat->type ) > 1 )
        CV_ERROR( CV_BadNumChannels, cvUnsupportedFormat );

    if( windowIn.height <= 0 || windowIn.width <= 0 )
        CV_ERROR( CV_StsBadArg, "Input window has non-positive sizes" );

    if( windowIn.x < 0 || windowIn.x + windowIn.width > mat->cols ||
        windowIn.y < 0 || windowIn.y + windowIn.height > mat->rows )
        CV_ERROR( CV_StsBadArg, "Initial window is not inside the image ROI" );

    CV_CALL( criteria = cvCheckTermCriteria( criteria, 1., 100 ));

    eps = cvRound( criteria.epsilon * criteria.epsilon );

    for( i = 0; i < criteria.max_iter; i++ )
    {
        int dx, dy, nx, ny;
        double inv_m00;

        CV_CALL( cvGetSubRect( mat, &cur_win, cur_rect )); 
        CV_CALL( cvMoments( &cur_win, &moments ));

        /* Calculating center of mass */
        if( fabs(moments.m00) < DBL_EPSILON )
            break;

        inv_m00 = moments.inv_sqrt_m00*moments.inv_sqrt_m00;
        dx = cvRound( moments.m10 * inv_m00 - windowIn.width*0.5 );
        dy = cvRound( moments.m01 * inv_m00 - windowIn.height*0.5 );

        nx = cur_rect.x + dx;
        ny = cur_rect.y + dy;

        if( nx < 0 )
            nx = 0;
        else if( nx + cur_rect.width > mat->cols )
            nx = mat->cols - cur_rect.width;

        if( ny < 0 )
            ny = 0;
        else if( ny + cur_rect.height > mat->rows )
            ny = mat->rows - cur_rect.height;

        dx = nx - cur_rect.x;
        dy = ny - cur_rect.y;
        cur_rect.x = nx;
        cur_rect.y = ny;

        /* Check for coverage centers mass & window */
        if( dx*dx + dy*dy < eps )
            break;
    }

    __END__;

    if( comp )
    {
        comp->rect = cur_rect;
        comp->area = (float)moments.m00;
    }

    return i;
}

/* 发现目标中心，尺寸和方向 

int cvCamShift( const CvArr* prob_image, CvRect window, CvTermCriteria criteria,
                CvConnectedComp* comp, CvBox2D* box=NULL );
prob_image 
目标直方图的反向投影 (见 cvCalcBackProject). 
window 
初始搜索窗口 
criteria 
确定窗口搜索停止的准则 
comp 
生成的结构，包含收敛的搜索窗口坐标 (comp->rect 字段) 与窗口内部所有象素点的和 (comp->area 字段). 
box 
目标的带边界盒子。如果非 NULL, 则包含目标的尺寸和方向。 
函数 cvCamShift 实现了 CAMSHIFT 目标跟踪算法([Bradski98]). 首先它调用函数 cvMeanShift 寻找目标中心，然后计算目标尺寸和方向。最后返回函数 cvMeanShift 中的迭代次数。 

CvCamShiftTracker 类在 cv.hpp 中被声明，函数实现了彩色目标的跟踪。 

 */
/*F///////////////////////////////////////////////////////////////////////////////////////
//    Name:    cvCamShift
//    Purpose: CAMSHIFT algorithm
//    Context:
//    Parameters:
//      imgProb     - 2D object probability distribution
//      windowIn    - CvRect of CAMSHIFT Window intial size
//      criteria    - criteria of stop finding window
//      windowOut   - Location, height and width of converged CAMSHIFT window
//      orientation - If != NULL, return distribution orientation
//      len         - If != NULL, return equivalent len
//      width       - If != NULL, return equivalent width
//      area        - sum of all elements in result window
//      itersUsed   - Returns number of iterations CAMSHIFT took to converge
//    Returns:
//      The function itself returns the area found
//    Notes:
//F*/
CV_IMPL int
cvCamShift( const void* imgProb, CvRect windowIn,
            CvTermCriteria criteria,
            CvConnectedComp* _comp,
            CvBox2D* box )
{
    const int TOLERANCE = 10;
    CvMoments moments;
    double m00 = 0, m10, m01, mu20, mu11, mu02, inv_m00;
    double a, b, c, xc, yc;
    double rotate_a, rotate_c;
    double theta = 0, square;
    double cs, sn;
    double length = 0, width = 0;
    int itersUsed = 0;
    CvConnectedComp comp;
    CvMat  cur_win, stub, *mat = (CvMat*)imgProb;

    CV_FUNCNAME( "cvCamShift" );

    comp.rect = windowIn;

    __BEGIN__;

    CV_CALL( mat = cvGetMat( mat, &stub ));

    CV_CALL( itersUsed = cvMeanShift( mat, windowIn, criteria, &comp ));
    windowIn = comp.rect;

    windowIn.x -= TOLERANCE;
    if( windowIn.x < 0 )
        windowIn.x = 0;

    windowIn.y -= TOLERANCE;
    if( windowIn.y < 0 )
        windowIn.y = 0;

    windowIn.width += 2 * TOLERANCE;
    if( windowIn.x + windowIn.width > mat->width )
        windowIn.width = mat->width - windowIn.x;

    windowIn.height += 2 * TOLERANCE;
    if( windowIn.y + windowIn.height > mat->height )
        windowIn.height = mat->height - windowIn.y;

    CV_CALL( cvGetSubRect( mat, &cur_win, windowIn ));

    /* Calculating moments in new center mass */
    CV_CALL( cvMoments( &cur_win, &moments ));

    m00 = moments.m00;
    m10 = moments.m10;
    m01 = moments.m01;
    mu11 = moments.mu11;
    mu20 = moments.mu20;
    mu02 = moments.mu02;

    if( fabs(m00) < DBL_EPSILON )
        EXIT;

    inv_m00 = 1. / m00;
    xc = cvRound( m10 * inv_m00 + windowIn.x );
    yc = cvRound( m01 * inv_m00 + windowIn.y );
    a = mu20 * inv_m00;
    b = mu11 * inv_m00;
    c = mu02 * inv_m00;

    /* Calculating width & height */
    square = sqrt( 4 * b * b + (a - c) * (a - c) );

    /* Calculating orientation */
    theta = atan2( 2 * b, a - c + square );

    /* Calculating width & length of figure */
    cs = cos( theta );
    sn = sin( theta );

    rotate_a = cs * cs * mu20 + 2 * cs * sn * mu11 + sn * sn * mu02;
    rotate_c = sn * sn * mu20 - 2 * cs * sn * mu11 + cs * cs * mu02;
    length = sqrt( rotate_a * inv_m00 ) * 4;
    width = sqrt( rotate_c * inv_m00 ) * 4;

    /* In case, when tetta is 0 or 1.57... the Length & Width may be exchanged */
    if( length < width )
    {
        double t;
        
        CV_SWAP( length, width, t );
        CV_SWAP( cs, sn, t );
        theta = CV_PI*0.5 - theta;
    }

    /* Saving results */
    if( _comp || box )
    {
        int t0, t1;
        int _xc = cvRound( xc );
        int _yc = cvRound( yc );

        t0 = cvRound( fabs( length * cs ));
        t1 = cvRound( fabs( width * sn ));

        t0 = MAX( t0, t1 ) + 2;
        comp.rect.width = MIN( t0, (mat->width - _xc) * 2 );

        t0 = cvRound( fabs( length * sn ));
        t1 = cvRound( fabs( width * cs ));

        t0 = MAX( t0, t1 ) + 2;
        comp.rect.height = MIN( t0, (mat->height - _yc) * 2 );

        comp.rect.x = MAX( 0, _xc - comp.rect.width / 2 );
        comp.rect.y = MAX( 0, _yc - comp.rect.height / 2 );

        comp.rect.width = MIN( mat->width - comp.rect.x, comp.rect.width );
        comp.rect.height = MIN( mat->height - comp.rect.y, comp.rect.height );
        comp.area = (float) m00;
    }

    __END__;

    if( _comp )
        *_comp = comp;
    
    if( box )
    {
        box->size.height = (float)length;
        box->size.width = (float)width;
        box->angle = (float)(theta*180./CV_PI);
        box->center = cvPoint2D32f( comp.rect.x + comp.rect.width*0.5f,
                                    comp.rect.y + comp.rect.height*0.5f);
    }

    return itersUsed;
}

/* End of file. */
