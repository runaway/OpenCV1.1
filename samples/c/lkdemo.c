/* Demo of modified Lucas-Kanade optical flow algorithm.
   See the printf below */

#ifdef _CH_
#pragma package <opencv>
#endif

#ifndef _EiC
#include "cv.h"
#include "highgui.h"
#include <stdio.h>
#include <ctype.h>
#endif
// 基于光流的运动人体

/* 简介：在计算机视觉中，Lucas–Kanade光流算法是一种两帧差分的光流估计算法。它由Bruce D. Lucas 和 Takeo Kanade提出。

光流的概念:(Optical flow or optic flow)
它是一种运动模式，这种运动模式指的是一个物体、表面、边缘在一个视角下由一个观察者（比如眼睛、摄像头等）和背景之间形成的明显移动。光流技术，如运动检测和图像分割，时间碰撞，运动补偿编码，三维立体视差，都是利用了这种边缘或表面运动的技术。

二维图像的移动相对于观察者而言是三维物体移动的在图像平面的投影。
有序的图像可以估计出二维图像的瞬时图像速率或离散图像转移。

光流算法：
它评估了两幅图像的之间的变形，它的基本假设是体素和图像像素守恒。它假设一个物体的颜色在前后两帧没有巨大而明显的变化。基于这个思路，我们可以得到图像约束方程。不同的光流算法解决了假定了不同附加条件的光流问题。

Lucas–Kanade算法：
这个算法是最常见，最流行的。它计算两帧在时间t 到t + δt之间每个每个像素点位置的移动。由于它是基于图像信号的泰勒级数，这种方法称为差分，这就是对于空间和时间坐标使用偏导数。
图像约束方程可以写为I(x,y,z,t) = I(x + δx,y + δy,z + δz,t + δt)
I(x, y,z, t) 为在（x,y,z）位置的体素。
我们假设移动足够的小，那么对图像约束方程使用泰勒公式，我们可以得到：


H.O.T. 指更高阶，在移动足够小的情况下可以忽略。从这个方程中我们可以得到：

或者

我们得到：


Vx,Vy,Vz 分别是I(x,y,z,t)的光流向量中x，y，z的组成。 , , 和则是图像在(x,y,z,t)这一点向相应方向的差分。
所以

IxVx + IyVy + IzVz = ? It。

写做：

 

这个方程有三个未知量，尚不能被解决，这也就是所谓光流算法的光圈问题。那么要找到光流向量则需要另一套解决的方案。而Lucas-Kanade算法是一个非迭代的算法：

假设流(Vx,Vy,Vz)在一个大小为m*m*m(m>1)的小窗中是一个常数，那么从像素1...n, n = m3中可以得到下列一组方程：

 

 

 

 

三个未知数但是有多于三个的方程，这个方程组自然是个超定方程，也就是说方程组内有冗余，方程组可以表示为：

I_{x_1} & I_{y_1} & I_{z_1}\\
I_{x_2} & I_{y_2} & I_{z_2}\\
\vdots & \vdots & \vdots\\
I_{x_n} & I_{y_n} & I_{z_n}
\end{bmatrix}
\begin{bmatrix}
V_x\\
V_y\\
V_z
\end{bmatrix}
=
\begin{bmatrix}
-I_{t_1}\\
-I_{t_2}\\
\vdots \\
-I_{t_n}
\end{bmatrix} ">

记作：

为了解决这个超定问题，我们采用最小二乘法：

or

 

得到：

V_x\\
V_y\\
V_z
\end{bmatrix}
=
\begin{bmatrix}
\sum I_{x_i}^2 & \sum I_{x_i}I_{y_i} & \sum I_{x_i}I_{z_i} \\
\sum I_{x_i}I_{y_i} & \sum I_{y_i}^2 & \sum I_{y_i}I_{z_i} \\
\sum I_{x_i}I_{z_i} & \sum I_{y_i}I_{z_i} & \sum I_{z_i}^2 \\
\end{bmatrix}^{-1}
\begin{bmatrix}
-\sum I_{x_i}I_{t_i} \\
-\sum I_{y_i}I_{t_i} \\
-\sum I_{z_i}I_{t_i}
\end{bmatrix}">

其中的求和是从1到n。

这也就是说寻找光流可以通过在四维上图像导数的分别累加得出。我们还需要一个权重函数W(i, j,k)， 来突出窗口中心点的坐标。高斯函数做这项工作是非常合适的，

这个算法的不足在于它不能产生一个密度很高的流向量，例如在运动的边缘和黑大的同质区域中的微小移动方面流信息会很快的褪去。它的优点在于有噪声存在的鲁棒性还是可以的。

 

实现Lucas-Kanade 光流算法可以分为三个步骤。

第一步骤为初始化需要跟踪的点。

第二步骤为根据两帧之间的光流来计算由初始化的需要跟踪的点的目标点，为此要先计算出两帧的光流金字塔。

第三步骤为把输入输出点进行互换，还有上一帧与目前帧的互换以及上一帧与当前帧金字塔的互换。

下面我们来具体看一下实现的过程：

 

//第三步，当第一次进入方法时候，因为输入点的容器是空的，所以并不运行这段代码，而是运行到第一步。当再次进入这个方法时。因为由第一步得到的输出点已经由第二步转换为输入点，所以输入点非空，我们就可以进入下面的方法。该方法时为opencv计算光流跟踪的主要方法。由前一帧与当前帧的图像和两帧的金字塔得到了新的输出点。如果输入点的光流小时。对应的m_status 的值为0。我们可以把没有光流的点进行删除。

if(m_lInputPoints.size())

    {

        cvCalcOpticalFlowPyrLK(m_prev_grey, m_grey, m_prev_pyramid, m_pyramid,

&m_lInputPoints[0], &m_lOutputPoints[0], m_lInputPoints.size(), cvSize(m_win_size, m_win_size), 3, m_status, 0,

cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03), m_flags);

 

        m_flags |= CV_LKFLOW_PYR_A_READY;

}  

//第一步从这里开始，假设我们已经初始化完毕输入点，但是我们要先把输入点放在输出的点的容器内，然后计算该容器内点的光流。

    cvFindCornerSubPix(m_grey, &m_lOutputPoints[m_lOutputPoints.size() - 1], 1,

                cvSize(m_win_size, m_win_size), cvSize(-1, -1),

                cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03));

//第二步，把得到的输出点的光流放到输入点的容器内，以及互换前一帧与当前帧，还有前一帧与当前帧的光流来进行下次的运算。这里运行完毕，并不运行到第三步，而是退出该方法。并得到下面一帧。

m_lInputPoints = m_lOutputPoints;

    CV_SWAP(m_prev_grey, m_grey, m_swap_temp);

    CV_SWAP(m_prev_pyramid, m_pyramid, m_swap_temp);

至此，Lucas-Kanade 光流算法已经成功的实现。可以对设定点进行跟踪。但是如何选取这些被跟踪的点来进行最优化的跟踪，是需要解决的下一个问题。 

在上一节提到了Lucas-Kanade光流跟踪算法，是一种准确，成熟，比较容易实现的物体跟踪算法，对画面中固定点会进行准确快速的跟踪。但是在视频中如何对移动物体进行跟踪以及跟踪点的选择，则是另一个需要解决的问题。下面我们来详细了解一下。

 

首先，在视频中移动的物体与静止的物体的一种区别就是，在前一帧与当前帧之间，静止物体是没有运动的，运动物体的运动造成了在这两帧之中有一个运动的区域，两帧的差我们可以用cvAbsDiff来计算出，我们可以通过定位这个区域来设置需要跟踪的点。这个运动的区域我们可以通过OpenCV的查找轮廓的方法来找到，这里使用了cvFindContours 方法，因为我们只需找出运动区域的轮廓，所以我们需要找到的是物体外部轮廓，因而我们需要查找轮廓的方法为：CV_RETR_EXTERNAL，而生成的轮廓会保存在一个CvSeq中。需要注意的是，我们可以在视频画面中加入画面的平滑处理来去掉一些噪点，比如高斯滤波。

 

其次，既然我们有了运动区域的轮廓，我们就要把这个区域转化为需要被跟踪的点，一种普遍的方法就是我们遍历生成轮廓的CvSeq 中的每一个轮廓，再遍历每一个轮廓中的每个点，把这些点加在一起算出平均值，这个平均值就是这个轮廓的中心点。这种方法虽然正确，但是计算速度比较慢，因为计算量比较大，对于大的轮廓计算的点会非常多。另一种方法则是比较取巧的方法，计算速度很快，效果也非常好。因为我们需要的是对运动物体准确的跟踪，对于定位运动的物体则不需要很精确，只要知道物体在哪里运动就可以了。所以在这里，我们把查找到的每个轮廓用cvMinAreaRect2 转化为CvBox2D类型。这个类型的好处就是可以直接调用CvBox2D 的 Center 变量来生成中心点，这个中心店为轮廓近似四边形的中心点，与轮廓点的平均值点在效果上等价。

 

最后，我们把两帧之间所有运动生成的这些点保存在一个Vector 里，把这个Vector 作为输入点的序列发送到L-K算法中。通过L-K算法计算出这些点在当前帧的运动终点，再在下一帧把这些终点作为起始点再传入L-K算法继续计算。在L-K计算过程中，如果有新的运动的点加入，我们就继续把Vector中的新的点传入到L-K 中。

 

至此，L-K算法的自动跟踪已经完毕，我们可以通过加入一些滤波以及边缘检测来更准确的定位需要跟踪的点，而检测出来的运动点的位移偏量，比如X方向与Y方向，或者Z方向（需要两个摄像头），可以当做参数传入到相应的运动或者显示模块中，产生相对应的效果。
*/

IplImage *image = 0, *grey = 0, *prev_grey = 0, *pyramid = 0, *prev_pyramid = 0, *swap_temp;

int win_size = 10;
const int MAX_COUNT = 500;
CvPoint2D32f* points[2] = {0,0}, *swap_points;
char* status = 0;
int count = 0;
int need_to_init = 0;
int night_mode = 0;
int flags = 0;
int add_remove_pt = 0;
CvPoint pt;


void on_mouse( int event, int x, int y, int flags, void* param )
{
    if( !image )
        return;

    if( image->origin )
        y = image->height - y;

    if( event == CV_EVENT_LBUTTONDOWN )
    {
        pt = cvPoint(x,y);
        add_remove_pt = 1;
    }
}


int main( int argc, char** argv )
{
    CvCapture* capture = 0;
    
    if( argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0])))
        capture = cvCaptureFromCAM( argc == 2 ? argv[1][0] - '0' : 0 );
    else if( argc == 2 )
        capture = cvCaptureFromAVI( argv[1] );

    if( !capture )
    {
        fprintf(stderr,"Could not initialize capturing...\n");
        return -1;
    }

    /* print a welcome message, and the OpenCV version */
    printf ("Welcome to lkdemo, using OpenCV version %s (%d.%d.%d)\n",
	    CV_VERSION,
	    CV_MAJOR_VERSION, CV_MINOR_VERSION, CV_SUBMINOR_VERSION);

    printf( "Hot keys: \n"
            "\tESC - quit the program\n"
            "\tr - auto-initialize tracking\n"
            "\tc - delete all the points\n"
            "\tn - switch the \"night\" mode on/off\n"
            "To add/remove a feature point click it\n" );

    cvNamedWindow( "LkDemo", 0 );
    cvSetMouseCallback( "LkDemo", on_mouse, 0 );

    for(;;)
    {
        IplImage* frame = 0;
        int i, k, c;

        frame = cvQueryFrame( capture );
        if( !frame )
            break;

        if( !image )
        {
            /* allocate all the buffers */
            image = cvCreateImage( cvGetSize(frame), 8, 3 );
            image->origin = frame->origin;
            grey = cvCreateImage( cvGetSize(frame), 8, 1 );
            prev_grey = cvCreateImage( cvGetSize(frame), 8, 1 );
            pyramid = cvCreateImage( cvGetSize(frame), 8, 1 );
            prev_pyramid = cvCreateImage( cvGetSize(frame), 8, 1 );
            points[0] = (CvPoint2D32f*)cvAlloc(MAX_COUNT*sizeof(points[0][0]));
            points[1] = (CvPoint2D32f*)cvAlloc(MAX_COUNT*sizeof(points[0][0]));
            status = (char*)cvAlloc(MAX_COUNT);
            flags = 0;
        }

        cvCopy( frame, image, 0 );
        cvCvtColor( image, grey, CV_BGR2GRAY );

        if( night_mode )
            cvZero( image );
        
        if( need_to_init )
        {
            /* automatic initialization */
            IplImage* eig = cvCreateImage( cvGetSize(grey), 32, 1 );
            IplImage* temp = cvCreateImage( cvGetSize(grey), 32, 1 );
            double quality = 0.01;
            double min_distance = 10;

            count = MAX_COUNT;
            cvGoodFeaturesToTrack( grey, eig, temp, points[1], &count,
                                   quality, min_distance, 0, 3, 0, 0.04 );
            cvFindCornerSubPix( grey, points[1], count,
                cvSize(win_size,win_size), cvSize(-1,-1),
                cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));
            cvReleaseImage( &eig );
            cvReleaseImage( &temp );

            add_remove_pt = 0;
        }
        else if( count > 0 )
        {
            cvCalcOpticalFlowPyrLK( prev_grey, grey, prev_pyramid, pyramid,
                points[0], points[1], count, cvSize(win_size,win_size), 3, status, 0,
                cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03), flags );
            flags |= CV_LKFLOW_PYR_A_READY;
            for( i = k = 0; i < count; i++ )
            {
                if( add_remove_pt )
                {
                    double dx = pt.x - points[1][i].x;
                    double dy = pt.y - points[1][i].y;

                    if( dx*dx + dy*dy <= 25 )
                    {
                        add_remove_pt = 0;
                        continue;
                    }
                }
                
                if( !status[i] )
                    continue;
                
                points[1][k++] = points[1][i];
                cvCircle( image, cvPointFrom32f(points[1][i]), 3, CV_RGB(0,255,0), -1, 8,0);
            }
            count = k;
        }

        if( add_remove_pt && count < MAX_COUNT )
        {
            points[1][count++] = cvPointTo32f(pt);
            cvFindCornerSubPix( grey, points[1] + count - 1, 1,
                cvSize(win_size,win_size), cvSize(-1,-1),
                cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS,20,0.03));
            add_remove_pt = 0;
        }

        CV_SWAP( prev_grey, grey, swap_temp );
        CV_SWAP( prev_pyramid, pyramid, swap_temp );
        CV_SWAP( points[0], points[1], swap_points );
        need_to_init = 0;
        cvShowImage( "LkDemo", image );

        c = cvWaitKey(10);
        if( (char)c == 27 )
            break;
        switch( (char) c )
        {
        case 'r':
            need_to_init = 1;
            break;
        case 'c':
            count = 0;
            break;
        case 'n':
            night_mode ^= 1;
            break;
        default:
            ;
        }
    }

    cvReleaseCapture( &capture );
    cvDestroyWindow("LkDemo");

    return 0;
}

#ifdef _EiC
main(1,"lkdemo.c");
#endif
