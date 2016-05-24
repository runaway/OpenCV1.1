#ifdef _CH_
#pragma package <opencv>
#endif

#ifndef _EiC
// motion templates sample code
#include "cv.h"
#include "highgui.h"
#include <time.h>
#include <math.h>
#include <ctype.h>
#include <stdio.h>
#endif
// 运动模板
// various tracking parameters (in seconds)
const double MHI_DURATION = 1;
const double MAX_TIME_DELTA = 0.5;
const double MIN_TIME_DELTA = 0.05;
// number of cyclic frame buffer used for motion detection
// (should, probably, depend on FPS)
const int N = 4;

// ring image buffer
IplImage **buf = 0;
int last = 0;

// temporary images
IplImage *mhi = 0; // MHI
IplImage *orient = 0; // orientation
IplImage *mask = 0; // valid orientation mask
IplImage *segmask = 0; // motion segmentation map
CvMemStorage* storage = 0; // temporary storage

/*******************************************************************************
*   Func Name: update_mhi                                                
* Description: 更新运动历史图像的函数
*       Input:                                   
*      Output:                                               
*      Return:                                      
*     Caution:
*-------------------------------------------------------------------------------
* Modification History
*      1.Date: 2009-06-18
*      Author: Runaway
* Description:                                           
*******************************************************************************/
// parameters:
//  img - input video frame
//  dst - resultant motion picture
//  args - optional parameters
void update_mhi(IplImage* img, IplImage* dst, int diff_threshold)
{
    // 获取当前时间
    double timestamp = (double)clock()/CLOCKS_PER_SEC; // get current time in seconds

    // 获取当前帧大小
    CvSize size = cvSize(img->width,img->height); // get current frame size
    int i, idx1 = last, idx2;
    IplImage* silh;
    CvSeq* seq;
    CvRect comp_rect;
    double count;
    double angle;
    CvPoint center;
    double magnitude;          
    CvScalar color;

    // 给图像分配空间或者在尺寸改变的时候重新分配
    // allocate images at the beginning or
    // reallocate them if the frame size is changed
    if (!mhi || mhi->width != size.width || mhi->height != size.height) 
    {
        if (buf == 0) 
        {
            buf = (IplImage**)malloc(N*sizeof(buf[0]));
            memset( buf, 0, N*sizeof(buf[0]));
        }
        
        for( i = 0; i < N; i++ ) {
            cvReleaseImage( &buf[i] );
            buf[i] = cvCreateImage( size, IPL_DEPTH_8U, 1 );
            cvZero( buf[i] );
        }
        cvReleaseImage( &mhi );
        cvReleaseImage( &orient );
        cvReleaseImage( &segmask );
        cvReleaseImage( &mask );
        
        mhi = cvCreateImage( size, IPL_DEPTH_32F, 1 );

        // 在开始时清空MHI
        cvZero( mhi ); // clear MHI at the beginning

		// 按img的尺寸创建图像
        orient = cvCreateImage( size, IPL_DEPTH_32F, 1 );
        segmask = cvCreateImage( size, IPL_DEPTH_32F, 1 );
        mask = cvCreateImage( size, IPL_DEPTH_8U, 1 );
    }

    // 转换为灰度
    cvCvtColor( img, buf[last], CV_BGR2GRAY ); // convert frame to grayscale

    idx2 = (last + 1) % N; // index of (last - (N-1))th frame
    last = idx2;

    silh = buf[idx2];

    // 获取两帧间的差异，当前帧跟背景图相减，放到silh里面
    cvAbsDiff(buf[idx1], buf[idx2], silh); // get difference between frames

    // 二值化
    cvThreshold(silh, silh, diff_threshold, 1, CV_THRESH_BINARY); // and threshold it

    // 去掉影像(silhouette) 以更新运动历史图像
    cvUpdateMotionHistory(silh, mhi, timestamp, MHI_DURATION); // update MHI

    // 转换MHI到蓝色8位图
    // convert MHI to blue 8u image
    cvCvtScale( mhi, mask, 255./MHI_DURATION,
                (MHI_DURATION - timestamp)*255./MHI_DURATION );
    cvZero( dst );
    cvCvtPlaneToPix( mask, 0, 0, 0, dst );

    // 计算运动历史图像的梯度方向 
    // 计算运动梯度趋向和合法的趋向掩码
    // calculate motion gradient orientation and valid orientation mask
    cvCalcMotionGradient( mhi, mask, orient, MAX_TIME_DELTA, MIN_TIME_DELTA, 3 );
    
    if( !storage )
        storage = cvCreateMemStorage(0);
    else
        cvClearMemStorage(storage);

    // 将整个运动分割为独立的运动部分 
    // 分割运动:获取运动组件序列
    // 分割掩码是运动组件图标识出来的，不再过多的使用
    // segment motion: get sequence of motion components
    // segmask is marked motion components map. It is not used further
    seq = cvSegmentMotion(mhi, segmask, storage, timestamp, MAX_TIME_DELTA);

	// 按运动组件的数目来循环
    // 通过运动组件迭代
    // 根据整幅图像(全局运动)进行相应的一次或多次迭代
    // iterate through the motion components,
    // One more iteration (i == -1) corresponds to the whole image (global motion)
    for (i = -1; i < seq->total; i++) 
    {

        if (i < 0) 
        { 
            // 全局运动事件
            // case of the whole image
            // 获取当前帧的范围
            comp_rect = cvRect( 0, 0, size.width, size.height );

			// 设置颜色为白色
			color = CV_RGB(255,255,255);

			// 设置放大倍数为100
            magnitude = 100;
        }
        else 
        { 
            // 第i个运动组件
            // i-th motion component
            // 获取当前运动组件的范围
            comp_rect = ((CvConnectedComp*)cvGetSeqElem( seq, i ))->rect;

			// 丢弃很小的组件
			if( comp_rect.width + comp_rect.height < 100 ) // reject very small components
                continue;

			// 设置颜色为红色
			color = CV_RGB(255,0,0);

			// 设置放大倍数为30
            magnitude = 30;
        }

        // 选择组件感兴趣的区域
        // select component ROI
        cvSetImageROI( silh, comp_rect );
        cvSetImageROI( mhi, comp_rect );
        cvSetImageROI( orient, comp_rect );
        cvSetImageROI( mask, comp_rect );

        // 计算某些选择区域的全局运动方向 
        // 每个运动部件的运动方向就可以被这个函数利用提取的特定部件的掩模(mask)计算出来(使用cvCmp) 
        // 计算趋势
        // calculate orientation
        angle = cvCalcGlobalOrientation( orient, mask, mhi, timestamp, MHI_DURATION);

        // 根据左上角的原点来调整图像的角度
        angle = 360.0 - angle;  // adjust for images with top-left origin

        // 计算数组的绝对范数， 绝对差分范数或者相对差分范数 
        // 计算轮廓感兴趣区域中点的个数
        count = cvNorm( silh, 0, CV_L1, 0 ); // calculate number of points within silhouette ROI

        cvResetImageROI( mhi );
        cvResetImageROI( orient );
        cvResetImageROI( mask );
        cvResetImageROI( silh );

        // 检测小运动事件
        // check for the case of little motion
        if (count < comp_rect.width*comp_rect.height * 0.05)
        {
            continue;
        }

        // 画一个带箭头的时钟来指示方向
        // draw a clock with arrow indicating the direction
        center = cvPoint( (comp_rect.x + comp_rect.width/2),
                          (comp_rect.y + comp_rect.height/2) );

        cvCircle( dst, center, cvRound(magnitude*1.2), color, 3, CV_AA, 0 );
        cvLine( dst, center, cvPoint( cvRound( center.x + magnitude*cos(angle*CV_PI/180)),
                cvRound( center.y - magnitude*sin(angle*CV_PI/180))), color, 3, CV_AA, 0 );
    }
}


int main(int argc, char** argv)
{
    IplImage* motion = 0;
    CvCapture* capture = 0;
    
    if( argc == 1 || (argc == 2 && strlen(argv[1]) == 1 && isdigit(argv[1][0])))
        capture = cvCaptureFromCAM( argc == 2 ? argv[1][0] - '0' : 0 );
    else if( argc == 2 )
        capture = cvCaptureFromFile( argv[1] );

    // 如果捕获到图像
    if (capture)
    {
        cvNamedWindow( "Motion", 1 );
        
        for(;;)
        {
            IplImage* image;
            if( !cvGrabFrame( capture ))
                break;
            image = cvRetrieveFrame( capture );

            if( image )
            {
                if( !motion )
                {
                    motion = cvCreateImage( cvSize(image->width,image->height), 8, 3 );
                    cvZero( motion );

					// 将采集到的图像指针赋给运动图像
                    motion->origin = image->origin;
                }
            }

			// 更新运动历史图像
            update_mhi(image, motion, 30);

			// 显示运动图像
            cvShowImage( "Motion", motion );

            if( cvWaitKey(10) >= 0 )
                break;
        }

		cvReleaseCapture( &capture );
        cvDestroyWindow( "Motion" );
    }

    return 0;
}
                                
#ifdef _EiC
main(1,"motempl.c");
#endif
