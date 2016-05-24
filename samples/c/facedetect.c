#include "cv.h"
#include "highgui.h"

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <assert.h>
#include <math.h>
#include <float.h>
#include <limits.h>
#include <time.h>
#include <ctype.h>

#ifdef _EiC
#define WIN32
#endif

static CvMemStorage* storage = 0;
static CvHaarClassifierCascade* cascade = 0; // 特征器分类
static CvHaarClassifierCascade* nested_cascade = 0;
int use_nested_cascade = 0;

void detect_and_draw( IplImage* image );

// 建立一个字符串，包含cascade的名字,这个应该是训练的地方
//const char* cascade_name =
//    "../../data/haarcascades/haarcascade_frontalface_alt.xml";
const char* cascade_name =
"haarcascade_frontalface_alt.xml";
/*    "haarcascade_profileface.xml";*/
const char* nested_cascade_name =
    "../../data/haarcascades/haarcascade_eye_tree_eyeglasses.xml";
//    "../../data/haarcascades/haarcascade_eye.xml";
//1.3是预处理时的缩放系数,主要作用是把被处理的图象变小.
//1.1是检测函数用的一个输入参数,是用来缩放搜索窗口的.
double scale = 1;

int main(int argc, char** argv)
{
    // 建立从video 或者avi中得到图像的结构
    CvCapture* capture = 0;

    // 结构转换成的图像
    IplImage *frame, *frame_copy = 0;
    IplImage *image = 0;
    const char* scale_opt = "--scale=";

    // 定义整形的int用于计算
    int scale_opt_len = (int)strlen(scale_opt);
    const char* cascade_opt = "--cascade=";
    int cascade_opt_len = (int)strlen(cascade_opt);
    const char* nested_cascade_opt = "--nested-cascade";
    int nested_cascade_opt_len = (int)strlen(nested_cascade_opt);
    int i;

    // 从avi或者图像文件输入的文件名
    const char* input_name = 0;

    // 检查命令行
    for (i = 1; i < argc; i++)
    {
        if( strncmp( argv[i], cascade_opt, cascade_opt_len) == 0 )
            cascade_name = argv[i] + cascade_opt_len;
        else if( strncmp( argv[i], nested_cascade_opt, nested_cascade_opt_len ) == 0 )
        {
            if( argv[i][nested_cascade_opt_len] == '=' )
                nested_cascade_name = argv[i] + nested_cascade_opt_len + 1;
            nested_cascade = (CvHaarClassifierCascade*)cvLoad( nested_cascade_name, 0, 0, 0 );
            if( !nested_cascade )
                fprintf( stderr, "WARNING: Could not load classifier cascade for nested objects\n" );
        }
        else if( strncmp( argv[i], scale_opt, scale_opt_len ) == 0 )
        {
            if( !sscanf( argv[i] + scale_opt_len, "%lf", &scale ) || scale < 1 )
                scale = 1;
        }
        else if( argv[i][0] == '-' )
        {
            fprintf( stderr, "WARNING: Unknown option %s\n", argv[i] );
        }
        else
            input_name = argv[i];
    }

    // 读取训练好的分类器。
    // 加载（分类器层叠）训练库
    cascade = (CvHaarClassifierCascade*)cvLoad( cascade_name, 0, 0, 0 );

    // 加载不成功则显示错误讯息，并退出
    if (!cascade)
    {
        fprintf( stderr, "ERROR: Could not load classifier cascade\n" );
        fprintf( stderr,
        "Usage: facedetect [--cascade=\"<cascade_path>\"]\n"
        "   [--nested-cascade[=\"nested_cascade_path\"]]\n"
        "   [--scale[=<image scale>\n"
        "   [filename|camera_index]\n" );
        return -1;
    }

    // 确定内存大小默认64k
    storage = cvCreateMemStorage(0);

    // 确定是否目标来自文件或摄像头
    if( !input_name || (isdigit(input_name[0]) && input_name[1] == '\0') )
        capture = cvCaptureFromCAM( !input_name ? 0 : input_name[0] - '0' );
    else if( input_name )
    {
        image = cvLoadImage( input_name, 1 );
        if( !image )
            capture = cvCaptureFromAVI( input_name );
    }
    else
        image = cvLoadImage( "lena.jpg", 1 );

    // 建立一个新的window窗口，以result命名
    cvNamedWindow( "result", 1 );

    // 如果成功加载capture
    if (capture)
    {
        for(;;)
        {
            // 捕获一帧，并存到图像中
            if( !cvGrabFrame( capture ))
                break;
            frame = cvRetrieveFrame( capture );

            // 如果帧不存在，退出循环
            if( !frame )
                break;

            // 分配图像和frame一样大小 frame->nChannels
            if( !frame_copy )
                frame_copy = cvCreateImage( cvSize(frame->width,frame->height),
                                            IPL_DEPTH_8U, frame->nChannels );

            // 确定图像原点，如果是顶左结构，拷贝图像
            if (frame->origin == IPL_ORIGIN_TL)
            {
                cvCopy( frame, frame_copy, 0 );
            }
            else
            {
                // 反转图像，沿x轴
                cvFlip( frame, frame_copy, 0 );
            }

            // 调用函数确定脸的位置
            detect_and_draw( frame_copy );

            // 在处理下一帧前等待一会儿
            if( cvWaitKey( 10 ) >= 0 )
                goto _cleanup_;
        }

        cvWaitKey(0);
_cleanup_:
        // 释放图像和捕获帧的内存
        cvReleaseImage( &frame_copy );
        cvReleaseCapture( &capture );
    }
    else // 如果捕获没有成功，则
    {
        // 如果图像成功加载，则调用处理函数
        if (image)
        {
            // 调用人脸检测与标示事件
            detect_and_draw( image );

            // 等待
            cvWaitKey(0);

            // 释放内存
            cvReleaseImage( &image );
        }
        else if (input_name)
        {
            // 假定它是一个文档类的文件包含要被处理图像名称，每行一个
            /* assume it is a text file containing the
               list of the image filenames to be processed - one per line */
            FILE* f = fopen( input_name, "rt" );
            if( f )
            {
                char buf[1000+1];

                // 从f中读取1000个数据
                while (fgets(buf, 1000, f))
                {
                    // 从中移去空格并且确定名字长度
                    int len = (int)strlen(buf), c;

                    // 判断是否为空格
                    while( len > 0 && isspace(buf[len-1]) )
                        len--;
                    buf[len] = '\0';
                    printf( "file %s\n", buf ); 

                    // 加载确定名字的图像
                    image = cvLoadImage( buf, 1 );

                    // 如果文件加载成功，则
                    if( image )
                    {
                        // 检测人脸
                        detect_and_draw( image );
                        c = cvWaitKey(0);
                        if( c == 27 || c == 'q' || c == 'Q' )
                            break;
                        cvReleaseImage( &image );
                    }
                }
                fclose(f);
            }
        }
    }

    // 销毁窗口
    cvDestroyWindow("result");

    return 0;
}

void detect_and_draw( IplImage* img )
{
    // 用于设置标示图像中人脸的颜色
    static CvScalar colors[] = 
    {
        {{0,0,255}},
        {{0,128,255}},
        {{0,255,255}},
        {{0,255,0}},
        {{255,128,0}},
        {{255,255,0}},
        {{255,0,0}},
        {{255,0,255}}
    };

    IplImage *gray, *small_img;
    int i, j;

    // 创建基于输入图像的新的图像
    gray = cvCreateImage( cvSize(img->width,img->height), 8, 1 );
    small_img = cvCreateImage(cvSize(cvRound(img->width / scale),
                         cvRound(img->height / scale)), 8, 1);

    // 将img转换成灰度图像gray
    cvCvtColor(img, gray, CV_BGR2GRAY);

    // 重整灰度图像大小输出到small_img
    cvResize(gray, small_img, CV_INTER_LINEAR);

    // 直方图均衡
    cvEqualizeHist(small_img, small_img);

    // 在使用之前，清空内存
    cvClearMemStorage( storage );

    // 看cascade是否被加载，如果加载，则
    if (cascade)
    {
        double t = (double)cvGetTickCount();

        // 也许图像中，有不止一个face，所以创建一增长的face序列
        // 检测图像中所有人脸并将其存储在序列中
        CvSeq* faces = cvHaarDetectObjects(small_img, 
                                           cascade, 
                                           storage,
                                           1.2, // 检测窗口的放大系数1.1 
                                           2, // 2
                                           0
                                           //|CV_HAAR_FIND_BIGGEST_OBJECT
                                           //|CV_HAAR_DO_ROUGH_SEARCH
                                           |CV_HAAR_DO_CANNY_PRUNING,
                                           //|CV_HAAR_SCALE_IMAGE
                                           cvSize(30, 30));

        t = (double)cvGetTickCount() - t;

        // 打印检测时间
        printf("detection time = %gms\n", 
            t / ((double)cvGetTickFrequency() * 1000.));

        // 循环发现的face
        // 若检测出人脸，则画出人脸的位置
        for (i = 0; i < (faces ? faces->total : 0); i++)
        {
            CvRect* r = (CvRect*)cvGetSeqElem( faces, i );
            CvMat small_img_roi;
            CvSeq* nested_objects;

            // 定义点来确定人脸位置，因为用cvRetangle嘛
            CvPoint center;

            // 不同的人脸用不用颜色区分
            CvScalar color = colors[i % 8];
            int radius;
            center.x = cvRound((r->x + r->width*0.5)*scale);
            center.y = cvRound((r->y + r->height*0.5)*scale);
            radius = cvRound((r->width + r->height)*0.25*scale);
            cvCircle( img, center, radius, color, 3, 8, 0 );
            if( !nested_cascade )
                continue;
            cvGetSubRect( small_img, &small_img_roi, *r );

            // 检测嵌套目标
            nested_objects = cvHaarDetectObjects(&small_img_roi, 
                                                 nested_cascade, 
                                                 storage,
                                                 1.1, // 1.1 
                                                 2, 
                                                 0
                                                //|CV_HAAR_FIND_BIGGEST_OBJECT
                                                //|CV_HAAR_DO_ROUGH_SEARCH
                                                //|CV_HAAR_DO_CANNY_PRUNING
                                                //|CV_HAAR_SCALE_IMAGE
                                                 ,
                                                 cvSize(0, 0));

            for (j = 0; j < (nested_objects ? nested_objects->total : 0); j++)
            {
                // 在脸上画一个圆
                // 返回索引所指定的元素指针
                CvRect* nr = (CvRect*)cvGetSeqElem(nested_objects, j);

                // 找到画圆的中心点
                center.x = cvRound((r->x + nr->x + nr->width*0.5)*scale);
                center.y = cvRound((r->y + nr->y + nr->height*0.5)*scale);
                radius = cvRound((nr->width + nr->height)*0.25*scale);

                // 画圆
                cvCircle( img, center, radius, color, 3, 8, 0 );
            }
        }
    }

    cvShowImage( "result", img );
    cvReleaseImage( &gray );
    cvReleaseImage( &small_img );
}
