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

#include "_highgui.h"

#if _MSC_VER >= 1200
#pragma warning( disable: 4711 )
#endif

#if defined WIN64 && defined EM64T && defined _MSC_VER && !defined __ICL
#pragma optimize("",off)
#endif


/************************* Reading AVIs & Camera data **************************/
// 释放由cvCaptureFromFile 或者cvCaptureFromCAM申请的CvCapture结构

CV_IMPL void cvReleaseCapture( CvCapture** pcapture )
{
    if( pcapture && *pcapture )
    {
        delete *pcapture;
        *pcapture = 0;
    }
}
/* 从摄像头或者文件中抓取并返回一帧。

capture  
视频获取结构。 
函数cvQueryFrame从摄像头或者文件中抓取一帧，然后解压并返回这一帧。这个函数仅仅是函数cvGrabFrame和函数cvRetrieveFrame在一起调用的组合。返回的图像不可以被用户释放或者修改。

 */
CV_IMPL IplImage* cvQueryFrame( CvCapture* capture )
{
    return capture ? capture->queryFrame() : 0;
}

/* 从摄像头或者视频文件中抓取帧。

capture  
视频获取结构指针。 
函数cvGrabFrame从摄像头或者文件中抓取帧。被抓取的帧在内部被存储。这个函数的目的是快速的抓取帧，这一点对同时从几个摄像头读取数据的同步是很重要的。被抓取的帧可能是压缩的格式（由摄像头／驱动定义），所以没有被公开出来。如果要取回获取的帧，请使用cvRetrieveFrame.

 */
CV_IMPL int cvGrabFrame( CvCapture* capture )
{
    return capture ? capture->grabFrame() : 0;
}

// 取得cvCreateCameraCapture的返回值
/* 取回由cvGrabFrame抓取的图像。

capture  
视频获取结构。 
函数cvRetrieveFrame返回由函数cvGrabFrame 抓取的图像的指针。返回的图像不可以被用户释放或者修改。
 */
CV_IMPL IplImage* cvRetrieveFrame( CvCapture* capture )
{
    return capture ? capture->retrieveFrame() : 0;
}

/* 获得视频获取结构的属性。

capture  
视频获取结构。 
property_id  
属性标识。可以是下面之一： 
CV_CAP_PROP_POS_MSEC - 影片目前位置，为毫秒数或者视频获取时间戳 
CV_CAP_PROP_POS_FRAMES - 将被下一步解压／获取的帧索引，以0为起点 
CV_CAP_PROP_POS_AVI_RATIO - 视频文件的相对位置（0 - 影片的开始，1 - 影片的结尾) 
CV_CAP_PROP_FRAME_WIDTH - 视频流中的帧宽度 
CV_CAP_PROP_FRAME_HEIGHT - 视频流中的帧高度 
CV_CAP_PROP_FPS - 帧率 
CV_CAP_PROP_FOURCC - 表示codec的四个字符 
CV_CAP_PROP_FRAME_COUNT - 视频文件中帧的总数 
函数cvGetCaptureProperty获得摄像头或者视频文件的指定属性。 

译者注：有时候这个函数在cvQueryFrame被调用一次后，再调用cvGetCaptureProperty才会返回正确的数值。

那昨天晚上的不能改变图像的高度和宽度是不是这样处理就可以，首先调用一次cvQueryFrame，可以试试。

 */
CV_IMPL double cvGetCaptureProperty( CvCapture* capture, int id )
{
    return capture ? capture->getProperty(id) : 0;
}

/* capture  
视频获取结构。 
property_id  
属性标识符。可以是下面之一： 
CV_CAP_PROP_POS_MSEC - 从文件开始的位置，单位为毫秒 
CV_CAP_PROP_POS_FRAMES - 单位为帧数的位置（只对视频文件有效） 
CV_CAP_PROP_POS_AVI_RATIO - 视频文件的相对位置（0 - 影片的开始，1 - 影片的结尾) 
CV_CAP_PROP_FRAME_WIDTH - 视频流的帧宽度（只对摄像头有效） 
CV_CAP_PROP_FRAME_HEIGHT - 视频流的帧高度（只对摄像头有效） 
CV_CAP_PROP_FPS - 帧率（只对摄像头有效） 
CV_CAP_PROP_FOURCC - 表示codec的四个字符（只对摄像头有效） 
value  
属性的值。 
函数cvSetCaptureProperty设置指定视频获取的属性。目前这个函数对视频文件只支持： CV_CAP_PROP_POS_MSEC, CV_CAP_PROP_POS_FRAMES, CV_CAP_PROP_POS_AVI_RATIO 

好像是不行，目前只支持CV_CAP_PROP_POS_MSEC, CV_CAP_PROP_POS_FRAMES, CV_CAP_PROP_POS_AVI_RATIO 。

 */
CV_IMPL int cvSetCaptureProperty( CvCapture* capture, int id, double value )
{
    return capture ? capture->setProperty(id, value) : 0;
}

// 从摄像头取得图像
/* index  
要使用的摄像头索引。如果只有一个摄像头或者用哪个摄像头也无所谓，那使用参数-1应该便可以。 
函数cvCreateCameraCapture给从摄像头的视频流分配和初始化CvCapture结构。目前在Windows下可使用两种接口：Video for Windows（VFW）和Matrox Imaging Library（MIL）； Linux下也有两种接口：V4L和FireWire（IEEE1394）。 

释放这个结构，使用函数cvReleaseCapture。

 */
/**
 * Camera dispatching method: index is the camera number.
 * If given an index from 0 to 99, it tries to find the first
 * API that can access a given camera index.
 * Add multiples of 100 to select an API.
 */
CV_IMPL CvCapture * cvCreateCameraCapture (int index)
{
	int  domains[] =
	{
#ifdef HAVE_VIDEOINPUT
        CV_CAP_DSHOW,
#endif
		CV_CAP_IEEE1394,   // identical to CV_CAP_DC1394
		CV_CAP_STEREO,
		CV_CAP_VFW,        // identical to CV_CAP_V4L
		CV_CAP_MIL,
		CV_CAP_QT,
		CV_CAP_UNICAP,
		-1
	};

	// interpret preferred interface (0 = autodetect)
	int pref = (index / 100) * 100;
	if (pref)
	{
		domains[0]=pref;
		index %= 100;
		domains[1]=-1;
	}

	// try every possibly installed camera API
	for (int i = 0; domains[i] >= 0; i++)
	{
		// local variable to memorize the captured device
		CvCapture *capture;

		switch (domains[i])
		{
        #ifdef HAVE_VIDEOINPUT
        case CV_CAP_DSHOW:
            capture = cvCreateCameraCapture_DShow (index);
            if (capture)
                return capture;
            break;
        #endif

		#ifdef HAVE_TYZX
		case CV_CAP_STEREO:
			capture = cvCreateCameraCapture_TYZX (index);
			if (capture)
				return capture;
			break;
		#endif

		case CV_CAP_VFW:
		#ifdef HAVE_VFW
			capture = cvCreateCameraCapture_VFW (index);
			if (capture)
				return capture;
		#endif
		#if defined (HAVE_CAMV4L) || defined (HAVE_CAMV4L2)
			capture = cvCreateCameraCapture_V4L (index);
			if (capture)
				return capture;
		#endif
		#ifdef HAVE_GSTREAMER
			capture = cvCreateCapture_GStreamer(CV_CAP_GSTREAMER_V4L2, 0);
			if (capture)
				return capture;
			capture = cvCreateCapture_GStreamer(CV_CAP_GSTREAMER_V4L, 0);
			if (capture)
				return capture;
		#endif
			break;

		case CV_CAP_FIREWIRE:
		#ifdef HAVE_DC1394
			capture = cvCreateCameraCapture_DC1394 (index);
			if (capture)
				return capture;
		#endif
		#ifdef HAVE_CMU1394
			capture = cvCreateCameraCapture_CMU (index);
			if (capture)
				return capture;
		#endif
		#ifdef HAVE_GSTREAMER
			capture = cvCreateCapture_GStreamer(CV_CAP_GSTREAMER_1394, 0);
			if (capture)
				return capture;
		#endif
			break;

		#ifdef HAVE_MIL
		case CV_CAP_MIL:
			capture = cvCreateCameraCapture_MIL (index);
			if (capture)
				return capture;
			break;
		#endif

		#ifdef HAVE_QUICKTIME
		case CV_CAP_QT:
			capture = cvCreateCameraCapture_QT (index);
			if (capture)
				return capture;
			break;
		#endif
			
		#ifdef HAVE_UNICAP
		case CV_CAP_UNICAP:
		  capture = cvCreateCameraCapture_Unicap (index);
		  if (capture)
		    return capture;
		  break;
		#endif

		}
	}

	// failed open a camera
	return 0;
}

/* filename  
视频文件名。 
函数cvCreateFileCapture给指定文件中的视频流分配和初始化CvCapture结构。 

当分配的结构不再使用的时候，它应该使用cvReleaseCapture函数释放掉

 */
/**
 * Videoreader dispatching method: it tries to find the first
 * API that can access a given filename.
 */
CV_IMPL CvCapture * cvCreateFileCapture (const char * filename)
{
    CvCapture * result = 0;

    if (! result)
        result = cvCreateFileCapture_Images (filename);

    #ifdef WIN32
    if (! result)
        result = cvCreateFileCapture_Win32 (filename);
    #endif

    #ifdef HAVE_XINE
    if (! result)
        result = cvCreateFileCapture_XINE (filename);
    #endif

    #ifdef HAVE_GSTREAMER
    if (! result)
        result = cvCreateCapture_GStreamer (CV_CAP_GSTREAMER_FILE, filename);
    #endif

    #ifdef HAVE_FFMPEG
    if (! result)
        result = cvCreateFileCapture_FFMPEG (filename);
    #endif

    #ifdef HAVE_QUICKTIME
    if (! result)
        result = cvCreateFileCapture_QT (filename);
    #endif

    return result;
}

/* 创建视频文件写入器。

filename  
输出视频文件名。 
fourcc  
四个字符用来表示压缩帧的codec 例如，CV_FOURCC('P','I','M','1')是MPEG-1 codec， CV_FOURCC('M','J','P','G')是motion-jpeg codec等。 在Win32下，如果传入参数-1，可以从一个对话框中选择压缩方法和压缩参数。 
fps  
被创建视频流的帧率。 
frame_size  
视频流的大小。 
is_color  
如果非零，编码器将希望得到彩色帧并进行编码；否则，是灰度帧（只有在Windows下支持这个标志）。 
函数cvCreateVideoWriter创建视频写入器结构。 

 */
/**
 * Videowriter dispatching method: it tries to find the first
 * API that can write a given stream.
 */
CV_IMPL CvVideoWriter* cvCreateVideoWriter( const char* filename, int fourcc,
                                            double fps, CvSize frameSize, int is_color )
{
	CV_FUNCNAME( "cvCreateVideoWriter" );

	CvVideoWriter *result = 0;

	if(!fourcc || !fps)
		result = cvCreateVideoWriter_Images(filename);

	#ifdef WIN32
	if(!result)
		result = cvCreateVideoWriter_Win32(filename, fourcc, fps, frameSize, is_color);
	#endif

/*	#ifdef HAVE_XINE
	if(!result)
		result = cvCreateVideoWriter_XINE(filename, fourcc, fps, frameSize, is_color);
	#endif
*/
	#ifdef HAVE_FFMPEG
	if(!result)
		result = cvCreateVideoWriter_FFMPEG(filename, fourcc, fps, frameSize, is_color);
	#endif

	#ifdef HAVE_QUICKTIME
	if(!result)
		result = cvCreateVideoWriter_QT(filename, fourcc, fps, frameSize, is_color);
	#endif

	if(!result)
		result = cvCreateVideoWriter_Images(filename);

	return result;
}

/* 视频写入器结构。 
image  
被写入的帧。 
函数cvWriteFrame写入／附加到视频文件一帧。 */
CV_IMPL int cvWriteFrame( CvVideoWriter* writer, const IplImage* image )
{
    return writer ? writer->writeFrame(image) : 0;
}

/* writer  
指向视频写入器的指针。 
函数cvReleaseVideoWriter结束视频文件的写入并且释放这个结构。

 */
CV_IMPL void cvReleaseVideoWriter( CvVideoWriter** pwriter )
{
    if( pwriter && *pwriter )
    {
        delete *pwriter;
        *pwriter = 0;
    }
}
