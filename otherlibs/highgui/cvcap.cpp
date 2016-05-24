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
// �ͷ���cvCaptureFromFile ����cvCaptureFromCAM�����CvCapture�ṹ

CV_IMPL void cvReleaseCapture( CvCapture** pcapture )
{
    if( pcapture && *pcapture )
    {
        delete *pcapture;
        *pcapture = 0;
    }
}
/* ������ͷ�����ļ���ץȡ������һ֡��

capture  
��Ƶ��ȡ�ṹ�� 
����cvQueryFrame������ͷ�����ļ���ץȡһ֡��Ȼ���ѹ��������һ֡��������������Ǻ���cvGrabFrame�ͺ���cvRetrieveFrame��һ����õ���ϡ����ص�ͼ�񲻿��Ա��û��ͷŻ����޸ġ�

 */
CV_IMPL IplImage* cvQueryFrame( CvCapture* capture )
{
    return capture ? capture->queryFrame() : 0;
}

/* ������ͷ������Ƶ�ļ���ץȡ֡��

capture  
��Ƶ��ȡ�ṹָ�롣 
����cvGrabFrame������ͷ�����ļ���ץȡ֡����ץȡ��֡���ڲ����洢�����������Ŀ���ǿ��ٵ�ץȡ֡����һ���ͬʱ�Ӽ�������ͷ��ȡ���ݵ�ͬ���Ǻ���Ҫ�ġ���ץȡ��֡������ѹ���ĸ�ʽ��������ͷ���������壩������û�б��������������Ҫȡ�ػ�ȡ��֡����ʹ��cvRetrieveFrame.

 */
CV_IMPL int cvGrabFrame( CvCapture* capture )
{
    return capture ? capture->grabFrame() : 0;
}

// ȡ��cvCreateCameraCapture�ķ���ֵ
/* ȡ����cvGrabFrameץȡ��ͼ��

capture  
��Ƶ��ȡ�ṹ�� 
����cvRetrieveFrame�����ɺ���cvGrabFrame ץȡ��ͼ���ָ�롣���ص�ͼ�񲻿��Ա��û��ͷŻ����޸ġ�
 */
CV_IMPL IplImage* cvRetrieveFrame( CvCapture* capture )
{
    return capture ? capture->retrieveFrame() : 0;
}

/* �����Ƶ��ȡ�ṹ�����ԡ�

capture  
��Ƶ��ȡ�ṹ�� 
property_id  
���Ա�ʶ������������֮һ�� 
CV_CAP_PROP_POS_MSEC - ӰƬĿǰλ�ã�Ϊ������������Ƶ��ȡʱ��� 
CV_CAP_PROP_POS_FRAMES - ������һ����ѹ����ȡ��֡��������0Ϊ��� 
CV_CAP_PROP_POS_AVI_RATIO - ��Ƶ�ļ������λ�ã�0 - ӰƬ�Ŀ�ʼ��1 - ӰƬ�Ľ�β) 
CV_CAP_PROP_FRAME_WIDTH - ��Ƶ���е�֡��� 
CV_CAP_PROP_FRAME_HEIGHT - ��Ƶ���е�֡�߶� 
CV_CAP_PROP_FPS - ֡�� 
CV_CAP_PROP_FOURCC - ��ʾcodec���ĸ��ַ� 
CV_CAP_PROP_FRAME_COUNT - ��Ƶ�ļ���֡������ 
����cvGetCaptureProperty�������ͷ������Ƶ�ļ���ָ�����ԡ� 

����ע����ʱ�����������cvQueryFrame������һ�κ��ٵ���cvGetCaptureProperty�Ż᷵����ȷ����ֵ��

���������ϵĲ��ܸı�ͼ��ĸ߶ȺͿ���ǲ�����������Ϳ��ԣ����ȵ���һ��cvQueryFrame���������ԡ�

 */
CV_IMPL double cvGetCaptureProperty( CvCapture* capture, int id )
{
    return capture ? capture->getProperty(id) : 0;
}

/* capture  
��Ƶ��ȡ�ṹ�� 
property_id  
���Ա�ʶ��������������֮һ�� 
CV_CAP_PROP_POS_MSEC - ���ļ���ʼ��λ�ã���λΪ���� 
CV_CAP_PROP_POS_FRAMES - ��λΪ֡����λ�ã�ֻ����Ƶ�ļ���Ч�� 
CV_CAP_PROP_POS_AVI_RATIO - ��Ƶ�ļ������λ�ã�0 - ӰƬ�Ŀ�ʼ��1 - ӰƬ�Ľ�β) 
CV_CAP_PROP_FRAME_WIDTH - ��Ƶ����֡��ȣ�ֻ������ͷ��Ч�� 
CV_CAP_PROP_FRAME_HEIGHT - ��Ƶ����֡�߶ȣ�ֻ������ͷ��Ч�� 
CV_CAP_PROP_FPS - ֡�ʣ�ֻ������ͷ��Ч�� 
CV_CAP_PROP_FOURCC - ��ʾcodec���ĸ��ַ���ֻ������ͷ��Ч�� 
value  
���Ե�ֵ�� 
����cvSetCaptureProperty����ָ����Ƶ��ȡ�����ԡ�Ŀǰ�����������Ƶ�ļ�ֻ֧�֣� CV_CAP_PROP_POS_MSEC, CV_CAP_PROP_POS_FRAMES, CV_CAP_PROP_POS_AVI_RATIO 

�����ǲ��У�Ŀǰֻ֧��CV_CAP_PROP_POS_MSEC, CV_CAP_PROP_POS_FRAMES, CV_CAP_PROP_POS_AVI_RATIO ��

 */
CV_IMPL int cvSetCaptureProperty( CvCapture* capture, int id, double value )
{
    return capture ? capture->setProperty(id, value) : 0;
}

// ������ͷȡ��ͼ��
/* index  
Ҫʹ�õ�����ͷ���������ֻ��һ������ͷ�������ĸ�����ͷҲ����ν����ʹ�ò���-1Ӧ�ñ���ԡ� 
����cvCreateCameraCapture��������ͷ����Ƶ������ͳ�ʼ��CvCapture�ṹ��Ŀǰ��Windows�¿�ʹ�����ֽӿڣ�Video for Windows��VFW����Matrox Imaging Library��MIL���� Linux��Ҳ�����ֽӿڣ�V4L��FireWire��IEEE1394���� 

�ͷ�����ṹ��ʹ�ú���cvReleaseCapture��

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
��Ƶ�ļ����� 
����cvCreateFileCapture��ָ���ļ��е���Ƶ������ͳ�ʼ��CvCapture�ṹ�� 

������Ľṹ����ʹ�õ�ʱ����Ӧ��ʹ��cvReleaseCapture�����ͷŵ�

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

/* ������Ƶ�ļ�д������

filename  
�����Ƶ�ļ����� 
fourcc  
�ĸ��ַ�������ʾѹ��֡��codec ���磬CV_FOURCC('P','I','M','1')��MPEG-1 codec�� CV_FOURCC('M','J','P','G')��motion-jpeg codec�ȡ� ��Win32�£�����������-1�����Դ�һ���Ի�����ѡ��ѹ��������ѹ�������� 
fps  
��������Ƶ����֡�ʡ� 
frame_size  
��Ƶ���Ĵ�С�� 
is_color  
������㣬��������ϣ���õ���ɫ֡�����б��룻�����ǻҶ�֡��ֻ����Windows��֧�������־���� 
����cvCreateVideoWriter������Ƶд�����ṹ�� 

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

/* ��Ƶд�����ṹ�� 
image  
��д���֡�� 
����cvWriteFrameд�룯���ӵ���Ƶ�ļ�һ֡�� */
CV_IMPL int cvWriteFrame( CvVideoWriter* writer, const IplImage* image )
{
    return writer ? writer->writeFrame(image) : 0;
}

/* writer  
ָ����Ƶд������ָ�롣 
����cvReleaseVideoWriter������Ƶ�ļ���д�벢���ͷ�����ṹ��

 */
CV_IMPL void cvReleaseVideoWriter( CvVideoWriter** pwriter )
{
    if( pwriter && *pwriter )
    {
        delete *pwriter;
        *pwriter = 0;
    }
}
