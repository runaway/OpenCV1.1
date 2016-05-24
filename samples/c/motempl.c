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
// �˶�ģ��
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
* Description: �����˶���ʷͼ��ĺ���
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
    // ��ȡ��ǰʱ��
    double timestamp = (double)clock()/CLOCKS_PER_SEC; // get current time in seconds

    // ��ȡ��ǰ֡��С
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

    // ��ͼ�����ռ�����ڳߴ�ı��ʱ�����·���
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

        // �ڿ�ʼʱ���MHI
        cvZero( mhi ); // clear MHI at the beginning

		// ��img�ĳߴ紴��ͼ��
        orient = cvCreateImage( size, IPL_DEPTH_32F, 1 );
        segmask = cvCreateImage( size, IPL_DEPTH_32F, 1 );
        mask = cvCreateImage( size, IPL_DEPTH_8U, 1 );
    }

    // ת��Ϊ�Ҷ�
    cvCvtColor( img, buf[last], CV_BGR2GRAY ); // convert frame to grayscale

    idx2 = (last + 1) % N; // index of (last - (N-1))th frame
    last = idx2;

    silh = buf[idx2];

    // ��ȡ��֡��Ĳ��죬��ǰ֡������ͼ������ŵ�silh����
    cvAbsDiff(buf[idx1], buf[idx2], silh); // get difference between frames

    // ��ֵ��
    cvThreshold(silh, silh, diff_threshold, 1, CV_THRESH_BINARY); // and threshold it

    // ȥ��Ӱ��(silhouette) �Ը����˶���ʷͼ��
    cvUpdateMotionHistory(silh, mhi, timestamp, MHI_DURATION); // update MHI

    // ת��MHI����ɫ8λͼ
    // convert MHI to blue 8u image
    cvCvtScale( mhi, mask, 255./MHI_DURATION,
                (MHI_DURATION - timestamp)*255./MHI_DURATION );
    cvZero( dst );
    cvCvtPlaneToPix( mask, 0, 0, 0, dst );

    // �����˶���ʷͼ����ݶȷ��� 
    // �����˶��ݶ�����ͺϷ�����������
    // calculate motion gradient orientation and valid orientation mask
    cvCalcMotionGradient( mhi, mask, orient, MAX_TIME_DELTA, MIN_TIME_DELTA, 3 );
    
    if( !storage )
        storage = cvCreateMemStorage(0);
    else
        cvClearMemStorage(storage);

    // �������˶��ָ�Ϊ�������˶����� 
    // �ָ��˶�:��ȡ�˶��������
    // �ָ��������˶����ͼ��ʶ�����ģ����ٹ����ʹ��
    // segment motion: get sequence of motion components
    // segmask is marked motion components map. It is not used further
    seq = cvSegmentMotion(mhi, segmask, storage, timestamp, MAX_TIME_DELTA);

	// ���˶��������Ŀ��ѭ��
    // ͨ���˶��������
    // ��������ͼ��(ȫ���˶�)������Ӧ��һ�λ��ε���
    // iterate through the motion components,
    // One more iteration (i == -1) corresponds to the whole image (global motion)
    for (i = -1; i < seq->total; i++) 
    {

        if (i < 0) 
        { 
            // ȫ���˶��¼�
            // case of the whole image
            // ��ȡ��ǰ֡�ķ�Χ
            comp_rect = cvRect( 0, 0, size.width, size.height );

			// ������ɫΪ��ɫ
			color = CV_RGB(255,255,255);

			// ���÷Ŵ���Ϊ100
            magnitude = 100;
        }
        else 
        { 
            // ��i���˶����
            // i-th motion component
            // ��ȡ��ǰ�˶�����ķ�Χ
            comp_rect = ((CvConnectedComp*)cvGetSeqElem( seq, i ))->rect;

			// ������С�����
			if( comp_rect.width + comp_rect.height < 100 ) // reject very small components
                continue;

			// ������ɫΪ��ɫ
			color = CV_RGB(255,0,0);

			// ���÷Ŵ���Ϊ30
            magnitude = 30;
        }

        // ѡ���������Ȥ������
        // select component ROI
        cvSetImageROI( silh, comp_rect );
        cvSetImageROI( mhi, comp_rect );
        cvSetImageROI( orient, comp_rect );
        cvSetImageROI( mask, comp_rect );

        // ����ĳЩѡ�������ȫ���˶����� 
        // ÿ���˶��������˶�����Ϳ��Ա��������������ȡ���ض���������ģ(mask)�������(ʹ��cvCmp) 
        // ��������
        // calculate orientation
        angle = cvCalcGlobalOrientation( orient, mask, mhi, timestamp, MHI_DURATION);

        // �������Ͻǵ�ԭ��������ͼ��ĽǶ�
        angle = 360.0 - angle;  // adjust for images with top-left origin

        // ��������ľ��Է����� ���Բ�ַ���������Բ�ַ��� 
        // ������������Ȥ�����е�ĸ���
        count = cvNorm( silh, 0, CV_L1, 0 ); // calculate number of points within silhouette ROI

        cvResetImageROI( mhi );
        cvResetImageROI( orient );
        cvResetImageROI( mask );
        cvResetImageROI( silh );

        // ���С�˶��¼�
        // check for the case of little motion
        if (count < comp_rect.width*comp_rect.height * 0.05)
        {
            continue;
        }

        // ��һ������ͷ��ʱ����ָʾ����
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

    // �������ͼ��
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

					// ���ɼ�����ͼ��ָ�븳���˶�ͼ��
                    motion->origin = image->origin;
                }
            }

			// �����˶���ʷͼ��
            update_mhi(image, motion, 30);

			// ��ʾ�˶�ͼ��
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
