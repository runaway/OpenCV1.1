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
static CvHaarClassifierCascade* cascade = 0; // ����������
static CvHaarClassifierCascade* nested_cascade = 0;
int use_nested_cascade = 0;

void detect_and_draw( IplImage* image );

// ����һ���ַ���������cascade������,���Ӧ����ѵ���ĵط�
//const char* cascade_name =
//    "../../data/haarcascades/haarcascade_frontalface_alt.xml";
const char* cascade_name =
"haarcascade_frontalface_alt.xml";
/*    "haarcascade_profileface.xml";*/
const char* nested_cascade_name =
    "../../data/haarcascades/haarcascade_eye_tree_eyeglasses.xml";
//    "../../data/haarcascades/haarcascade_eye.xml";
//1.3��Ԥ����ʱ������ϵ��,��Ҫ�����ǰѱ������ͼ���С.
//1.1�Ǽ�⺯���õ�һ���������,�����������������ڵ�.
double scale = 1;

int main(int argc, char** argv)
{
    // ������video ����avi�еõ�ͼ��Ľṹ
    CvCapture* capture = 0;

    // �ṹת���ɵ�ͼ��
    IplImage *frame, *frame_copy = 0;
    IplImage *image = 0;
    const char* scale_opt = "--scale=";

    // �������ε�int���ڼ���
    int scale_opt_len = (int)strlen(scale_opt);
    const char* cascade_opt = "--cascade=";
    int cascade_opt_len = (int)strlen(cascade_opt);
    const char* nested_cascade_opt = "--nested-cascade";
    int nested_cascade_opt_len = (int)strlen(nested_cascade_opt);
    int i;

    // ��avi����ͼ���ļ�������ļ���
    const char* input_name = 0;

    // ���������
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

    // ��ȡѵ���õķ�������
    // ���أ������������ѵ����
    cascade = (CvHaarClassifierCascade*)cvLoad( cascade_name, 0, 0, 0 );

    // ���ز��ɹ�����ʾ����ѶϢ�����˳�
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

    // ȷ���ڴ��СĬ��64k
    storage = cvCreateMemStorage(0);

    // ȷ���Ƿ�Ŀ�������ļ�������ͷ
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

    // ����һ���µ�window���ڣ���result����
    cvNamedWindow( "result", 1 );

    // ����ɹ�����capture
    if (capture)
    {
        for(;;)
        {
            // ����һ֡�����浽ͼ����
            if( !cvGrabFrame( capture ))
                break;
            frame = cvRetrieveFrame( capture );

            // ���֡�����ڣ��˳�ѭ��
            if( !frame )
                break;

            // ����ͼ���frameһ����С frame->nChannels
            if( !frame_copy )
                frame_copy = cvCreateImage( cvSize(frame->width,frame->height),
                                            IPL_DEPTH_8U, frame->nChannels );

            // ȷ��ͼ��ԭ�㣬����Ƕ���ṹ������ͼ��
            if (frame->origin == IPL_ORIGIN_TL)
            {
                cvCopy( frame, frame_copy, 0 );
            }
            else
            {
                // ��תͼ����x��
                cvFlip( frame, frame_copy, 0 );
            }

            // ���ú���ȷ������λ��
            detect_and_draw( frame_copy );

            // �ڴ�����һ֡ǰ�ȴ�һ���
            if( cvWaitKey( 10 ) >= 0 )
                goto _cleanup_;
        }

        cvWaitKey(0);
_cleanup_:
        // �ͷ�ͼ��Ͳ���֡���ڴ�
        cvReleaseImage( &frame_copy );
        cvReleaseCapture( &capture );
    }
    else // �������û�гɹ�����
    {
        // ���ͼ��ɹ����أ�����ô�����
        if (image)
        {
            // ��������������ʾ�¼�
            detect_and_draw( image );

            // �ȴ�
            cvWaitKey(0);

            // �ͷ��ڴ�
            cvReleaseImage( &image );
        }
        else if (input_name)
        {
            // �ٶ�����һ���ĵ�����ļ�����Ҫ������ͼ�����ƣ�ÿ��һ��
            /* assume it is a text file containing the
               list of the image filenames to be processed - one per line */
            FILE* f = fopen( input_name, "rt" );
            if( f )
            {
                char buf[1000+1];

                // ��f�ж�ȡ1000������
                while (fgets(buf, 1000, f))
                {
                    // ������ȥ�ո���ȷ�����ֳ���
                    int len = (int)strlen(buf), c;

                    // �ж��Ƿ�Ϊ�ո�
                    while( len > 0 && isspace(buf[len-1]) )
                        len--;
                    buf[len] = '\0';
                    printf( "file %s\n", buf ); 

                    // ����ȷ�����ֵ�ͼ��
                    image = cvLoadImage( buf, 1 );

                    // ����ļ����سɹ�����
                    if( image )
                    {
                        // �������
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

    // ���ٴ���
    cvDestroyWindow("result");

    return 0;
}

void detect_and_draw( IplImage* img )
{
    // �������ñ�ʾͼ������������ɫ
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

    // ������������ͼ����µ�ͼ��
    gray = cvCreateImage( cvSize(img->width,img->height), 8, 1 );
    small_img = cvCreateImage(cvSize(cvRound(img->width / scale),
                         cvRound(img->height / scale)), 8, 1);

    // ��imgת���ɻҶ�ͼ��gray
    cvCvtColor(img, gray, CV_BGR2GRAY);

    // �����Ҷ�ͼ���С�����small_img
    cvResize(gray, small_img, CV_INTER_LINEAR);

    // ֱ��ͼ����
    cvEqualizeHist(small_img, small_img);

    // ��ʹ��֮ǰ������ڴ�
    cvClearMemStorage( storage );

    // ��cascade�Ƿ񱻼��أ�������أ���
    if (cascade)
    {
        double t = (double)cvGetTickCount();

        // Ҳ��ͼ���У��в�ֹһ��face�����Դ���һ������face����
        // ���ͼ������������������洢��������
        CvSeq* faces = cvHaarDetectObjects(small_img, 
                                           cascade, 
                                           storage,
                                           1.2, // ��ⴰ�ڵķŴ�ϵ��1.1 
                                           2, // 2
                                           0
                                           //|CV_HAAR_FIND_BIGGEST_OBJECT
                                           //|CV_HAAR_DO_ROUGH_SEARCH
                                           |CV_HAAR_DO_CANNY_PRUNING,
                                           //|CV_HAAR_SCALE_IMAGE
                                           cvSize(30, 30));

        t = (double)cvGetTickCount() - t;

        // ��ӡ���ʱ��
        printf("detection time = %gms\n", 
            t / ((double)cvGetTickFrequency() * 1000.));

        // ѭ�����ֵ�face
        // �������������򻭳�������λ��
        for (i = 0; i < (faces ? faces->total : 0); i++)
        {
            CvRect* r = (CvRect*)cvGetSeqElem( faces, i );
            CvMat small_img_roi;
            CvSeq* nested_objects;

            // �������ȷ������λ�ã���Ϊ��cvRetangle��
            CvPoint center;

            // ��ͬ�������ò�����ɫ����
            CvScalar color = colors[i % 8];
            int radius;
            center.x = cvRound((r->x + r->width*0.5)*scale);
            center.y = cvRound((r->y + r->height*0.5)*scale);
            radius = cvRound((r->width + r->height)*0.25*scale);
            cvCircle( img, center, radius, color, 3, 8, 0 );
            if( !nested_cascade )
                continue;
            cvGetSubRect( small_img, &small_img_roi, *r );

            // ���Ƕ��Ŀ��
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
                // �����ϻ�һ��Բ
                // ����������ָ����Ԫ��ָ��
                CvRect* nr = (CvRect*)cvGetSeqElem(nested_objects, j);

                // �ҵ���Բ�����ĵ�
                center.x = cvRound((r->x + nr->x + nr->width*0.5)*scale);
                center.y = cvRound((r->y + nr->y + nr->height*0.5)*scale);
                radius = cvRound((nr->width + nr->height)*0.25*scale);

                // ��Բ
                cvCircle( img, center, radius, color, 3, 8, 0 );
            }
        }
    }

    cvShowImage( "result", img );
    cvReleaseImage( &gray );
    cvReleaseImage( &small_img );
}
