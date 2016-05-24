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
// ���ڹ������˶�����

/* ��飺�ڼ�����Ӿ��У�Lucas�CKanade�����㷨��һ����֡��ֵĹ��������㷨������Bruce D. Lucas �� Takeo Kanade�����

�����ĸ���:(Optical flow or optic flow)
����һ���˶�ģʽ�������˶�ģʽָ����һ�����塢���桢��Ե��һ���ӽ�����һ���۲��ߣ������۾�������ͷ�ȣ��ͱ���֮���γɵ������ƶ����������������˶�����ͼ��ָʱ����ײ���˶��������룬��ά�����Ӳ�������������ֱ�Ե������˶��ļ�����

��άͼ����ƶ�����ڹ۲��߶�������ά�����ƶ�����ͼ��ƽ���ͶӰ��
�����ͼ����Թ��Ƴ���άͼ���˲ʱͼ�����ʻ���ɢͼ��ת�ơ�

�����㷨��
������������ͼ���֮��ı��Σ����Ļ������������غ�ͼ�������غ㡣������һ���������ɫ��ǰ����֡û�о޴�����Եı仯���������˼·�����ǿ��Եõ�ͼ��Լ�����̡���ͬ�Ĺ����㷨����˼ٶ��˲�ͬ���������Ĺ������⡣

Lucas�CKanade�㷨��
����㷨������������еġ���������֡��ʱ��t ��t + ��t֮��ÿ��ÿ�����ص�λ�õ��ƶ����������ǻ���ͼ���źŵ�̩�ռ��������ַ�����Ϊ��֣�����Ƕ��ڿռ��ʱ������ʹ��ƫ������
ͼ��Լ�����̿���дΪI(x,y,z,t) = I(x + ��x,y + ��y,z + ��z,t + ��t)
I(x, y,z, t) Ϊ�ڣ�x,y,z��λ�õ����ء�
���Ǽ����ƶ��㹻��С����ô��ͼ��Լ������ʹ��̩�չ�ʽ�����ǿ��Եõ���


H.O.T. ָ���߽ף����ƶ��㹻С������¿��Ժ��ԡ���������������ǿ��Եõ���

����

���ǵõ���


Vx,Vy,Vz �ֱ���I(x,y,z,t)�Ĺ���������x��y��z����ɡ� , , ������ͼ����(x,y,z,t)��һ������Ӧ����Ĳ�֡�
����

IxVx + IyVy + IzVz = ? It��

д����

 

�������������δ֪�����в��ܱ��������Ҳ������ν�����㷨�Ĺ�Ȧ���⡣��ôҪ�ҵ�������������Ҫ��һ�׽���ķ�������Lucas-Kanade�㷨��һ���ǵ������㷨��

������(Vx,Vy,Vz)��һ����СΪm*m*m(m>1)��С������һ����������ô������1...n, n = m3�п��Եõ�����һ�鷽�̣�

 

 

 

 

����δ֪�������ж��������ķ��̣������������Ȼ�Ǹ��������̣�Ҳ����˵�������������࣬��������Ա�ʾΪ��

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

������

Ϊ�˽������������⣬���ǲ�����С���˷���

or

 

�õ���

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

���е�����Ǵ�1��n��

��Ҳ����˵Ѱ�ҹ�������ͨ������ά��ͼ�����ķֱ��ۼӵó������ǻ���Ҫһ��Ȩ�غ���W(i, j,k)�� ��ͻ���������ĵ�����ꡣ��˹������������Ƿǳ����ʵģ�

����㷨�Ĳ������������ܲ���һ���ܶȺܸߵ����������������˶��ı�Ե�ͺڴ��ͬ�������е�΢С�ƶ���������Ϣ��ܿ����ȥ�������ŵ��������������ڵ�³���Ի��ǿ��Եġ�

 

ʵ��Lucas-Kanade �����㷨���Է�Ϊ�������衣

��һ����Ϊ��ʼ����Ҫ���ٵĵ㡣

�ڶ�����Ϊ������֮֡��Ĺ����������ɳ�ʼ������Ҫ���ٵĵ��Ŀ��㣬Ϊ��Ҫ�ȼ������֡�Ĺ�����������

��������Ϊ�������������л�����������һ֡��Ŀǰ֡�Ļ����Լ���һ֡�뵱ǰ֡�������Ļ�����

�������������忴һ��ʵ�ֵĹ��̣�

 

//������������һ�ν��뷽��ʱ����Ϊ�����������ǿյģ����Բ���������δ��룬�������е���һ�������ٴν����������ʱ����Ϊ�ɵ�һ���õ���������Ѿ��ɵڶ���ת��Ϊ����㣬���������ǿգ����ǾͿ��Խ�������ķ������÷���ʱΪopencv����������ٵ���Ҫ��������ǰһ֡�뵱ǰ֡��ͼ�����֡�Ľ������õ����µ�����㡣��������Ĺ���Сʱ����Ӧ��m_status ��ֵΪ0�����ǿ��԰�û�й����ĵ����ɾ����

if(m_lInputPoints.size())

    {

        cvCalcOpticalFlowPyrLK(m_prev_grey, m_grey, m_prev_pyramid, m_pyramid,

&m_lInputPoints[0], &m_lOutputPoints[0], m_lInputPoints.size(), cvSize(m_win_size, m_win_size), 3, m_status, 0,

cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03), m_flags);

 

        m_flags |= CV_LKFLOW_PYR_A_READY;

}  

//��һ�������￪ʼ�����������Ѿ���ʼ���������㣬��������Ҫ�Ȱ�������������ĵ�������ڣ�Ȼ�����������ڵ�Ĺ�����

    cvFindCornerSubPix(m_grey, &m_lOutputPoints[m_lOutputPoints.size() - 1], 1,

                cvSize(m_win_size, m_win_size), cvSize(-1, -1),

                cvTermCriteria(CV_TERMCRIT_ITER|CV_TERMCRIT_EPS, 20, 0.03));

//�ڶ������ѵõ��������Ĺ����ŵ������������ڣ��Լ�����ǰһ֡�뵱ǰ֡������ǰһ֡�뵱ǰ֡�Ĺ����������´ε����㡣����������ϣ��������е��������������˳��÷��������õ�����һ֡��

m_lInputPoints = m_lOutputPoints;

    CV_SWAP(m_prev_grey, m_grey, m_swap_temp);

    CV_SWAP(m_prev_pyramid, m_pyramid, m_swap_temp);

���ˣ�Lucas-Kanade �����㷨�Ѿ��ɹ���ʵ�֡����Զ��趨����и��١��������ѡȡ��Щ�����ٵĵ����������Ż��ĸ��٣�����Ҫ�������һ�����⡣ 

����һ���ᵽ��Lucas-Kanade���������㷨����һ��׼ȷ�����죬�Ƚ�����ʵ�ֵ���������㷨���Ի����й̶�������׼ȷ���ٵĸ��١���������Ƶ����ζ��ƶ�������и����Լ����ٵ��ѡ��������һ����Ҫ��������⡣������������ϸ�˽�һ�¡�

 

���ȣ�����Ƶ���ƶ��������뾲ֹ�������һ��������ǣ���ǰһ֡�뵱ǰ֮֡�䣬��ֹ������û���˶��ģ��˶�������˶������������֮֡����һ���˶���������֡�Ĳ����ǿ�����cvAbsDiff������������ǿ���ͨ����λ���������������Ҫ���ٵĵ㡣����˶����������ǿ���ͨ��OpenCV�Ĳ��������ķ������ҵ�������ʹ����cvFindContours ��������Ϊ����ֻ���ҳ��˶����������������������Ҫ�ҵ����������ⲿ���������������Ҫ���������ķ���Ϊ��CV_RETR_EXTERNAL�������ɵ������ᱣ����һ��CvSeq�С���Ҫע����ǣ����ǿ�������Ƶ�����м��뻭���ƽ��������ȥ��һЩ��㣬�����˹�˲���

 

��Σ���Ȼ���������˶���������������Ǿ�Ҫ���������ת��Ϊ��Ҫ�����ٵĵ㣬һ���ձ�ķ����������Ǳ�������������CvSeq �е�ÿһ���������ٱ���ÿһ�������е�ÿ���㣬����Щ�����һ�����ƽ��ֵ�����ƽ��ֵ����������������ĵ㡣���ַ�����Ȼ��ȷ�����Ǽ����ٶȱȽ�������Ϊ�������Ƚϴ󣬶��ڴ����������ĵ��ǳ��ࡣ��һ�ַ������ǱȽ�ȡ�ɵķ����������ٶȺܿ죬Ч��Ҳ�ǳ��á���Ϊ������Ҫ���Ƕ��˶�����׼ȷ�ĸ��٣����ڶ�λ�˶�����������Ҫ�ܾ�ȷ��ֻҪ֪�������������˶��Ϳ����ˡ�������������ǰѲ��ҵ���ÿ��������cvMinAreaRect2 ת��ΪCvBox2D���͡�������͵ĺô����ǿ���ֱ�ӵ���CvBox2D �� Center �������������ĵ㣬������ĵ�Ϊ���������ı��ε����ĵ㣬���������ƽ��ֵ����Ч���ϵȼۡ�

 

������ǰ���֮֡�������˶����ɵ���Щ�㱣����һ��Vector ������Vector ��Ϊ���������з��͵�L-K�㷨�С�ͨ��L-K�㷨�������Щ���ڵ�ǰ֡���˶��յ㣬������һ֡����Щ�յ���Ϊ��ʼ���ٴ���L-K�㷨�������㡣��L-K��������У�������µ��˶��ĵ���룬���Ǿͼ�����Vector�е��µĵ㴫�뵽L-K �С�

 

���ˣ�L-K�㷨���Զ������Ѿ���ϣ����ǿ���ͨ������һЩ�˲��Լ���Ե�������׼ȷ�Ķ�λ��Ҫ���ٵĵ㣬�����������˶����λ��ƫ��������X������Y���򣬻���Z������Ҫ��������ͷ�������Ե����������뵽��Ӧ���˶�������ʾģ���У��������Ӧ��Ч����
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
