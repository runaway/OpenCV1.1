// Added by Runaway at 20090515
#define _AFXDLL
#include "stdafx.h" 

#ifdef _CH_
#pragma package <opencv>
#endif

#ifndef _EiC
#include "cv.h"
#include "highgui.h"
#include <STDIO.H>  

// Added by Runaway at 20090515
#include <TIME.H>    
#include <VECTOR>     
using namespace std;   
#endif

/* ��Open CV �е�ƽ�滮����غ���ʹ��̽����ת��(2009-04-02 21:21:02)��ǩ��it   ���ࣺѧϰ�ʼ� 
ת�ԣ�http://vckbase.com/document/viewdoc/?id=1794 ���ߣ���ѫ
ժҪ��Delaunay�����ʷ��ڹ���Ӧ���зǳ����ã���Դ��OpenCVҲ�ṩ����Ӧ�ĺ���������
����ԭʼ�ĵ����Ǻ���ϸ����ʹ�ù�������Ȼ�������ܶ��鷳�����߾��Լ�������������
�ص��ܽᣬ�������ʵ��Ӧ���е�������⡣
�ؼ��֣�open cv, delaunay, ƽ�滮��
����㼯����������(triangulation)����һֱ���������й�ע�����⡣��������������
��������й㷺Ӧ�á�Delaunay �����ʷ���Ŀǰ�о�Ӧ������һ���ʷַ���������߱���
���ŵ㣬���¼��о�������
�����Բ���ʣ����ɵ㼯V-���ɵ�D-�������У�ÿ�������ε����Բ���������õ㼯������
����㡣
�����С�Ƕ����ʣ����ɵ㼯V-���ɵ�D-�������У������������е���С�Ƕ������ġ�
Open CV����Delaunay��ʵ�֣�����ط����˹����й����ߡ�����Open CV�ṩ����ϸ����
���������ṩ�����sample���������ڶ�ԭ�ĵ����ο��鼮[1,3,4]������ϵĲ��㣬������
ʹ�ù�������Ȼ�����ܶ����⣬���潫�Լ���һЩ��⼰̽�������ܽᣬ����֮���������
��ָ������ע�⣬���ﲢ������ϸ����Open CV��ν���delaunay���֣���ο�Open CV�Դ�
��ʾ������delaunay.c.

1. Ҳ˵���ķ���Ե��Quad-edge�����ṹ
ͼ1 ��e�Լ����e��صı�(��ͼ����Open CV�ĵ�)
����ṹͼ�ǳ��Ѷ�(���Ҷ���)�����Ƿǳ��ؼ�����Open CV ƽ�滮�ֵ������Ԫ�أ�����
�ṹ���£�
#define CV_QUADEDGE2D_FIELDS() \ int flags; \ struct CvSubdiv2DPoint* pt[4]; 
\ CvSubdiv2DEdge next[4]; typedef struct CvQuadEdge2D{CV_QUADEDGE2D_FIELDS()} 
CvQuadEdge2D;
����ṹ�Ĺؼ�����������next[4],�䰴˳���������ߣ����룩��e,eRot�Լ����ǵķ���
�ߡ�ע���������á��ߴ��롱��ԭ����Open CV����long�͵Ĵ�������ʾƽ�滮�ֵ�һ���ߡ�
eLnext: e�������򣨻��Ϸ�������һ������Σ�e������һ���ߣ�eLnext��ָ�������
�����һ���ߣ���������е����������ݽṹ�е�ʮ������ı�ʾ������e��������һ��ģ�
eRnext: ���ͬeLnext,ֻ������ָ��e���ҷ����򣨻��·����򣩵���һ���ߣ���e������
��һ��ģ�
eDnext: ��e����Ŀ�ĵ㡱����һ���ߣ�
eOnext: ��e���������㡱����һ���ߣ�
eRot: e�Ķ�ż�ߣ����ûʲô�ý��͵��ˡ�
���˽���Щ֪ʶ������ǿ��Ի�����µ�Ӧ�ã� 

2. Ӧ��1����Delaunay���ֽ������ȡ���������ӹ�ϵ
(1) ���ȣ��Ա�e��ʼѭ���������������������߾Ϳ����ҵ�һ�������Σ������б߽�����ͬ
�������Ϳ����ҵ���������Σ�ע�⣬������������ظ��ıߣ���Ҫ�����жϡ��������£�
for( i = 0; i < total; i++ ) // total�Ǳ���Ŀ�����Բο�Open CVʾ������delaunay.c
{ CvQuadEdge2D* edge = (CvQuadEdge2D*)(reader.ptr); if( CV_IS_SET_ELEM( edge )) 
{ CvSubdiv2DEdge e = (CvSubdiv2DEdge)edge; CvSubdiv2DEdge t = e; CvPoint buf[3]; 
int iPointNum = 3; for(int j = 0; j < iPointNum; j++ ){ CvSubdiv2DPoint* pt = 
cvSubdiv2DEdgeOrg( t ); if( !pt ) break; buf[j] = cvPoint( cvRound(pt->pt.x), 
cvRound(pt->pt.y)); t = cvSubdiv2DGetEdge( t, CV_NEXT_AROUND_LEFT ); } 
if (j == iPointNum) { AddTriangle(buf); // ��������� } 
CV_NEXT_SEQ_ELEM( elem_size, reader ); }
(2) ��Σ���Ϊ��Delaunay�����У����б����з���ģ���ͨ��e������ѭ���ܻ���ʧ������
���Σ����ͬʱ������e�ķ���߽�����ѭ������Ĵ�����Ը�Ϊ���£� 
Bool FindTriangleFromEdge(CvSubdiv2DEdge e) { CvSubdiv2DEdge t = e; CvPoint 
buf[3]; CvPoint *pBuf = buf; int iPointNum = 3; for(int j = 0; j < iPointNum; 
j++ ){ CvSubdiv2DPoint* pt = cvSubdiv2DEdgeOrg( t ); if( !pt ) break; buf[j] = 
cvPoint( cvRound(pt->pt.x), cvRound(pt->pt.y)); t = cvSubdiv2DGetEdge( t, 
CV_NEXT_AROUND_LEFT ); } if (j == iPointNum) { AddTriangle(buf); // ��������� 
return true; } return false; } // ���ô������� for( i = 0; i < total; i++ ) 
{ CvQuadEdge2D* edge = (CvQuadEdge2D*)(reader.ptr); if( CV_IS_SET_ELEM( edge )) 
{ CvSubdiv2DEdge e = (CvSubdiv2DEdge)edge; FindTriangleFromEdge(e); 
CvSubdiv2DEdge e1 = (CvSubdiv2DEdge)edge+2; //��next[2] FindTriangleFromEdge(e1); } 
CV_NEXT_SEQ_ELEM( elem_size, reader ); }
������Ĵ����У���ֱ�Ӳ�������λ�Ʒ����и��ֱߵĶ�Ӧ��(��edge+2)������Open CV�Ѿ�
�����Լ���ʵ�ֺ�����cvSubdiv2DRotateEdge�������ú�ɫ�����ע�������Ի�Ϊ�� 
CvSubdiv2DEdge e1 = cvSubdiv2DRotateEdge((CvSubdiv2DEdge)edge,2);
����16��������룬Delaunay�ָ�Ľ����ͼ2��ʾ�� 

3. Ӧ��2����Vonoroi���ֽ������ȡ�����
�ο�Delaunay.c�еĺ�����void paint_voronoi( CvSubdiv2D* subdiv, IplImage* img );
�����ͼ3��ʾ������Ӧ��1�ķ����������δ���Ҳ�����ס�
ͼ2. 16�����Delaunay�����ʷֽ�� ͼ3. ��Ӧ��Voronoi���ֽ��   */

typedef struct TRI   
{   
    CvPoint2D32f a;   
    CvPoint2D32f b;   
    CvPoint2D32f c;   
}   
Tri;   
   
vector<Tri> tri;//����Ѽ�⵽���������б�    
   
char win[] = "source";   
char win2[] = "win2";   
IplImage* img, *img2;   
const int N = 16;   
CvPoint2D32f fp[] = 
{   // Test data    
/*  
    50,50,  
    200,400,  
    500,50,  
    200,200   
*/   
107, 64,   
172, 62,   
102, 107,   
126, 106,   
157, 108,   
182, 106,   
142, 156,   
134, 167,   
148, 167,   
123, 203,   
161, 202,   
142, 253,   
63, 166,   
209, 159,   
141, 80,   
141, 108   
};   
   
bool Search(Tri triTemp) //���Ҽ�⵽���������Ƿ���ڣ����ڷ���true,���򷵻�false    
{   
    int i;   
    int total = tri.size();   
    for (i = 0; i < total; i++)   
    {          
        if ((tri[i].a.x == triTemp.a.x)&&(tri[i].a.y == triTemp.a.y)&&   
            (tri[i].b.x == triTemp.b.x)&&(tri[i].b.y == triTemp.b.y)&&   
            (tri[i].c.x == triTemp.c.x)&&(tri[i].c.y == triTemp.c.y))   
        {   
            return true;   
        }   
        if ((tri[i].a.x == triTemp.b.x)&&(tri[i].a.y == triTemp.b.y)&&   
            (tri[i].b.x == triTemp.c.x)&&(tri[i].b.y == triTemp.c.y)&&   
            (tri[i].c.x == triTemp.a.x)&&(tri[i].c.y == triTemp.a.y))   
        {   
            return true;   
        }   
        if ((tri[i].a.x == triTemp.c.x)&&(tri[i].a.y == triTemp.c.y)&&   
            (tri[i].b.x == triTemp.a.x)&&(tri[i].b.y == triTemp.a.y)&&   
            (tri[i].c.x == triTemp.b.x)&&(tri[i].c.y == triTemp.b.y))   
        {   
            return true;   
        }   
        if ((tri[i].a.x == triTemp.a.x)&&(tri[i].a.y == triTemp.a.y)&&   
            (tri[i].b.x == triTemp.c.x)&&(tri[i].b.y == triTemp.c.y)&&   
            (tri[i].c.x == triTemp.b.x)&&(tri[i].c.y == triTemp.b.y))   
        {   
            return true;   
        }   
        if ((tri[i].a.x == triTemp.b.x)&&(tri[i].a.y == triTemp.b.y)&&   
            (tri[i].b.x == triTemp.a.x)&&(tri[i].b.y == triTemp.a.y)&&   
            (tri[i].c.x == triTemp.c.x)&&(tri[i].c.y == triTemp.c.y))   
        {   
            return true;   
        }   
        if ((tri[i].a.x == triTemp.c.x)&&(tri[i].a.y == triTemp.c.y)&&   
            (tri[i].b.x == triTemp.b.x)&&(tri[i].b.y == triTemp.b.y)&&   
            (tri[i].c.x == triTemp.a.x)&&(tri[i].c.y == triTemp.a.y))   
        {   
            return true;   
        }   
    }   
    return false;   
}   


/* the script demostrates iterative construction of
   delaunay triangulation and voronoi tesselation */

CvSubdiv2D* init_delaunay( CvMemStorage* storage,
                           CvRect rect )
{
    CvSubdiv2D* subdiv;

    subdiv = cvCreateSubdiv2D( CV_SEQ_KIND_SUBDIV2D, sizeof(*subdiv),
                               sizeof(CvSubdiv2DPoint),
                               sizeof(CvQuadEdge2D),
                               storage );
    cvInitSubdivDelaunay2D( subdiv, rect );

    return subdiv;
}


void draw_subdiv_point( IplImage* img, CvPoint2D32f fp, CvScalar color )
{
    cvCircle( img, cvPoint(cvRound(fp.x), cvRound(fp.y)), 3, color, CV_FILLED, 8, 0 );
}


void draw_subdiv_edge( IplImage* img, CvSubdiv2DEdge edge, CvScalar color )
{
    CvSubdiv2DPoint* org_pt;
    CvSubdiv2DPoint* dst_pt;
    CvPoint2D32f org;
    CvPoint2D32f dst;
    CvPoint iorg, idst;

    // ���ر�Ե��ԭ��,��ñ�edge����ʼ��
    org_pt = cvSubdiv2DEdgeOrg(edge);

    // ���ر�Ե���յ�,��ñ�edge����ֹ��
    dst_pt = cvSubdiv2DEdgeDst(edge);

    if( org_pt && dst_pt )
    {
        org = org_pt->pt;
        dst = dst_pt->pt;

        // cvRound�ǽ�������ת��Ϊ���� 
        iorg = cvPoint( cvRound( org.x ), cvRound( org.y ));
        idst = cvPoint( cvRound( dst.x ), cvRound( dst.y ));

        cvLine( img, iorg, idst, color, 1, CV_AA, 0 );
    }
}


void draw_subdiv( IplImage* img, CvSubdiv2D* subdiv,
                  CvScalar delaunay_color, CvScalar voronoi_color )
{
    CvSeqReader  reader;
    int i, total = subdiv->edges->total;
    int elem_size = subdiv->edges->elem_size;

    cvStartReadSeq( (CvSeq*)(subdiv->edges), &reader, 0 );

    for( i = 0; i < total; i++ )
    {
        CvQuadEdge2D* edge = (CvQuadEdge2D*)(reader.ptr);

        if( CV_IS_SET_ELEM( edge ))
        {
            draw_subdiv_edge( img, (CvSubdiv2DEdge)edge + 1, voronoi_color );
            draw_subdiv_edge( img, (CvSubdiv2DEdge)edge, delaunay_color );
        }

        CV_NEXT_SEQ_ELEM( elem_size, reader );
    }
}


void locate_point(CvSubdiv2D* subdiv, 
                  CvPoint2D32f fp, 
                  IplImage* img,
                  CvScalar active_color)
{
    CvSubdiv2DEdge e;

    // ������������Ӧ�������Ե(�������ϻ�������) 
    CvSubdiv2DEdge e0 = 0;

    // ������������Ӧ�������������(ָ��double����)����ѡ��
    CvSubdiv2DPoint* p = 0;

    // ����һ����,����һ����������Ӧ�������Ե(�������ϻ�������)����������Ӧ
    // �������������(ָ��double����) 
    // �� Delaunay���ǲ����ж�λ�����
    cvSubdiv2DLocate(subdiv, fp, &e0, &p);

    if (e0)
    {
        e = e0;

        do
        {
            // ���ݱ�e����ʼ�㻭��
            draw_subdiv_edge( img, e, active_color );

            // ���ظ����ı�Ե֮һ 
            e = cvSubdiv2DGetEdge(e,CV_NEXT_AROUND_LEFT);
        }
        while( e != e0 );
    }

    // ����fp�����껭��
    draw_subdiv_point( img, fp, active_color );
}


void draw_subdiv_facet( IplImage* img, CvSubdiv2DEdge edge )
{
    CvSubdiv2DEdge t = edge;
    int i, count = 0;
    CvPoint* buf = 0;

    // count number of edges in facet
    do
    {
        count++;

        // ���ظ����ı�Ե֮һ 
        t = cvSubdiv2DGetEdge( t, CV_NEXT_AROUND_LEFT );
    } while (t != edge );

    buf = (CvPoint*)malloc( count * sizeof(buf[0]));

    // gather points
    t = edge;
    for( i = 0; i < count; i++ )
    {
        CvSubdiv2DPoint* pt = cvSubdiv2DEdgeOrg( t );
        if( !pt ) break;
        buf[i] = cvPoint( cvRound(pt->pt.x), cvRound(pt->pt.y));
        t = cvSubdiv2DGetEdge( t, CV_NEXT_AROUND_LEFT );
    }

    if( i == count )
    {
        CvSubdiv2DPoint* pt = cvSubdiv2DEdgeDst( cvSubdiv2DRotateEdge( edge, 1 ));

        // ���͹�����
        cvFillConvexPoly( img, buf, count, CV_RGB(rand()&255,rand()&255,rand()&255), CV_AA, 0 );
        cvPolyLine( img, &buf, &count, 1, 1, CV_RGB(0,0,0), 1, CV_AA, 0);
        draw_subdiv_point( img, pt->pt, CV_RGB(0,0,0));
    }
    free( buf );
}

/* Voronoiͼ���ֽ�̩ɭ����λ�Dirichletͼ��������һ�����������ڵ�ֱ�ߵĴ�ֱƽ����
��ɵ������������ɡ�N����ƽ����������ĵ㣬�������ڽ�ԭ�򻮷�ƽ�棻ÿ����������
����������������Delaunay����������������Voronoi����ι���һ���ߵ���ص����Ӷ���
�������Ρ�Delaunay�����ε����ԲԲ��������������ص�Voronoi����ε�һ�����㡣
Voronoi��������Delaunayͼ��żͼ */
void paint_voronoi( CvSubdiv2D* subdiv, IplImage* img )
{
    CvSeqReader  reader;
    int i, total = subdiv->edges->total;
    int elem_size = subdiv->edges->elem_size;

    // ����Voronoiͼ���ϸ���ṹ
    cvCalcSubdivVoronoi2D( subdiv );

    cvStartReadSeq( (CvSeq*)(subdiv->edges), &reader, 0 );

    for( i = 0; i < total; i++ )
    {
        CvQuadEdge2D* edge = (CvQuadEdge2D*)(reader.ptr);

        if( CV_IS_SET_ELEM( edge ))
        {
            CvSubdiv2DEdge e = (CvSubdiv2DEdge)edge;

            // ����ͬһ���ķ���Ե�ṹ�е���һ����Ե
            // left
            draw_subdiv_facet( img, cvSubdiv2DRotateEdge( e, 1 ));

            // right
            draw_subdiv_facet( img, cvSubdiv2DRotateEdge( e, 3 ));
        }

        CV_NEXT_SEQ_ELEM( elem_size, reader );
    }
}

void triangles(CvSubdiv2D* subdiv, IplImage* img)   
{   
	int j = 0;

    CvSeqReader  reader;   
    int i, total = subdiv->edges->total;   
    int elem_size = subdiv->edges->elem_size;   
   
    Tri triTemp;   
    cvStartReadSeq( (CvSeq*)(subdiv->edges), &reader, 0 );   
    int num = 0;   
    srand( (unsigned)time( NULL ) );   
    for( i = 0; i < total; i++ )   
    {   
        CvQuadEdge2D* edge = (CvQuadEdge2D*)(reader.ptr);   
   
        if( CV_IS_SET_ELEM( edge ))   
        {   
            CvSubdiv2DEdge e = (CvSubdiv2DEdge)edge;   
            CvSubdiv2DEdge t = e;   
            CvPoint buf[3];   
            CvPoint *pBuf = buf;   
            int iPointNum = 3;             
   
            for(j = 0; j < iPointNum; j++ ){   
                CvSubdiv2DPoint* pt = cvSubdiv2DEdgeOrg( t );   
                if( !pt )    
                    break;   
                buf[j] = cvPoint( cvRound(pt->pt.x), cvRound(pt->pt.y));   
                t = cvSubdiv2DGetEdge( t, CV_NEXT_AROUND_LEFT );   
            }   
            if (abs(buf[0].x) != 1800 && abs(buf[0].y) != 1800 &&   
                abs(buf[1].x) != 1800 && abs(buf[1].y) != 1800 &&   
                abs(buf[2].x) != 1800 && abs(buf[2].y) != 1800 && j == 3)   
            {   
                triTemp.a.x = buf[0].x;   
                triTemp.a.y = buf[0].y;   
                triTemp.b.x = buf[1].x;   
                triTemp.b.y = buf[1].y;   
                triTemp.c.x = buf[2].x;   
                triTemp.c.y = buf[2].y;   
                if (!Search(triTemp))//û�м�⵽    
                {   
                    tri.push_back(triTemp);   
                    cvFillConvexPoly( img, buf, iPointNum, CV_RGB(rand()&255,rand()&255,rand()&255), CV_AA, 0 );   
                    cvPolyLine( img, &pBuf, &iPointNum, 1, 1, CV_RGB(0,0,0), 1, CV_AA, 0);   
                       
                    cvShowImage(win,img);   
                    cvWaitKey(-1);   
                       
                    printf("%d: (%d, %d)-(%d, %d)-(%d, %d)\n", num++, buf[0].x, buf[0].y, buf[1].x, buf[1].y, buf[2].x, buf[2].y);   
                }   
            }   
   
//////////////////////////////////////////////////////////////////////////    
            CvSubdiv2DEdge e1 = (CvSubdiv2DEdge)edge+2;   
            t = e1;   
               
            for(j = 0; j < iPointNum; j++ ){   
                CvSubdiv2DPoint* pt = cvSubdiv2DEdgeOrg( t );   
                if( !pt ) break;   
                buf[j] = cvPoint( cvRound(pt->pt.x), cvRound(pt->pt.y));   
                t = cvSubdiv2DGetEdge( t, CV_NEXT_AROUND_LEFT );   
            }   
            if (abs(buf[0].x) != 1800 && abs(buf[0].y) != 1800 &&   
                abs(buf[1].x) != 1800 && abs(buf[1].y) != 1800 &&   
                abs(buf[2].x) != 1800 && abs(buf[2].y) != 1800 && j == 3)    
            {   
                triTemp.a.x = buf[0].x;   
                triTemp.a.y = buf[0].y;   
                triTemp.b.x = buf[1].x;   
                triTemp.b.y = buf[1].y;   
                triTemp.c.x = buf[2].x;   
                triTemp.c.y = buf[2].y;   
                if (!Search(triTemp))   
                {   
                    tri.push_back(triTemp);   
                    cvFillConvexPoly( img, buf, iPointNum, CV_RGB(rand()&255,rand()&255,rand()&255), CV_AA, 0 );   
                    cvPolyLine( img, &pBuf, &iPointNum, 1, 1, CV_RGB(0,0,0), 1, CV_AA, 0);   
   
                    cvShowImage(win,img);   
                    cvWaitKey(-1);   
   
                    printf("%d: (%d, %d)-(%d, %d)-(%d, %d)\n", num++, buf[0].x, buf[0].y, buf[1].x, buf[1].y, buf[2].x, buf[2].y);   
                }   
            }          
        }          
        CV_NEXT_SEQ_ELEM( elem_size, reader );   
    }   
}   


void run(void)
{
    char win[] = "source";
    int i;
    CvRect rect = { 0, 0, 600, 600 }; // ����һ�����ο�

    // �ڴ�洢����һ���������洢��������,����,ͼ��,�ӻ��ֵȶ�̬�������ݽṹ�ĵײ�ṹ�� 
    CvMemStorage* storage;
    CvSubdiv2D* subdiv;
    IplImage* img;
    CvScalar active_facet_color, delaunay_color, voronoi_color, bkgnd_color;

    
    active_facet_color = CV_RGB(255, 0, 0); // �²���ĵ��ʶΪ��ɫ 
    delaunay_color = CV_RGB(0, 0, 0); // delaunay�����α�ʶΪ��ɫ
    voronoi_color = CV_RGB(0, 180, 0); // voronoi�߿��ʶΪ��ɫ
    bkgnd_color = CV_RGB(255, 255, 255); // ����ɫΪ��ɫ 

	// ����ͼ��img��ʵ�ʻ���D-����ͼ
    img = cvCreateImage(cvSize(rect.width,rect.height), 8, 3);

    // ���ñ���
    cvSet(img, bkgnd_color, 0);  

    cvNamedWindow( win, 1 );

    storage = cvCreateMemStorage(0);

    // ����ڴ����ĳ�ʼ���Ĺ���
    subdiv = init_delaunay(storage, rect);

    printf("Delaunay triangulation will be build now interactively.\n"
           "To stop the process, press any key\n\n");

    // �ܹ���N������
    for (i = 0; i < 1000; i++) // 200
    {
        // ���������������Ķ�ά��
        CvPoint2D32f fp = cvPoint2D32f((float)(rand() % (rect.width - 10) + 5),
                                       (float)(rand() % (rect.height - 10) + 5));

        // ���²���ĵ���ͼ�ϱ�ʶ����
        locate_point(subdiv, fp, img, active_facet_color);
        cvShowImage(win, img);

        if (cvWaitKey(100) >= 0)
            break;

        // �²���һ������������Delaunay����ͼ 
        // ��Delaunay���ǲ����в���һ����
        cvSubdivDelaunay2DInsert(subdiv, fp);

        // ����Voronoiͼ���ϸ���ṹ 
        cvCalcSubdivVoronoi2D( subdiv );
        cvSet( img, bkgnd_color, 0 );
        draw_subdiv( img, subdiv, delaunay_color, voronoi_color );
        cvShowImage( win, img );

        if (cvWaitKey(100) >= 0)
            break;
    }

    cvSet( img, bkgnd_color, 0 );

    // ��Voronoiͼ��
    paint_voronoi( subdiv, img );
    cvShowImage( win, img );

    // triangles(subdiv, img); //����D-����ͼ�ڵ����������� 

    cvWaitKey(0);

    cvReleaseMemStorage( &storage );
    cvReleaseImage(&img);
    cvDestroyWindow( win );
}

int main( int argc, char** argv )
{
    argc, argv;
    run();
    return 0;
}

#ifdef _EiC
main( 1, "delaunay.c" );
#endif
