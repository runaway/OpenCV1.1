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

/* 对Open CV 中的平面划分相关函数使用探索【转】(2009-04-02 21:21:02)标签：it   分类：学习笔记 
转自：http://vckbase.com/document/viewdoc/?id=1794 作者：龚勋
摘要：Delaunay三角剖分在工程应用中非常有用，开源库OpenCV也提供了相应的函数，但是
由于原始文档不是很详细，在使用过程中仍然会遇到很多麻烦，笔者就自己的理解进行了相
关的总结，并解决了实际应用中的相关问题。
关键字：open cv, delaunay, 平面划分
任意点集的三角网格化(triangulation)问题一直是人们密切关注的问题。三角网格化问题在
许多领域有广泛应用。Delaunay 三角剖分是目前研究应用最广的一种剖分方法，因其具备很
多优点，以下简单列举两条：
空外接圆性质：在由点集V-生成的D-三角网中，每个三角形的外接圆均不包含该点集的其他
任意点。
最大最小角度性质：在由点集V-生成的D-三角网中，所有三角形中的最小角度是最大的。
Open CV中有Delaunay的实现，极大地方便了广大科研工作者。尽管Open CV提供了详细的文
档，并且提供了相关sample，但是由于对原文档及参考书籍[1,3,4]的理解上的不足，笔者在
使用过程中仍然遇到很多问题，下面将自己的一些理解及探索进行总结，谬误之处望大家批
评指正。请注意，这里并不会详细介绍Open CV如何进行delaunay划分，请参考Open CV自带
的示例程序delaunay.c.

1. 也说“四方边缘（Quad-edge）”结构
图1 边e以及与边e相关的边(该图来自Open CV文档)
这个结构图非常难懂(对我而言)，但是非常关键，是Open CV 平面划分的最基本元素，数据
结构如下：
#define CV_QUADEDGE2D_FIELDS() \ int flags; \ struct CvSubdiv2DPoint* pt[4]; 
\ CvSubdiv2DEdge next[4]; typedef struct CvQuadEdge2D{CV_QUADEDGE2D_FIELDS()} 
CvQuadEdge2D;
这个结构的关键数据是数组next[4],其按顺序存放四条边（代码）：e,eRot以及它们的反向
边。注：我这里用“边代码”，原因是Open CV是用long型的代码来表示平面划分的一条边。
eLnext: e的左方区域（或上方区域）是一个多边形，e是其中一条边，eLnext是指向这个区
域的另一条边，这个描述有点类似于数据结构中的十字链表的表示法，与e是连接在一起的；
eRnext: 理解同eLnext,只不过是指向e的右方区域（或下方区域）的另一条边，与e是连接
在一起的；
eDnext: 与e共“目的点”的另一条边；
eOnext: 与e共“出发点”的另一条边；
eRot: e的对偶边，这就没什么好解释的了。
在了解这些知识点后，我们可以获得如下的应用： 

2. 应用1－在Delaunay划分结束后获取三角形连接关系
(1) 首先，以边e开始循环查找与其相连的两条边就可以找到一个三角形，对所有边进行相同
操作，就可以找到许多三角形，注意，这其中有许多重复的边，需要进行判断。代码如下：
for( i = 0; i < total; i++ ) // total是边数目，可以参考Open CV示例程序delaunay.c
{ CvQuadEdge2D* edge = (CvQuadEdge2D*)(reader.ptr); if( CV_IS_SET_ELEM( edge )) 
{ CvSubdiv2DEdge e = (CvSubdiv2DEdge)edge; CvSubdiv2DEdge t = e; CvPoint buf[3]; 
int iPointNum = 3; for(int j = 0; j < iPointNum; j++ ){ CvSubdiv2DPoint* pt = 
cvSubdiv2DEdgeOrg( t ); if( !pt ) break; buf[j] = cvPoint( cvRound(pt->pt.x), 
cvRound(pt->pt.y)); t = cvSubdiv2DGetEdge( t, CV_NEXT_AROUND_LEFT ); } 
if (j == iPointNum) { AddTriangle(buf); // 添加三角形 } 
CV_NEXT_SEQ_ELEM( elem_size, reader ); }
(2) 其次，因为在Delaunay划分中，所有边是有方向的，光通过e进行轮循可能会遗失部分三
角形，因此同时还得以e的反向边进行轮循，上面的代码可以改为如下： 
Bool FindTriangleFromEdge(CvSubdiv2DEdge e) { CvSubdiv2DEdge t = e; CvPoint 
buf[3]; CvPoint *pBuf = buf; int iPointNum = 3; for(int j = 0; j < iPointNum; 
j++ ){ CvSubdiv2DPoint* pt = cvSubdiv2DEdgeOrg( t ); if( !pt ) break; buf[j] = 
cvPoint( cvRound(pt->pt.x), cvRound(pt->pt.y)); t = cvSubdiv2DGetEdge( t, 
CV_NEXT_AROUND_LEFT ); } if (j == iPointNum) { AddTriangle(buf); // 添加三角形 
return true; } return false; } // 调用代码如下 for( i = 0; i < total; i++ ) 
{ CvQuadEdge2D* edge = (CvQuadEdge2D*)(reader.ptr); if( CV_IS_SET_ELEM( edge )) 
{ CvSubdiv2DEdge e = (CvSubdiv2DEdge)edge; FindTriangleFromEdge(e); 
CvSubdiv2DEdge e1 = (CvSubdiv2DEdge)edge+2; //即next[2] FindTriangleFromEdge(e1); } 
CV_NEXT_SEQ_ELEM( elem_size, reader ); }
在上面的代码中，是直接采用数组位移法进行各种边的对应的(即edge+2)，不过Open CV已经
有了自己的实现函数：cvSubdiv2DRotateEdge，上面用红色粗体标注的语句可以换为： 
CvSubdiv2DEdge e1 = cvSubdiv2DRotateEdge((CvSubdiv2DEdge)edge,2);
对于16个点的输入，Delaunay分割的结果如图2所示。 

3. 应用2－在Vonoroi划分结束后获取多边形
参考Delaunay.c中的函数：void paint_voronoi( CvSubdiv2D* subdiv, IplImage* img );
结果如图3所示。有了应用1的分析，理解这段代码也很容易。
图2. 16个点的Delaunay三角剖分结果 图3. 相应的Voronoi划分结果   */

typedef struct TRI   
{   
    CvPoint2D32f a;   
    CvPoint2D32f b;   
    CvPoint2D32f c;   
}   
Tri;   
   
vector<Tri> tri;//存放已检测到的三角形列表    
   
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
   
bool Search(Tri triTemp) //查找检测到的三角形是否存在，存在返回true,否则返回false    
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

    // 返回边缘的原点,获得边edge的起始点
    org_pt = cvSubdiv2DEdgeOrg(edge);

    // 返回边缘的终点,获得边edge的终止点
    dst_pt = cvSubdiv2DEdgeDst(edge);

    if( org_pt && dst_pt )
    {
        org = org_pt->pt;
        dst = dst_pt->pt;

        // cvRound是将浮点数转换为整数 
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

    // 定义与输入点对应的输入边缘(点在其上或者其右) 
    CvSubdiv2DEdge e0 = 0;

    // 定义与输入点对应的输出顶点坐标(指向double类型)，可选。
    CvSubdiv2DPoint* p = 0;

    // 输入一个点,返回一个与输入点对应的输入边缘(点在其上或者其右)，与输入点对应
    // 的输出顶点坐标(指向double类型) 
    // 在 Delaunay三角测量中定位输入点
    cvSubdiv2DLocate(subdiv, fp, &e0, &p);

    if (e0)
    {
        e = e0;

        do
        {
            // 根据边e的起始点画边
            draw_subdiv_edge( img, e, active_color );

            // 返回给定的边缘之一 
            e = cvSubdiv2DGetEdge(e,CV_NEXT_AROUND_LEFT);
        }
        while( e != e0 );
    }

    // 根据fp的坐标画点
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

        // 返回给定的边缘之一 
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

        // 填充凸多边形
        cvFillConvexPoly( img, buf, count, CV_RGB(rand()&255,rand()&255,rand()&255), CV_AA, 0 );
        cvPolyLine( img, &buf, &count, 1, 1, CV_RGB(0,0,0), 1, CV_AA, 0);
        draw_subdiv_point( img, pt->pt, CV_RGB(0,0,0));
    }
    free( buf );
}

/* Voronoi图，又叫泰森多边形或Dirichlet图，它是由一组由连接两邻点直线的垂直平分线
组成的连续多边形组成。N个在平面上有区别的点，按照最邻近原则划分平面；每个点与它的
最近邻区域相关联。Delaunay三角形是由与相邻Voronoi多边形共享一条边的相关点连接而成
的三角形。Delaunay三角形的外接圆圆心是与三角形相关的Voronoi多边形的一个顶点。
Voronoi三角形是Delaunay图的偶图 */
void paint_voronoi( CvSubdiv2D* subdiv, IplImage* img )
{
    CvSeqReader  reader;
    int i, total = subdiv->edges->total;
    int elem_size = subdiv->edges->elem_size;

    // 计算Voronoi图表的细胞结构
    cvCalcSubdivVoronoi2D( subdiv );

    cvStartReadSeq( (CvSeq*)(subdiv->edges), &reader, 0 );

    for( i = 0; i < total; i++ )
    {
        CvQuadEdge2D* edge = (CvQuadEdge2D*)(reader.ptr);

        if( CV_IS_SET_ELEM( edge ))
        {
            CvSubdiv2DEdge e = (CvSubdiv2DEdge)edge;

            // 返回同一个四方边缘结构中的另一条边缘
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
                if (!Search(triTemp))//没有检测到    
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
    CvRect rect = { 0, 0, 600, 600 }; // 构造一个矩形框

    // 内存存储器是一个可用来存储诸如序列,轮廓,图形,子划分等动态增长数据结构的底层结构。 
    CvMemStorage* storage;
    CvSubdiv2D* subdiv;
    IplImage* img;
    CvScalar active_facet_color, delaunay_color, voronoi_color, bkgnd_color;

    
    active_facet_color = CV_RGB(255, 0, 0); // 新插入的点标识为红色 
    delaunay_color = CV_RGB(0, 0, 0); // delaunay三角形标识为黑色
    voronoi_color = CV_RGB(0, 180, 0); // voronoi边框标识为绿色
    bkgnd_color = CV_RGB(255, 255, 255); // 背景色为白色 

	// 创建图像，img是实际画的D-三角图
    img = cvCreateImage(cvSize(rect.width,rect.height), 8, 3);

    // 设置背景
    cvSet(img, bkgnd_color, 0);  

    cvNamedWindow( win, 1 );

    storage = cvCreateMemStorage(0);

    // 完成内存分配的初始化的工作
    subdiv = init_delaunay(storage, rect);

    printf("Delaunay triangulation will be build now interactively.\n"
           "To stop the process, press any key\n\n");

    // 总共有N个顶点
    for (i = 0; i < 1000; i++) // 200
    {
        // 随机产生浮点坐标的二维点
        CvPoint2D32f fp = cvPoint2D32f((float)(rand() % (rect.width - 10) + 5),
                                       (float)(rand() % (rect.height - 10) + 5));

        // 将新插入的点在图上标识出来
        locate_point(subdiv, fp, img, active_facet_color);
        cvShowImage(win, img);

        if (cvWaitKey(100) >= 0)
            break;

        // 新插入一个点重新生成Delaunay三角图 
        // 向Delaunay三角测量中插入一个点
        cvSubdivDelaunay2DInsert(subdiv, fp);

        // 计算Voronoi图表的细胞结构 
        cvCalcSubdivVoronoi2D( subdiv );
        cvSet( img, bkgnd_color, 0 );
        draw_subdiv( img, subdiv, delaunay_color, voronoi_color );
        cvShowImage( win, img );

        if (cvWaitKey(100) >= 0)
            break;
    }

    cvSet( img, bkgnd_color, 0 );

    // 画Voronoi图表
    paint_voronoi( subdiv, img );
    cvShowImage( win, img );

    // triangles(subdiv, img); //遍历D-三角图内的所有三角形 

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
