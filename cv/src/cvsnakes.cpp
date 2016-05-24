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
#include "_cv.h"

#define _CV_SNAKE_BIG 2.e+38f
#define _CV_SNAKE_IMAGE 1
#define _CV_SNAKE_GRAD  2


/*F///////////////////////////////////////////////////////////////////////////////////////
//    Name:      icvSnake8uC1R     
//    Purpose:   
//    Context:   
//    Parameters:
//               src - source image,
//               srcStep - its step in bytes,
//               roi - size of ROI,
//               pt - pointer to snake points array
//               n - size of points array, 
//               alpha - pointer to coefficient of continuity energy, 
//               beta - pointer to coefficient of curvature energy,  
//               gamma - pointer to coefficient of image energy,  
//               coeffUsage - if CV_VALUE - alpha, beta, gamma point to single value
//                            if CV_MATAY - point to arrays
//               criteria - termination criteria.
//               scheme - image energy scheme
//                         if _CV_SNAKE_IMAGE - image intensity is energy
//                         if _CV_SNAKE_GRAD  - magnitude of gradient is energy
//    Returns:   
//F*/

static CvStatus
icvSnake8uC1R( unsigned char *src,
               int srcStep,
               CvSize roi,
               CvPoint * pt,
               int n,
               float *alpha,
               float *beta,
               float *gamma,
               int coeffUsage, CvSize win, CvTermCriteria criteria, int scheme )
{
    int i, j, k;
    int neighbors = win.height * win.width;

    int centerx = win.width >> 1;
    int centery = win.height >> 1;

    float invn;
    int iteration = 0;
    int converged = 0;
	

    float *Econt;
    float *Ecurv;
    float *Eimg;
    float *E;

    float _alpha, _beta, _gamma;

    /*#ifdef GRAD_SNAKE */
    float *gradient = NULL;
    uchar *map = NULL;
    int map_width = ((roi.width - 1) >> 3) + 1;
    int map_height = ((roi.height - 1) >> 3) + 1;
    CvSepFilter pX, pY;
    #define WTILE_SIZE 8
    #define TILE_SIZE (WTILE_SIZE + 2)        
    short dx[TILE_SIZE*TILE_SIZE], dy[TILE_SIZE*TILE_SIZE];
    CvMat _dx = cvMat( TILE_SIZE, TILE_SIZE, CV_16SC1, dx );
    CvMat _dy = cvMat( TILE_SIZE, TILE_SIZE, CV_16SC1, dy );
    CvMat _src = cvMat( roi.height, roi.width, CV_8UC1, src );

    /* inner buffer of convolution process */
    //char ConvBuffer[400];

    /*#endif */


    /* check bad arguments */
    if( src == NULL )
        return CV_NULLPTR_ERR;
    if( (roi.height <= 0) || (roi.width <= 0) )
        return CV_BADSIZE_ERR;
    if( srcStep < roi.width )
        return CV_BADSIZE_ERR;
    if( pt == NULL )
        return CV_NULLPTR_ERR;
    if( n < 3 )
        return CV_BADSIZE_ERR;
    if( alpha == NULL )
        return CV_NULLPTR_ERR;
    if( beta == NULL )
        return CV_NULLPTR_ERR;
    if( gamma == NULL )
        return CV_NULLPTR_ERR;
    if( coeffUsage != CV_VALUE && coeffUsage != CV_ARRAY )
        return CV_BADFLAG_ERR;
    if( (win.height <= 0) || (!(win.height & 1)))
        return CV_BADSIZE_ERR;
    if( (win.width <= 0) || (!(win.width & 1)))
        return CV_BADSIZE_ERR;

    invn = 1 / ((float) n);

    if( scheme == _CV_SNAKE_GRAD )
    {
        pX.init_deriv( TILE_SIZE+2, CV_8UC1, CV_16SC1, 1, 0, 3 );
        pY.init_deriv( TILE_SIZE+2, CV_8UC1, CV_16SC1, 0, 1, 3 );

        gradient = (float *) cvAlloc( roi.height * roi.width * sizeof( float ));

        if( !gradient )
            return CV_OUTOFMEM_ERR;
        map = (uchar *) cvAlloc( map_width * map_height );
        if( !map )
        {
            cvFree( &gradient );
            return CV_OUTOFMEM_ERR;
        }
        /* clear map - no gradient computed */
        memset( (void *) map, 0, map_width * map_height );
    }
    Econt = (float *) cvAlloc( neighbors * sizeof( float ));
    Ecurv = (float *) cvAlloc( neighbors * sizeof( float ));
    Eimg = (float *) cvAlloc( neighbors * sizeof( float ));
    E = (float *) cvAlloc( neighbors * sizeof( float ));

    while( !converged )
    {
        float ave_d = 0;
        int moved = 0;

        converged = 0;
        iteration++;
        /* compute average distance */
        for( i = 1; i < n; i++ )
        {
            int diffx = pt[i - 1].x - pt[i].x;
            int diffy = pt[i - 1].y - pt[i].y;

            ave_d += cvSqrt( (float) (diffx * diffx + diffy * diffy) );
        }
        ave_d += cvSqrt( (float) ((pt[0].x - pt[n - 1].x) *
                                  (pt[0].x - pt[n - 1].x) +
                                  (pt[0].y - pt[n - 1].y) * (pt[0].y - pt[n - 1].y)));

        ave_d *= invn;
        /* average distance computed */
        for( i = 0; i < n; i++ )
        {
            /* Calculate Econt */
            float maxEcont = 0;
            float maxEcurv = 0;
            float maxEimg = 0;
            float minEcont = _CV_SNAKE_BIG;
            float minEcurv = _CV_SNAKE_BIG;
            float minEimg = _CV_SNAKE_BIG;
            float Emin = _CV_SNAKE_BIG;

            int offsetx = 0;
            int offsety = 0;
            float tmp;

            /* compute bounds */
            int left = MIN( pt[i].x, win.width >> 1 );
            int right = MIN( roi.width - 1 - pt[i].x, win.width >> 1 );
            int upper = MIN( pt[i].y, win.height >> 1 );
            int bottom = MIN( roi.height - 1 - pt[i].y, win.height >> 1 );

            maxEcont = 0;
            minEcont = _CV_SNAKE_BIG;
            for( j = -upper; j <= bottom; j++ )
            {
                for( k = -left; k <= right; k++ )
                {
                    int diffx, diffy;
                    float energy;

                    if( i == 0 )
                    {
                        diffx = pt[n - 1].x - (pt[i].x + k);
                        diffy = pt[n - 1].y - (pt[i].y + j);
                    }
                    else
                    {
                        diffx = pt[i - 1].x - (pt[i].x + k);
                        diffy = pt[i - 1].y - (pt[i].y + j);
                    }
                    Econt[(j + centery) * win.width + k + centerx] = energy =
                        (float) fabs( ave_d -
                                      cvSqrt( (float) (diffx * diffx + diffy * diffy) ));

                    maxEcont = MAX( maxEcont, energy );
                    minEcont = MIN( minEcont, energy );
                }
            }
            tmp = maxEcont - minEcont;
            tmp = (tmp == 0) ? 0 : (1 / tmp);
            for( k = 0; k < neighbors; k++ )
            {
                Econt[k] = (Econt[k] - minEcont) * tmp;
            }

            /*  Calculate Ecurv */
            maxEcurv = 0;
            minEcurv = _CV_SNAKE_BIG;
            for( j = -upper; j <= bottom; j++ )
            {
                for( k = -left; k <= right; k++ )
                {
                    int tx, ty;
                    float energy;

                    if( i == 0 )
                    {
                        tx = pt[n - 1].x - 2 * (pt[i].x + k) + pt[i + 1].x;
                        ty = pt[n - 1].y - 2 * (pt[i].y + j) + pt[i + 1].y;
                    }
                    else if( i == n - 1 )
                    {
                        tx = pt[i - 1].x - 2 * (pt[i].x + k) + pt[0].x;
                        ty = pt[i - 1].y - 2 * (pt[i].y + j) + pt[0].y;
                    }
                    else
                    {
                        tx = pt[i - 1].x - 2 * (pt[i].x + k) + pt[i + 1].x;
                        ty = pt[i - 1].y - 2 * (pt[i].y + j) + pt[i + 1].y;
                    }
                    Ecurv[(j + centery) * win.width + k + centerx] = energy =
                        (float) (tx * tx + ty * ty);
                    maxEcurv = MAX( maxEcurv, energy );
                    minEcurv = MIN( minEcurv, energy );
                }
            }
            tmp = maxEcurv - minEcurv;
            tmp = (tmp == 0) ? 0 : (1 / tmp);
            for( k = 0; k < neighbors; k++ )
            {
                Ecurv[k] = (Ecurv[k] - minEcurv) * tmp;
            }

            /* Calculate Eimg */
            for( j = -upper; j <= bottom; j++ )
            {
                for( k = -left; k <= right; k++ )
                {
                    float energy;

                    if( scheme == _CV_SNAKE_GRAD )
                    {
                        /* look at map and check status */
                        int x = (pt[i].x + k)/WTILE_SIZE;
                        int y = (pt[i].y + j)/WTILE_SIZE;

                        if( map[y * map_width + x] == 0 )
                        {
                            int l, m;							

                            /* evaluate block location */
                            int upshift = y ? 1 : 0;
                            int leftshift = x ? 1 : 0;
                            int bottomshift = MIN( 1, roi.height - (y + 1)*WTILE_SIZE );
                            int rightshift = MIN( 1, roi.width - (x + 1)*WTILE_SIZE );
                            CvRect g_roi = { x*WTILE_SIZE - leftshift, y*WTILE_SIZE - upshift,
                                leftshift + WTILE_SIZE + rightshift, upshift + WTILE_SIZE + bottomshift };
                            CvMat _src1;
                            cvGetSubArr( &_src, &_src1, g_roi );

                            pX.process( &_src1, &_dx );
                            pY.process( &_src1, &_dy );

                            for( l = 0; l < WTILE_SIZE + bottomshift; l++ )
                            {
                                for( m = 0; m < WTILE_SIZE + rightshift; m++ )
                                {
                                    gradient[(y*WTILE_SIZE + l) * roi.width + x*WTILE_SIZE + m] =
                                        (float) (dx[(l + upshift) * TILE_SIZE + m + leftshift] *
                                                 dx[(l + upshift) * TILE_SIZE + m + leftshift] +
                                                 dy[(l + upshift) * TILE_SIZE + m + leftshift] *
                                                 dy[(l + upshift) * TILE_SIZE + m + leftshift]);
                                }
                            }
                            map[y * map_width + x] = 1;
                        }
                        Eimg[(j + centery) * win.width + k + centerx] = energy =
                            gradient[(pt[i].y + j) * roi.width + pt[i].x + k];
                    }
                    else
                    {
                        Eimg[(j + centery) * win.width + k + centerx] = energy =
                            src[(pt[i].y + j) * srcStep + pt[i].x + k];
                    }

                    maxEimg = MAX( maxEimg, energy );
                    minEimg = MIN( minEimg, energy );
                }
            }

            tmp = (maxEimg - minEimg);
            tmp = (tmp == 0) ? 0 : (1 / tmp);

            for( k = 0; k < neighbors; k++ )
            {
                Eimg[k] = (minEimg - Eimg[k]) * tmp;
            }

            /* locate coefficients */
            if( coeffUsage == CV_VALUE)
            {
                _alpha = *alpha;
                _beta = *beta;
                _gamma = *gamma;
            }
            else
            {                   
                _alpha = alpha[i];
                _beta = beta[i];
                _gamma = gamma[i];
            }

            /* Find Minimize point in the neighbors */
            for( k = 0; k < neighbors; k++ )
            {
                E[k] = _alpha * Econt[k] + _beta * Ecurv[k] + _gamma * Eimg[k];
            }
            Emin = _CV_SNAKE_BIG;
            for( j = -upper; j <= bottom; j++ )
            {
                for( k = -left; k <= right; k++ )
                {

                    if( E[(j + centery) * win.width + k + centerx] < Emin )
                    {
                        Emin = E[(j + centery) * win.width + k + centerx];
                        offsetx = k;
                        offsety = j;
                    }
                }
            }

            if( offsetx || offsety )
            {
                pt[i].x += offsetx;
                pt[i].y += offsety;
                moved++;
            }
        }
        converged = (moved == 0);
        if( (criteria.type & CV_TERMCRIT_ITER) && (iteration >= criteria.max_iter) )
            converged = 1;
        if( (criteria.type & CV_TERMCRIT_EPS) && (moved <= criteria.epsilon) )
            converged = 1;
    }

    cvFree( &Econt );
    cvFree( &Ecurv );
    cvFree( &Eimg );
    cvFree( &E );

    if( scheme == _CV_SNAKE_GRAD )
    {
        cvFree( &gradient );
        cvFree( &map );
    }
    return CV_OK;
}

/* 改变轮廓位置使得它的能量最小 

void cvSnakeImage( const IplImage* image, CvPoint* points, int length,
                   float* alpha, float* beta, float* gamma, int coeff_usage,
                   CvSize win, CvTermCriteria criteria, int calc_gradient=1 );
image 
输入图像或外部能量域 
points 
轮廓点 (snake). 
length 
轮廓点的数目 
alpha 
连续性能量的权 Weight[s]，单个浮点数或长度为 length 的浮点数数组，每个轮廓点有一个权 
beta 
曲率能量的权 Weight[s]，与 alpha 类似 
gamma 
图像能量的权 Weight[s]，与 alpha 类似 
coeff_usage 
前面三个参数的不同使用方法： 
CV_VALUE 表示每个 alpha, beta, gamma 都是指向为所有点所用的一个单独数值; 
CV_ARRAY 表示每个 alpha, beta, gamma 是一个指向系数数组的指针，snake 上面各点的系数都不相同。因此，各个系数数组必须与轮廓具有同样的大小。所有数组必须与轮廓具有同样大小 
win 
每个点用于搜索最小值的邻域尺寸，两个 win.width 和 win.height 都必须是奇数 
criteria 
终止条件 
calc_gradient 
梯度符号。如果非零，函数为每一个图像象素计算梯度幅值，且把它当成能量场，否则考虑输入图像本身。 
函数 cvSnakeImage 更新 snake 是为了最小化 snake 的整个能量，其中能量是依赖于轮廓形状的内部能量(轮廓越光滑，内部能量越小)以及依赖于能量场的外部能量之和，外部能量通常在哪些局部能量极值点中达到最小值(这些局部能量极值点与图像梯度表示的图像边缘相对应)。 

参数 criteria.epsilon 用来定义必须从迭代中除掉以保证迭代正常运行的点的最少数目。 

如果在迭代中去掉的点数目小于 criteria.epsilon 或者函数达到了最大的迭代次数 criteria.max_iter ，则终止函数。 

 */
CV_IMPL void
cvSnakeImage( const IplImage* src, CvPoint* points,
              int length, float *alpha,
              float *beta, float *gamma,
              int coeffUsage, CvSize win,
              CvTermCriteria criteria, int calcGradient )
{

    CV_FUNCNAME( "cvSnakeImage" );

    __BEGIN__;

    uchar *data;
    CvSize size;
    int step;

    if( src->nChannels != 1 )
        CV_ERROR( CV_BadNumChannels, "input image has more than one channel" );

    if( src->depth != IPL_DEPTH_8U )
        CV_ERROR( CV_BadDepth, cvUnsupportedFormat );

    cvGetRawData( src, &data, &step, &size );

    IPPI_CALL( icvSnake8uC1R( data, step, size, points, length,
                              alpha, beta, gamma, coeffUsage, win, criteria,
                              calcGradient ? _CV_SNAKE_GRAD : _CV_SNAKE_IMAGE ));
    __END__;
}

/* end of file */
