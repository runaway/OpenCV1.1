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

IPCVAPI_IMPL( CvStatus, icvUpdateMotionHistory_8u32f_C1IR,
    (const uchar * silIm, int silStep, float *mhiIm, int mhiStep,
     CvSize size, float timestamp, float mhi_duration),
     (silIm, silStep, mhiIm, mhiStep, size, timestamp, mhi_duration) )
{
    int x, y;

    /* function processes floating-point images using integer arithmetics */
    Cv32suf v;
    int ts, delbound;
    int *mhi = (int *) mhiIm;

    v.f = timestamp;
    ts = v.i;

    if( !silIm || !mhiIm )
        return CV_NULLPTR_ERR;

    if( size.height <= 0 || size.width <= 0 ||
        silStep < size.width || mhiStep < size.width * CV_SIZEOF_FLOAT ||
        (mhiStep & (CV_SIZEOF_FLOAT - 1)) != 0 )
        return CV_BADSIZE_ERR;

    if( mhi_duration < 0 )
        return CV_BADFACTOR_ERR;

    mhi_duration = timestamp - mhi_duration;

    v.f = mhi_duration;
    delbound = CV_TOGGLE_FLT( v.i );

    mhiStep /= sizeof(mhi[0]);

    if( mhiStep == size.width && silStep == size.width )
    {
        size.width *= size.height;
        size.height = 1;
    }

    if( delbound > 0 )
        for( y = 0; y < size.height; y++, silIm += silStep, mhi += mhiStep )
            for( x = 0; x < size.width; x++ )
            {
                int val = mhi[x];

                /* val = silIm[x] ? ts : val < delbound ? 0 : val; */
                val &= (val < delbound) - 1;
                val ^= (ts ^ val) & ((silIm[x] == 0) - 1);
                mhi[x] = val;
            }
    else
        for( y = 0; y < size.height; y++, silIm += silStep, mhi += mhiStep )
            for( x = 0; x < size.width; x++ )
            {
                int val = mhi[x];

                /* val = silIm[x] ? ts : val < delbound ? 0 : val; */
                val &= (CV_TOGGLE_FLT( val ) < delbound) - 1;
                val ^= (ts ^ val) & ((silIm[x] == 0) - 1);
                mhi[x] = val;
            }

    return CV_OK;
}

/* silhouette 
影像 mask，运动发生地方具有非零象素 
mhi 
运动历史图像(单通道, 32-比特 浮点数)，为本函数所更新 
timestamp 
当前时间，毫秒或其它单位 
duration 
运动跟踪的最大持续时间，用 timestamp 一样的时间单位 
函数 cvUpdateMotionHistory 用下面方式更新运动历史图像：
mhi(x,y)=timestamp  if silhouette(x,y)!=0
         0          if silhouette(x,y)=0 and mhi(x,y)<timestamp-duration
         mhi(x,y)   otherwise
也就是，MHI（motion history image） 中在运动发生的象素点被设置为当前时间戳，而运
动发生较久的象素点被清除。 */
/* motion templates */
CV_IMPL void
cvUpdateMotionHistory( const void* silhouette, void* mhimg,
                       double timestamp, double mhi_duration )
{
    CvSize size;
    CvMat  silhstub, *silh = (CvMat*)silhouette;
    CvMat  mhistub, *mhi = (CvMat*)mhimg;
    int mhi_step, silh_step;

    CV_FUNCNAME( "cvUpdateMHIByTime" );

    __BEGIN__;

    CV_CALL( silh = cvGetMat( silh, &silhstub ));
    CV_CALL( mhi = cvGetMat( mhi, &mhistub ));

    if( !CV_IS_MASK_ARR( silh ))
        CV_ERROR( CV_StsBadMask, "" );

    if( CV_MAT_CN( mhi->type ) > 1 )
        CV_ERROR( CV_BadNumChannels, "" );

    if( CV_MAT_DEPTH( mhi->type ) != CV_32F )
        CV_ERROR( CV_BadDepth, "" );

    if( !CV_ARE_SIZES_EQ( mhi, silh ))
        CV_ERROR( CV_StsUnmatchedSizes, "" );

    size = cvGetMatSize( mhi );

    mhi_step = mhi->step;
    silh_step = silh->step;

    if( CV_IS_MAT_CONT( mhi->type & silh->type ))
    {
        size.width *= size.height;
        mhi_step = silh_step = CV_STUB_STEP;
        size.height = 1;
    }

    IPPI_CALL( icvUpdateMotionHistory_8u32f_C1IR( (const uchar*)(silh->data.ptr), silh_step,
                                                  mhi->data.fl, mhi_step, size,
                                                  (float)timestamp, (float)mhi_duration ));
    __END__;
}

/* mhi 
运动历史图像 
mask 
Mask 图像；用来标注运动梯度数据正确的点，为输出参数。 
orientation 
运动梯度的方向图像，包含从 0 到 360 角度 
delta1, delta2 
函数在每个象素点 (x,y) 邻域寻找 MHI 的最小值 (m(x,y)) 和最大值 (M(x,y))，并且假
设梯度是正确的，当且仅当： 
min(delta1,delta2) <= M(x,y)-m(x,y) <= max(delta1,delta2). 
aperture_size 
函数所用微分算子的开孔尺寸 CV_SCHARR, 1, 3, 5 or 7 (见 cvSobel). 
函数 cvCalcMotionGradient 计算 MHI 的差分 Dx 和 Dy ，然后计算梯度方向如下式： 

orientation(x,y)=arctan(Dy(x,y)/Dx(x,y)) 
其中都要考虑 Dx(x,y)' 和 Dy(x,y)' 的符号 (如 cvCartToPolar 类似). 然后填充mask以
表示哪些方向是正确的(见 delta1 和delta2 的描述).  */
CV_IMPL void
cvCalcMotionGradient( const CvArr* mhiimg, CvArr* maskimg,
                      CvArr* orientation,
                      double delta1, double delta2,
                      int aperture_size )
{
    CvMat *dX_min = 0, *dY_max = 0;
    IplConvKernel* el = 0;

    CV_FUNCNAME( "cvCalcMotionGradient" );

    __BEGIN__;

    CvMat  mhistub, *mhi = (CvMat*)mhiimg;
    CvMat  maskstub, *mask = (CvMat*)maskimg;
    CvMat  orientstub, *orient = (CvMat*)orientation;
    CvMat  dX_min_row, dY_max_row, orient_row, mask_row;
    CvSize size;
    int x, y;

    float  gradient_epsilon = 1e-4f * aperture_size * aperture_size;
    float  min_delta, max_delta;

    CV_CALL( mhi = cvGetMat( mhi, &mhistub ));
    CV_CALL( mask = cvGetMat( mask, &maskstub ));
    CV_CALL( orient = cvGetMat( orient, &orientstub ));

    if( !CV_IS_MASK_ARR( mask ))
        CV_ERROR( CV_StsBadMask, "" );

    if( aperture_size < 3 || aperture_size > 7 || (aperture_size & 1) == 0 )
        CV_ERROR( CV_StsOutOfRange, "aperture_size must be 3, 5 or 7" );

    if( delta1 <= 0 || delta2 <= 0 )
        CV_ERROR( CV_StsOutOfRange, "both delta's must be positive" );

    if( CV_MAT_TYPE( mhi->type ) != CV_32FC1 || CV_MAT_TYPE( orient->type ) != CV_32FC1 )
        CV_ERROR( CV_StsUnsupportedFormat,
        "MHI and orientation must be single-channel floating-point images" );

    if( !CV_ARE_SIZES_EQ( mhi, mask ) || !CV_ARE_SIZES_EQ( orient, mhi ))
        CV_ERROR( CV_StsUnmatchedSizes, "" );

    if( orient->data.ptr == mhi->data.ptr )
        CV_ERROR( CV_StsInplaceNotSupported, "orientation image must be different from MHI" );

    if( delta1 > delta2 )
    {
        double t;
        CV_SWAP( delta1, delta2, t );
    }

	// 获取mhi的尺寸
    size = cvGetMatSize( mhi );
    min_delta = (float)delta1;
    max_delta = (float)delta2;
    CV_CALL( dX_min = cvCreateMat( mhi->rows, mhi->cols, CV_32F ));
    CV_CALL( dY_max = cvCreateMat( mhi->rows, mhi->cols, CV_32F ));

	// 计算Dx和Dy
    /* calc Dx and Dy */
    CV_CALL( cvSobel( mhi, dX_min, 1, 0, aperture_size ));
    CV_CALL( cvSobel( mhi, dY_max, 0, 1, aperture_size ));
    cvGetRow( dX_min, &dX_min_row, 0 );
    cvGetRow( dY_max, &dY_max_row, 0 );
    cvGetRow( orient, &orient_row, 0 );
    cvGetRow( mask, &mask_row, 0 );

	// 计算梯度
    /* calc gradient */
    for( y = 0; y < size.height; y++ )
    {
        dX_min_row.data.ptr = dX_min->data.ptr + y*dX_min->step;
        dY_max_row.data.ptr = dY_max->data.ptr + y*dY_max->step;
        orient_row.data.ptr = orient->data.ptr + y*orient->step;
        mask_row.data.ptr = mask->data.ptr + y*mask->step;

		// 计算2D向量的角度和(或)幅度
		// Calculates the magnitude and/or angle of 2d vectors.
        cvCartToPolar( &dX_min_row, &dY_max_row, 0, &orient_row, 1 );

		// 将梯度很小的地方的方向清零
        /* make orientation zero where the gradient is very small */
        for( x = 0; x < size.width; x++ )
        {
            float dY = dY_max_row.data.fl[x];
            float dX = dX_min_row.data.fl[x];

            if( fabs(dX) < gradient_epsilon && fabs(dY) < gradient_epsilon )
            {
                mask_row.data.ptr[x] = 0;
                orient_row.data.i[x] = 0;
            }
            else
                mask_row.data.ptr[x] = 1;
        }
    }

	// 创建一个结构元件
	// Creates a structuring element.
    CV_CALL( el = cvCreateStructuringElementEx( aperture_size, aperture_size,
                            aperture_size/2, aperture_size/2, CV_SHAPE_RECT ));

	// 用结构元件腐蚀膨胀mhi到dX_min,dY_max
	cvErode( mhi, dX_min, el );
    cvDilate( mhi, dY_max, el );

	// 将与邻居有小运动差异的像素作掩码
    /* mask off pixels which have little motion difference in their neighborhood */
    for( y = 0; y < size.height; y++ )
    {
        dX_min_row.data.ptr = dX_min->data.ptr + y*dX_min->step;
        dY_max_row.data.ptr = dY_max->data.ptr + y*dY_max->step;
        mask_row.data.ptr = mask->data.ptr + y*mask->step;
        orient_row.data.ptr = orient->data.ptr + y*orient->step;
        
        for( x = 0; x < size.width; x++ )
        {
            float d0 = dY_max_row.data.fl[x] - dX_min_row.data.fl[x];

            if( mask_row.data.ptr[x] == 0 || d0 < min_delta || max_delta < d0 )
            {
                mask_row.data.ptr[x] = 0;
                orient_row.data.i[x] = 0;
            }
        }
    }

    __END__;

    cvReleaseMat( &dX_min );
    cvReleaseMat( &dY_max );

	// 释放结构元件
    cvReleaseStructuringElement( &el );
}

/* orientation 
运动梯度方向图像，由函数 cvCalcMotionGradient 得到 
mask 
Mask 图像. 它可以是正确梯度 mask (由函数 cvCalcMotionGradient 得到)与区域mask的
结合，其中区域 mask 确定哪些方向需要计算。 
mhi 
运动历史图象 
timestamp 
当前时间（单位毫秒或其它）最好在传递它到函数 cvUpdateMotionHistory 之前存储一下
以便以后的重用，因为对大图像运行 cvUpdateMotionHistory 和 cvCalcMotionGradient会
花费一些时间 
duration 
运动跟踪的最大持续时间，用法与 cvUpdateMotionHistory 中的一致 
函数 cvCalcGlobalOrientation 在选择的区域内计算整个运动方向，并且返回0°到360°
之间的角度值。首先函数创建运动直方图，寻找基本方向做为直方图最大值的坐标。然后函
数计算与基本方向的相对偏移量，做为所有方向向量的加权和：运行越近，权重越大。得到
的角度是基本方向和偏移量的循环和。  */
CV_IMPL double
cvCalcGlobalOrientation( const void* orientation, const void* maskimg, const void* mhiimg,
                         double curr_mhi_timestamp, double mhi_duration )
{
    double  angle = 0;
    int hist_size = 12;
    CvHistogram* hist = 0;

    CV_FUNCNAME( "cvCalcGlobalOrientation" );

    __BEGIN__;

    CvMat  mhistub, *mhi = (CvMat*)mhiimg;
    CvMat  maskstub, *mask = (CvMat*)maskimg;
    CvMat  orientstub, *orient = (CvMat*)orientation;
    void*  _orient;
    float _ranges[] = { 0, 360 };
    float* ranges = _ranges;
    int base_orient;
    double shift_orient = 0, shift_weight = 0, fbase_orient;
    double a, b;
    float delbound;
    CvMat mhi_row, mask_row, orient_row;
    int x, y, mhi_rows, mhi_cols;

    CV_CALL( mhi = cvGetMat( mhi, &mhistub ));
    CV_CALL( mask = cvGetMat( mask, &maskstub ));
    CV_CALL( orient = cvGetMat( orient, &orientstub ));

    if( !CV_IS_MASK_ARR( mask ))
        CV_ERROR( CV_StsBadMask, "" );

    if( CV_MAT_TYPE( mhi->type ) != CV_32FC1 || CV_MAT_TYPE( orient->type ) != CV_32FC1 )
        CV_ERROR( CV_StsUnsupportedFormat,
        "MHI and orientation must be single-channel floating-point images" );

    if( !CV_ARE_SIZES_EQ( mhi, mask ) || !CV_ARE_SIZES_EQ( orient, mhi ))
        CV_ERROR( CV_StsUnmatchedSizes, "" );

    if( mhi_duration <= 0 )
        CV_ERROR( CV_StsOutOfRange, "MHI duration must be positive" );

    if( orient->data.ptr == mhi->data.ptr )
        CV_ERROR( CV_StsInplaceNotSupported, "orientation image must be different from MHI" );

	// 计算方向差分值的直方图
    // calculate histogram of different orientation values
    CV_CALL( hist = cvCreateHist( 1, &hist_size, CV_HIST_ARRAY, &ranges ));
    _orient = orient;
    cvCalcArrHist( &_orient, hist, 0, mask );

	// 找到最大索引(占优势的方向)
    // find the maximum index (the dominant orientation)
    cvGetMinMaxHistValue( hist, 0, 0, 0, &base_orient );
    base_orient *= 360/hist_size;

	// 用MHI的最大值覆盖时间戳
    // override timestamp with the maximum value in MHI
    cvMinMaxLoc( mhi, 0, &curr_mhi_timestamp, 0, 0, mask );

	// 找到占优势的方向的位移邋作为相关角度和的加权
    // find the shift relative to the dominant orientation as weighted sum of relative angles
    a = 254. / 255. / mhi_duration;
    b = 1. - curr_mhi_timestamp * a;
    fbase_orient = base_orient;
    delbound = (float)(curr_mhi_timestamp - mhi_duration);
    mhi_rows = mhi->rows;
    mhi_cols = mhi->cols;

    if( CV_IS_MAT_CONT( mhi->type & mask->type & orient->type ))
    {
        mhi_cols *= mhi_rows;
        mhi_rows = 1;
    }

    cvGetRow( mhi, &mhi_row, 0 );
    cvGetRow( mask, &mask_row, 0 );
    cvGetRow( orient, &orient_row, 0 );

    /*
       a = 254/(255*dt)
       b = 1 - t*a = 1 - 254*t/(255*dur) =
       (255*dt - 254*t)/(255*dt) =
       (dt - (t - dt)*254)/(255*dt);
       --------------------------------------------------------
       ax + b = 254*x/(255*dt) + (dt - (t - dt)*254)/(255*dt) =
       (254*x + dt - (t - dt)*254)/(255*dt) =
       ((x - (t - dt))*254 + dt)/(255*dt) =
       (((x - (t - dt))/dt)*254 + 1)/255 = (((x - low_time)/dt)*254 + 1)/255
     */
    for( y = 0; y < mhi_rows; y++ )
    {
        mhi_row.data.ptr = mhi->data.ptr + mhi->step*y;
        mask_row.data.ptr = mask->data.ptr + mask->step*y;
        orient_row.data.ptr = orient->data.ptr + orient->step*y;

        for( x = 0; x < mhi_cols; x++ )
            if( mask_row.data.ptr[x] != 0 && mhi_row.data.fl[x] > delbound )
            {
                /*
                   orient in 0..360, base_orient in 0..360
                   -> (rel_angle = orient - base_orient) in -360..360.
                   rel_angle is translated to -180..180
                 */
                double weight = mhi_row.data.fl[x] * a + b;
                int rel_angle = cvRound( orient_row.data.fl[x] - fbase_orient );

                rel_angle += (rel_angle < -180 ? 360 : 0);
                rel_angle += (rel_angle > 180 ? -360 : 0);

                if( abs(rel_angle) < 90 )
                {
                    shift_orient += weight * rel_angle;
                    shift_weight += weight;
                }
            }
    }

	// 加入占优势的方向和相关的位移
    // add the dominant orientation and the relative shift
    if( shift_weight == 0 )
        shift_weight = 0.01;

    base_orient = base_orient + cvRound( shift_orient / shift_weight );
    base_orient -= (base_orient < 360 ? 0 : 360);
    base_orient += (base_orient >= 0 ? 0 : 360);

    angle = base_orient;

    __END__;

    cvReleaseHist( &hist );
    return angle;
}

// 将整个运动分割为独立的运动部分 
/* mhi 
运动历史图像 
seg_mask 
发现应当存储的 mask 的图像, 单通道, 32bits， 浮点数. 
storage 
包含运动连通域序列的内存存储仓 
timestamp 
当前时间，毫秒单位 
seg_thresh 
分割阈值，推荐等于或大于运动历史“每步”之间的间隔。 
函数 cvSegmentMotion 寻找所有的运动分割，并且在seg_mask 用不同的单独数字
(1,2,...)标识它们。它也返回一个具有CvConnectedComp结构的序列，其中每个结构对应一
个运动部件。在这之后，每个运动部件的运动方向就可以被函数cvCalcGlobalOrientation 
利用提取的特定部件的掩模(mask)计算出来(使用cvCmp) 
 */
CV_IMPL CvSeq*
cvSegmentMotion( const CvArr* mhiimg, CvArr* segmask, CvMemStorage* storage,
                 double timestamp, double seg_thresh )
{
    CvSeq* components = 0;
    CvMat* mask8u = 0;

    CV_FUNCNAME( "cvSegmentMotion" );

    __BEGIN__;

    CvMat  mhistub, *mhi = (CvMat*)mhiimg;
    CvMat  maskstub, *mask = (CvMat*)segmask;
    Cv32suf v, comp_idx;
    int stub_val, ts;
    int x, y;

    if( !storage )
        CV_ERROR( CV_StsNullPtr, "NULL memory storage" );

    CV_CALL( mhi = cvGetMat( mhi, &mhistub ));
    CV_CALL( mask = cvGetMat( mask, &maskstub ));

    if( CV_MAT_TYPE( mhi->type ) != CV_32FC1 || CV_MAT_TYPE( mask->type ) != CV_32FC1 )
        CV_ERROR( CV_BadDepth, "Both MHI and the destination mask" );

    if( !CV_ARE_SIZES_EQ( mhi, mask ))
        CV_ERROR( CV_StsUnmatchedSizes, "" );

	// 创建掩码矩阵
    CV_CALL( mask8u = cvCreateMat( mhi->rows + 2, mhi->cols + 2, CV_8UC1 ));
    cvZero( mask8u );
    cvZero( mask );
    CV_CALL( components = cvCreateSeq( CV_SEQ_KIND_GENERIC, sizeof(CvSeq),
                                       sizeof(CvConnectedComp), storage ));
    
    v.f = (float)timestamp; ts = v.i;
    v.f = FLT_MAX*0.1f; stub_val = v.i;
    comp_idx.f = 1;

    for( y = 0; y < mhi->rows; y++ )
    {
    	// 定义指向当前行的指针
        int* mhi_row = (int*)(mhi->data.ptr + y*mhi->step);

		// 按列数循环
        for( x = 0; x < mhi->cols; x++ )
        {
        	// 如果行内的元素为零则设置为桩值
            if( mhi_row[x] == 0 )
                mhi_row[x] = stub_val;
        }
    }

	// 按行数循环
    for( y = 0; y < mhi->rows; y++ )
    {
	    // 定义指向当前行的指针
        int* mhi_row = (int*)(mhi->data.ptr + y*mhi->step);

		// 定义指向掩码矩阵当前行的指针
        uchar* mask8u_row = mask8u->data.ptr + (y+1)*mask8u->step + 1;

		// 按列数循环
        for( x = 0; x < mhi->cols; x++ )
        {
            if( mhi_row[x] == ts && mask8u_row[x] == 0 )
            {
            	// 定义连通域
                CvConnectedComp comp;
                int x1, y1;
                CvScalar _seg_thresh = cvRealScalar(seg_thresh);
                CvPoint seed = cvPoint(x,y);

				// 用给定的颜色填充连通域
				// Fills a connected component with the given color.
                CV_CALL( cvFloodFill( mhi, seed, cvRealScalar(0), _seg_thresh, _seg_thresh,
                                      &comp, CV_FLOODFILL_MASK_ONLY + 2*256 + 4, mask8u ));

				// 按连通域的高度循环
                for( y1 = 0; y1 < comp.rect.height; y1++ )
                {
                    int* mask_row1 = (int*)(mask->data.ptr +
                                    (comp.rect.y + y1)*mask->step) + comp.rect.x;
                    uchar* mask8u_row1 = mask8u->data.ptr +
                                    (comp.rect.y + y1+1)*mask8u->step + comp.rect.x+1;

					// 按连通域的宽度循环
                    for( x1 = 0; x1 < comp.rect.width; x1++ )
                    {
                        if( mask8u_row1[x1] > 1 )
                        {
                            mask8u_row1[x1] = 1;
                            mask_row1[x1] = comp_idx.i;
                        }
                    }
                }

				// 连通域索引+1
                comp_idx.f++;

				// 将此连通域推入序列
                cvSeqPush( components, &comp );
            }
        }
    }

	// 按行数循环
    for( y = 0; y < mhi->rows; y++ )
    {
        int* mhi_row = (int*)(mhi->data.ptr + y*mhi->step);

		// 按列数循环
        for( x = 0; x < mhi->cols; x++ )
        {
            if( mhi_row[x] == stub_val )
                mhi_row[x] = 0;
        }
    }

    __END__;

    cvReleaseMat( &mask8u );
    return components;
}

/* End of file. */
