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

/* 1．    什么是卡尔曼滤波器

（What is the Kalman Filter?）

 

在学习卡尔曼滤波器之前，首先看看为什么叫“卡尔曼”。跟其他著名的理论（例如傅立叶变换，泰勒级数等等）一样，卡尔曼也是一个人的名字，而跟他们不同的是，他是个现代人！

 

卡尔曼全名Rudolf Emil Kalman，匈牙利数学家，1930年出生于匈牙利首都布达佩斯。1953，1954年于麻省理工学院分别获得电机工程学士及硕士学位。1957年于哥伦比亚大学获得博士学位。我们现在要学习的卡尔曼滤波器，正是源于他的博士论文和1960年发表的论文《A New Approach to Linear Filtering and Prediction Problems》（线性滤波与预测问题的新方法）。如果对这编论文有兴趣，可以到这里的地址下载： http://www.cs.unc.edu/~welch/media/pdf/Kalman1960.pdf。

 

简单来说，卡尔曼滤波器是一个“optimal recursive data processing algorithm（最优化自回归数据处理算法）”。对于解决很大部分的问题，他是最优，效率最高甚至是最有用的。他的广泛应用已经超过30年，包括机器人导航，控制，传感器数据融合甚至在军事方面的雷达系统以及导弹追踪等等。近年来更被应用于计算机图像处理，例如头脸识别，图像分割，图像边缘检测等等。

 

2．卡尔曼滤波器的介绍

（Introduction to the Kalman Filter）

 

为了可以更加容易的理解卡尔曼滤波器，这里会应用形象的描述方法来讲解，而不是像大多数参考书那样罗列一大堆的数学公式和数学符号。但是，他的5条公式是其核心内容。结合现代的计算机，其实卡尔曼的程序相当的简单，只要你理解了他的那5条公式。

 

在介绍他的5条公式之前，先让我们来根据下面的例子一步一步的探索。

 

假设我们要研究的对象是一个房间的温度。根据你的经验判断，这个房间的温度是恒定的，也就是下一分钟的温度等于现在这一分钟的温度（假设我们用一分钟来做时间单位）。假设你对你的经验不是100%的相信，可能会有上下偏差几度。我们把这些偏差看成是高斯白噪声（White Gaussian Noise），也就是这些偏差跟前后时间是没有关系的而且符合高斯分配（Gaussian Distribution）。另外，我们在房间里放一个温度计，但是这个温度计也不准确的，测量值会比实际值偏差。我们也把这些偏差看成是高斯白噪声。

 

好了，现在对于某一分钟我们有两个有关于该房间的温度值：你根据经验的预测值（系统的预测值）和温度计的值（测量值）。下面我们要用这两个值结合他们各自的噪声来估算出房间的实际温度值。

 

假如我们要估算k时刻的是实际温度值。首先你要根据k-1时刻的温度值，来预测k时刻的温度。因为你相信温度是恒定的，所以你会得到k时刻的温度预测值是跟k-1时刻一样的，假设是23度，同时该值的高斯噪声的偏差是5度（5是这样得到的：如果k-1时刻估算出的最优温度值的偏差是3，你对自己预测的不确定度是4度，他们平方相加再开方，就是5）。然后，你从温度计那里得到了k时刻的温度值，假设是25度，同时该值的偏差是4度。

 

由于我们用于估算k时刻的实际温度有两个温度值，分别是23度和25度。究竟实际温度是多少呢？相信自己还是相信温度计呢？究竟相信谁多一点，我们可以用他们的covariance来判断。因为Kg^2=5^2/(5^2+4^2)，所以Kg=0.78，我们可以估算出k时刻的实际温度值是：23+0.78*(25-23)=24.56度。可以看出，因为温度计的covariance比较小（比较相信温度计），所以估算出的最优温度值偏向温度计的值。

 

现在我们已经得到k时刻的最优温度值了，下一步就是要进入k+1时刻，进行新的最优估算。到现在为止，好像还没看到什么自回归的东西出现。对了，在进入k+1时刻之前，我们还要算出k时刻那个最优值（24.56度）的偏差。算法如下：((1-Kg)*5^2)^0.5=2.35。这里的5就是上面的k时刻你预测的那个23度温度值的偏差，得出的2.35就是进入k+1时刻以后k时刻估算出的最优温度值的偏差（对应于上面的3）。

 

就是这样，卡尔曼滤波器就不断的把covariance递归，从而估算出最优的温度值。他运行的很快，而且它只保留了上一时刻的covariance。上面的Kg，就是卡尔曼增益（Kalman Gain）。他可以随不同的时刻而改变他自己的值，是不是很神奇！

 

下面就要言归正传，讨论真正工程系统上的卡尔曼。

 

3．    卡尔曼滤波器算法

（The Kalman Filter Algorithm）

 

在这一部分，我们就来描述源于Dr Kalman 的卡尔曼滤波器。下面的描述，会涉及一些基本的概念知识，包括概率（Probability），随即变量（Random Variable），高斯或正态分配（Gaussian Distribution）还有State-space Model等等。但对于卡尔曼滤波器的详细证明，这里不能一一描述。

 

首先，我们先要引入一个离散控制过程的系统。该系统可用一个线性随机微分方程（Linear Stochastic Difference equation）来描述：

X(k)=A X(k-1)+B U(k)+W(k)

再加上系统的测量值：

Z(k)=H X(k)+V(k)

上两式子中，X(k)是k时刻的系统状态，U(k)是k时刻对系统的控制量。A和B是系统参数，对于多模型系统，他们为矩阵。Z(k)是k时刻的测量值，H是测量系统的参数，对于多测量系统，H为矩阵。W(k)和V(k)分别表示过程和测量的噪声。他们被假设成高斯白噪声(White Gaussian Noise)，他们的covariance 分别是Q，R（这里我们假设他们不随系统状态变化而变化）。

 

对于满足上面的条件(线性随机微分系统，过程和测量都是高斯白噪声)，卡尔曼滤波器是最优的信息处理器。下面我们来用他们结合他们的covariances 来估算系统的最优化输出（类似上一节那个温度的例子）。

 

首先我们要利用系统的过程模型，来预测下一状态的系统。假设现在的系统状态是k，根据系统的模型，可以基于系统的上一状态而预测出现在状态：

X(k|k-1)=A X(k-1|k-1)+B U(k) ……….. (1)

式(1)中，X(k|k-1)是利用上一状态预测的结果，X(k-1|k-1)是上一状态最优的结果，U(k)为现在状态的控制量，如果没有控制量，它可以为0。

 

到现在为止，我们的系统结果已经更新了，可是，对应于X(k|k-1)的covariance还没更新。我们用P表示covariance：

P(k|k-1)=A P(k-1|k-1) A’+Q ……… (2)

式(2)中，P(k|k-1)是X(k|k-1)对应的covariance，P(k-1|k-1)是X(k-1|k-1)对应的covariance，A’表示A的转置矩阵，Q是系统过程的covariance。式子1，2就是卡尔曼滤波器5个公式当中的前两个，也就是对系统的预测。

 

现在我们有了现在状态的预测结果，然后我们再收集现在状态的测量值。结合预测值和测量值，我们可以得到现在状态(k)的最优化估算值X(k|k)：

X(k|k)= X(k|k-1)+Kg(k) (Z(k)-H X(k|k-1)) ……… (3)

其中Kg为卡尔曼增益(Kalman Gain)：

Kg(k)= P(k|k-1) H’ / (H P(k|k-1) H’ + R) ……… (4)

 

到现在为止，我们已经得到了k状态下最优的估算值X(k|k)。但是为了要另卡尔曼滤波器不断的运行下去直到系统过程结束，我们还要更新k状态下X(k|k)的covariance：

P(k|k)=（I-Kg(k) H）P(k|k-1) ……… (5)

其中I 为1的矩阵，对于单模型单测量，I=1。当系统进入k+1状态时，P(k|k)就是式子(2)的P(k-1|k-1)。这样，算法就可以自回归的运算下去。

 

卡尔曼滤波器的原理基本描述了，式子1，2，3，4和5就是他的5 个基本公式。根据这5个公式，可以很容易的实现计算机的程序 */

// 分配 Kalman 滤波器结构
/* dynam_params 
状态向量维数 
measure_params 
测量向量维数 
control_params 
控制向量维数 
函数 cvCreateKalman 分配 CvKalman 以及它的所有矩阵和初始参数  */
CV_IMPL CvKalman*
cvCreateKalman( int DP, int MP, int CP )
{
    CvKalman *kalman = 0;

    CV_FUNCNAME( "cvCreateKalman" );
    
    __BEGIN__;

    if( DP <= 0 || MP <= 0 )
        CV_ERROR( CV_StsOutOfRange,
        "state and measurement vectors must have positive number of dimensions" );

    if( CP < 0 )
        CP = DP;
    
    /* allocating memory for the structure */
    CV_CALL( kalman = (CvKalman *)cvAlloc( sizeof( CvKalman )));
    memset( kalman, 0, sizeof(*kalman));
    
    kalman->DP = DP;
    kalman->MP = MP;
    kalman->CP = CP;

    CV_CALL( kalman->state_pre = cvCreateMat( DP, 1, CV_32FC1 ));
    cvZero( kalman->state_pre );
    
    CV_CALL( kalman->state_post = cvCreateMat( DP, 1, CV_32FC1 ));
    cvZero( kalman->state_post );
    
    CV_CALL( kalman->transition_matrix = cvCreateMat( DP, DP, CV_32FC1 ));
    cvSetIdentity( kalman->transition_matrix );

    CV_CALL( kalman->process_noise_cov = cvCreateMat( DP, DP, CV_32FC1 ));
    cvSetIdentity( kalman->process_noise_cov );
    
    CV_CALL( kalman->measurement_matrix = cvCreateMat( MP, DP, CV_32FC1 ));
    cvZero( kalman->measurement_matrix );

    CV_CALL( kalman->measurement_noise_cov = cvCreateMat( MP, MP, CV_32FC1 ));
    cvSetIdentity( kalman->measurement_noise_cov );

    CV_CALL( kalman->error_cov_pre = cvCreateMat( DP, DP, CV_32FC1 ));
    
    CV_CALL( kalman->error_cov_post = cvCreateMat( DP, DP, CV_32FC1 ));
    cvZero( kalman->error_cov_post );

    CV_CALL( kalman->gain = cvCreateMat( DP, MP, CV_32FC1 ));

    if( CP > 0 )
    {
        CV_CALL( kalman->control_matrix = cvCreateMat( DP, CP, CV_32FC1 ));
        cvZero( kalman->control_matrix );
    }

    CV_CALL( kalman->temp1 = cvCreateMat( DP, DP, CV_32FC1 ));
    CV_CALL( kalman->temp2 = cvCreateMat( MP, DP, CV_32FC1 ));
    CV_CALL( kalman->temp3 = cvCreateMat( MP, MP, CV_32FC1 ));
    CV_CALL( kalman->temp4 = cvCreateMat( MP, DP, CV_32FC1 ));
    CV_CALL( kalman->temp5 = cvCreateMat( MP, 1, CV_32FC1 ));

#if 1
    kalman->PosterState = kalman->state_pre->data.fl;
    kalman->PriorState = kalman->state_post->data.fl;
    kalman->DynamMatr = kalman->transition_matrix->data.fl;
    kalman->MeasurementMatr = kalman->measurement_matrix->data.fl;
    kalman->MNCovariance = kalman->measurement_noise_cov->data.fl;
    kalman->PNCovariance = kalman->process_noise_cov->data.fl;
    kalman->KalmGainMatr = kalman->gain->data.fl;
    kalman->PriorErrorCovariance = kalman->error_cov_pre->data.fl;
    kalman->PosterErrorCovariance = kalman->error_cov_post->data.fl;
#endif    

    __END__;

    if( cvGetErrStatus() < 0 )
        cvReleaseKalman( &kalman );

    return kalman;
}

// 释放 Kalman 滤波器结构
/* kalman 
指向 Kalman 滤波器结构的双指针 
函数 cvReleaseKalman 释放结构 CvKalman 和里面所有矩阵  */

CV_IMPL void
cvReleaseKalman( CvKalman** _kalman )
{
    CvKalman *kalman;

    CV_FUNCNAME( "cvReleaseKalman" );
    __BEGIN__;
    
    if( !_kalman )
        CV_ERROR( CV_StsNullPtr, "" );
    
    kalman = *_kalman;
    
    /* freeing the memory */
    cvReleaseMat( &kalman->state_pre );
    cvReleaseMat( &kalman->state_post );
    cvReleaseMat( &kalman->transition_matrix );
    cvReleaseMat( &kalman->control_matrix );
    cvReleaseMat( &kalman->measurement_matrix );
    cvReleaseMat( &kalman->process_noise_cov );
    cvReleaseMat( &kalman->measurement_noise_cov );
    cvReleaseMat( &kalman->error_cov_pre );
    cvReleaseMat( &kalman->gain );
    cvReleaseMat( &kalman->error_cov_post );
    cvReleaseMat( &kalman->temp1 );
    cvReleaseMat( &kalman->temp2 );
    cvReleaseMat( &kalman->temp3 );
    cvReleaseMat( &kalman->temp4 );
    cvReleaseMat( &kalman->temp5 );

    memset( kalman, 0, sizeof(*kalman));

    /* deallocating the structure */
    cvFree( _kalman );

    __END__;
}


// 估计后来的模型状态
/* 
kalman 
Kalman 滤波器状态 
control 
控制向量 (uk), 如果没有外部控制   (control_params=0) 应该为 NULL 
函数 cvKalmanPredict 根据当前状态估计后来的随机模型状态，并存储于 kalman->state_pre:

     x'k=A?xk+B?uk
     P'k=A?Pk-1*AT + Q,
其中
x'k 是预测状态 (kalman->state_pre),
xk-1 是前一步的矫正状态 (kalman->state_post)
                 (应该在开始的某个地方初始化，即缺省为零向量),
uk 是外部控制(control 参数),
P'k 是先验误差相关矩阵 (kalman->error_cov_pre)
Pk-1 是前一步的后验误差相关矩阵(kalman->error_cov_post)
                 (应该在开始的某个地方初始化，即缺省为单位矩阵),

函数返回估计得到的状态值  */
CV_IMPL const CvMat*
cvKalmanPredict( CvKalman* kalman, const CvMat* control )
{
    CvMat* result = 0;
    
    CV_FUNCNAME( "cvKalmanPredict" );

    __BEGIN__;
    
    if( !kalman )
        CV_ERROR( CV_StsNullPtr, "" );

    /* update the state */
    /* x'(k) = A*x(k) */
    CV_CALL( cvMatMulAdd( kalman->transition_matrix, kalman->state_post, 0, kalman->state_pre ));

    if( control && kalman->CP > 0 )
        /* x'(k) = x'(k) + B*u(k) */
        CV_CALL( cvMatMulAdd( kalman->control_matrix, control, kalman->state_pre, kalman->state_pre ));
    
    /* update error covariance matrices */
    /* temp1 = A*P(k) */
    CV_CALL( cvMatMulAdd( kalman->transition_matrix, kalman->error_cov_post, 0, kalman->temp1 ));
    
    /* P'(k) = temp1*At + Q */
    CV_CALL( cvGEMM( kalman->temp1, kalman->transition_matrix, 1, kalman->process_noise_cov, 1,
                     kalman->error_cov_pre, CV_GEMM_B_T ));

    result = kalman->state_pre;

    __END__;

    return result;
}

// 调节模型状态
/* kalman 
被更新的 Kalman 结构的指针 
measurement 
指向测量向量的指针，向量形式为 CvMat  
函数 cvKalmanCorrect 在给定的模型状态的测量基础上，调节随机模型状态：

Kk=P'k?HT?(H?P'k?HT+R)-1
xk=x'k+Kk?(zk-H?x'k)
Pk=(I-Kk?H)?P'k
其中
zk - 给定测量(mesurement parameter)
Kk - Kalman "增益" 矩阵

函数存储调节状态到 kalman->state_post 中并且输出时返回它  */
CV_IMPL const CvMat*
cvKalmanCorrect( CvKalman* kalman, const CvMat* measurement )
{
    CvMat* result = 0;

    CV_FUNCNAME( "cvKalmanCorrect" );

    __BEGIN__;
    
    if( !kalman || !measurement )
        CV_ERROR( CV_StsNullPtr, "" );

    /* temp2 = H*P'(k) */
    CV_CALL( cvMatMulAdd( kalman->measurement_matrix,
                          kalman->error_cov_pre, 0, kalman->temp2 ));
    /* temp3 = temp2*Ht + R */
    CV_CALL( cvGEMM( kalman->temp2, kalman->measurement_matrix, 1,
                     kalman->measurement_noise_cov, 1, kalman->temp3, CV_GEMM_B_T ));

    /* temp4 = inv(temp3)*temp2 = Kt(k) */
    CV_CALL( cvSolve( kalman->temp3, kalman->temp2, kalman->temp4, CV_SVD ));

    /* K(k) */
    CV_CALL( cvTranspose( kalman->temp4, kalman->gain ));
    
    /* temp5 = z(k) - H*x'(k) */
    CV_CALL( cvGEMM( kalman->measurement_matrix, kalman->state_pre, -1, measurement, 1, kalman->temp5 ));

    /* x(k) = x'(k) + K(k)*temp5 */
    CV_CALL( cvMatMulAdd( kalman->gain, kalman->temp5, kalman->state_pre, kalman->state_post ));

    /* P(k) = P'(k) - K(k)*temp2 */
    CV_CALL( cvGEMM( kalman->gain, kalman->temp2, -1, kalman->error_cov_pre, 1,
                     kalman->error_cov_post, 0 ));

    result = kalman->state_post;

    __END__;

    return result;
}
