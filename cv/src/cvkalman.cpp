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

/* 1��    ʲô�ǿ������˲���

��What is the Kalman Filter?��

 

��ѧϰ�������˲���֮ǰ�����ȿ���Ϊʲô�С��������������������������ۣ����縵��Ҷ�任��̩�ռ����ȵȣ�һ����������Ҳ��һ���˵����֣��������ǲ�ͬ���ǣ����Ǹ��ִ��ˣ�

 

������ȫ��Rudolf Emil Kalman����������ѧ�ң�1930��������������׶�������˹��1953��1954������ʡ��ѧԺ�ֱ��õ������ѧʿ��˶ʿѧλ��1957���ڸ��ױ��Ǵ�ѧ��ò�ʿѧλ����������Ҫѧϰ�Ŀ������˲���������Դ�����Ĳ�ʿ���ĺ�1960�귢������ġ�A New Approach to Linear Filtering and Prediction Problems���������˲���Ԥ��������·�����������������������Ȥ�����Ե�����ĵ�ַ���أ� http://www.cs.unc.edu/~welch/media/pdf/Kalman1960.pdf��

 

����˵���������˲�����һ����optimal recursive data processing algorithm�����Ż��Իع����ݴ����㷨���������ڽ���ܴ󲿷ֵ����⣬�������ţ�Ч����������������õġ����Ĺ㷺Ӧ���Ѿ�����30�꣬���������˵��������ƣ������������ں������ھ��·�����״�ϵͳ�Լ�����׷�ٵȵȡ�����������Ӧ���ڼ����ͼ��������ͷ��ʶ��ͼ��ָͼ���Ե���ȵȡ�

 

2���������˲����Ľ���

��Introduction to the Kalman Filter��

 

Ϊ�˿��Ը������׵���⿨�����˲����������Ӧ��������������������⣬�������������ο�����������һ��ѵ���ѧ��ʽ����ѧ���š����ǣ�����5����ʽ����������ݡ�����ִ��ļ��������ʵ�������ĳ����൱�ļ򵥣�ֻҪ�������������5����ʽ��

 

�ڽ�������5����ʽ֮ǰ�������������������������һ��һ����̽����

 

��������Ҫ�о��Ķ�����һ��������¶ȡ�������ľ����жϣ����������¶��Ǻ㶨�ģ�Ҳ������һ���ӵ��¶ȵ���������һ���ӵ��¶ȣ�����������һ��������ʱ�䵥λ�������������ľ��鲻��100%�����ţ����ܻ�������ƫ��ȡ����ǰ���Щƫ����Ǹ�˹��������White Gaussian Noise����Ҳ������Щƫ���ǰ��ʱ����û�й�ϵ�Ķ��ҷ��ϸ�˹���䣨Gaussian Distribution�������⣬�����ڷ������һ���¶ȼƣ���������¶ȼ�Ҳ��׼ȷ�ģ�����ֵ���ʵ��ֵƫ�����Ҳ����Щƫ����Ǹ�˹��������

 

���ˣ����ڶ���ĳһ���������������й��ڸ÷�����¶�ֵ������ݾ����Ԥ��ֵ��ϵͳ��Ԥ��ֵ�����¶ȼƵ�ֵ������ֵ������������Ҫ��������ֵ������Ǹ��Ե�����������������ʵ���¶�ֵ��

 

��������Ҫ����kʱ�̵���ʵ���¶�ֵ��������Ҫ����k-1ʱ�̵��¶�ֵ����Ԥ��kʱ�̵��¶ȡ���Ϊ�������¶��Ǻ㶨�ģ��������õ�kʱ�̵��¶�Ԥ��ֵ�Ǹ�k-1ʱ��һ���ģ�������23�ȣ�ͬʱ��ֵ�ĸ�˹������ƫ����5�ȣ�5�������õ��ģ����k-1ʱ�̹�����������¶�ֵ��ƫ����3������Լ�Ԥ��Ĳ�ȷ������4�ȣ�����ƽ������ٿ���������5����Ȼ������¶ȼ�����õ���kʱ�̵��¶�ֵ��������25�ȣ�ͬʱ��ֵ��ƫ����4�ȡ�

 

�����������ڹ���kʱ�̵�ʵ���¶��������¶�ֵ���ֱ���23�Ⱥ�25�ȡ�����ʵ���¶��Ƕ����أ������Լ����������¶ȼ��أ���������˭��һ�㣬���ǿ��������ǵ�covariance���жϡ���ΪKg^2=5^2/(5^2+4^2)������Kg=0.78�����ǿ��Թ����kʱ�̵�ʵ���¶�ֵ�ǣ�23+0.78*(25-23)=24.56�ȡ����Կ�������Ϊ�¶ȼƵ�covariance�Ƚ�С���Ƚ������¶ȼƣ������Թ�����������¶�ֵƫ���¶ȼƵ�ֵ��

 

���������Ѿ��õ�kʱ�̵������¶�ֵ�ˣ���һ������Ҫ����k+1ʱ�̣������µ����Ź��㡣������Ϊֹ������û����ʲô�Իع�Ķ������֡����ˣ��ڽ���k+1ʱ��֮ǰ�����ǻ�Ҫ���kʱ���Ǹ�����ֵ��24.56�ȣ���ƫ��㷨���£�((1-Kg)*5^2)^0.5=2.35�������5���������kʱ����Ԥ����Ǹ�23���¶�ֵ��ƫ��ó���2.35���ǽ���k+1ʱ���Ժ�kʱ�̹�����������¶�ֵ��ƫ���Ӧ�������3����

 

�����������������˲����Ͳ��ϵİ�covariance�ݹ飬�Ӷ���������ŵ��¶�ֵ�������еĺܿ죬������ֻ��������һʱ�̵�covariance�������Kg�����ǿ��������棨Kalman Gain�����������治ͬ��ʱ�̶��ı����Լ���ֵ���ǲ��Ǻ����棡

 

�����Ҫ�Թ�������������������ϵͳ�ϵĿ�������

 

3��    �������˲����㷨

��The Kalman Filter Algorithm��

 

����һ���֣����Ǿ�������Դ��Dr Kalman �Ŀ������˲�������������������漰һЩ�����ĸ���֪ʶ���������ʣ�Probability�����漴������Random Variable������˹����̬���䣨Gaussian Distribution������State-space Model�ȵȡ������ڿ������˲�������ϸ֤�������ﲻ��һһ������

 

���ȣ�������Ҫ����һ����ɢ���ƹ��̵�ϵͳ����ϵͳ����һ���������΢�ַ��̣�Linear Stochastic Difference equation����������

X(k)=A X(k-1)+B U(k)+W(k)

�ټ���ϵͳ�Ĳ���ֵ��

Z(k)=H X(k)+V(k)

����ʽ���У�X(k)��kʱ�̵�ϵͳ״̬��U(k)��kʱ�̶�ϵͳ�Ŀ�������A��B��ϵͳ���������ڶ�ģ��ϵͳ������Ϊ����Z(k)��kʱ�̵Ĳ���ֵ��H�ǲ���ϵͳ�Ĳ��������ڶ����ϵͳ��HΪ����W(k)��V(k)�ֱ��ʾ���̺Ͳ��������������Ǳ�����ɸ�˹������(White Gaussian Noise)�����ǵ�covariance �ֱ���Q��R���������Ǽ������ǲ���ϵͳ״̬�仯���仯����

 

�����������������(�������΢��ϵͳ�����̺Ͳ������Ǹ�˹������)���������˲��������ŵ���Ϣ�����������������������ǽ�����ǵ�covariances ������ϵͳ�����Ż������������һ���Ǹ��¶ȵ����ӣ���

 

��������Ҫ����ϵͳ�Ĺ���ģ�ͣ���Ԥ����һ״̬��ϵͳ���������ڵ�ϵͳ״̬��k������ϵͳ��ģ�ͣ����Ի���ϵͳ����һ״̬��Ԥ�������״̬��

X(k|k-1)=A X(k-1|k-1)+B U(k) ������.. (1)

ʽ(1)�У�X(k|k-1)��������һ״̬Ԥ��Ľ����X(k-1|k-1)����һ״̬���ŵĽ����U(k)Ϊ����״̬�Ŀ����������û�п�������������Ϊ0��

 

������Ϊֹ�����ǵ�ϵͳ����Ѿ������ˣ����ǣ���Ӧ��X(k|k-1)��covariance��û���¡�������P��ʾcovariance��

P(k|k-1)=A P(k-1|k-1) A��+Q ������ (2)

ʽ(2)�У�P(k|k-1)��X(k|k-1)��Ӧ��covariance��P(k-1|k-1)��X(k-1|k-1)��Ӧ��covariance��A����ʾA��ת�þ���Q��ϵͳ���̵�covariance��ʽ��1��2���ǿ������˲���5����ʽ���е�ǰ������Ҳ���Ƕ�ϵͳ��Ԥ�⡣

 

����������������״̬��Ԥ������Ȼ���������ռ�����״̬�Ĳ���ֵ�����Ԥ��ֵ�Ͳ���ֵ�����ǿ��Եõ�����״̬(k)�����Ż�����ֵX(k|k)��

X(k|k)= X(k|k-1)+Kg(k) (Z(k)-H X(k|k-1)) ������ (3)

����KgΪ����������(Kalman Gain)��

Kg(k)= P(k|k-1) H�� / (H P(k|k-1) H�� + R) ������ (4)

 

������Ϊֹ�������Ѿ��õ���k״̬�����ŵĹ���ֵX(k|k)������Ϊ��Ҫ�������˲������ϵ�������ȥֱ��ϵͳ���̽��������ǻ�Ҫ����k״̬��X(k|k)��covariance��

P(k|k)=��I-Kg(k) H��P(k|k-1) ������ (5)

����I Ϊ1�ľ��󣬶��ڵ�ģ�͵�������I=1����ϵͳ����k+1״̬ʱ��P(k|k)����ʽ��(2)��P(k-1|k-1)���������㷨�Ϳ����Իع��������ȥ��

 

�������˲�����ԭ����������ˣ�ʽ��1��2��3��4��5��������5 ��������ʽ��������5����ʽ�����Ժ����׵�ʵ�ּ�����ĳ��� */

// ���� Kalman �˲����ṹ
/* dynam_params 
״̬����ά�� 
measure_params 
��������ά�� 
control_params 
��������ά�� 
���� cvCreateKalman ���� CvKalman �Լ��������о���ͳ�ʼ����  */
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

// �ͷ� Kalman �˲����ṹ
/* kalman 
ָ�� Kalman �˲����ṹ��˫ָ�� 
���� cvReleaseKalman �ͷŽṹ CvKalman ���������о���  */

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


// ���ƺ�����ģ��״̬
/* 
kalman 
Kalman �˲���״̬ 
control 
�������� (uk), ���û���ⲿ����   (control_params=0) Ӧ��Ϊ NULL 
���� cvKalmanPredict ���ݵ�ǰ״̬���ƺ��������ģ��״̬�����洢�� kalman->state_pre:

     x'k=A?xk+B?uk
     P'k=A?Pk-1*AT + Q,
����
x'k ��Ԥ��״̬ (kalman->state_pre),
xk-1 ��ǰһ���Ľ���״̬ (kalman->state_post)
                 (Ӧ���ڿ�ʼ��ĳ���ط���ʼ������ȱʡΪ������),
uk ���ⲿ����(control ����),
P'k �����������ؾ��� (kalman->error_cov_pre)
Pk-1 ��ǰһ���ĺ��������ؾ���(kalman->error_cov_post)
                 (Ӧ���ڿ�ʼ��ĳ���ط���ʼ������ȱʡΪ��λ����),

�������ع��Ƶõ���״ֵ̬  */
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

// ����ģ��״̬
/* kalman 
�����µ� Kalman �ṹ��ָ�� 
measurement 
ָ�����������ָ�룬������ʽΪ CvMat  
���� cvKalmanCorrect �ڸ�����ģ��״̬�Ĳ��������ϣ��������ģ��״̬��

Kk=P'k?HT?(H?P'k?HT+R)-1
xk=x'k+Kk?(zk-H?x'k)
Pk=(I-Kk?H)?P'k
����
zk - ��������(mesurement parameter)
Kk - Kalman "����" ����

�����洢����״̬�� kalman->state_post �в������ʱ������  */
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
