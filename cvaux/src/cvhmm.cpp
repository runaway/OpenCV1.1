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

#include "_cvaux.h"

#define LN2PI 1.837877f
#define BIG_FLT 1.e+10f


#define _CV_ERGODIC 1
#define _CV_CAUSAL 2

#define _CV_LAST_STATE 1
#define _CV_BEST_STATE 2  

/* HMM中间的过程自己还是有地方没有看明白，看来一个算法说出来真不是自己想的那么简单，明天接着看
1.评价问题，就是人脸中的识别，给定的人脸观测序列，和 r模型，来计算P最大的就是最
相似的。就是说人脸数据库中有A1，A2,....几个模型，现在把给定的观测序列o1，o2，。。。
来计算看看那一个P（o|A）大，那么就是哪一个。这里的算法就是前向后向算法。
2.解码问题，有给定的lamuda和观测序列，确定最优的状态。viterbi伴随给定观测序列产
生的最佳状态序列。称为对观测序列的viterbi分割。
3.学习问题，又给定的观测序列建立模型是概率最大，就是一个参数估计的问题，人脸识别
中就是同一个人的不同人脸的训练过程。Baum-Welch，迭代的方法。

观测到的人脸的特征被看成是不可观测的状态（隐的）产生的实现。一个合理的或者好的
HMM模型应该是这样的：给定一组观测序列，由关于状态的适当的初始状态出发，得到一个
观测序列的实现，非常好的逼近实际的观测序列。

训练的过程就是：给定的人脸提取DCT，然后确定参数，这里要用到viterbi分割来确定状态
数目或者状态参数。用前向后向算法计算观测序列在在这个观测模型下的P，看是不是很接
近观测序列的模型。然后用用Baum-Welch算法参数估计得到最优的参数模型，这里可以给定
一个概率的阈值之差。

识别的过程就是：对要识别的人脸图像提取出DCT观测序列，然后用前向后向算法计算P，最
大的就是那一类。

EHMM中每个状态的密度函数可采用包含一个或者多个分量混合高斯密度函数。

EHMM的训练过程：
（1）2D-DCT观测序列
（2）确定超状态和其中的子状态数目5，3，6，6，6，3.
（3）根据超状态的个数和内部个数将人脸均匀分割。
（4）根据状态数和图像均匀分割得到的观测向量得到EHMM的参数
（5）Baum-Welch迭代计算。

识别和HMM差不多，其实训练和识别的过程和HMM都差不多 */
/* 二维隐马尔科夫模型 
我们前面介绍的采样窗是垂直方向的，自然想到是不是也可以在水平方向建立状态这就是二
维隐马尔科夫模型，但是二维HMM的训练和识别算法的复杂性，使用不是很理想，嵌入式隐
马尔科夫模型由一系列超状态组成，每个超状态又包含若干状态，称为嵌入状态，超状态反
应其中的一维信息，嵌入状态反映另一维的信息，但是由于超状态内的状态之间没有状态的
转移，所以不是真正的二维。只能看作是一个简化的二维。
人脸的超状态模型就是刚才的从上到下的五个状态（前面的HMM中的五个状态），在各个超
状态之间增加水平信息。转移关系还是从一个超状态到另一个超状态。嵌入的水平状态数为
3，6，6，6，3用一个或多个分量的混合高斯密度函数表示。

观测向量的提取，首先把人脸分为图像快。然后取图像块的灰度值或者变换系数组成一个观
测向量，图像快采用遍历的方法进行采样，就是从上到下，从左到右，来获取图像的采样快。
2D-DCT的低频分量。 */
//*F///////////////////////////////////////////////////////////////////////////////////////
//    Name: _cvCreateObsInfo
//    Purpose: The function allocates memory for CvImgObsInfo structure 
//             and its inner stuff
//    Context:
//    Parameters: obs_info - addres of pointer to CvImgObsInfo structure
//                num_hor_obs - number of horizontal observation vectors
//                num_ver_obs - number of horizontal observation vectors
//                obs_size - length of observation vector
//
//    Returns: error status
//
//    Notes:   
//F*/      
static CvStatus CV_STDCALL icvCreateObsInfo(  CvImgObsInfo** obs_info, 
                                           CvSize num_obs, int obs_size )
{
    int total = num_obs.height * num_obs.width;
 
    CvImgObsInfo* obs = (CvImgObsInfo*)cvAlloc( sizeof( CvImgObsInfo) );
    
    obs->obs_x = num_obs.width;
    obs->obs_y = num_obs.height;

    obs->obs = (float*)cvAlloc( total * obs_size * sizeof(float) );

    obs->state = (int*)cvAlloc( 2 * total * sizeof(int) );
    obs->mix = (int*)cvAlloc( total * sizeof(int) );  
        
    obs->obs_size = obs_size;

    obs_info[0] = obs;
 
    return CV_NO_ERR;
}

static CvStatus CV_STDCALL icvReleaseObsInfo( CvImgObsInfo** p_obs_info )
{
    CvImgObsInfo* obs_info = p_obs_info[0];

    cvFree( &(obs_info->obs) );
    cvFree( &(obs_info->mix) );
    cvFree( &(obs_info->state) ); 
    cvFree( &(obs_info) );

    p_obs_info[0] = NULL;

    return CV_NO_ERR;
} 

    
//*F///////////////////////////////////////////////////////////////////////////////////////
//    Name: icvCreate2DHMM
//    Purpose: The function allocates memory for 2-dimensional embedded HMM model 
//             and its inner stuff
//    Context:
//    Parameters: hmm - addres of pointer to CvEHMM structure
//                state_number - array of hmm sizes (size of array == state_number[0]+1 )
//                num_mix - number of gaussian mixtures in low-level HMM states 
//                          size of array is defined by previous array values
//                obs_size - length of observation vectors
//
//    Returns: error status
//
//    Notes: state_number[0] - number of states in external HMM.
//           state_number[i] - number of states in embedded HMM
//           
//           example for face recognition: state_number = { 5 3 6 6 6 3 },
//                                         length of num_mix array = 3+6+6+6+3 = 24//
//
//F*/
static CvStatus CV_STDCALL icvCreate2DHMM(CvEHMM** this_hmm,
                                          int* state_number, 
                                          int* num_mix, 
                                          int obs_size)
{
    int i;
    int real_states = 0;

    CvEHMMState* all_states;
    CvEHMM* hmm;
    int total_mix = 0;
    float* pointers;

	// 计算2维HMM里面所有级别的状态总数
    // compute total number of states of all level in 2d EHMM
    for (i = 1; i <= state_number[0]; i++)
    {
        real_states += state_number[i];
    }

	// 给所有的HMM分配空间
    /* allocate memory for all hmms (from all levels) */
    hmm = (CvEHMM*)cvAlloc((state_number[0] + 1) * sizeof(CvEHMM));

    // 设置超状态的数目
    /* set number of superstates */
    hmm[0].num_states = state_number[0];
    hmm[0].level = 1;

    // 给所有的状态分配空间    
    /* allocate memory for all states */
    all_states = (CvEHMMState *)cvAlloc(real_states * sizeof(CvEHMMState));

	// 指定混合分量的数目
    /* assign number of mixtures */
    for (i = 0; i < real_states; i++)
    {
        all_states[i].num_mix = num_mix[i];
    }

	// 计算所有实际状态的内部长度
    /* compute size of inner of all real states */
    for (i = 0; i < real_states; i++)
    {
        total_mix += num_mix[i]; // total_mix 72
    } 

    // 给所有状态成员分配空间
    /* allocate memory for states stuff */
    pointers = (float*)cvAlloc(total_mix * (2/*for mu invvar */ * obs_size 
    						 + 2/*for weight and log_var_val*/ ) 
    						 * sizeof(float));

    // 组织内存
    /* organize memory */
    for (i = 0; i < real_states; i++)
    {
        all_states[i].mu      = pointers; pointers += num_mix[i] * obs_size;  
        all_states[i].inv_var = pointers; pointers += num_mix[i] * obs_size;

        all_states[i].log_var_val = pointers; pointers += num_mix[i];
        all_states[i].weight      = pointers; pointers += num_mix[i];
    }          

    // 给EHMM数组设置指针
    /* set pointer to embedded hmm array */
    hmm->u.ehmm = hmm + 1;
    
    for (i = 0; i < hmm[0].num_states; i++)
    {
        hmm[i + 1].u.state = all_states;
        all_states += state_number[i + 1];
        hmm[i + 1].num_states = state_number[i + 1];
    }                              
    
    for (i = 0; i <= state_number[0]; i++)
    {
    	// 设置转移概率矩阵，观察值概率矩阵的指针
        hmm[i].transP = icvCreateMatrix_32f(hmm[i].num_states, hmm[i].num_states);
        hmm[i].obsProb = NULL;
        hmm[i].level = i ? 0 : 1;
    }

    // 如果所有的正常则返回指针
    /* if all ok - return pointer */
    *this_hmm = hmm;
    
    return CV_NO_ERR;
} 

static CvStatus CV_STDCALL icvRelease2DHMM( CvEHMM** phmm )
{
    CvEHMM* hmm = phmm[0]; 
    int i;
    for( i = 0; i < hmm[0].num_states + 1; i++ )
    {
        icvDeleteMatrix( hmm[i].transP );
    } 

    if (hmm->obsProb != NULL)
    {
        int* tmp = ((int*)(hmm->obsProb)) - 3;
        cvFree( &(tmp)  );
    }

    cvFree( &(hmm->u.ehmm->u.state->mu) );
    cvFree( &(hmm->u.ehmm->u.state) );


    /* free hmm structures */
    cvFree( phmm );

    phmm[0] = NULL;

    return CV_NO_ERR;
}     

/* distance between 2 vectors */
static float icvSquareDistance( CvVect32f v1, CvVect32f v2, int len )
{
    int i;
    double dist0 = 0;
    double dist1 = 0;

    for( i = 0; i <= len - 4; i += 4 )
    {
        double t0 = v1[i] - v2[i];
        double t1 = v1[i+1] - v2[i+1];
        dist0 += t0*t0;
        dist1 += t1*t1;

        t0 = v1[i+2] - v2[i+2];
        t1 = v1[i+3] - v2[i+3];
        dist0 += t0*t0;
        dist1 += t1*t1;
    }

    for( ; i < len; i++ )
    {
        double t0 = v1[i] - v2[i];
        dist0 += t0*t0;
    }

    return (float)(dist0 + dist1);
} 

/*******************************************************************************
*   Func Name: icvUniformImgSegm                                                
* Description: 对图像HMM状态执行统一分割的函数
*       Input:                                   
*      Output:                                               
*      Return:                                      
*     Caution:
*-------------------------------------------------------------------------------
* Modification History
*      1.Date: 2009-06-12
*      Author: Runaway
* Description:                                           
*******************************************************************************/
// Performs uniform segmentation of image observations by HMM states
// 可以用于CHMM和DHMM
/*can be used in CHMM & DHMM */
static CvStatus CV_STDCALL
icvUniformImgSegm(CvImgObsInfo* obs_info, CvEHMM* hmm)
{
#if 1
	// 执行得很不好
    /* implementation is very bad */
    int  i, j, counter = 0;
    CvEHMMState* first_state;
    float inv_x = 1.f/obs_info->obs_x;
    float inv_y = 1.f/obs_info->obs_y;

	// 检测参数
    /* check arguments */
    if (!obs_info || !hmm) 
    {
    	return CV_NULLPTR_ERR;
    }

    first_state = hmm->u.ehmm->u.state;
            
    for (i = 0; i < obs_info->obs_y; i++)
    {
        //bad line (division )
        // 计算超状态个数
        int superstate = (int)((i * hmm->num_states)*inv_y);/* /obs_info->obs_y; */
        
        int index = (int)(hmm->u.ehmm[superstate].u.state - first_state);

		// 嵌入状态的的状态数分别为3、6、6、6、3，用来表示水平方向的抽象化状态。
        for (j = 0; j < obs_info->obs_x; j++, counter++)
        {
            int state = (int)((j * hmm->u.ehmm[superstate].num_states)* inv_x); /* / obs_info->obs_x; */
            
            obs_info->state[2 * counter] = superstate;
            obs_info->state[2 * counter + 1] = state + index;
        }
    } 
#else
    //this is not ready yet

    int i,j,k,m;
    CvEHMMState* first_state = hmm->u.ehmm->u.state; 

    /* check bad arguments */
    if ( hmm->num_states > obs_info->obs_y ) return CV_BADSIZE_ERR;

    //compute vertical subdivision
    float row_per_state = (float)obs_info->obs_y / hmm->num_states;
    float col_per_state[1024]; /* maximum 1024 superstates */
    
    //for every horizontal band compute subdivision
    for( i = 0; i < hmm->num_states; i++ )
    {
        CvEHMM* ehmm = &(hmm->u.ehmm[i]);
        col_per_state[i] = (float)obs_info->obs_x / ehmm->num_states;
    }

    //compute state bounds
    int ss_bound[1024];
    for( i = 0; i < hmm->num_states - 1; i++ )
    {
        ss_bound[i] = floor( row_per_state * ( i+1 ) );
    }
    ss_bound[hmm->num_states - 1] = obs_info->obs_y;

    //work inside every superstate

    int row = 0;

    for( i = 0; i < hmm->num_states; i++ )
    {
        CvEHMM* ehmm = &(hmm->u.ehmm[i]);
        int index = ehmm->u.state - first_state;

        //calc distribution in superstate
        int es_bound[1024];
        for( j = 0; j < ehmm->num_states - 1; j++ )
        {
            es_bound[j] = floor( col_per_state[i] * ( j+1 ) );
        }
        es_bound[ehmm->num_states - 1] = obs_info->obs_x;

        //assign states to first row of superstate
        int col = 0;
        for( j = 0; j < ehmm->num_states; j++ )
        {
            for( k = col; k < es_bound[j]; k++, col++ )
            {
                obs_info->state[row * obs_info->obs_x + 2 * k] = i;
                obs_info->state[row * obs_info->obs_x + 2 * k + 1] = j + index;
            }
            col = es_bound[j]; 
        }

        //copy the same to other rows of superstate
        for( m = row; m < ss_bound[i]; m++ )
        {
            memcpy( &(obs_info->state[m * obs_info->obs_x * 2]), 
                    &(obs_info->state[row * obs_info->obs_x * 2]), obs_info->obs_x * 2 * sizeof(int) );
        }

        row = ss_bound[i];    
    }   

#endif

    return CV_NO_ERR;
}
           
/*******************************************************************************
*   Func Name: icvInitMixSegm                                                
* Description: 执行EHMM状态的混合分量分割的函数
*       Input:                                   
*      Output:                                               
*      Return:                                      
*     Caution:
*-------------------------------------------------------------------------------
* Modification History
*      1.Date: 2009-06-19
*      Author: Runaway
* Description:                                           
*******************************************************************************/
/*F///////////////////////////////////////////////////////////////////////////////////////
//    Name: InitMixSegm
//    Purpose: The function implements the mixture segmentation of the states of the
//             embedded HMM
//    Context: used with the Viterbi training of the embedded HMM
//             Function uses K-Means algorithm for clustering
//
//    Parameters:  obs_info_array - array of pointers to image observations 
//                 num_img - length of above array
//                 hmm - pointer to HMM structure   
//     
//    Returns: error status
//
//    Notes: 
//F*/
static CvStatus CV_STDCALL
icvInitMixSegm(CvImgObsInfo** obs_info_array, int num_img, CvEHMM* hmm)
{                                      
    int  k, i, j; 

    // 定义每个状态的观察值的数目
    int* num_samples; /* number of observations in every state */

    // 定义每个状态得计数器的矩阵
    int* counter;     /* array of counters for every state */

    // 对于每个状态，定义指向特征矩阵的指针
    int**  a_class;   /* for every state - characteristic array */

    // 对于每个状态，定义指向观察向量的指针
    CvVect32f** samples; /* for every state - pointer to observation vectors */

	// 对于每个状态，定义指向向量混合量的指针数组
    int***  samples_mix;   /* for every state - array of pointers to vectors mixtures */   
    
    CvTermCriteria criteria = cvTermCriteria( CV_TERMCRIT_EPS|CV_TERMCRIT_ITER,
                                              1000,    /* iter */
                                              0.01f ); /* eps  */
    
    int total = 0;
    
    CvEHMMState* first_state = hmm->u.ehmm->u.state; 

    // 计算所有的状态数
    for (i = 0 ; i < hmm->num_states; i++)
    {
        total += hmm->u.ehmm[i].num_states;
    }                                  

    // 对于每一个状态分配整数大小的空间 - 状态内的向量数
    /* for every state integer is allocated - number of vectors in state */
    num_samples = (int*)cvAlloc( total * sizeof(int) );

    // 对于每一个状态分配一个整数计数器
    /* integer counter is allocated for every state */
    counter = (int*)cvAlloc( total * sizeof(int) );
    
    samples = (CvVect32f**)cvAlloc( total * sizeof(CvVect32f*) ); 
    samples_mix = (int***)cvAlloc( total * sizeof(int**) ); 
    
    /* clear */
    memset( num_samples, 0 , total*sizeof(int) );
    memset( counter, 0 , total*sizeof(int) );
    
    // 对于每个状态，属于它的向量的数目都会被计算(有点像直方图)
    /* for every state the number of vectors which belong to it is computed (smth. like histogram) */
    for (k = 0; k < num_img; k++)
    {  
        CvImgObsInfo* obs = obs_info_array[k];
        int count = 0;
        
        for (i = 0; i < obs->obs_y; i++)
        {
            for (j = 0; j < obs->obs_x; j++, count++)
            {
                int state = obs->state[2 * count + 1];
                num_samples[state] += 1;
            }
        }
    } 

    // 对于每一个状态分配一个int*
    /* for every state int* is allocated */
    a_class = (int**)cvAlloc( total*sizeof(int*) );
    
    for (i = 0; i < total; i++)
    {
        a_class[i] = (int*)cvAlloc( num_samples[i] * sizeof(int) );
        samples[i] = (CvVect32f*)cvAlloc( num_samples[i] * sizeof(CvVect32f) );
        samples_mix[i] = (int**)cvAlloc( num_samples[i] * sizeof(int*) );
    }

    // 对于每个状态，属于它的向量都会被收集
    /* for every state vectors which belong to state are gathered */
    for (k = 0; k < num_img; k++)
    {  
        CvImgObsInfo* obs = obs_info_array[k];
        int num_obs = ( obs->obs_x ) * ( obs->obs_y );
        float* vector = obs->obs;

        for (i = 0; i < num_obs; i++, vector+=obs->obs_size )
        {
            int state = obs->state[2*i+1];
            
            samples[state][counter[state]] = vector;
            samples_mix[state][counter[state]] = &(obs->mix[i]);
            counter[state]++;            
        }
    } 

    // 清空计数器
    /* clear counters */
    memset( counter, 0, total*sizeof(int) );

    // 用K Means算法做实际的聚类
    /* do the actual clustering using the K Means algorithm */
    for (i = 0; i < total; i++)
    {
        if ( first_state[i].num_mix == 1)
        {   
            for (k = 0; k < num_samples[i]; k++)
            {  
            	// 所有的向量属于一个混合分量
                /* all vectors belong to one mixture */
                a_class[i][k] = 0;
            }
        }      
        else if( num_samples[i] )
        {
        	// 聚类向量
            /* clusterize vectors  */
            cvKMeans( first_state[i].num_mix, samples[i], num_samples[i], 
                      obs_info_array[0]->obs_size, criteria, a_class[i] );
        } 
    }

    // 对每个向量指定混合分量的数目
    /* for every vector number of mixture is assigned */
    for( i = 0; i < total; i++ )
    {
        for (j = 0; j < num_samples[i]; j++)
        {
            samples_mix[i][j][0] = a_class[i][j];
        }
    }
    
    for (i = 0; i < total; i++)
    {
        cvFree( &(a_class[i]) );
        cvFree( &(samples[i]) );
        cvFree( &(samples_mix[i]) );
    }

    cvFree( &a_class );
    cvFree( &samples );
    cvFree( &samples_mix );
    cvFree( &counter );
    cvFree( &num_samples );  
    
    return CV_NO_ERR;
}

/*F///////////////////////////////////////////////////////////////////////////////////////
//    Name: ComputeUniModeGauss
//    Purpose: The function computes the Gaussian pdf for a sample vector 
//    Context:
//    Parameters:  obsVeq - pointer to the sample vector
//                 mu - pointer to the mean vector of the Gaussian pdf
//                 var - pointer to the variance vector of the Gaussian pdf
//                 VecSize - the size of sample vector
//                 
//    Returns: the pdf of the sample vector given the specified Gaussian 
//
//    Notes: 
//F*/
/*static float icvComputeUniModeGauss(CvVect32f vect, CvVect32f mu, 
                              CvVect32f inv_var, float log_var_val, int vect_size)           
{
    int n; 
    double tmp;
    double prob;

    prob = -log_var_val;

    for (n = 0; n < vect_size; n++)
    {
        tmp = (vect[n] - mu[n]) * inv_var[n];
        prob = prob - tmp * tmp;
   }
   //prob *= 0.5f;
  
   return (float)prob;
}*/                        

/*F///////////////////////////////////////////////////////////////////////////////////////
//    Name: ComputeGaussMixture
//    Purpose: The function computes the mixture Gaussian pdf of a sample vector. 
//    Context:
//    Parameters:  obsVeq - pointer to the sample vector
//                 mu  - two-dimensional pointer to the mean vector of the Gaussian pdf;
//                       the first dimension is indexed over the number of mixtures and 
//                       the second dimension is indexed along the size of the mean vector
//                 var - two-dimensional pointer to the variance vector of the Gaussian pdf;
//                       the first dimension is indexed over the number of mixtures and 
//                       the second dimension is indexed along the size of the variance vector
//                 VecSize - the size of sample vector
//                 weight - pointer to the wights of the Gaussian mixture
//                 NumMix - the number of Gaussian mixtures
//                 
//    Returns: the pdf of the sample vector given the specified Gaussian mixture.  
//
//    Notes: 
//F*/
/* Calculate probability of observation at state in logarithmic scale*/
/*static float
icvComputeGaussMixture( CvVect32f vect, float* mu, 
                        float* inv_var, float* log_var_val, 
                        int vect_size, float* weight, int num_mix )
{       
    double prob, l_prob;
    
    prob = 0.0f; 

    if (num_mix == 1)
    {
        return icvComputeUniModeGauss( vect, mu, inv_var, log_var_val[0], vect_size);    
    }
    else
    {
        int m;
        for (m = 0; m < num_mix; m++)
        {
            if ( weight[m] > 0.0)
            { 
                l_prob = icvComputeUniModeGauss(vect, mu + m*vect_size, 
                                                        inv_var + m * vect_size,
                                                        log_var_val[m], 
                                                        vect_size); 

                prob = prob + weight[m]*exp((double)l_prob);
            }
        } 
        prob = log(prob);    
    }                        
    return (float)prob;   
}*/                            


/*F///////////////////////////////////////////////////////////////////////////////////////
//    Name: EstimateObsProb
//    Purpose: The function computes the probability of every observation in every state 
//    Context:
//    Parameters:  obs_info - observations
//                 hmm      - hmm
//    Returns: error status  
//
//    Notes: 
//F*/
static CvStatus CV_STDCALL icvEstimateObsProb(CvImgObsInfo* obs_info, 
											  CvEHMM* hmm)
{
    int i, j;
    int total_states = 0;

	// 检察矩阵是否存在并检查当前的大小是否不够
    /* check if matrix exist and check current size
       if not sufficient - realloc */
    int status = 0; /* 1 - not allocated, 2 - allocated but small size, 
                       3 - size is enough, but distribution is bad, 0 - all ok */

    for( j = 0; j < hmm->num_states; j++ )
    {
       total_states += hmm->u.ehmm[j].num_states;
    }

    if ( hmm->obsProb == NULL ) 
    {
    	// 分配内存
        /* allocare memory */
        int need_size = ( obs_info->obs_x * obs_info->obs_y * total_states * sizeof(float) +
                          obs_info->obs_y * hmm->num_states * sizeof( CvMatr32f) );

        int* buffer = (int*)cvAlloc( need_size + 3 * sizeof(int) );
        buffer[0] = need_size;
        buffer[1] = obs_info->obs_y;
        buffer[2] = obs_info->obs_x;
        hmm->obsProb = (float**) (buffer + 3);
        status = 3;
        
    }
    else
    {   
    	// 检察当前的大小
        /* check current size */
        int* total= (int*)(((int*)(hmm->obsProb)) - 3);
        int need_size = ( obs_info->obs_x * obs_info->obs_y * total_states * sizeof(float) +
                          obs_info->obs_y * hmm->num_states * sizeof( CvMatr32f/*(float*)*/ ) );

        assert( sizeof(float*) == sizeof(int) );

        if ( need_size > (*total) ) 
        {
            int* buffer = ((int*)(hmm->obsProb)) - 3;
            cvFree( &buffer);
            buffer = (int*)cvAlloc( need_size + 3 * sizeof(int));
            buffer[0] = need_size;
            buffer[1] = obs_info->obs_y;
            buffer[2] = obs_info->obs_x;

            hmm->obsProb = (float**)(buffer + 3);
            
            status = 3;
        }          
    }

    if (!status)
    {
        int* obsx = ((int*)(hmm->obsProb)) - 1;
        int* obsy = ((int*)(hmm->obsProb)) - 2;
                
        assert( (*obsx > 0) && (*obsy > 0) );

		// 是好的分布状态
        /* is good distribution? */
        if ( (obs_info->obs_x > (*obsx) ) || (obs_info->obs_y > (*obsy) ) ) 
            status = 3;        
    }

    // 如果是坏的状态，这再分配动作
    /* if bad status - do reallocation actions */
    assert( (status == 0) || (status == 3) );

    if ( status )
    {
        float** tmp = hmm->obsProb;
        float*  tmpf;

		// 分配ehmm->obsProb的指针
        /* distribute pointers of ehmm->obsProb */
        for( i = 0; i < hmm->num_states; i++ )
        {
            hmm->u.ehmm[i].obsProb = tmp; 
            tmp += obs_info->obs_y;
        }

        tmpf = (float*)tmp;

		// 分配ehmm->obsProb[j]的指针
        /* distribute pointers of ehmm->obsProb[j] */
        for( i = 0; i < hmm->num_states; i++ )
        {
            CvEHMM* ehmm = &( hmm->u.ehmm[i] );
            
            for( j = 0; j < obs_info->obs_y; j++ )
            {
                ehmm->obsProb[j] = tmpf;
                tmpf += ehmm->num_states * obs_info->obs_x;
            }           
        }
    }/* end of pointer distribution */ 

#if 1    
    {
#define MAX_BUF_SIZE  1200
        float  local_log_mix_prob[MAX_BUF_SIZE];
        double local_mix_prob[MAX_BUF_SIZE];
        int    vect_size = obs_info->obs_size;
        CvStatus res = CV_NO_ERR;

        float*  log_mix_prob = local_log_mix_prob;
        double* mix_prob = local_mix_prob;
        
        int  max_size = 0;
        int  obs_x = obs_info->obs_x;

		// 计算临时缓冲区大小
        /* calculate temporary buffer size */
        for( i = 0; i < hmm->num_states; i++ )
        {
            CvEHMM* ehmm = &(hmm->u.ehmm[i]);
            CvEHMMState* state = ehmm->u.state;

            int max_mix = 0;
            for( j = 0; j < ehmm->num_states; j++ )
            {
                int t = state[j].num_mix;
                if( max_mix < t ) max_mix = t;
            }
            max_mix *= ehmm->num_states;
            if( max_size < max_mix ) max_size = max_mix;
        }

        max_size *= obs_x * vect_size;

        // 分配缓冲区
        /* allocate buffer */
        if( max_size > MAX_BUF_SIZE )
        {
            log_mix_prob = (float*)cvAlloc( max_size*(sizeof(float) + sizeof(double)));
            if( !log_mix_prob ) return CV_OUTOFMEM_ERR;
            mix_prob = (double*)(log_mix_prob + max_size);
        }

        memset( log_mix_prob, 0, max_size*sizeof(float));

		// 计算概率
        /*****************computing probabilities***********************/

        // 通过外部HMM状态的数目来循环
        /* loop through external states */
        for (i = 0; i < hmm->num_states; i++)
        {
            CvEHMM* ehmm = &(hmm->u.ehmm[i]);
            CvEHMMState* state = ehmm->u.state;
            
            int max_mix = 0;
            int n_states = ehmm->num_states;

			// 决定最大混合分量的数目
            /* determine maximal number of mixtures (again) */
            for( j = 0; j < ehmm->num_states; j++ )
            {
                int t = state[j].num_mix;
                if( max_mix < t ) max_mix = t;
            }

			// 通过观察矩阵的行数来循环	
            /* loop through rows of the observation matrix */
            for (j = 0; j < obs_info->obs_y; j++)
            {
                int  m, n;
                       
                float* obs = obs_info->obs + j * obs_x * vect_size;
                float* log_mp = max_mix > 1 ? log_mix_prob : ehmm->obsProb[j];
                double* mp = mix_prob;
            
                /* several passes are done below */

                // 对每个混合分量计算概率的对数
                /* 1. calculate logarithms of probabilities for each mixture */

				// 通过混合分量来循环
                /* loop through mixtures */
                for (m = 0; m < max_mix; m++)
                {
                	// 把指针指向一行的第一个观察值
                    /* set pointer to first observation in the line */
                    float* vect = obs;

					// 遍历一行的每一个观察值
                    /* cycles through obseravtions in the line */
                    for( n = 0; n < obs_x; n++, vect += vect_size, log_mp += n_states )
                    {
                        int k, l;
                        for( l = 0; l < n_states; l++ )
                        {
                            if( state[l].num_mix > m )
                            {
                                float* mu = state[l].mu + m*vect_size;
                                float* inv_var = state[l].inv_var + m*vect_size;
                                double prob = -state[l].log_var_val[m];
                                for( k = 0; k < vect_size; k++ )
                                {
                                    double t = (vect[k] - mu[k])*inv_var[k];
                                    prob -= t*t;
                                }
                                log_mp[l] = MAX( (float)prob, -500 );
                            }
                        }
                    }
                }

				// 如果有一个单一的混合分量，则跳过剩余的
                /* skip the rest if there is a single mixture */
                if( max_mix == 1 ) continue;

				// 计算log_mix_prob的指数，例如每个混合分量的概率
                /* 2. calculate exponent of log_mix_prob
                      (i.e. probability for each mixture) */
                cvbFastExp( log_mix_prob, mix_prob, max_mix * obs_x * n_states );

				// 对所有混合分量带权重求和
                /* 3. sum all mixtures with weights */
                /* 3a. first mixture - simply scale by weight */
                for( n = 0; n < obs_x; n++, mp += n_states )
                {
                    int l;
                    for( l = 0; l < n_states; l++ )
                    {
                        mp[l] *= state[l].weight[0];
                    }
                }

				// 加上其他的混合分量
                /* 3b. add other mixtures */
                for( m = 1; m < max_mix; m++ )
                {
                    int ofs = -m*obs_x*n_states;
                    for( n = 0; n < obs_x; n++, mp += n_states )
                    {
                        int l;
                        for( l = 0; l < n_states; l++ )
                        {
                            if( m < state[l].num_mix )
                            {
                                mp[l + ofs] += mp[l] * state[l].weight[m];
                            }
                        }
                    }
                }

				// 把概率摘要的对数推给矩阵目标
                /* 4. Put logarithms of summary probabilities to the destination matrix */
                cvbFastLog( mix_prob, ehmm->obsProb[j], obs_x * n_states );
            }
        }

        if( log_mix_prob != local_log_mix_prob ) cvFree( &log_mix_prob );
        return res;
#undef MAX_BUF_SIZE
    }
#else
    for( i = 0; i < hmm->num_states; i++ )
    {
        CvEHMM* ehmm = &(hmm->u.ehmm[i]);
        CvEHMMState* state = ehmm->u.state;

        for( j = 0; j < obs_info->obs_y; j++ )
        {
            int k,m;
                       
            int obs_index = j * obs_info->obs_x;

            float* B = ehmm->obsProb[j];
            
            /* cycles through obs and states */
            for( k = 0; k < obs_info->obs_x; k++ )
            {
                CvVect32f vect = (obs_info->obs) + (obs_index + k) * vect_size;
                
                float* matr_line = B + k * ehmm->num_states;

                for( m = 0; m < ehmm->num_states; m++ )
                {
                    matr_line[m] = icvComputeGaussMixture( vect, state[m].mu, state[m].inv_var, 
                                                             state[m].log_var_val, vect_size, state[m].weight,
                                                             state[m].num_mix );
                }
            }
        }
    }
#endif
}


/*F///////////////////////////////////////////////////////////////////////////////////////
//    Name: EstimateTransProb
//    Purpose: The function calculates the state and super state transition probabilities 
//             of the model given the images, 
//             the state segmentation and the input parameters
//    Context:
//    Parameters: obs_info_array - array of pointers to image observations 
//                num_img - length of above array
//                hmm - pointer to HMM structure                 
//    Returns: void
//
//    Notes:   
//F*/
static CvStatus CV_STDCALL
icvEstimateTransProb( CvImgObsInfo** obs_info_array, int num_img, CvEHMM* hmm )
{
    int  i, j, k;

    CvEHMMState* first_state = hmm->u.ehmm->u.state;
    /* as a counter we will use transP matrix */
    
    /* initialization */
    
    /* clear transP */
    icvSetZero_32f( hmm->transP, hmm->num_states, hmm->num_states );
    for (i = 0; i < hmm->num_states; i++ )
    {
        icvSetZero_32f( hmm->u.ehmm[i].transP , hmm->u.ehmm[i].num_states, hmm->u.ehmm[i].num_states );
    }
        
    /* compute the counters */
    for (i = 0; i < num_img; i++)
    {
        int counter = 0;
        CvImgObsInfo* info = obs_info_array[i];
        
        for (j = 0; j < info->obs_y; j++)
        {
            for (k = 0; k < info->obs_x; k++, counter++)
            {
                /* compute how many transitions from state to state
                   occured both in horizontal and vertical direction */ 
                int superstate, state;
                int nextsuperstate, nextstate;
                int begin_ind;

                superstate = info->state[2 * counter];
                begin_ind = (int)(hmm->u.ehmm[superstate].u.state - first_state);
                state = info->state[ 2 * counter + 1] - begin_ind; 
                
                if (j < info->obs_y - 1)
                {
                    int transP_size = hmm->num_states;
                    
                    nextsuperstate = info->state[ 2*(counter + info->obs_x) ];

                    hmm->transP[superstate * transP_size + nextsuperstate] += 1;
                }
                
                if (k < info->obs_x - 1)
                {   
                    int transP_size = hmm->u.ehmm[superstate].num_states;

                    nextstate = info->state[2*(counter+1) + 1] - begin_ind;
                    hmm->u.ehmm[superstate].transP[ state * transP_size + nextstate] += 1;
                }
            }
        }
    }
    /* estimate superstate matrix */
    for( i = 0; i < hmm->num_states; i++)
    {
        float total = 0;
        float inv_total;
        for( j = 0; j < hmm->num_states; j++)
        {
            total += hmm->transP[i * hmm->num_states + j];
        }
        //assert( total );

        inv_total = total ? 1.f/total : 0;
        
        for( j = 0; j < hmm->num_states; j++)
        {                   
            hmm->transP[i * hmm->num_states + j] = 
                hmm->transP[i * hmm->num_states + j] ? 
                (float)log( hmm->transP[i * hmm->num_states + j] * inv_total ) : -BIG_FLT;
        }
    }
    
    /* estimate other matrices */
    for( k = 0; k < hmm->num_states; k++ )
    {
        CvEHMM* ehmm = &(hmm->u.ehmm[k]);

        for( i = 0; i < ehmm->num_states; i++)
        {
            float total = 0;
            float inv_total;
            for( j = 0; j < ehmm->num_states; j++)
            {
                total += ehmm->transP[i*ehmm->num_states + j];
            }
            //assert( total );
            inv_total = total ? 1.f/total :  0;
            
            for( j = 0; j < ehmm->num_states; j++)
            {                   
                ehmm->transP[i * ehmm->num_states + j] = 
                    (ehmm->transP[i * ehmm->num_states + j]) ?
                    (float)log( ehmm->transP[i * ehmm->num_states + j] * inv_total) : -BIG_FLT ;
            }
        }
    }
    return CV_NO_ERR;
} 
                      

/*F///////////////////////////////////////////////////////////////////////////////////////
//    Name: MixSegmL2
//    Purpose: The function implements the mixture segmentation of the states of the
//             embedded HMM
//    Context: used with the Viterbi training of the embedded HMM
//
//    Parameters:  
//             obs_info_array
//             num_img
//             hmm
//    Returns: void
//
//    Notes: 
//F*/
static CvStatus CV_STDCALL
icvMixSegmL2( CvImgObsInfo** obs_info_array, int num_img, CvEHMM* hmm )
{
    int     k, i, j, m;
       
    CvEHMMState* state = hmm->u.ehmm[0].u.state;
    
    
    for (k = 0; k < num_img; k++)
    {   
        int counter = 0;
        CvImgObsInfo* info = obs_info_array[k];

        for (i = 0; i < info->obs_y; i++)
        {
            for (j = 0; j < info->obs_x; j++, counter++)
            {
                int e_state = info->state[2 * counter + 1];
                float min_dist;
                                                
                min_dist = icvSquareDistance((info->obs) + (counter * info->obs_size), 
                                               state[e_state].mu, info->obs_size);
                info->mix[counter] = 0;  
                
                for (m = 1; m < state[e_state].num_mix; m++)
                {
                    float dist=icvSquareDistance( (info->obs) + (counter * info->obs_size),
                                                    state[e_state].mu + m * info->obs_size,
                                                    info->obs_size);
                    if (dist < min_dist)
                    {
                        min_dist = dist;
                        /* assign mixture with smallest distance */ 
                        info->mix[counter] = m;
                    }
                }
            }
        }
    }
    return CV_NO_ERR;
} 

/*
CvStatus icvMixSegmProb(CvImgObsInfo* obs_info, int num_img, CvEHMM* hmm )
{
    int     k, i, j, m;
       
    CvEHMMState* state = hmm->ehmm[0].state_info;
    
    
    for (k = 0; k < num_img; k++)
    {   
        int counter = 0;
        CvImgObsInfo* info = obs_info + k;

        for (i = 0; i < info->obs_y; i++)
        {
            for (j = 0; j < info->obs_x; j++, counter++)
            {
                int e_state = info->in_state[counter];
                float max_prob;
                                                
                max_prob = icvComputeUniModeGauss( info->obs[counter], state[e_state].mu[0], 
                                                    state[e_state].inv_var[0], 
                                                    state[e_state].log_var[0],
                                                    info->obs_size );
                info->mix[counter] = 0;  
                
                for (m = 1; m < state[e_state].num_mix; m++)
                {
                    float prob=icvComputeUniModeGauss(info->obs[counter], state[e_state].mu[m],
                                                       state[e_state].inv_var[m], 
                                                       state[e_state].log_var[m],
                                                       info->obs_size);
                    if (prob > max_prob)
                    {
                        max_prob = prob;
                        // assign mixture with greatest probability. 
                        info->mix[counter] = m;
                    }
                }
            }
        }
    } 

    return CV_NO_ERR;
} 
*/
static CvStatus CV_STDCALL
icvViterbiSegmentation( int num_states, int /*num_obs*/, CvMatr32f transP,
                        CvMatr32f B, int start_obs, int prob_type,
                        int** q, int min_num_obs, int max_num_obs,
                        float* prob )
{
    // memory allocation 
    int i, j, last_obs;
    int m_HMMType = _CV_ERGODIC; /* _CV_CAUSAL or _CV_ERGODIC */
    
    int m_ProbType   = prob_type; /* _CV_LAST_STATE or _CV_BEST_STATE */
    
    int m_minNumObs  = min_num_obs; /*??*/
    int m_maxNumObs  = max_num_obs; /*??*/
    
    int m_numStates  = num_states;
    
    float* m_pi = (float*)cvAlloc( num_states* sizeof(float) );
    CvMatr32f m_a = transP;

    // offset brobability matrix to starting observation 
    CvMatr32f m_b = B + start_obs * num_states;
    //so m_xl will not be used more

    //m_xl = start_obs; 

    /*     if (muDur != NULL){ 
    m_d = new int[m_numStates];
    m_l = new double[m_numStates];
    for (i = 0; i < m_numStates; i++){
    m_l[i] = muDur[i]; 
    }
    } 
    else{
    m_d = NULL;
    m_l = NULL;
    }
    */
    
    CvMatr32f m_Gamma = icvCreateMatrix_32f( num_states, m_maxNumObs );
    int* m_csi = (int*)cvAlloc( num_states * m_maxNumObs * sizeof(int) );
    
    //stores maximal result for every ending observation */
    CvVect32f   m_MaxGamma = prob;
    

//    assert( m_xl + max_num_obs <= num_obs );

    /*??m_q          = new int*[m_maxNumObs - m_minNumObs];
      ??for (i = 0; i < m_maxNumObs - m_minNumObs; i++)
      ??     m_q[i] = new int[m_minNumObs + i + 1];
    */

    /******************************************************************/
    /*    Viterbi initialization                                      */
    /* set initial state probabilities, in logarithmic scale */
    for (i = 0; i < m_numStates; i++)
    {
        m_pi[i] = -BIG_FLT;
    }
    m_pi[0] = 0.0f;
    
    for  (i = 0; i < num_states; i++)
    {
        m_Gamma[0 * num_states + i] = m_pi[i] + m_b[0 * num_states + i];
        m_csi[0 * num_states + i] = 0;   
    }
    
    /******************************************************************/
    /*    Viterbi recursion                                           */
    
    if ( m_HMMType == _CV_CAUSAL ) //causal model
    {
        int t,j;
        
        for (t = 1 ; t < m_maxNumObs; t++)
        {
            // evaluate self-to-self transition for state 0
            m_Gamma[t * num_states + 0] = m_Gamma[(t-1) * num_states + 0] + m_a[0];
            m_csi[t * num_states + 0] = 0;
            
            for (j = 1; j < num_states; j++)
            {  
                float self = m_Gamma[ (t-1) * num_states + j] + m_a[ j * num_states + j];
                float prev = m_Gamma[ (t-1) * num_states +(j-1)] + m_a[ (j-1) * num_states + j];
                
                if ( prev > self )
                {
                    m_csi[t * num_states + j] = j-1;
                    m_Gamma[t * num_states + j] = prev;
                }
                else
                {
                    m_csi[t * num_states + j] = j;
                    m_Gamma[t * num_states + j] = self;
                }
                
                m_Gamma[t * num_states + j] = m_Gamma[t * num_states + j] + m_b[t * num_states + j];   
            }                                                                    
        }
    }
    else if ( m_HMMType == _CV_ERGODIC ) //ergodic model 
    { 
        int t;
        for (t = 1 ; t < m_maxNumObs; t++)
        {     
            for (j = 0; j < num_states; j++)
            {   
                int i;
                m_Gamma[ t*num_states + j] = m_Gamma[(t-1) * num_states + 0] + m_a[0*num_states+j];
                m_csi[t *num_states + j] = 0;
                
                for (i = 1; i < num_states; i++)
                {
                    float currGamma = m_Gamma[(t-1) *num_states + i] + m_a[i *num_states + j];       
                    if (currGamma > m_Gamma[t *num_states + j])
                    { 
                        m_Gamma[t * num_states + j] = currGamma;
                        m_csi[t * num_states + j] = i;
                    }
                } 
                m_Gamma[t *num_states + j] = m_Gamma[t *num_states + j] + m_b[t * num_states + j];
            }             
        }                  
    }

    for( last_obs = m_minNumObs-1, i = 0; last_obs < m_maxNumObs; last_obs++, i++ )
    {
        int t;

        /******************************************************************/
        /*    Viterbi termination                                         */
    
        if ( m_ProbType == _CV_LAST_STATE )
        {
            m_MaxGamma[i] = m_Gamma[last_obs * num_states + num_states - 1];
            q[i][last_obs] = num_states - 1;
        }
        else if( m_ProbType == _CV_BEST_STATE )
        {
            int k;
            q[i][last_obs] = 0;  
            m_MaxGamma[i] = m_Gamma[last_obs * num_states + 0]; 
        
            for(k = 1; k < num_states; k++)
            { 
                if ( m_Gamma[last_obs * num_states + k] > m_MaxGamma[i] )
                {
                    m_MaxGamma[i] = m_Gamma[last_obs * num_states + k];
                    q[i][last_obs] = k;
                }    
            }
        } 
    
        /******************************************************************/
        /*    Viterbi backtracking                                        */
        for  (t = last_obs-1; t >= 0; t--)
        {
            q[i][t] = m_csi[(t+1) * num_states + q[i][t+1] ];   
        } 
    }            
    
    /* memory free */
    cvFree( &m_pi );
    cvFree( &m_csi );
    icvDeleteMatrix( m_Gamma );   
       
    return CV_NO_ERR;
} 

/*F///////////////////////////////////////////////////////////////////////////////////////
//    Name: icvEViterbi
//    Purpose: The function calculates the embedded Viterbi algorithm
//             for 1 image 
//    Context:
//    Parameters:  
//             obs_info - observations
//             hmm      - HMM
//                
//    Returns: the Embedded Viterbi probability (float) 
//             and do state segmentation of observations
//
//    Notes: 
//F*/
static float CV_STDCALL icvEViterbi( CvImgObsInfo* obs_info, CvEHMM* hmm )
{
    int    i, j, counter;
    float  log_likelihood;

    float inv_obs_x = 1.f / obs_info->obs_x;

    CvEHMMState* first_state = hmm->u.ehmm->u.state;
    
    /* memory allocation for superB */
    CvMatr32f superB = icvCreateMatrix_32f(hmm->num_states, obs_info->obs_y );
    
    /* memory allocation for q */
    int*** q = (int***)cvAlloc( hmm->num_states * sizeof(int**) );
    int* super_q = (int*)cvAlloc( obs_info->obs_y * sizeof(int) );
    
    for (i = 0; i < hmm->num_states; i++)
    {
        q[i] = (int**)cvAlloc( obs_info->obs_y * sizeof(int*) );
        
        for (j = 0; j < obs_info->obs_y ; j++)
        {
            q[i][j] = (int*)cvAlloc( obs_info->obs_x * sizeof(int) );
        }
    }             

    // 开始Viterbi分割
    /* start Viterbi segmentation */
    for (i = 0; i < hmm->num_states; i++)
    {
        CvEHMM* ehmm = &(hmm->u.ehmm[i]);
        
        for (j = 0; j < obs_info->obs_y; j++)
        {
            float max_gamma;

            // 1维HMM Viterbi分割
            /* 1D HMM Viterbi segmentation */
            icvViterbiSegmentation( ehmm->num_states, obs_info->obs_x, 
                ehmm->transP, ehmm->obsProb[j], 0, 
                _CV_LAST_STATE, &q[i][j], obs_info->obs_x,
                obs_info->obs_x, &max_gamma);
            
            superB[j * hmm->num_states + i] = max_gamma * inv_obs_x;
        }
    }

    // 执行全局Viterbi分割
    /* perform global Viterbi segmentation (i.e. process higher-level HMM) */
    
    icvViterbiSegmentation( hmm->num_states, obs_info->obs_y, 
                             hmm->transP, superB, 0, 
                             _CV_LAST_STATE, &super_q, obs_info->obs_y,
                             obs_info->obs_y, &log_likelihood );
    
    log_likelihood /= obs_info->obs_y ;   
    
    
    counter = 0;

    // 给观察向量指定新的状态
    /* assign new state to observation vectors */
    for (i = 0; i < obs_info->obs_y; i++)
    {   
        for (j = 0; j < obs_info->obs_x; j++, counter++)
        {
            int superstate = super_q[i];
            int state = (int)(hmm->u.ehmm[superstate].u.state - first_state);
            
            obs_info->state[2 * counter] = superstate;
            obs_info->state[2 * counter + 1] = state + q[superstate][i][j];
        }
    }

    // 释放superB的内存
    /* memory deallocation for superB */
    icvDeleteMatrix( superB );

    // 释放q的内存
    /*memory deallocation for q */
    for (i = 0; i < hmm->num_states; i++)
    {
        for (j = 0; j < obs_info->obs_y ; j++)
        {
            cvFree( &q[i][j] );
        }
        cvFree( &q[i] );
    }
    
    cvFree( &q );
    cvFree( &super_q );
    
    return log_likelihood;
}  

static CvStatus CV_STDCALL
icvEstimateHMMStateParams( CvImgObsInfo** obs_info_array, int num_img, CvEHMM* hmm )
{
    /* compute gamma, weights, means, vars */
    int k, i, j, m;
    int total = 0;
    int vect_len = obs_info_array[0]->obs_size;

    float start_log_var_val = LN2PI * vect_len;

    CvVect32f tmp_vect = icvCreateVector_32f( vect_len );
    
    CvEHMMState* first_state = hmm->u.ehmm[0].u.state;

    assert( sizeof(float) == sizeof(int) );

    for(i = 0; i < hmm->num_states; i++ )
    {
        total+= hmm->u.ehmm[i].num_states;
    }

    /***************Gamma***********************/
    /* initialize gamma */
    for( i = 0; i < total; i++ )
    {
        for (m = 0; m < first_state[i].num_mix; m++)
        {
            ((int*)(first_state[i].weight))[m] = 0;
        }     
    }
    
    /* maybe gamma must be computed in mixsegm process ?? */

    /* compute gamma */
    for (k = 0; k < num_img; k++)
    {
        CvImgObsInfo* info = obs_info_array[k];
        int num_obs = info->obs_y * info->obs_x;

        for (i = 0; i < num_obs; i++)
        {
            int state, mixture;
            state = info->state[2*i + 1];
            mixture = info->mix[i];
            /* computes gamma - number of observations corresponding 
               to every mixture of every state */ 
            ((int*)(first_state[state].weight))[mixture] += 1;              
        }
    }     
    /***************Mean and Var***********************/
    /* compute means and variances of every item */
    /* initially variance placed to inv_var */
    /* zero mean and variance */
    for (i = 0; i < total; i++)
    {
        memset( (void*)first_state[i].mu, 0, first_state[i].num_mix * vect_len * 
                                                                         sizeof(float) );
        memset( (void*)first_state[i].inv_var, 0, first_state[i].num_mix * vect_len * 
                                                                         sizeof(float) );
    }
    
    /* compute sums */
    for (i = 0; i < num_img; i++)
    {
        CvImgObsInfo* info = obs_info_array[i];
        int total_obs = info->obs_x * info->obs_y;

        float* vector = info->obs;

        for (j = 0; j < total_obs; j++, vector+=vect_len )
        {   
            int state = info->state[2 * j + 1];
            int mixture = info->mix[j]; 
            
            CvVect32f mean  = first_state[state].mu + mixture * vect_len;
            CvVect32f mean2 = first_state[state].inv_var + mixture * vect_len;
            
            icvAddVector_32f( mean, vector, mean, vect_len );
            for( k = 0; k < vect_len; k++ )
                mean2[k] += vector[k]*vector[k];
        }   
    }
    
    /*compute the means and variances */
    /* assume gamma already computed */
    for (i = 0; i < total; i++)
    {           
        CvEHMMState* state = &(first_state[i]);

        for (m = 0; m < state->num_mix; m++)
        {
            int k;
            CvVect32f mu  = state->mu + m * vect_len;
            CvVect32f invar = state->inv_var + m * vect_len;             
            
            if ( ((int*)state->weight)[m] > 1)
            {
                float inv_gamma = 1.f/((int*)(state->weight))[m];
            
                icvScaleVector_32f( mu, mu, vect_len, inv_gamma);
                icvScaleVector_32f( invar, invar, vect_len, inv_gamma);
            }

            icvMulVectors_32f(mu, mu, tmp_vect, vect_len);
            icvSubVector_32f( invar, tmp_vect, invar, vect_len);     
            
            /* low bound of variance - 100 (Ara's experimental result) */
            for( k = 0; k < vect_len; k++ )
            {
                invar[k] = (invar[k] > 100.f) ? invar[k] : 100.f;
            }

            /* compute log_var */
            state->log_var_val[m] = start_log_var_val;
            for( k = 0; k < vect_len; k++ )
            {
                state->log_var_val[m] += (float)log( invar[k] );
            }           

            /* SMOLI 27.10.2000 */
            state->log_var_val[m] *= 0.5;


            /* compute inv_var = 1/sqrt(2*variance) */
            icvScaleVector_32f(invar, invar, vect_len, 2.f );
            cvbInvSqrt( invar, invar, vect_len );
        }
    }
  
    /***************Weights***********************/
    /* normilize gammas - i.e. compute mixture weights */
    
    //compute weights
    for (i = 0; i < total; i++)
    {           
        int gamma_total = 0;
        float norm;

        for (m = 0; m < first_state[i].num_mix; m++)
        {
            gamma_total += ((int*)(first_state[i].weight))[m];  
        }

        norm = gamma_total ? (1.f/(float)gamma_total) : 0.f;
            
        for (m = 0; m < first_state[i].num_mix; m++)
        {
            first_state[i].weight[m] = ((int*)(first_state[i].weight))[m] * norm;
        } 
    }                                               

    icvDeleteVector( tmp_vect);
    return CV_NO_ERR; 
}   

/*
CvStatus icvLightingCorrection8uC1R( uchar* img, CvSize roi, int src_step )
{
    int i, j;
    int width = roi.width;
    int height = roi.height;
    
    float x1, x2, y1, y2;
    int f[3] = {0, 0, 0};
    float a[3] = {0, 0, 0};
    
    float h1;
    float h2;
    
    float c1,c2;
    
    float min = FLT_MAX;
    float max = -FLT_MAX;
    float correction;
    
    float* float_img = icvAlloc( width * height * sizeof(float) );
    
    x1 = width * (width + 1) / 2.0f; // Sum (1, ... , width)
    x2 = width * (width + 1 ) * (2 * width + 1) / 6.0f; // Sum (1^2, ... , width^2)
    y1 = height * (height + 1)/2.0f; // Sum (1, ... , width)
    y2 = height * (height + 1 ) * (2 * height + 1) / 6.0f; // Sum (1^2, ... , width^2)
    
    
    // extract grayvalues
    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            f[2] = f[2] + j * img[i*src_step + j];
            f[1] = f[1] + i * img[i*src_step + j];
            f[0] = f[0] +     img[i*src_step + j];
        }
    }
    
    h1 = (float)f[0] * (float)x1 / (float)width;
    h2 = (float)f[0] * (float)y1 / (float)height;
    
    a[2] = ((float)f[2] - h1) / (float)(x2*height - x1*x1*height/(float)width);
    a[1] = ((float)f[1] - h2) / (float)(y2*width - y1*y1*width/(float)height);
    a[0] = (float)f[0]/(float)(width*height) - (float)y1*a[1]/(float)height - 
        (float)x1*a[2]/(float)width;
    
    for (i = 0; i < height; i++) 
    {    
        for (j = 0; j < width; j++)
        {
            
            correction = a[0] + a[1]*(float)i + a[2]*(float)j;
            
            float_img[i*width + j] = img[i*src_step + j] - correction;
            
            if (float_img[i*width + j] < min) min = float_img[i*width+j];
            if (float_img[i*width + j] > max) max = float_img[i*width+j];
        }
    }
    
    //rescaling to the range 0:255
    c2 = 0;
    if (max == min)
        c2 = 255.0f;
    else
        c2 = 255.0f/(float)(max - min);
    
    c1 = (-(float)min)*c2;
    
    for (i = 0; i < height; i++)
    {
        for (j = 0; j < width; j++)
        {
            int value = (int)floor(c2*float_img[i*width + j] + c1);
            if (value < 0) value = 0;
            if (value > 255) value = 255;
            img[i*src_step + j] = (uchar)value;
        }
    }

    cvFree( &float_img );
    return CV_NO_ERR;
}
                              

CvStatus icvLightingCorrection( icvImage* img ) 
{
    CvSize roi;
    if ( img->type != IPL_DEPTH_8U || img->channels != 1 )
    return CV_BADFACTOR_ERR;

    roi = _cvSize( img->roi.width, img->roi.height );
    
    return _cvLightingCorrection8uC1R( img->data + img->roi.y * img->step + img->roi.x, 
                                        roi, img->step );

}

*/

/*******************************************************************************
*   Func Name: cvCreate2DHMM                                                 
* Description: 创建二维的EHMM对象的函数
*       Input:                                   
*      Output:                                               
*      Return:                                      
*     Caution:
*-------------------------------------------------------------------------------
* Modification History
*      1.Date: 2009-06-19
*      Author: Runaway
* Description:                                           
*******************************************************************************/
/* 参数stateNumber是数组，其中第一个元素是在HMM中的超状态（就是外部的HMM状态）后
面的元素就是在每一个超状态中的状态数目，就是我们前面说的人脸中的水平的3，6，6，6，
3，数组的长度是stateNumber [0]+1。
numMix每一个内部状态的混合高斯函数是一个指针，在数组中的状态数和这里是一样的，所
以这里不用计算。
obsSize观测向量的大小
这个函数返回一个指向有特定参数的CvEHMM的结构体*/
CV_IMPL CvEHMM*
cvCreate2DHMM(int *state_number, int *num_mix, int obs_size)
{
    CvEHMM* hmm = 0;

    CV_FUNCNAME( "cvCreate2DHMM" );

    __BEGIN__;

    IPPI_CALL( icvCreate2DHMM( &hmm, state_number, num_mix, obs_size ));

    __END__;

    return hmm;
}

// 释放所有HMM占用的内存和指向HMM的指针
CV_IMPL void
cvRelease2DHMM( CvEHMM ** hmm )
{
    CV_FUNCNAME( "cvRelease2DHMM" );

    __BEGIN__;

    IPPI_CALL( icvRelease2DHMM( hmm ));
    __END__;
}

//  创建一个结构体CvImgObsInfo来保存图像的观测向量。
/* numObs:: Numbers of observations in the horizontal and vertical directions.在水平方向和垂直方向的观测数目。 For the given image and scheme of extracting observations the parameter can be computed via the macro 对给定的图像来提取观测的参数可以通过下面的宏

CV_COUNT_OBS( roi, dctSize, delta, numObs ), where 

roi, dctSize, delta, numObs are the pointers to structures of the type CvSize . The pointer 其中roi, dctSize, delta, numObs 是指向CvSize的指针

roi means size of roi of image observed, roi观测图像的大小

numObsis the output parameter of the macro.obsSize:: Size of observation vectors to be stored in the structure. numObsis 是宏的obsSize的输出参数，观测向量被存储在这个结构体中。

The function cvCreateObsInfo creates new structures to store image observation vectors. For definitions of the parameters 函数cvCreateObsInfo 创建了一个新的结构体来存储图像的观测向量

roi, dctSize, and deltas ee the specification of The function 

cvImgToObs_DCT为了确定roi, dctSize, and deltas 这几个参数需要看cvImgToObs_DCT这个函数

Image Observation Structure 

typedef struct CvImgObsInfo { int obs_x; int obs_y; int obs_size; float** obs; int* state; int* mix; } CvImgObsInfo;

obs_x:: Number of observations in the horizontal direction.obs_y:: Number of observations in the vertical direction.obs_size:: Length of every observation vector.obs:: Pointer to observation vectors stored consequently. Number of vectors is obs_x*obs_y

图像观测向量的结构体 obs_x水平向量的观测数目，obs_y垂直向量的观测数目。obs_size每一个观测向量的长度，vector.obs观测向量的指针，观测向量的数目obs_x*obs_y，

.state:: Array of indices of states, assigned to every observation vector.mix:: Index of mixture component, corresponding to the observation vector within an assigned state.每一个观测向量的状态，数组。各部分组成的索引。mix确保观测向量在一个指派值内。

  */
CV_IMPL CvImgObsInfo*
cvCreateObsInfo( CvSize num_obs, int obs_size )
{
    CvImgObsInfo *obs_info = 0;
    
    CV_FUNCNAME( "cvCreateObsInfo" );

    __BEGIN__;

    IPPI_CALL( icvCreateObsInfo( &obs_info, num_obs, obs_size ));

    __END__;

    return obs_info;
}

// 释放观测结构的函数
CV_IMPL void
cvReleaseObsInfo( CvImgObsInfo ** obs_info )
{
    CV_FUNCNAME( "cvReleaseObsInfo" );

    __BEGIN__;

    IPPI_CALL( icvReleaseObsInfo( obs_info ));

    __END__;
}

/* 对HMM的状态块进行统一的处理obsInfo::Observations structure.hmm::HMM 
structureHMM的观测结构。这个函数统一由see __Initial Segmentation__ for 2D 
Embedded HMM for 2D embedded HMM with 5 superstates and 3, 6, 6, 6, 3 internal 
states of every corresponding superstate就是5个超状态每一个超状态中有3，6，6，6，
3个内部状态。*/
CV_IMPL void
cvUniformImgSegm( CvImgObsInfo * obs_info, CvEHMM * hmm )
{
    CV_FUNCNAME( "cvUniformImgSegm" );

    __BEGIN__;

    IPPI_CALL( icvUniformImgSegm( obs_info, hmm ));
    __CLEANUP__;
    __END__;
}

// 每一个内部状态的混合组成部分。The function cvInitMixSegm takes a group of observations from several training images already segmented by states and splits a set of observation vectors within every internal HMM state into as many clusters as the number of mixture components in the state.函数cvInitMixSegm把训练图像中的观测序列内部的HMM分为混合状态中的一clusters。
CV_IMPL void
cvInitMixSegm( CvImgObsInfo ** obs_info_array, int num_img, CvEHMM * hmm )
{
    CV_FUNCNAME( "cvInitMixSegm" );

    __BEGIN__;

    IPPI_CALL( icvInitMixSegm( obs_info_array, num_img, hmm ));

    __END__;
}

// CvImgObsInfo** obsInfoArray（指向观测数组的指针）, int numImg（数组的长度）, CvEHMM* hmm );估计每一个HMM的参数。这个函数计算所有内部状态的参数。
CV_IMPL void
cvEstimateHMMStateParams( CvImgObsInfo ** obs_info_array, int num_img, CvEHMM * hmm )
{
    CV_FUNCNAME( "cvEstimateHMMStateParams" );

    __BEGIN__;

    IPPI_CALL( icvEstimateHMMStateParams( obs_info_array, num_img, hmm ));

    __END__;
}

// 估计转移概率
CV_IMPL void
cvEstimateTransProb( CvImgObsInfo ** obs_info_array, int num_img, CvEHMM * hmm )
{
    CV_FUNCNAME( "cvEstimateTransProb" );

    __BEGIN__;

    IPPI_CALL( icvEstimateTransProb( obs_info_array, num_img, hmm ));

    __END__;

}

// 计算在内部HMM中观测向量中发生的可能性computes Gaussian probabilities of each observation to occur in each of the internal HMM states。

CV_IMPL void
cvEstimateObsProb( CvImgObsInfo * obs_info, CvEHMM * hmm )
{
    CV_FUNCNAME( "cvEstimateObsProb" );

    __BEGIN__;

    IPPI_CALL( icvEstimateObsProb( obs_info, hmm ));

    __END__;
}

// executes Viterbi algorithm for embedded HMM. Viterbi algorithm evaluates the likelihood of the best match between the given image observations and the given HMM and performs segmentation of image observations by HMM states. The segmentation is done on the basis of the match found 相似匹配，给定的观测向量或者HMM作用于有HMM状态的图像观测向量分块。

CV_IMPL float
cvEViterbi( CvImgObsInfo * obs_info, CvEHMM * hmm )
{
    float result = FLT_MAX;

    CV_FUNCNAME( "cvEViterbi" );

    __BEGIN__;

    if( (obs_info == NULL) || (hmm == NULL) )
        CV_ERROR( CV_BadDataPtr, "Null pointer." );

    result = icvEViterbi( obs_info, hmm );
    
    __END__;
    
    return result;
}

// segments observations from all training images by mixture components of newly Viterbi algorithm-assigned states. The function uses Euclidean distance to group vectors around the existing mixtures centers. 
CV_IMPL void
cvMixSegmL2( CvImgObsInfo ** obs_info_array, int num_img, CvEHMM * hmm )
{
    CV_FUNCNAME( "cvMixSegmL2" );

    __BEGIN__;

    IPPI_CALL( icvMixSegmL2( obs_info_array, num_img, hmm ));

    __END__;
}

/* End of file */

