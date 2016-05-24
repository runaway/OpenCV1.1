// eigenface.c, by Robin Hewitt, 2007
// edited by Xin QIao, AUG 2007
//
// Example program showing how to implement eigenface with OpenCV

// Usage:
//
// First, you need some face images. I used the ORL face database.
// You can download it for free at
//    www.cl.cam.ac.uk/research/dtg/attarchive/facedatabase.html
//
// List the training and test face images you want to use in the
// input files train.txt and test.txt. (Example input files are provided
// in the download.) To use these input files exactly as provided, unzip
// the ORL face database, and place train.txt, test.txt, and eigenface.exe
// at the root of the unzipped database.
//
// To run the learning phase of eigenface, enter
//    eigenface train
// at the command prompt. To run the recognition phase, enter
//    eigenface test


#include <stdio.h>
#include <string.h>
#include <math.h> 
#include "cv.h"
#include "cvaux.h"
#include "highgui.h"


//// Global variables
IplImage ** faceImgArr        = 0; // array of face images
CvMat    *  personNumTruthMat = 0; // array of person numbers
int nTrainFaces              = 0; // the number of training images
int nEigens                  = 0; // the number of eigenvalues
IplImage * g_pAvgTrainImg      = 0; // the average image
IplImage ** eigenVectArr      = 0; // eigenvectors
CvMat * eigenValMat          = 0; // eigenvalues
CvMat * projectedTrainFaceMat = 0; // projected training faces


//// Function prototypes
void learn();
void recognize();
void doPCA();
void storeTrainingData();
int  loadTrainingData(CvMat ** pTrainPersonNumMat);
double * findNearestNeighbor(float * projectedTestFace);
int  loadFaceImgArray(char * filename);
void printUsage();


//////////////////////////////////
// main()
//
int main(int argc, char** argv)
{
    // validate that an input was specified
    if (argc != 2)
    {
        printUsage();
        return -1;
    }

    if (!strcmp(argv[1], "train"))
    {
        // 训练
        learn();
    }
    else if (!strcmp(argv[1], "test"))
    {
        // 识别
        recognize();
    }
    else
    {
        printf("Unknown command: %s\n", argv[1]);
        printUsage();
    }
    
    return 0;
}


//////////////////////////////////
// learn()
//
void learn()
{
    int i, j, k, offset;
	FILE * TrainfaceFile = 0;

    // 装入训练数据
    // load training data
    nTrainFaces = loadFaceImgArray("train.txt");

    if (nTrainFaces < 2)
    {
        fprintf(stderr,
               "Need 2 or more training faces\n"
               "Input file contains only %d\n", nTrainFaces);
        return;
    } 

    // 对训练中的人脸进行主成分分析
    // do PCA on the training faces
    doPCA();

    // 把测试数据投射到PCA子空间
    // project the training images onto the PCA subspace
    projectedTrainFaceMat = cvCreateMat(nTrainFaces, nEigens, CV_32FC1);

    offset = projectedTrainFaceMat->step / sizeof(float);

    // 对每个人脸进行特征值分解
    for (i = 0; i < nTrainFaces; i++)
    {
        //int offset = i * nEigens;
        cvEigenDecomposite(*(faceImgArr + i), // 
                           nEigens,
                           eigenVectArr,
                           0, 
                           0,
                           g_pAvgTrainImg, // 输入参数
                           //projectedTrainFaceMat->data.fl + i*nEigens);
                           projectedTrainFaceMat->data.fl + i*offset);
    }

    // 将训练集人脸图片的PCA结果输出到TrainFace.txt
    // 存储投影训练人脸矩阵到TrainFace.txt
    // store the projectedTrainFaceMat as TrainFace.txt
    //FILE * TrainfaceFile = 0;
    if (TrainfaceFile = fopen("TrainFace.txt", "w"))
    {
        for (j = 0 ; j < nTrainFaces ; j++)
        {
              fprintf(TrainfaceFile,"%d    ", j); 

              for (k = 0; k < nEigens ; k++)
              {
                  fprintf(TrainfaceFile, " %d : %f ", k, (projectedTrainFaceMat->data.fl + j*offset)[k] );  
              } 
              
              fprintf(TrainfaceFile," -1 : ? \n");
        }
    } 

    // 把识别数据保存为xml文件
    // store the recognition data as an xml file
    storeTrainingData();
}


//////////////////////////////////
// recognize()
//
void recognize()
{
    int i, j, k;

    int truth, nearest;
    int * order = malloc(nTrainFaces * sizeof(int)); 
    double * temp = malloc(nTrainFaces * sizeof(double));
    double dSmallestDistance;
    int result;

    // 待测试人脸的数目
    int nTestFaces  = 0;        // the number of test images

    // 训练过程中的人数
    CvMat * trainPersonNumMat = 0;  // the person numbers during training
    float * projectedTestFace = 0;

	double * sim = NULL; 
	FILE * TestfaceFile; 

    // 装入测试图片文件名并获取人数
    // load test images and ground truth for person number
    nTestFaces = loadFaceImgArray("test.txt");
    printf("%d test faces loaded\n", nTestFaces);

    // 装入已保存的训练数据
    // load the saved training data
    if (!loadTrainingData(&trainPersonNumMat))
    {
        return;
    }

    // 给测试数据分配空间
    projectedTestFace = (float *)cvAlloc(nEigens * sizeof(float));

    // 按特征值数目分配空间
    sim = (double *)cvAlloc(nEigens * sizeof(double));

    // 根据待测试的人脸数目
    for (i = 0; i < nTestFaces; i++)
    {
        int iNearest = 0;

        //int star[nTrainFaces];
        // 把测试数据投射到PCA子空间,特征值分解
        // project the test image onto the PCA subspace
        cvEigenDecomposite(*(faceImgArr + i),
                           nEigens,
                           eigenVectArr,
                           0, 
                           0,
                           g_pAvgTrainImg, // input
                           projectedTestFace);

        // 测试人脸图片的PCA结果输出到TestFace.txt中
        // 存储测试人脸到TestFace.txt  
        // store the projectedTestFace as TestFace.txt
        // FILE * TestfaceFile = 0;
        if (TestfaceFile = fopen("TestFace.txt", "w"))
        {       
              fprintf(TestfaceFile,"%d    ", 0); 

              for (k = 0; k < nEigens ; k++)
              {
                  fprintf(TestfaceFile, " %d : %f ", k, projectedTestFace[k]);  
              }
              
              fprintf(TestfaceFile," -1 : ? \n");
        } 

        sim = findNearestNeighbor(projectedTestFace);
        truth    = personNumTruthMat->data.i;
        nearest  = trainPersonNumMat->data.i[iNearest];


        // sort

        order[0] = iNearest;


        for (j = 0; j <  nTrainFaces; j++)
        {
            temp[j] = sim[j]; 
        }
                    
        //for (k = 0; k <  nTrainFaces; k++)
        //{
            // 记录temp中最小元素的index 
            result = 0; 
            
            dSmallestDistance = DBL_MAX;
            
            // 找到temp中最小元素的index，记录入result
            for (j = 0; j <  nTrainFaces; j++)
            {                  
                if (temp[j] < dSmallestDistance)
                {
                    dSmallestDistance = temp[j];
                    result = j;
                }                
            }
                        
            //order = result;
            
            // 将temp中找到的最小元素置为无穷大 
            //temp[result] = DBL_MAX;
        //}      
        
        
        
        printf("%d test image : \n",i+1); 

        // 打印每张脸的相似度
        for (k = 0; k < nTrainFaces; k++)
        {
            printf("star : %d, simility : %f\n", k + 1, sim[k]);
        }

        //printf("nearest = %d, Truth = %d\n\n", nearest, truth);
        printf("dSmallestDistance = %f, result = %d\n\n", dSmallestDistance, result + 1);
        
    }
}


//////////////////////////////////
// loadTrainingData()
//
int loadTrainingData(CvMat ** pTrainPersonNumMat)
{
    CvFileStorage * fileStorage;
    int i;

    // 创建一个文件存储接口
    // create a file-storage interface
    fileStorage = cvOpenFileStorage( "facedata.xml", 0, CV_STORAGE_READ );

    if (!fileStorage)
    {
        fprintf(stderr, "Can't open facedata.xml\n");
        return 0;
    }

    nEigens = cvReadIntByName(fileStorage, 0, "nEigens", 0);
    nTrainFaces = cvReadIntByName(fileStorage, 0, "nTrainFaces", 0);
    *pTrainPersonNumMat = (CvMat *)cvReadByName(fileStorage, 0, "trainPersonNumMat", 0);
    eigenValMat  = (CvMat *)cvReadByName(fileStorage, 0, "eigenValMat", 0);
    projectedTrainFaceMat = (CvMat *)cvReadByName(fileStorage, 0, "projectedTrainFaceMat", 0);
    g_pAvgTrainImg = (IplImage *)cvReadByName(fileStorage, 0, "avgTrainImg", 0);
    eigenVectArr = (IplImage **)cvAlloc(nTrainFaces*sizeof(IplImage *));

    // 按特征值数目读取特征向量
    for (i = 0; i < nEigens; i++)
    {
        char varname[200];
        sprintf( varname, "eigenVect_%d", i );
        *(eigenVectArr + i) = (IplImage *)cvReadByName(fileStorage, 0, varname, 0);
    }

    // 释放文件存储接口
    // release the file-storage interface
    cvReleaseFileStorage(&fileStorage);

    return 1;
}


//////////////////////////////////
// storeTrainingData()
//
void storeTrainingData()
{
    CvFileStorage * fileStorage;
    int i;

    // 创建一个文件存储接口
    // create a file-storage interface
    //fileStorage = cvOpenFileStorage( "./data/facedata.xml", 0, CV_STORAGE_WRITE );
	fileStorage = cvOpenFileStorage( "facedata.xml", 0, CV_STORAGE_WRITE );


    // 存储所有数据
    // store all the data
    cvWriteInt( fileStorage, "nEigens", nEigens );
    cvWriteInt( fileStorage, "nTrainFaces", nTrainFaces );
    cvWrite(fileStorage, "trainPersonNumMat", personNumTruthMat, cvAttrList(0,0));
    cvWrite(fileStorage, "eigenValMat", eigenValMat, cvAttrList(0,0));
    cvWrite(fileStorage, "projectedTrainFaceMat", projectedTrainFaceMat, cvAttrList(0,0));
    cvWrite(fileStorage, "avgTrainImg", g_pAvgTrainImg, cvAttrList(0,0));

    // 按特征值数目写特征向量
    for (i = 0; i < nEigens; i++)
    {
        char varname[200];
        sprintf( varname, "eigenVect_%d", i );
        cvWrite(fileStorage, varname, *eigenVectArr, cvAttrList(0,0));
    }

    // 释放文件存储接口
    // release the file-storage interface
    cvReleaseFileStorage( &fileStorage );
}

/*******************************************************************************
*   Func Name: findNearestNeighbor                                                
* Description: 的函数
*       Input:                                   
*      Output:                                               
*      Return:                                      
*     Caution:
*-------------------------------------------------------------------------------
* Modification History
*      1.Date: 2009-05-07
*      Author: Runaway
* Description:                                           
*******************************************************************************/
//////////////////////////////////
// findNearestNeighbor()
//
double * findNearestNeighbor(float * projectedTestFace)
{
    //double leastDistSq = 1e12;
    double leastDistSq = DBL_MAX;
    int i, iTrain, iNearest = 0;
    double * sim = (double *)malloc(nTrainFaces * sizeof(double)); 

    // 按已训练的人脸的数目
    for (iTrain = 0; iTrain < nTrainFaces; iTrain++)
    {
        double distSq = 0;
        long double distSx = 0;
        long double distSy = 0;

        for (i = 0; i < nEigens; i++)
        {
              
            float d_i = *projectedTestFace -
                projectedTrainFaceMat->data.fl[iTrain * nEigens + i];
            //distSq += d_i*d_i / eigenValMat->data.fl;  // Mahalanobis

            // 计算欧几里德距离
            distSq += d_i * d_i; // Euclidean

            // 计算余弦距离
            /*
            /////////////////////////////////////////////////////////////////  
            double d_i =
                projectedTestFace *
                projectedTrainFaceMat->data.fl[iTrain*nEigens + i];
            distSq +=  d_i;
            double d_x = 
                projectedTestFace * projectedTestFace;
            double d_y = 
                projectedTrainFaceMat->data.fl[iTrain*nEigens + i]
                * projectedTrainFaceMat->data.fl[iTrain*nEigens + i];
            distSx += d_x;
            distSy += d_y;
            
            /////////////////////////////////////////////////////////////////
            */
        }
        
        
        // 输出的距离，可以作为初步的度量，看看效果如何，作为以后提高的baseline
        sim[iTrain] = distSq; 
        
        /*
            ///////////////////////////////////////////////////////////////// 
            
        sim[iTrain] = fabs ( (double) (distSq / ( sqrt(distSx) *  sqrt(distSy) )));
        
        /////////////////////////////////////////////////////////////////
        */
        
        //if(distSq < leastDistSq)
        //{
        //    leastDistSq = distSq;
        //    iNearest = iTrain;
        //}
    }

    return sim;
}

/* 对于人脸数据，采用了主成分分析（ Principal Component Analysis ， PCA ）方法来
做数据降维。那么，什么是PCA呢？
主成分分析 （ Principal Component Analysis ， PCA ）是一种掌握事物主要矛盾的统计
分析方法，它可以从多元事物中解析出主要影响因素，揭示事物的本质，简化复杂的问题。
计算主成分的目的是将高维数据投影到较低维空间。给定 n 个变量的 m 个观察值，形成一
个 n ′ m 的数据矩阵， n 通常比较大。对于一个由多个变量描述的复杂事物，人们难以认
识，那么是否可以抓住事物主要方面进行重点分析呢？如果事物的主要方面刚好体现在几个
主要变量上，我们只需要将这几个变量分离出来，进行详细分析。但是，在一般情况下，并
不能直接找出这样的关键变量。这时我们可以用原有变量的线性组合来表示事物的主要方面， 
PCA 就是这样一种分析方法。 
PCA 的目标是寻找 r （ r<n ）个新变量，使它们反映事物的主要特征，压缩原有数据矩阵
的规模。每个新变量是原有变量的线性组合，体现原有变量的综合效果，具有一定的实际含
义。这 r 个新变量称为“主成分”，它们可以在很大程度上反映原来 n 个变量的影响，并
且这些新变量是互不相关的，也是正交的。通过主成分分析，压缩数据空间，将多元数据的
特征在低维空间里直观地表示出来。
Opencv 中，事先预置了PCA方法的函数，我们去调用即可。在下面给出的程序中，将训练集
人脸图片的PCA结果输出到TrainFace.txt，测试人脸图片的PCA结果输出到TestFace.txt中，
以备以后调用别的学习算法（如SVM等）使用。在程序中，直接使用欧式距离或余弦距离（需
要在代码中手动更改编译）度量测试人脸和各训练人脸的相似度并输出，以供作简单的观察。*/
/*******************************************************************************
*   Func Name: doPCA                                                
* Description: 主成分分析函数
*       Input:                                   
*      Output:                                               
*      Return:                                      
*     Caution:
*-------------------------------------------------------------------------------
* Modification History
*      1.Date: 2009-05-07
*      Author: Runaway
* Description:                                           
*******************************************************************************/
void doPCA()
{
    int i;
    CvTermCriteria calcLimit;
    CvSize faceImgSize;

    // 设置要使用的特征值数目
    // set the number of eigenvalues to use
    nEigens = nTrainFaces - 1;

    // 分配特征向量图片空间
    // allocate the eigenvector images
    faceImgSize.width  = faceImgArr[0]->width;
    faceImgSize.height = faceImgArr[0]->height;
    eigenVectArr = (IplImage**)cvAlloc(sizeof(IplImage*) * nEigens);

    // 按特征值数目来创建图像
    for (i = 0; i < nEigens; i++)
    {
        *(eigenVectArr + i) = cvCreateImage(faceImgSize, IPL_DEPTH_32F, 1);
    }

    // 分配特征值数组
    // allocate the eigenvalue array
    eigenValMat = cvCreateMat(1, nEigens, CV_32FC1);

    // 分配均值图像
    // allocate the averaged image
    g_pAvgTrainImg = cvCreateImage(faceImgSize, IPL_DEPTH_32F, 1);

    // 设置PCA决断准则
    // set the PCA termination criterion
    calcLimit = cvTermCriteria(CV_TERMCRIT_ITER, nEigens, 1);

    // 计算均值图像，特征值，特征向量
    // compute average image, eigenvalues, and eigenvectors
    cvCalcEigenObjects(nTrainFaces,
                       (void*)faceImgArr,
                       (void*)eigenVectArr,
                       CV_EIGOBJ_NO_CALLBACK,
                       0,
                       0,
                       &calcLimit,
                       g_pAvgTrainImg,
                       eigenValMat->data.fl);

    // 将数组规范化到一个确定的规范或数据范围
    cvNormalize(eigenValMat, eigenValMat, 1, 0, CV_L1, 0);
}


//////////////////////////////////
// loadFaceImgArray()
//
int loadFaceImgArray(char * filename)
{
    FILE * imgListFile = 0;
    char imgFilename[512];
    int iFace, nFaces=0;

	IplImage* dst; 

    // 打开输入文件
    // open the input file
    if (!(imgListFile = fopen(filename, "r")))
    {
        fprintf(stderr, "Can\'t open file %s\n", filename);
        return 0;
    }

    // 从列表文件中获取文件名
    // count the number of faces
    while (fgets(imgFilename, 512, imgListFile))
    {
        // 人脸计数
        ++nFaces;
    }

    // 将指针移到文件开始
    rewind(imgListFile);

    // 分配人脸图象数组和人数矩阵
    // allocate the face-image array and person number matrix
    faceImgArr = (IplImage **)cvAlloc(nFaces*sizeof(IplImage *));
    personNumTruthMat = cvCreateMat(1, nFaces, CV_32SC1);

    // 存储人脸图像到数组
    // store the face images in an array
    for (iFace = 0; iFace < nFaces; iFace++)
    {
                              
        /*
        // read person number and name of image file
        fscanf(imgListFile,
            "%d %s", personNumTruthMat->data.i+iFace, imgFilename);
        */
        
        fscanf(imgListFile, "%s", imgFilename);
            
        ///////////////////////////////////////////////////////////////////// 
        // 装入人脸图像
        // load the face image
        faceImgArr[iFace] = cvLoadImage(imgFilename, CV_LOAD_IMAGE_ANYCOLOR);
        
        dst = cvCreateImage(cvSize(92,112), 
                  faceImgArr[iFace]->depth, faceImgArr[iFace]->nChannels);
        cvResize(faceImgArr[iFace], dst, CV_INTER_LINEAR);
        //strcat(FileName,".pgm");
        //faceImgArr[iFace] = dst; 
        cvSaveImage(imgFilename, dst);
        
        cvReleaseImage(&dst); 
        
        faceImgArr[iFace] = cvLoadImage(imgFilename, CV_LOAD_IMAGE_GRAYSCALE);

        ///////////////////////////////////////////////////////////////////// 
        if (!faceImgArr[iFace])
        {
            fprintf(stderr, "Can\'t load image from %s\n", imgFilename);
            return 0;
        }
    }

    fclose(imgListFile);

    return nFaces;
}


//////////////////////////////////
// printUsage()
//
void printUsage()
{
    printf("Usage: eigenface <command>\n",
           "  Valid commands are\n"
           "    train\n"
           "    test\n");
}

