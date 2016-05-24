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
        // ѵ��
        learn();
    }
    else if (!strcmp(argv[1], "test"))
    {
        // ʶ��
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

    // װ��ѵ������
    // load training data
    nTrainFaces = loadFaceImgArray("train.txt");

    if (nTrainFaces < 2)
    {
        fprintf(stderr,
               "Need 2 or more training faces\n"
               "Input file contains only %d\n", nTrainFaces);
        return;
    } 

    // ��ѵ���е������������ɷַ���
    // do PCA on the training faces
    doPCA();

    // �Ѳ�������Ͷ�䵽PCA�ӿռ�
    // project the training images onto the PCA subspace
    projectedTrainFaceMat = cvCreateMat(nTrainFaces, nEigens, CV_32FC1);

    offset = projectedTrainFaceMat->step / sizeof(float);

    // ��ÿ��������������ֵ�ֽ�
    for (i = 0; i < nTrainFaces; i++)
    {
        //int offset = i * nEigens;
        cvEigenDecomposite(*(faceImgArr + i), // 
                           nEigens,
                           eigenVectArr,
                           0, 
                           0,
                           g_pAvgTrainImg, // �������
                           //projectedTrainFaceMat->data.fl + i*nEigens);
                           projectedTrainFaceMat->data.fl + i*offset);
    }

    // ��ѵ��������ͼƬ��PCA��������TrainFace.txt
    // �洢ͶӰѵ����������TrainFace.txt
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

    // ��ʶ�����ݱ���Ϊxml�ļ�
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

    // ��������������Ŀ
    int nTestFaces  = 0;        // the number of test images

    // ѵ�������е�����
    CvMat * trainPersonNumMat = 0;  // the person numbers during training
    float * projectedTestFace = 0;

	double * sim = NULL; 
	FILE * TestfaceFile; 

    // װ�����ͼƬ�ļ�������ȡ����
    // load test images and ground truth for person number
    nTestFaces = loadFaceImgArray("test.txt");
    printf("%d test faces loaded\n", nTestFaces);

    // װ���ѱ����ѵ������
    // load the saved training data
    if (!loadTrainingData(&trainPersonNumMat))
    {
        return;
    }

    // ���������ݷ���ռ�
    projectedTestFace = (float *)cvAlloc(nEigens * sizeof(float));

    // ������ֵ��Ŀ����ռ�
    sim = (double *)cvAlloc(nEigens * sizeof(double));

    // ���ݴ����Ե�������Ŀ
    for (i = 0; i < nTestFaces; i++)
    {
        int iNearest = 0;

        //int star[nTrainFaces];
        // �Ѳ�������Ͷ�䵽PCA�ӿռ�,����ֵ�ֽ�
        // project the test image onto the PCA subspace
        cvEigenDecomposite(*(faceImgArr + i),
                           nEigens,
                           eigenVectArr,
                           0, 
                           0,
                           g_pAvgTrainImg, // input
                           projectedTestFace);

        // ��������ͼƬ��PCA��������TestFace.txt��
        // �洢����������TestFace.txt  
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
            // ��¼temp����СԪ�ص�index 
            result = 0; 
            
            dSmallestDistance = DBL_MAX;
            
            // �ҵ�temp����СԪ�ص�index����¼��result
            for (j = 0; j <  nTrainFaces; j++)
            {                  
                if (temp[j] < dSmallestDistance)
                {
                    dSmallestDistance = temp[j];
                    result = j;
                }                
            }
                        
            //order = result;
            
            // ��temp���ҵ�����СԪ����Ϊ����� 
            //temp[result] = DBL_MAX;
        //}      
        
        
        
        printf("%d test image : \n",i+1); 

        // ��ӡÿ���������ƶ�
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

    // ����һ���ļ��洢�ӿ�
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

    // ������ֵ��Ŀ��ȡ��������
    for (i = 0; i < nEigens; i++)
    {
        char varname[200];
        sprintf( varname, "eigenVect_%d", i );
        *(eigenVectArr + i) = (IplImage *)cvReadByName(fileStorage, 0, varname, 0);
    }

    // �ͷ��ļ��洢�ӿ�
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

    // ����һ���ļ��洢�ӿ�
    // create a file-storage interface
    //fileStorage = cvOpenFileStorage( "./data/facedata.xml", 0, CV_STORAGE_WRITE );
	fileStorage = cvOpenFileStorage( "facedata.xml", 0, CV_STORAGE_WRITE );


    // �洢��������
    // store all the data
    cvWriteInt( fileStorage, "nEigens", nEigens );
    cvWriteInt( fileStorage, "nTrainFaces", nTrainFaces );
    cvWrite(fileStorage, "trainPersonNumMat", personNumTruthMat, cvAttrList(0,0));
    cvWrite(fileStorage, "eigenValMat", eigenValMat, cvAttrList(0,0));
    cvWrite(fileStorage, "projectedTrainFaceMat", projectedTrainFaceMat, cvAttrList(0,0));
    cvWrite(fileStorage, "avgTrainImg", g_pAvgTrainImg, cvAttrList(0,0));

    // ������ֵ��Ŀд��������
    for (i = 0; i < nEigens; i++)
    {
        char varname[200];
        sprintf( varname, "eigenVect_%d", i );
        cvWrite(fileStorage, varname, *eigenVectArr, cvAttrList(0,0));
    }

    // �ͷ��ļ��洢�ӿ�
    // release the file-storage interface
    cvReleaseFileStorage( &fileStorage );
}

/*******************************************************************************
*   Func Name: findNearestNeighbor                                                
* Description: �ĺ���
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

    // ����ѵ������������Ŀ
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

            // ����ŷ����¾���
            distSq += d_i * d_i; // Euclidean

            // �������Ҿ���
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
        
        
        // ����ľ��룬������Ϊ�����Ķ���������Ч����Σ���Ϊ�Ժ���ߵ�baseline
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

/* �����������ݣ����������ɷַ����� Principal Component Analysis �� PCA ��������
�����ݽ�ά����ô��ʲô��PCA�أ�
���ɷַ��� �� Principal Component Analysis �� PCA ����һ������������Ҫì�ܵ�ͳ��
���������������ԴӶ�Ԫ�����н�������ҪӰ�����أ���ʾ����ı��ʣ��򻯸��ӵ����⡣
�������ɷֵ�Ŀ���ǽ���ά����ͶӰ���ϵ�ά�ռ䡣���� n �������� m ���۲�ֵ���γ�һ
�� n �� m �����ݾ��� n ͨ���Ƚϴ󡣶���һ���ɶ�����������ĸ����������������
ʶ����ô�Ƿ����ץס������Ҫ��������ص�����أ�����������Ҫ����պ������ڼ���
��Ҫ�����ϣ�����ֻ��Ҫ���⼸���������������������ϸ���������ǣ���һ������£���
����ֱ���ҳ������Ĺؼ���������ʱ���ǿ�����ԭ�б����������������ʾ�������Ҫ���棬 
PCA ��������һ�ַ��������� 
PCA ��Ŀ����Ѱ�� r �� r<n �����±�����ʹ���Ƿ�ӳ�������Ҫ������ѹ��ԭ�����ݾ���
�Ĺ�ģ��ÿ���±�����ԭ�б�����������ϣ�����ԭ�б������ۺ�Ч��������һ����ʵ�ʺ�
�塣�� r ���±�����Ϊ�����ɷ֡������ǿ����ںܴ�̶��Ϸ�ӳԭ�� n ��������Ӱ�죬��
����Щ�±����ǻ�����صģ�Ҳ�������ġ�ͨ�����ɷַ�����ѹ�����ݿռ䣬����Ԫ���ݵ�
�����ڵ�ά�ռ���ֱ�۵ر�ʾ������
Opencv �У�����Ԥ����PCA�����ĺ���������ȥ���ü��ɡ�����������ĳ����У���ѵ����
����ͼƬ��PCA��������TrainFace.txt����������ͼƬ��PCA��������TestFace.txt�У�
�Ա��Ժ���ñ��ѧϰ�㷨����SVM�ȣ�ʹ�á��ڳ����У�ֱ��ʹ��ŷʽ��������Ҿ��루��
Ҫ�ڴ������ֶ����ı��룩�������������͸�ѵ�����������ƶȲ�������Թ����򵥵Ĺ۲졣*/
/*******************************************************************************
*   Func Name: doPCA                                                
* Description: ���ɷַ�������
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

    // ����Ҫʹ�õ�����ֵ��Ŀ
    // set the number of eigenvalues to use
    nEigens = nTrainFaces - 1;

    // ������������ͼƬ�ռ�
    // allocate the eigenvector images
    faceImgSize.width  = faceImgArr[0]->width;
    faceImgSize.height = faceImgArr[0]->height;
    eigenVectArr = (IplImage**)cvAlloc(sizeof(IplImage*) * nEigens);

    // ������ֵ��Ŀ������ͼ��
    for (i = 0; i < nEigens; i++)
    {
        *(eigenVectArr + i) = cvCreateImage(faceImgSize, IPL_DEPTH_32F, 1);
    }

    // ��������ֵ����
    // allocate the eigenvalue array
    eigenValMat = cvCreateMat(1, nEigens, CV_32FC1);

    // �����ֵͼ��
    // allocate the averaged image
    g_pAvgTrainImg = cvCreateImage(faceImgSize, IPL_DEPTH_32F, 1);

    // ����PCA����׼��
    // set the PCA termination criterion
    calcLimit = cvTermCriteria(CV_TERMCRIT_ITER, nEigens, 1);

    // �����ֵͼ������ֵ����������
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

    // ������淶����һ��ȷ���Ĺ淶�����ݷ�Χ
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

    // �������ļ�
    // open the input file
    if (!(imgListFile = fopen(filename, "r")))
    {
        fprintf(stderr, "Can\'t open file %s\n", filename);
        return 0;
    }

    // ���б��ļ��л�ȡ�ļ���
    // count the number of faces
    while (fgets(imgFilename, 512, imgListFile))
    {
        // ��������
        ++nFaces;
    }

    // ��ָ���Ƶ��ļ���ʼ
    rewind(imgListFile);

    // ��������ͼ���������������
    // allocate the face-image array and person number matrix
    faceImgArr = (IplImage **)cvAlloc(nFaces*sizeof(IplImage *));
    personNumTruthMat = cvCreateMat(1, nFaces, CV_32SC1);

    // �洢����ͼ������
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
        // װ������ͼ��
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

