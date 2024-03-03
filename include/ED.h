/**************************************************************************************************************
* Edge Drawing (ED) and Edge Drawing Parameter Free (EDPF) source codes.
* Copyright (C) Cihan Topal & Cuneyt Akinlar 
* E-mails of the authors:  cihantopal@gmail.com, cuneytakinlar@gmail.com
*
* Please cite the following papers if you use Edge Drawing library:
*
* [1] C. Topal and C. Akinlar, “Edge Drawing: A Combined Real-Time Edge and Segment Detector,”
*     Journal of Visual Communication and Image Representation, 23(6), 862-872, DOI: 10.1016/j.jvcir.2012.05.004 (2012).
*
* [2] C. Akinlar and C. Topal, “EDPF: A Real-time Parameter-free Edge Segment Detector with a False Detection Control,”
*     International Journal of Pattern Recognition and Artificial Intelligence, 26(1), DOI: 10.1142/S0218001412550026 (2012).
**************************************************************************************************************/

#ifndef _ED_
#define _ED_

#include <opencv2/opencv.hpp>
#include "EDColor.h"

/// Special defines
#define EDGE_VERTICAL   1
#define EDGE_HORIZONTAL 2

#define ANCHOR_PIXEL  254
#define EDGE_PIXEL    255

#define LEFT  1
#define RIGHT 2
#define UP    3
#define DOWN  4

enum GradientOperator { PREWITT_OPERATOR = 101, SOBEL_OPERATOR = 102, SCHARR_OPERATOR = 103, LSD_OPERATOR = 104 };

struct StackNode {
	int r, c;   // starting pixel
	int parent; // parent chain (-1 if no parent)
	int dir;    // direction where you are supposed to go
};

// Used during Edge Linking
struct Chain {

	int dir;                   // Direction of the chain
	int len;                   // # of pixels in the chain
	int parent;                // Parent of this node (-1 if no parent)
	int children[2];           // Children of this node (-1 if no children)
	cv::Point *pixels;         // Pointer to the beginning of the pixels array
	bool isAnchor;
};

class ED {
							
public:
	ED(cv::Mat _srcImage, GradientOperator _op = PREWITT_OPERATOR, int _gradThresh = 20, int _anchorThresh = 0, int _scanInterval = 1, int _minPathLen = 10, double _sigma = 1.0, bool _sumFlag = true);
	ED(cv::Mat _srcImage ,const std::vector<cv::KeyPoint>& keypoints, GradientOperator _op = PREWITT_OPERATOR, int _gradThresh = 20, int _anchorThresh = 0, int _scanInterval = 1, int _minPathLen = 10, double _sigma = 1.0, bool _sumFlag = true);
	ED(const ED &cpyObj); 
	ED(short* gradImg, uchar *dirImg, int _width, int _height, int _gradThresh, int _anchorThresh, int _scanInterval = 1, int _minPathLen = 10, bool selectStableAnchors = true);
	ED(EDColor &cpyObj);
	ED();

	cv::Mat getEdgeImage();
	cv::Mat getAnchorImage();
	cv::Mat getSmoothImage();
	cv::Mat getGradImage();
	
	int getSegmentNo();
	int getAnchorNo();
	
	std::vector<cv::Point> getAnchorPoints();
	std::vector<std::vector<cv::Point>> getSegments();
	std::vector<std::vector<cv::Point>> getSortedSegments();
	
	void Keypoint2Points(const std::vector<cv::KeyPoint>& keypoints) {
       
        for (const auto& kp : keypoints) {
			edgeImg[static_cast<int>(kp.pt.y)*width + static_cast<int>(kp.pt.x)] = ANCHOR_PIXEL;
            anchorPoints.push_back(cv::Point(static_cast<int>(kp.pt.x), static_cast<int>(kp.pt.y)));
        }
		anchorNos = (int)anchorPoints.size(); // get the total number of anchor points
    }

	cv::Mat drawParticularSegments(std::vector<int> list);

	bool isPointAnAnchor(const cv::Point& pt) {
		for (const auto& anchor : anchorPoints) {
			if (pt == anchor) return true;
		}
		return false;
	}

	cv::Mat equalizeHist16SC1(const cv::Mat& src) {
    // 首先，检查输入图像是否为CV_16SC1类型
    if (src.type() != CV_16SC1) {
        throw std::runtime_error("Input type must be CV_16SC1");
    }

    // 将图像转换为CV_16UC1以避免负值和数据溢出
    cv::Mat temp;
    src.convertTo(temp, CV_16U);

    // 将16位图像缩放到8位范围以应用equalizeHist
    double minVal, maxVal;
    cv::minMaxLoc(temp, &minVal, &maxVal);
    temp.convertTo(temp, CV_8U, 255.0 / (maxVal - minVal), -minVal * 255.0 / (maxVal - minVal));

    // 应用直方图均衡化
    cv::Mat equalized;
    cv::equalizeHist(temp, equalized);

    // 如果需要，可以将处理后的图像转换回CV_16SC1
    equalized.convertTo(equalized, CV_16S, (maxVal - minVal) / 255.0, minVal);

    return equalized;
	}
	
protected:
	int width; // width of source image
	int height; // height of source image
	uchar *srcImg; 
	std::vector<std::vector< cv::Point> > segmentPoints;
	double sigma; // Gaussian sigma
	cv::Mat smoothImage;
	uchar *edgeImg; // pointer to edge image data
	uchar *smoothImg; // pointer to smoothed image data
	int segmentNos;
	int minPathLen;
	cv::Mat srcImage;

private:
	void ComputeGradient();
	void ComputeAnchorPoints();
	void JoinAnchorPointsUsingSortedAnchors(); 
	void sortAnchorsByGradValue();
	int* sortAnchorsByGradValue1();

	static bool AdjustTreeAndCheckParent(Chain* chains, int root, int parent);
	static int LongestChain(Chain *chains, int root);
	static int RetrieveChainNos(Chain *chains, int root, int chainNos[]);

	int anchorNos;
	std::vector<cv::Point> anchorPoints;
	std::vector<cv::Point> edgePoints;

	cv::Mat edgeImage;
	cv::Mat gradImage;

	uchar *dirImg; // pointer to direction image data
	short *gradImg; // pointer to gradient image data

	GradientOperator op; // operation used in gradient calculation
	int gradThresh; // gradient threshold
	int anchorThresh; // anchor point threshold
	int scanInterval;
	bool sumFlag;
};


#endif
