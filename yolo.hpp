#ifndef YOLO_HPP
#define YOLO_HPP

#include <vector>
#include "net.h"
#include <opencv2/opencv.hpp>

struct Object {
	    float x, y, w, h;
	        float prob;
		    int label;
		    cv::Rect_<float> rect;
};


float intersection_area(const Object& a, const Object& b);
void qsort_descent_inplace(std::vector<Object>& faceobjects, int left, int right);
void qsort_descent_inplace(std::vector<Object>& faceobjects);
void nms_sorted_bboxes(const std::vector<Object>& faceobjects, std::vector<int>& picked, float nms_threshold, bool agnostic = false);
float sigmoid(float x);
void generate_proposals(const ncnn::Mat& anchors, int stride, const ncnn::Mat& in_pad, const ncnn::Mat& feat_blob, float prob_threshold, std::vector<Object>& objects);
int detect_yolov5_for_person(const cv::Mat& bgr, bool& person_detected);

#endif 
