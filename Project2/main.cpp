#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

#define M_PI 3.14159265

cv::Mat img = cv::imread("C:\\Users\\sangsu lee\\Desktop\\1_1274(iou_0.00).bmp", 0);

// distance mode only-----------------------------------------
double THRESHOLD = 0.8; 
//------------------------------------------------------------

// angle mode only -------------------------------------------
double THRESHOLD_ANGLE = 0.1;//30 / (180.0 / M_PI); //0.40; 
double DELETE_MAX_DISTANCE = 30; // angle mode only
double FORCE_DELETE_DISTANCE = 4.0;
//------------------------------------------------------------

bool DEBUG_MODE = false;
int LOOP = 4;

double ang_max = -9999;
double ang_min = 9999;

double get_angle(cv::Point a, cv::Point b, cv::Point c, bool isDbug = true) {
	double result = atan2(c.y - a.y, c.x - a.x) - atan2(b.y - a.y, b.x - a.x);
	if (result < 0){
		/*result = result  + 2.0 * M_PI;
		cout << result << endl;*/
	}
	if (isDbug) cout << "angle3: " << abs(result) << endl;
	if (isDbug) cout << "angle3 to degree: " << abs(result) * 180 / M_PI << endl;
	return abs(result);
}

double distance_point(cv::Point pt1, cv::Point pt2) {
	return pow(pow(pt1.x - pt2.x, 2) + pow(pt1.y - pt2.y, 2), 0.5);
}

void ReduceContourPoint(std::vector<cv::Point>& vtContour, double fDistThresh, int loop = 4, bool isDbug = true)
{
	for (int i = 0; i < loop; ++i) {
		std::vector<cv::Point> remove_pt;
		int count = 0;
		for (std::vector<cv::Point>::iterator it = std::begin(vtContour); it != std::end(vtContour) - 1; /*++it,*/ ++count)
		{
			cv::Point pt1, pt2, pt3;

			if ((it + 2) == vtContour.end())
			{
				pt1 = *it;
				pt2 = *(it + 1);
				pt3 = *(vtContour.begin());
			}
			else if ((it + 1) == vtContour.end())
			{
				pt1 = *it;
				pt2 = *(vtContour.begin());
				pt3 = *(vtContour.begin() + 1);
			}
			else
			{
				pt1 = *it;
				pt2 = *(it + 1);
				pt3 = *(it + 2);
			}

			if (isDbug) cout << pt1 << endl;
			if (isDbug) cout << pt2 << endl;
			if (isDbug) cout << pt3 << endl;

			double pt2_angle = get_angle(pt1, pt2, pt3, DEBUG_MODE);
			double distance_pt1_pt2 = distance_point(pt1, pt2);
			if (isDbug) cout << "angle: " << pt2_angle << endl;
			if (isDbug) cout << "distance_pt1_pt3: " << distance_pt1_pt2 << endl;
			ang_max = max(ang_max, pt2_angle);
			ang_min = min(ang_min, pt2_angle);
			if (((pt2_angle < THRESHOLD_ANGLE) && (distance_pt1_pt2 < DELETE_MAX_DISTANCE)) || distance_pt1_pt2 < FORCE_DELETE_DISTANCE) {
				//컨투어벡터에서 pt2에 해당하는것 제거
				//remove_pt.push_back(pt2);
				//or 바로 제거
				vtContour.erase(std::remove(vtContour.begin(), vtContour.end(), pt2), vtContour.end());
			}
			else {
				it++;
			}
		}

		//컨투어벡터에서 pt2에 해당하는것 제거
		/*for (std::vector<cv::Point>::iterator it = std::begin(remove_pt); it != std::end(remove_pt); ++it)
		{
			vtContour.erase(std::remove(vtContour.begin(), vtContour.end(), *it), vtContour.end());
			if (isDbug) cout << "deleted:" << (*it) << endl;
		}*/
	}
	if (isDbug) cout << vtContour.size() << endl;
}

int main()
{
	cv::RNG rng(1332345);

	std::vector<std::vector<cv::Point>> contours;
	std::vector<std::vector<cv::Point>>& contours_ref = contours;
	std::vector<cv::Vec4i> hierarchy;
	vector<int> before_size, after_size;

	//findContours(img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_TC89_L1 /*cv::CHAIN_APPROX_TC89_KCOS*/ /*cv::CHAIN_APPROX_SIMPLE*/);
	findContours(img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_TC89_L1);

	cv::Mat padded;
	img.copyTo(padded);
	cv::cvtColor(padded, padded, CV_GRAY2RGB);

	for (size_t i = 0; i < contours.size(); i++)
	{
		cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
		drawContours(padded, contours, (int)i, color, 3, cv::LINE_8, hierarchy, 0);
		before_size.push_back((int)contours[i].size());

		for (int j = 0; j < contours[i].size(); ++j) {
			circle(padded, contours[i].at(j), 1, Scalar(251, 255, 0), -1);
		}
	}

	std::cout << "before" << contours[0].size() << std::endl;

	for (size_t i = 0; i < contours.size(); i++)
		ReduceContourPoint(contours[i], THRESHOLD, LOOP, DEBUG_MODE);

	std::cout << "after" << contours[0].size() << std::endl;

	imshow("before image", padded);
	waitKey(50);

	cv::Mat padded_after;
	img.copyTo(padded_after);
	cv::cvtColor(padded_after, padded_after, CV_GRAY2RGB);
	for (size_t i = 0; i < contours.size(); i++)
	{
		cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
		drawContours(padded_after, contours, (int)i, color, 3, cv::LINE_8, hierarchy, 0);
		after_size.push_back((int)contours[i].size());

		for (int j = 0; j < contours[i].size(); ++j) {
			circle(padded_after, contours[i].at(j), 1, Scalar(251, 255, 0), -1);
		}
	}
	imshow("after image", padded_after);
	waitKey(50);

	cout << "ang_min: " << ang_min << endl;
	cout << "ang_max: " << ang_max << endl;

	//std::cout << "threshold: " << THRESHOLD << "  loop: " << LOOP << std::endl;
	//std::cout << "THRESHOLD_ANGLE: " << THRESHOLD_ANGLE << " (" << THRESHOLD_ANGLE * (180.0 / M_PI) << "°)" << std::endl;
	std::cout << "LOOP: " << LOOP << std::endl;
	std::cout << "THRESHOLD_ANGLE: " << THRESHOLD_ANGLE << std::endl;
	std::cout << "THRESHOLD_DISTANCE: " << DELETE_MAX_DISTANCE << std::endl;
	std::cout << "FORCE_DELETE_DISTANCE: " << FORCE_DELETE_DISTANCE << std::endl;
	std::cout << "result" << std::endl;
	assert(after_size.size() == before_size.size());
	for (int i = 0; i < after_size.size(); ++i) {
		cout << "before : " << before_size[i] << "\tafter : " << after_size[i] << endl;
	}
	
	while (true) {
		waitKey(50);
	}

	return 0;
}

//reference
//https://cubelover.tistory.com/2
//https://qastack.kr/programming/1211212/how-to-calculate-an-angle-from-three-points