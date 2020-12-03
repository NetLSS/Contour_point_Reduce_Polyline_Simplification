#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>

using namespace cv;
using namespace std;

void ReduceContourPoint(std::vector<cv::Point>& vtContour, double fDistThresh, int loop = 1);


cv::Mat img = cv::imread("C:\\Users\\sangsu lee\\Desktop\\test.png", 0);
float THRESHOLD = 0.6;
int LOOP = 8;
void main()
{
	cv::RNG rng(1332345);

	std::vector<std::vector<cv::Point>> contours;
	std::vector<std::vector<cv::Point>>& contours_ref = contours;
	std::vector<cv::Vec4i> hierarchy;
	vector<int> before_size, after_size;
	findContours(img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);
	cv::Mat padded;
	img.copyTo(padded);
	cv::cvtColor(padded, padded, CV_GRAY2RGB);
	for (size_t i = 0; i < contours.size(); i++)
	{
		cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
		drawContours(padded, contours, (int)i, color, 3, cv::LINE_8, hierarchy, 0);
		before_size.push_back(contours[i].size());
	}
	std::cout << "before" << contours[0].size() << std::endl;

	for (size_t i = 0; i < contours.size(); i++)
		ReduceContourPoint(contours[i], THRESHOLD, LOOP);

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
		after_size.push_back(contours[i].size());
	}
	imshow("after image", padded_after);
	waitKey(50);

	std::cout << "threshold: " << THRESHOLD << "  loop: " << LOOP << std::endl;
	std::cout << "result" << std::endl;
	assert(after_size.size() == before_size.size());
	for (int i = 0; i < after_size.size(); ++i) {
		cout << "before : " << before_size[i] << "\tafter : " << after_size[i] << endl;
	}

	while (true) {
		waitKey(50);
	}
}

void ReduceContourPoint(std::vector<cv::Point>& vtContour, double fDistThresh, int loop)
{
	for (int i = 0; i < loop; ++i) {
		std::vector<cv::Point> remove_pt;
		int count = 0;
		for (std::vector<cv::Point>::iterator it = std::begin(vtContour); it != std::end(vtContour) - 1; ++it, ++count)
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

			cout << pt1 << endl;
			cout << pt2 << endl;
			cout << pt3 << endl;


			double fDistance;
			float a, b, c;
			double area, AB;
			if (pt1.x == pt3.x) fDistance = abs(pt1.x - pt2.x);
			else if (pt1.y == pt3.y) fDistance = abs(pt1.y - pt2.y);
			else
			{
				area = abs((pt1.x - pt2.x) * (pt3.y - pt2.y) - (pt1.y - pt2.y) * (pt3.x - pt2.x));
				AB = pow(pow(pt1.x - pt3.x, 2) + pow(pt1.y - pt3.y, 2), 0.5);
				fDistance = (area / AB);
			}

			cout << count << " distance : " << fDistance << endl;

			if (fDistance < fDistThresh)
			{
				//컨투어벡터에서 pt2에 해당하는것 제거
				remove_pt.push_back(pt2);
			}

		}

		for (std::vector<cv::Point>::iterator it = std::begin(remove_pt); it != std::end(remove_pt); ++it)
		{
			vtContour.erase(std::remove(vtContour.begin(), vtContour.end(), *it), vtContour.end());
			cout << "deleted:" << (*it) << endl;
		}
	}
	cout << vtContour.size() << endl;
}


//reference
//https://cubelover.tistory.com/2