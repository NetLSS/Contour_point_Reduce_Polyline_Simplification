#include <opencv2/opencv.hpp>
#include <iostream>
#include <stdio.h>
#include <io.h>
#include <string>
#include <fstream>


using namespace cv;
using namespace std;

#define M_PI 3.14159265

cv::Mat img = cv::imread("C:\\Users\\sangsu lee\\Desktop\\1_1274(iou_0.00).bmp", 0);


// distance mode only-----------------------------------------
double THRESHOLD = 0.9; 
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
	/*cout << vtContour.size() << endl;*/
	if (vtContour.size() < 3)
		return;
	for (int i = 0; i < loop; ++i) {
		std::vector<cv::Point> remove_pt;
		int count = 0;
		for (std::vector<cv::Point>::iterator it = std::begin(vtContour); it != std::end(vtContour) - 1 && it != std::end(vtContour) - 2 && it != std::end(vtContour); /*++it,*/ ++count)
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

			/*double pt2_angle = get_angle(pt1, pt2, pt3, DEBUG_MODE);
			double distance_pt1_pt2 = distance_point(pt1, pt2);
			if (isDbug) cout << "angle: " << pt2_angle << endl;
			if (isDbug) cout << "distance_pt1_pt3: " << distance_pt1_pt2 << endl;
			ang_max = max(ang_max, pt2_angle);
			ang_min = min(ang_min, pt2_angle);
			if (((pt2_angle < THRESHOLD_ANGLE) && (distance_pt1_pt2 < DELETE_MAX_DISTANCE)) || distance_pt1_pt2 < FORCE_DELETE_DISTANCE) {
				vtContour.erase(std::remove(vtContour.begin(), vtContour.end(), pt2), vtContour.end());
			}
			else {
				it++;
			}*/

			#pragma region distance mode
			double fDistance;
			double area, AB;
			double distance_pt1_pt2 = distance_point(pt1, pt2);

			if (pt1.x == pt3.x) fDistance = abs(pt1.x - pt2.x);
			else if (pt1.y == pt3.y) fDistance = abs(pt1.y - pt2.y);
			else
			{
				area = abs((pt1.x - pt2.x) * (pt3.y - pt2.y) - (pt1.y - pt2.y) * (pt3.x - pt2.x));
				AB = pow(pow(pt1.x - pt3.x, 2) + pow(pt1.y - pt3.y, 2), 0.5);
				fDistance = (area / AB);
			}

			if (isDbug) cout << count << " distance : " << fDistance << endl;
			//cout << "distance" << endl;
			if (fDistance < THRESHOLD || distance_pt1_pt2 <= FORCE_DELETE_DISTANCE)
			{
				//컨투어벡터에서 pt2에 해당하는것 제거
				//remove_pt.push_back(pt2);
				vtContour.erase(std::remove(vtContour.begin(), vtContour.end(), pt2), vtContour.end());
			}
			else {
				it++;
			}
			#pragma endregion
		}

	}
	if (isDbug) cout << vtContour.size() << endl;
}

int main()
{
	ofstream csv_result, csv_result2, csv_result3;

	csv_result.open("csv_result.csv");
	csv_result2.open("csv_result2.csv");
	csv_result3.open("csv_result_capacity.csv");
	
	csv_result << "file,index,before_pt,after_pt,before_area,after_area" << endl;
	csv_result2 << "file,index,before_pt,after_pt,before_area,after_area" << endl;
	csv_result3 << "file,index,original_capa,reduced_capa" << endl;
	
	#pragma region file list read
	string find_file_pattern = "D:\\2020\\DS\\Project\\2020-12-07-contour_image\\FP_cut\\*.bmp";
	string root_path = "D:\\2020\\DS\\Project\\2020-12-07-contour_image\\FP_cut\\";
	vector<string>::iterator ptr;
	vector<string> files;
	unsigned long long sum_original=0, sum_pass1=0, sum_pass2=0;
	int sum_pt_before = 0, sum_pt_after = 0;

	struct _finddata_t fd;
	intptr_t handle;
	if ((handle = _findfirst(find_file_pattern.c_str(), &fd)) == -1L)
		cout << "No file in directory!" << endl;
	do {
		//cout << fd.name << endl;
		files.push_back(root_path + fd.name); // 특정 폴더 내 파일 목록 불러오기
	} while (_findnext(handle, &fd) == 0);
	_findclose(handle);
	#pragma endregion


	cout << files.size() << endl;
	int fnc_cnt = 0;
	for (int k=0; k<(int)files.size(); ++k, ++fnc_cnt){
		string file_name = files[k];
		cv::Mat img = cv::imread(file_name, 0);

		#pragma region 원본 라벨 영역 계산
		Mat labels, stats, centroides;
		int cnt = connectedComponentsWithStats(img, labels, stats, centroides);

		if(DEBUG_MODE) cout << "original label" << endl;
		for (int i = 1; i < cnt; ++i) {
			int* p = stats.ptr<int>(i);
			if (DEBUG_MODE) cout << "label " << i << ": " << p[4] << endl;
			sum_original += p[4];
			//cout << p[4] << endl;
		}
		#pragma endregion

		//----------------------------------------------------------
		cv::RNG rng(1332345);

		vector<vector<cv::Point>> contours, contours_APPROX_SIMPLE;
		//vector<vector<cv::Point>>& contours_ref = contours;
		vector<cv::Vec4i> hierarchy, hierarchy_APPROX_SIMPLE;
		vector<int> before_size, after_size;

		//findContours(img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_TC89_L1 /*cv::CHAIN_APPROX_TC89_KCOS*/ /*cv::CHAIN_APPROX_SIMPLE*/);
		findContours(img, contours, hierarchy, cv::RETR_TREE, cv::CHAIN_APPROX_TC89_L1);
		findContours(img, contours_APPROX_SIMPLE, hierarchy_APPROX_SIMPLE, cv::RETR_TREE, cv::CHAIN_APPROX_SIMPLE);

		Mat padded;
		Mat contours_pass1 = Mat::zeros(640, 640, CV_8U);
		img.copyTo(padded);
		cvtColor(padded, padded, CV_GRAY2RGB);

		for (size_t i = 0; i < contours.size(); i++)
		{
			cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
			Scalar white = Scalar(255, 255, 255);
			drawContours(contours_pass1, contours, (int)i, white, -1, cv::LINE_8, hierarchy, 0);
			//drawContours(padded, contours, (int)i, color, 3, cv::LINE_8, hierarchy, 0);
			before_size.push_back((int)contours[i].size());
			
			//sum_pt_before += (int)contours[i].size();
			sum_pt_before += (int)contours_APPROX_SIMPLE[i].size();

			/*for (int j = 0; j < contours[i].size(); ++j) {
				circle(padded, contours[i].at(j), 1, Scalar(251, 255, 0), -1);
			}*/
		}


		if (DEBUG_MODE) std::cout << "before" << contours[0].size() << std::endl;

		for (size_t i = 0; i < contours.size(); i++)
			ReduceContourPoint(contours[i], THRESHOLD, LOOP, DEBUG_MODE);

		if (DEBUG_MODE) std::cout << "after" << contours[0].size() << std::endl;

		#pragma region findContours 1회 후 영역 계산
		cnt = connectedComponentsWithStats(contours_pass1, labels, stats, centroides);
		
		vector<int> before_area(contours.size());

		if (DEBUG_MODE) cout << "pass1 label" << endl;
		for (int i = 1; i < cnt; ++i) {
			int* p = stats.ptr<int>(i);
			if (DEBUG_MODE) cout << "label " << i << ": " << p[4] << endl;
			sum_pass1 += p[4];
			//cout << sum_pass1 << endl;
			before_area[i-1]=(p[4]);
		}
		#pragma endregion


		cv::Mat padded_after;
		Mat contours_pass2 = Mat::zeros(640, 640, CV_8U);
		img.copyTo(padded_after);
		cv::cvtColor(padded_after, padded_after, CV_GRAY2RGB);
		for (size_t i = 0; i < contours.size(); i++)
		{
			cv::Scalar color = cv::Scalar(rng.uniform(0, 256), rng.uniform(0, 256), rng.uniform(0, 256));
			Scalar white = Scalar(255, 255, 255);
			drawContours(contours_pass2, contours, (int)i, white, -1, cv::LINE_8, hierarchy, 0);
			//drawContours(padded_after, contours, (int)i, color, 3, cv::LINE_8, hierarchy, 0);
			after_size.push_back((int)contours[i].size());
			sum_pt_after += (int)contours[i].size();
		/*	for (int j = 0; j < contours[i].size(); ++j) {
				circle(padded_after, contours[i].at(j), 1, Scalar(251, 255, 0), -1);
			}*/
		}

		#pragma region 2회(reduce) 후 영역 계산
		cnt = connectedComponentsWithStats(contours_pass2, labels, stats, centroides);

		vector<int> after_area(contours.size());

		if (DEBUG_MODE) cout << "pass2 label" << endl;
		for (int i = 1; i < cnt; ++i) {
			int* p = stats.ptr<int>(i);
			if (DEBUG_MODE) cout << "label " << i << ": " << p[4] << endl;
			sum_pass2 += p[4];
			after_area[i-1]=(p[4]);
			//cout << "pass 2:" << sum_pass2 << endl;
		}
		#pragma endregion


		assert(after_size.size() == before_size.size());

		for (int i = 0; i < after_size.size(); ++i) {
			if (DEBUG_MODE) cout << "before : " << before_size[i] << "\tafter : " << after_size[i] << endl;
			float percent=0.0;
			if(before_area[i] !=0)
				percent = (1.0 * abs(after_area[i] - before_area[i]) / before_area[i]) * 100.0;
		
			csv_result << format("%s,%d,%d,%d,%d,%d,%f\n", file_name.c_str(), k, before_size[i], after_size[i], before_area[i], after_area[i], percent);
		}

		float percent = 0.0;
		if (sum_pass2!=0)
			percent = (1.0 * abs(sum_pass1 - sum_pass2) / sum_pass1) * 100.0;

		csv_result2 << format("%s,%d,%d,%d,%d,%d,%f\n", file_name.c_str(), k, sum_pt_before, sum_pt_after, sum_pass1, sum_pass2, percent);
		csv_result3 << format("%s,%d,%d,%d\n", file_name.c_str(), k, sum_pt_before * 8, sum_pt_after * 8);
		cout << fnc_cnt <<","<<percent<<"%"<< endl;

		sum_original = 0;
		sum_pass1 = 0;
		sum_pass2 = 0;
		sum_pt_after = 0;
		sum_pt_before = 0;
	}

	/*cout << sum_original << endl;
	cout << sum_pass1 << endl;
	cout << sum_pass2 << endl;*/

	csv_result.close();
	csv_result2.close();
	csv_result3.close();
	
	while (true) {
		waitKey(50);
	}

	return 0;
}

//reference
//https://cubelover.tistory.com/2
//https://qastack.kr/programming/1211212/how-to-calculate-an-angle-from-three-points