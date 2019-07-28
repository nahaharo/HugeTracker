#pragma once
#include "stdafx.h"

namespace huge
{
	struct Img_process_option
	{
		float blur_sigma;
		int threshold;
		cv::Size blur_size;
		int offset;
	};

	Img_process_option Default_option = { 3.0, 220, cv::Size(5,5), 80 };

	class camera_intrinsic
	{
	public:
		camera_intrinsic(float fx, float fy, float ppx, float ppy, std::array<float, 5> dist) : fx(fx), fy(fy), ppx(ppx), ppy(ppy), dist(dist) {};
		const cv::Mat get_intr_mat() const { return (cv::Mat_<float>(3, 3) << fx, 0, ppx, 0, fy, ppy, 0, 0, 1); };
		const cv::Mat get_dist_mat() const { return cv::Mat(1, 5, CV_32F, const_cast<float*>(dist.data())); };

	private:
		//Setting for 4k option
		const float fx;
		const float fy;
		const float ppx;
		const float ppy;
		const std::array<float, 5> dist;
	};

	camera_intrinsic mode_4K = camera_intrinsic(2293.22138f, 2288.47566f, 1950.98829f, 1046.01488f, { -0.39900f, 0.18573f, 0.00166f, 0.00114f, 0.00000f });

	struct Object_Point
	{
		const float L1;
		const float L2;

		const cv::Point3f p0 = cv::Point3f(0.0f, -L2, 0.0f);
		const cv::Point3f p1 = cv::Point3f(0.0f, L1 - L2, 0.0f);
		const cv::Point3f p2 = cv::Point3f(-L1 / 2, 0.0f, 0.0f);
		const cv::Point3f p3 = cv::Point3f(L1 / 2, 0.0f, 0.0f);

		const std::vector<cv::Point3f> points = { p0, p1, p2, p3 };
	};

	Object_Point Basic = { 70.0f, 30.0f };

	class HugeTracker
	{
	public:
		static HugeTracker * GetInstance(std::string url)
		{
			if (m_pInstance == nullptr)
				m_pInstance = new HugeTracker(url);
			return m_pInstance;
		};

		static HugeTracker * GetInstance()
		{
			if (m_pInstance == nullptr)
				throw std::runtime_error("HugeTracker was not properly created.");
			return m_pInstance;
		};

		static void DestroyInstance()
		{
			if (m_pInstance)
			{
				delete m_pInstance;
				m_pInstance = nullptr;
			}
		};

		std::tuple<std::array<double, 3>, std::array<double, 9>> get_transform(size_t id)
		{
			result_lock.lock();
			auto tmp1 = trans;
			auto tmp2 = rots;
			result_lock.unlock();
			return std::make_tuple(tmp1, tmp2);
		};

	private:
		HugeTracker(std::string Source, Img_process_option& option = Default_option, Object_Point obj = Basic, camera_intrinsic& intr = mode_4K)
			: capture(Source), img_option(option), object(Basic), intr(intr)
		{
			capture = cv::VideoCapture(Source);

			if (!capture.isOpened())
				throw std::runtime_error("Error opening video stream or file");

			cv::Mat frame, gray;
			capture.read(frame);
			cv::cvtColor(frame, gray, cv::COLOR_RGB2GRAY);
			thresh_mask = cv::Mat::ones(frame.size(), CV_8U)*255;
			uchar* data = thresh_mask.data;
			for (int i = 0; i < 300; i++)
			{
				for (int j = 0; j < frame.size().height; j++)
				{
					data[j*frame.size().width + i] = 0;
				}
			}


			producer_thread = std::thread([=] { producer(); });
			consumer_thread = std::thread([=] { consumer(); });
		};
		~HugeTracker()
		{
			kill_sig = false;
			producer_thread.join();
			consumer_thread.join();
		};

		

		static HugeTracker * m_pInstance;

		Img_process_option img_option;
		Object_Point object;
		camera_intrinsic intr;

		cv::VideoCapture capture;

		concurrency::concurrent_queue<cv::Mat> image_queue;

		std::mutex result_lock;
		std::array<double, 3> trans;
		std::array<double, 9> rots;

		std::thread producer_thread;
		std::thread consumer_thread;

		std::atomic<bool> kill_sig = false;

		cv::Mat thresh_mask;

	private:
		bool Getcrosspoint(cv::Point2f &p1, cv::Point2f &p2, cv::Point2f &p3, cv::Point2f &p4, cv::Point2f &cross);
		bool isright(cv::Point2f &p1, cv::Point2f &p2, bool &ans);
		bool LabelingPoint(std::tuple<cv::Point2f, cv::Point2f, cv::Point2f, cv::Point2f> &param, std::tuple<cv::Point2f, cv::Point2f, cv::Point2f, cv::Point2f> &ans);
		void producer();
		void consumer();
	};

	HugeTracker* HugeTracker::m_pInstance = nullptr;
}


bool huge::HugeTracker::Getcrosspoint(cv::Point2f &p1, cv::Point2f &p2, cv::Point2f &p3, cv::Point2f &p4, cv::Point2f &cross)
{
	auto c1 = p1.x * p2.y - p2.x * p1.y;
	auto a1 = p2.y - p1.y;
	auto b1 = -p2.x + p1.x;

	auto c2 = p3.x * p4.y - p4.x * p3.y;
	auto a2 = p4.y - p3.y;
	auto b2 = -p4.x + p3.x;

	auto det = a1 * b2 - b1 * a2;
	if (det == 0) return false;
	auto x = (c1 * b2 - b1 * c2) / det;
	auto y = (-a2 * c1 + a1 * c2) / det;

	cross = cv::Point2f(x, y);
	return true;
}

bool huge::HugeTracker::isright(cv::Point2f &p1, cv::Point2f &p2, bool &rans)
{
	auto ans = p1.x * p2.y - p1.y * p2.x;
	if (ans == 0) return false;
	rans = ans;
	return true;
}

bool huge::HugeTracker::LabelingPoint(std::tuple<cv::Point2f, cv::Point2f, cv::Point2f, cv::Point2f> &param, std::tuple<cv::Point2f, cv::Point2f, cv::Point2f, cv::Point2f> &ans)
{

	auto& p0 = std::get<0>(param);
	auto& p1 = std::get<1>(param);
	auto& p2 = std::get<2>(param);
	auto& p3 = std::get<3>(param);
	//estimation
	std::array<cv::Point2f, 3> crosses;

	bool check = true;

	{
		check &= Getcrosspoint(p0, p1, p2, p3, crosses[0]);
		check &= Getcrosspoint(p0, p2, p1, p3, crosses[1]);
		check &= Getcrosspoint(p0, p3, p1, p2, crosses[2]);
	}
	if (!check) return false;

	cv::Point2f avg_point = p0 + p1 + p2 + p3;
	avg_point /= 4.0f;

	std::array<float, 3> cross_dist;
	cross_dist[0] = cv::norm(crosses[0] - avg_point);
	cross_dist[1] = cv::norm(crosses[1] - avg_point);
	cross_dist[2] = cv::norm(crosses[2] - avg_point);

	//argmin
	size_t min;
	if (cross_dist[0] > cross_dist[1])
	{
		if (cross_dist[1] > cross_dist[2]) min = 2;
		else min = 1;
	}
	else
	{
		if (cross_dist[0] > cross_dist[2]) min = 2;
		else min = 0;
	}

	std::pair<cv::Point2f, cv::Point2f> set1, set2;
	cv::Point2f cross = crosses[min];
	switch (min)
	{
	case 0:
		set1 = std::make_pair<>(p0, p1);
		set2 = std::make_pair<>(p2, p3);
		break;
	case 1:
		set1 = std::make_pair<>(p0, p2);
		set2 = std::make_pair<>(p1, p3);
		break;
	case 2:
		set1 = std::make_pair<>(p0, p3);
		set2 = std::make_pair<>(p1, p2);
		break;
	};

	std::array<float, 4> dist_from_center;
	dist_from_center[0] = cv::norm(set1.first - cross);
	dist_from_center[1] = cv::norm(set1.second - cross);
	dist_from_center[2] = cv::norm(set2.first - cross);
	dist_from_center[3] = cv::norm(set2.second - cross);

	cv::Point2f A, B, C, D;
	if (abs(dist_from_center[0] - dist_from_center[1]) > abs(dist_from_center[2] - dist_from_center[3]))
	{
		if (dist_from_center[0] > dist_from_center[1])
		{
			B = set1.first;
			A = set1.second;
		}
		else
		{
			B = set1.second;
			A = set1.first;
		}
		cv::Point2f AB = B - A;
		cv::Point2f tmp = set2.second - cross;
		bool res;
		if (!isright(tmp, AB, res)) return false;
		if (res)
		{
			D = set2.second;
			C = set2.first;
		}
		else
		{
			D = set2.first;
			C = set2.second;
		}
	}
	else
	{
		if (dist_from_center[2] > dist_from_center[3])
		{
			B = set2.first;
			A = set2.second;
		}
		else
		{
			B = set2.second;
			A = set2.first;
		}
		cv::Point2f AB = B - A;
		cv::Point2f tmp = set1.second - cross;

		bool res;
		if (!isright(tmp, AB, res)) return false;
		if (res)
		{
			D = set1.second;
			C = set1.first;
		}
		else
		{
			D = set1.first;
			C = set1.second;
		}
	}
	if (abs(cv::norm(B - A) - cv::norm(C - D)) > 50) return false;

	ans = std::make_tuple<>(A, B, C, D);
	return true;
}

void huge::HugeTracker::producer()
{
	while (!kill_sig)
	{
		cv::Mat frame;
		if (capture.grab())
		{
			capture.retrieve(frame);
			image_queue.push(frame);
		}
	}
}

void huge::HugeTracker::consumer()
{
	while (!kill_sig) {
		cv::Mat frame, gray, cutted, thresh, blured;
		if (!image_queue.try_pop(frame)) continue;

		// If the frame is empty, break immediately
		if (frame.empty())
			break;

		cv::cvtColor(frame, gray, cv::COLOR_RGB2GRAY);


		cv::GaussianBlur(gray, blured, img_option.blur_size, img_option.blur_sigma);

		cv::bitwise_and(blured, thresh_mask, cutted);

		//cv::compare(thresh_mask, blured, thresh, cv::CMP_LT);

		cv::threshold(cutted, thresh, 180, 255, cv::THRESH_BINARY);

		//Moments part
		std::vector<std::vector<cv::Point>> contours;
		cv::findContours(thresh, contours, cv::RETR_LIST, cv::CHAIN_APPROX_NONE);

		//Making momentum
		std::vector<cv::Point2f> centers(contours.size());
		size_t i = 0;
		for (auto c : contours)
		{
			auto m = cv::moments(c);
			if (m.m00 < 1e-7) continue;
			centers[i] = cv::Point2f(static_cast<float>(m.m10 / (m.m00)),
				static_cast<float>(m.m01 / (m.m00)));
			cv::circle(frame, centers[i], 10, cv::Scalar(0, 0, 255), 10);
			++i;
			
		}

		cv::Mat resized;
		cv::resize(cutted, resized, cv::Size(0, 0), 0.25, 0.25);
		cv::imshow("asdf", resized);
		cv::waitKey(1);


		size_t num_centers = i;
		if (num_centers < 4) continue;

		cv::Mat Dist(num_centers, num_centers, CV_64F);
		double* dist_data = (double*)Dist.data;
		for (size_t i = 0; i < num_centers; i++)
		{
			for (size_t j = i; j < num_centers; j++)
			{
				if (i == j) dist_data[i*Dist.cols + j] = 0;
				auto tmp = cv::norm(centers[i] - centers[j]);
				dist_data[i*Dist.cols + j] = tmp;
				dist_data[j*Dist.cols + i] = tmp;
			}
		}

		std::vector<std::tuple<int, int, int, int>> sig;
		std::vector<int> counts;
		for (size_t i = 0; i < num_centers; i++)
		{
			auto s = Dist.col(i);
			cv::Mat sorted;
			cv::sortIdx(s, sorted, cv::SORT_EVERY_COLUMN+cv::SORT_ASCENDING);
			int* sorted_data = (int*)sorted.data;
			std::sort(sorted_data, sorted_data + 4);
			auto comb_tuple = std::make_tuple<>(sorted_data[0], sorted_data[1], sorted_data[2], sorted_data[3]);
			bool flag = true;
			for (size_t i = 0; i< sig.size(); i++)
			{
				if (comb_tuple == sig[i])
				{
					counts[i] += 1;
					flag = false;
					break;
				}
			}
			if (flag)
			{
				sig.push_back(comb_tuple);
				counts.push_back(1);
			}
		}
		int max_idx = std::max_element(counts.begin(), counts.end()) - counts.begin();

		auto[i1, i2, i3, i4] = sig[max_idx];

		std::tuple<cv::Point2f, cv::Point2f, cv::Point2f, cv::Point2f> input_points, labeled_points;
		input_points = std::make_tuple<>(centers[i1], centers[i2], centers[i3], centers[i4]);
		if(!LabelingPoint(input_points, labeled_points)) continue;

		cv::Mat rvec, tvec, rod;	// rotation & translation vectors
		auto&[A, B, C, D] = labeled_points;
		std::vector<cv::Point2f> img_p = { A, B, C, D };
		cv::solvePnP(object.points, img_p, intr.get_intr_mat(), intr.get_dist_mat(), rvec, tvec);

		cv::Rodrigues(rvec, rod);
		std::array<double, 3> trans_arr = { tvec.at<double>(0), tvec.at<double>(1), tvec.at<double>(2) };
		std::array<double, 9>   rot_arr = { rod.at<double>(0,0), rod.at<double>(0,1), rod.at<double>(0,2),
										    rod.at<double>(1,0), rod.at<double>(1,1), rod.at<double>(1,2),
										    rod.at<double>(2,0), rod.at<double>(2,1), rod.at<double>(2,2) };

		cv::circle(frame, A, 10, cv::Scalar(0, 0, 255));
		cv::circle(frame, B, 10, cv::Scalar(0, 0, 255));
		cv::circle(frame, C, 10, cv::Scalar(0, 0, 255));
		cv::circle(frame, D, 10, cv::Scalar(0, 0, 255));

		//cv::Mat resized;
		//cv::resize(frame, resized, cv::Size(0,0), 0.25, 0.25);
		//cv::imshow("asdf", resized);
		//cv::waitKey(1);
		result_lock.lock();
		trans = trans_arr;
		rots = rot_arr;
		result_lock.unlock();
	}
}

inline std::tuple<double, double, double> mat_to_euler(const std::array<double, 9> m)
{
	double sy = sqrt(m[0]*m[0] + m[3]*m[3]);

	bool singular = sy < 1e-6;

	double x, y, z;
	if (!singular)
	{
		x = atan2(m[7], m[8]);
		y = atan2(-m[6], sy);
		z = atan2(m[3], m[0]);
	}
	else
	{
		x = atan2(-m[5], m[4]);
		y = atan2(-m[6], sy);
		z = 0;
	}
	return std::make_tuple(x, y, z);
}