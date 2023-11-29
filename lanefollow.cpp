#include <iostream>
#include <stdio.h>
#include <opencv2/opencv.hpp>
#include <signal.h>
using namespace std;
using namespace cv;

bool ctrl_c_pressed = false;
void ctrlc_handler(int){ ctrl_c_pressed = true; }
// lanefollow_100rpm_ccw
// lanefollow_100rpm_cw
int main(void)
{
	VideoCapture cap("lanefollow_100rpm_ccw.mp4");
	if (!cap.isOpened()) { cerr << "failed!" << endl; return -1; }
	/*
	string wr1 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! h264parse ! rtph264pay pt=96 ! udpsink host=192.168.0.72 port=8451 sync=false";
    VideoWriter writer1(wr1, 0, (double)30, Size(640, 360), true);
    if (!writer1.isOpened()) { cerr << "Writer open failed!" << endl; return -1;}

	string wr2 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! h264parse ! rtph264pay pt=96 ! udpsink host=192.168.0.72 port=8452 sync=false";
    VideoWriter writer2(wr2, 0, (double)30, Size(640, 90), false);
    if (!writer1.isOpened()) { cerr << "Writer open failed!" << endl; return -1;}

	string wr3 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! h264parse ! rtph264pay pt=96 ! udpsink host=192.168.0.72 port=8453 sync=false";
    VideoWriter writer3(wr3, 0, (double)30, Size(640, 90), true);
    if (!writer1.isOpened()) { cerr << "Writer open failed!" << endl; return -1;}
	*/

	Mat src, dst, src2;  //영상
	Mat labels, stats, centroids; 
	int cnt; 
	Dxl mx;  //다이나믹셀
    	struct timeval start,end1;
    	double time1;
	int rvel = 0, lvel = 0, error;
	Point pt1(160,45),pt2(480,45),center(320,45);
	signal(SIGINT, ctrlc_handler);
    	if(!mx.open()) { cout << "dynamixel open error"<<endl; return -1; }
	while (true) {
		cap >> src;
		if (src.empty()) break;
		imshow("src", src);
		cvtColor(src, src, COLOR_BGR2GRAY);
		src = src(Rect(0,270,640,90));
		src2 = src.clone();
		src2 = src + (100 - mean(src)[0]);
		threshold(src2, dst, 135, 255, THRESH_BINARY);
		cnt = connectedComponentsWithStats(dst, labels, stats, centroids);
		cvtColor(dst, dst, COLOR_GRAY2BGR);
		for (int i = 1;i < cnt;i++) {
			int* p = stats.ptr<int>(i);
			error = dst.cols / 2 - (pt1.x+pt2.x/2);
			if (p[4] > 2000) {
				if (abs(pt1.x - centroids.at<double>(i, 0)) < dst.cols / 8) {
					//rectangle(dst, Rect(p[0], p[1], p[2], p[3]), Scalar(255, 0, 0), 2);
					pt1 = Point(centroids.at<double>(i, 0), centroids.at<double>(i, 1));
				}
					
				if (abs(pt2.x - centroids.at<double>(i, 0)) < dst.cols / 8) {  //-160~160
					//rectangle(dst, Rect(p[0], p[1], p[2], p[3]), Scalar(255, 0, 0), 2);
					pt2 = Point(centroids.at<double>(i, 0), centroids.at<double>(i, 1));
				}
			}
			lvel = 100 - 0.16 * error;
			rvel = -(100 + 0.16 * error);

		}
		cout << "pt1: " << pt1 << "pt2: " << pt2 << endl;
		circle(dst, Point(pt1), 1, Scalar(255, 0, 0), 3);
		circle(dst, Point(pt2), 1, Scalar(255, 0, 0), 3);
		center = (pt1 + pt2) / 2;
		circle(dst, Point(center), 1, Scalar(0, 0, 255), 2);
		mx.setVelocity(lvel,rvel);
        	if (ctrl_c_pressed) break; //Ctrl+c입력시 탈출
       		usleep(20*1000);
        	gettimeofday(&end1,NULL);
        	time1 =end1.tv_sec-start.tv_sec+(end1.tv_usec-start.tv_usec)/1000000.0;
        	cout <<"err:"<<error<<", rvel:"<<rvel<<", lvel:"<<lvel<< endl;
		imshow("src2", src2);
		imshow("dst", dst);
		waitKey(33);
	}
	mx.close(); // 장치닫기
	return 0;
}
