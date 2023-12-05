#include <iostream>
#include "opencv2/opencv.hpp"
#include <unistd.h>
#include <sys/time.h>
#include <signal.h>
#include "dxl.hpp"

using namespace std;
using namespace cv;
bool mode = false;   //다이나믹셀 모드(false=동작X)
bool ctrl_c_pressed = false; //ctrl+c 여부
void ctrlc_handler(int) { ctrl_c_pressed = true; } //ctrl+c가 눌리면 true로 변경
int main(void)
{
	string wr1 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! h264parse ! rtph264pay pt=96 ! udpsink host=192.168.0.72 port=8451 sync=false";
   	VideoWriter writer1(wr1, 0, (double)30, Size(640, 360), true);
   	if (!writer1.isOpened()) { cerr << "Writer open failed!" << endl; return -1; }

  	string wr2 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! h264parse ! rtph264pay pt=96 ! udpsink host=192.168.0.72 port=8452 sync=false";
   	VideoWriter writer2(wr2, 0, (double)30, Size(640, 90), false);
   	if (!writer2.isOpened()) { cerr << "Writer open failed!" << endl; return -1; }

   	string wr3 = "appsrc ! videoconvert ! video/x-raw, format=BGRx ! nvvidconv ! nvv4l2h264enc insert-sps-pps=true ! h264parse ! rtph264pay pt=96 ! udpsink host=192.168.0.72 port=8453 sync=false";
   	VideoWriter writer3(wr3, 0, (double)30, Size(640, 90), true);
   	if (!writer3.isOpened()) { cerr << "Writer open failed!" << endl; return -1; }

	//영상파일
	VideoCapture source("lanefollow_100rpm_ccw.mp4");  //출력영상
	if (!source.isOpened()) { cerr << "failed!" << endl; return -1; } //예외처리

	Mat src, dst, src2;  //영상
	Mat labels, stats, centroids;  //레이블링 변수
	int cnt;
	Dxl mx;  //다이나믹셀
	int rvel = 0, lvel = 0, error = 0;
	struct timeval start, end1; //시간 변수
	double time1;
	Point l_pt(160, 45), r_pt(480, 45), center(320, 45);  //최소값 변
	Point min(320, 45), l_min(160, 45), r_min(480, 45);
	signal(SIGINT, ctrlc_handler); //시그널 핸들러 지정
	if (!mx.open()) { cout << "dynamixel open error" << endl; return -1; } //예외처리
	while (true) {
		gettimeofday(&start, NULL); //시작시간
		source >> src; //영상 불러오기
		if (src.empty()) { cerr << "frame empty!" << endl; break; }  //예외처리
		writer1 << src; //영상출력
		cvtColor(src, src, COLOR_BGR2GRAY); //컬러->그레이
		src = src(Rect(0, 270, 640, 90)); //ROI
		src2 = src.clone(); //ROI영상 src2에 복사
		src2 = src + (80 - mean(src)[0]); //밝기 조정
		GaussianBlur(src2, dst, Size(5, 5), 5); //노이즈 제거
		threshold(src2, dst, 135, 255, THRESH_BINARY); //이진화
		cnt = connectedComponentsWithStats(dst, labels, stats, centroids); //바운딩박스
		cvtColor(dst, dst, COLOR_GRAY2BGR); //그레이->컬러
		int c1[2] = {0, 500}, c2[2] = {0, 500};  //c[0]=현재값, c[1]=최소값
		int n1 = 0, n2 = 0;  //최소값의 cnt
		l_pt = l_min;  r_pt = r_min;  //각 라인의 중심
		if (mx.kbhit()) //키보드 입력 체크
		{
			char ch = mx.getch(); //키입력 받기
			if (ch == 'q') break; //q이면 종료
			else if (ch == 's') mode = true; //s면 다이나믹셀 모드 true
		}
		for (int i = 1;i < cnt;i++) {
			Point pp= Point(centroids.at<double>(i, 0), centroids.at<double>(i, 1));
			if (centroids.at<double>(i, 0) < min.x) {
				c1[0] = abs(sqrt(pow(l_pt.x - pp.x, 2) + pow(l_pt.y - pp.y, 2)));  //왼쪽라인 x좌표 - 바운딩박스 객체 x좌표
				if (c1[0] < dst.cols / 4 && c1[0] < c1[1]) {
					c1[1] = c1[0];   //최소값 저장 (int)
					l_min = Point(centroids.at<double>(i, 0), centroids.at<double>(i, 1));      //최소값의 무게중심 저장 (Point)
					n1 = i; //최소값의 cnt값 저장
				}
			}
			else if (centroids.at<double>(i, 0) > min.x) {
				c2[0] = abs(sqrt(pow(r_pt.x - pp.x, 2) + pow(r_pt.y - pp.y, 2))); //오른쪽라인 x좌표 - 바운딩박스 객체 x좌표
				if (c2[0] < dst.cols / 4 && c2[0] < c2[1]) {
					c2[1] = c2[0];   //최소값 저장 (int)
					r_min = Point(centroids.at<double>(i, 0), centroids.at<double>(i, 1));      //최소값의 무게중심 저장 (Point)
					n2 = i; //최소값의 cnt값 저장
				}
			}
		}
		if (n1 > 0 ) { //왼쪽라인 바운딩 박스
			rectangle(dst, Rect(stats.ptr<int>(n1)[0], stats.ptr<int>(n1)[1], stats.ptr<int>(n1)[2], stats.ptr<int>(n1)[3]), Scalar(0, 0, 255), 1);
		}
		if (n2 > 0) {  //오른쪽라인 바운딩 박
			rectangle(dst, Rect(stats.ptr<int>(n2)[0], stats.ptr<int>(n2)[1], stats.ptr<int>(n2)[2], stats.ptr<int>(n2)[3]), Scalar(0, 0, 255), 1);
		}
		min = (l_min + r_min) / 2; //두 라인의 중심 Point
		circle(dst, Point(min), 1, Scalar(255, 0, 0), 2); //중심 점
		circle(dst, Point(l_min), 1, Scalar(0, 0, 255), 2); //왼쪽 점
		circle(dst, Point(r_min), 1, Scalar(0, 0, 255), 2); //오른쪽 점
		error = dst.cols / 2 - min.x; //에러값(min.x=두 라인 중심의 x값)
		lvel = 100 - 0.15 * error;
		rvel = -(100 + 0.15 * error);
		if (ctrl_c_pressed) break; //Ctrl+c입력시 탈출
		if (mode) mx.setVelocity(lvel, rvel); //다이나믹 셀 모드가 true면 작동
		usleep(1000);
		gettimeofday(&end1, NULL); //끝 시간
		time1 = end1.tv_sec - start.tv_sec + (end1.tv_usec - start.tv_usec) / 1000000.0;
		cout << "err:" << error << ", lvel:" << lvel << ", rvel:" << rvel << ", time:" << time1 << endl;
		writer2 << src2; //ROI 영상출력
		writer3 << dst;  //라인검출 영상출력
		waitKey(33);
	}
	mx.close(); // 장치닫기
	return 0;
}
