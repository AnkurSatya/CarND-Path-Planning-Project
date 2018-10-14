#include <iostream>
#include <map>
#include <ctime>
#include <chrono>
#include <vector>
#include "Eigen-3.3/Eigen/Eigen"
#include "spline/src/spline.h"
// #include <opencv2>
// #include <opencv2/opencv.hpp>
// #include <opencv2/highgui/highgui.hpp>
// #include <opencv2/imgproc/imgproc.hpp>
using namespace std;

int main()
{
	// Eigen::VectorXd A(3);
	// A<<1,2,3;
	// A<<4,5,6;
	// cout<<A;

	// Gnuplot gp("lines");
	// gp.plot_slope(1.0,0.0,"y=x");

	// auto elapsed = chrono::high_resolution_clock::now() -start;
 //    long long ms = chrono::duration_cast<chrono::microseconds>(elapsed).count();
	// vector<int> a(3,10);
	// for(int i = 0; i<a.size();i++)
	// {
	// 	cout<<a[i]<<endl;
	// }
	// double max = 6945.554 - 0;
	// double a = 7000.0;
	// double b = fmod(a, max);
	// cout<<b<<endl;
	// for(int i = 0; i<11; i++)
	// {
	// 	cout<<exp(i)<<endl;
	// }
	// cout<<exp(10)<<endl;
	// cout<<1-exp(5)/exp(10)<<endl;
	// cout<<"OpenCV Version = "<<CV_VERSION<<endl;
	vector<int> a = {1,2,3,4,5,6,7,8,9,0};
	vector<int>b(a.begin()+1, a.end());
	// a = b;
	for(int i = 0; i<b.size(); i++)
	{
		cout<<b[i]<<endl;
	}
}
