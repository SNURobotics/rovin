#include <iostream>
#include <fstream>
#include <Eigen/Dense>
#include <rovin/Math/Constant.h>
#include <unsupported/Eigen/Splines>

#include <time.h>

#include <rovin/Math/Spline.h>

using namespace std;

int main()
{
	//ofstream out("C:\\Users\\±Ÿ¡ÿ\\Documents\\MATLAB\\output.txt");

	Eigen::Matrix<double, 9, 1> knots;
	knots << 1/20.0, 3 / 20.0, 5 / 20.0, 5 / 20.0, 11 / 20.0, 11 / 20.0, 13 / 20.0, 14 / 20.0, 20 / 20.0;
	Eigen::Matrix<double, 1, 5> controlPoints;
	controlPoints << 0.9, 4, -9, 88, 100;
	double d = 0;
	double c;
	c = clock();
	Eigen::Spline<double, 1, 20> a(knots, controlPoints);
	for (unsigned int i = 0; i < 100000; i++)
	{
		d = a(5 / 20.0)(0);
	}
	cout << d << endl;
	cout << clock() - c << endl;
	c = clock();
	rovin::Math::BSpline<5, 4, 1> sp(knots, controlPoints);
	for (unsigned int i = 0; i < 100000; i++)
	{
		d = sp(5 / 20.0)(0);
	}
	cout << d << endl;
	cout << clock() - c << endl;
	for (double x = 5 / 20.0; x < 1.0; x+=0.001)
	{
		if (abs(sp(x)(0) -  a(x)(0)) >= 0.000000000000001)
		{
			cout << "no" << endl;
			cout.precision(20);
			cout << x << endl;
			cout << sp(x) << endl;
			cout << a(x) << endl;
			break;
		}
	}
	//for (double x = 0.0; x <= 30.0; x += 0.01)
	//{
	//	out << sp.fval(x) << endl;
	//}
	//cout << clock() - c << endl;
	//cout << d << endl;

	return 0;
}