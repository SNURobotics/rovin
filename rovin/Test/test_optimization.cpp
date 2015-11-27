#include <iostream>
#include <ctime>
#include <rovin/Math/Common.h>
#include <rovin/Math/Optimization.h>

using namespace std;
using namespace rovin::Math;

class objective : public Function
{
	VectorX func(const VectorX& x) const
	{
		Eigen::Matrix< double, 1, 1 > f;
		f(0) = x(0)*x(1)+x(0)*x(0)*x(0)+x(1)*x(1)*x(1)+x(0)+x(1);
		return f;
	}
};
class equality : public Function
{
	VectorX func(const VectorX& x) const
	{
		Eigen::Matrix< double, 1, 1 > f;
		f(0) = x(0)*x(0) + x(1)*x(1) - 5;
		return f;
	}
};
class inequality : public Function
{
	VectorX func(const VectorX& x) const
	{
		Vector2 f;
		f(0) = x(0)+5;
		f(1) = x(1)+5;
		return f;
	}
};

int main()
{
	//Matrix3 L, G, C;
	//Vector3 g;
	//Vector3 A;
	//Eigen::Matrix<double, 1, 1> b;
	//Vector3 d;

	//L << 1, 2, 3, -2, -3, -4, 4, 3, 2;
	//G = L*L.transpose();
	//g << 1, 2, 3;
	//A << 1, 1, 1;
	//b << 5;
	//C.setIdentity();
	//d << 0, 0, 0;

	//Vector3 x;
	//x << 1, 2, 3;
	//x = QuadraticProgrammingEq(G, g, A, b, C, d, x);
	//cout << x << endl;
	objective obj_f;
	equality eq_f;
	inequality ineq_f;

	Vector2 x;
	x << 0, 0;

	Vector2 xf = NonlinearProgramming(obj_f, ineq_f, eq_f, x);

	cout << xf << endl;
	cout << obj_f(xf) << endl;

	return 0;
}