#include <iostream>
#include <ctime>
#include <rovin/Math/Common.h>
#include <rovin/Math/Optimization.h>

using namespace std;
using namespace rovin::Math;

class Example : public Function
{
	VectorX func(const VectorX& x)
	{
		Vector2 f;
		f(0) = sin(x[0]);
		f(1) = 2 * x(0) + 3;
		return f;
	}
};

int main()
{
	Example f;

	Eigen::Matrix<double, 1, 1> x;
	x(0) = 3;

	cout << f(x) << endl;
	cout << f.Jacobian(x) << endl;
	cout << f.Hessian(x)[0] << endl;

	return 0;
}