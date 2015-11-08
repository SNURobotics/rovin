#include <iostream>
#include <conio.h>
#include <cmath>

#include <rovin/Math/LieGroup.h>
#include <rovin/Math/Constraint.h>
#include <rovin/Model/Assembly.h>

using namespace std;
using namespace Eigen;
using namespace rovin;

class userConstraint : public Math::Constraint
{
public:
	userConstraint() : Constraint(2, 2) {}

	VectorXd func(const VectorXd& q)
	{
		VectorXd f(2);
		f(0) = sin(q(0)) + cos(q(0)) - 1;
		f(1) = q(0) + q(1) - 3.1415926535;
		return f;
	}
};

int main()
{
	cout << "ROVIN È­ÀÌÆÃ!" << endl;

	rovin::Math::SE3 T(rovin::Math::SO3::RotX(3.141592));
	cout << T << endl;
	
	userConstraint A;
	cout << A.sovleEq(Eigen::Vector2d(0, 2)) << endl;

	_getch();
	return 0;
}