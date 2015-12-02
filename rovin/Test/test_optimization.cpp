#include <iostream>
#include <ctime>
#include <rovin/Math/Common.h>
#include <rovin/Math/Optimization.h>

using namespace std;
using namespace rovin::Math;

class objective : public Function
{
public:
	objective() {}
	VectorX func(const VectorX& x) const
	{
		Eigen::Matrix< double, 1, 1 > f;
		f(0) = x(1)*x(3) - x(1)*x(1)*x(2)*x(3);
		return f;
	}
};
class equality : public Function
{
public:
	equality() {}
	VectorX func(const VectorX& x) const
	{
		Eigen::Matrix< double, 1, 1 > f;
		f(0) = +x(0)*x(0)*x(0) + x(1)*x(1)*x(1) - x(2)*x(3)*x(3)*x(3) - 5;
		return f;
	}
};
class inequality : public Function
{
public:
	inequality() {}
	VectorX func(const VectorX& x) const
	{
		VectorX f(4);
		f(0) = x(0) - 10;
		f(1) = x(1) - 10;
		f(2) = -x(0) + 1;
		f(3) = -x(1) + 1;
		return f;
	}
};

int main()
{
	FunctionPtr obj_f = FunctionPtr(new objective());
	FunctionPtr eq_f = FunctionPtr(new equality());
	FunctionPtr ineq_f = FunctionPtr(new inequality());

	Vector4 x;
	x << 1 ,   1 , 10 , 10;

	NonlinearOptimization nonlinearSolver;
	nonlinearSolver._objectiveFunc = obj_f;
	nonlinearSolver._eqFunc = eq_f;
	nonlinearSolver._ineqFunc = ineq_f;

	double c = clock();
	x = nonlinearSolver.solve(x);
	cout << nonlinearSolver._Iter << endl;
	cout << "x : " << endl << x << endl;
	cout << "obj : " << endl << (*obj_f)(x) << endl;
	cout << "eq : " << endl << (*eq_f)(x) << endl;
	cout << "ineq : " << endl << (*ineq_f)(x) << endl;
	cout << clock() - c << endl;

	return 0;
}