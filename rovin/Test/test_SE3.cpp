#include "rovin/Math/SE3.h"
#include "Rmatrix/rmatrix3.h"
#include "LieGroup/LieGroup.h"
#include <rovin/utils/Diagnostic.h>
#include <iostream>

using namespace std;

rovin::Math::SE3 A, B, C, D, E, F;
SE3 SE3_1, SE3_2, SE3_3, SE3_1_sq;
Eigen::Matrix4d AA, BB, CC, DD;

void test1()
{
	A = B * D * E * E;
	A *= A;
}
void test2()
{
	AA = (((BB * BB).eval() * BB).eval() * BB).eval();
	AA *= AA;
}
void test3()
{
	SE3_1_sq = SE3_1 * SE3_3 * SE3_2 * SE3_2;
	SE3_1_sq *= SE3_1_sq;
}

int main()
{
	se3 se3_1(1, 2, 3, 1, 3, 3);
	SE3_1.Exp(se3_1);
	SE3_2 = SE3_3 = SE3_1;

	B(0, 0) = SE3_1(0, 0);
	B(0, 1) = SE3_1(0, 1);
	B(0, 2) = SE3_1(0, 2);
	B(0, 3) = SE3_1(0, 3);

	B(1, 0) = SE3_1(1, 0);
	B(1, 1) = SE3_1(1, 1);
	B(1, 2) = SE3_1(1, 2);
	B(1, 3) = SE3_1(1, 3);

	B(2, 0) = SE3_1(2, 0);
	B(2, 1) = SE3_1(2, 1);
	B(2, 2) = SE3_1(2, 2);
	B(2, 3) = SE3_1(2, 3);

	B(3, 0) = SE3_1(3, 0);
	B(3, 1) = SE3_1(3, 1);
	B(3, 2) = SE3_1(3, 2);
	B(3, 3) = SE3_1(3, 3);

	D = C = E = F = B;
	BB = B;

	PERFORM_TEST(test1(), 1e+8);
	PERFORM_TEST(test2(), 1e+8);
	PERFORM_TEST(test3(), 1e+8);
	cout << B << endl;
	cout << A << endl;

	return 0;
}