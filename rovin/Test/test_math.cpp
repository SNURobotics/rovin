#include <iostream>
#include <conio.h>
#include <cmath>
#include <time.h>

#include <rovin/Math/LieGroup.h>
#include <rovin/Model/Assembly.h>
#include <rovin/utils/Diagnostic.h>

#include "Rmatrix/rmatrix3.h"
#include "LieGroup/LieGroup.h"

using namespace std;
using namespace Eigen;
using namespace rovin;

typedef int(*a)(int);
typedef int(*aa)(int, int);

int afunc(int c)
{
	return c + 1;
}


int main()
{
	//RMatrix	A(4, 4), B(4,4), C(4,4);
	//A.SetEye(4,4);
	//B.SetEye(4, 4);

	//Matrix4d eA, eB, eC;
	//eA.setRandom();
	//eB.setRandom();
	//
	//PERFORM_TEST(C = A*B, 1e+6);
	//PERFORM_TEST(eC = eA*eB, 1e+6);

	//cout << C << endl;
	//cout << eC << endl;

	//PERFORM_TEST(RMatrix D(4, 4), 1e+7);
	//PERFORM_TEST(Math::SE3 eD, 1e+7);


	SE3	A, B, C;
	Math::SE3 AA, BB, CC;
	PERFORM_TEST(C = A*B*A*B*A, 1e+7);
	PERFORM_TEST(CC = AA*BB*AA*BB*AA, 1e+7);
	cout << C << endl;
	cout << CC << endl;

	se3 se3_1(1,2,3,4,5,6);
	SE3 SE3_1;
	Math::so3 se3_rovin1(1, 2, 3);
	Math::so3 se3_rovin2(1, 2, 3);
	Math::SE3 SE3_rovin;
	PERFORM_TEST(SE3_1 = Exp(se3_1), 1e+7);
	PERFORM_TEST(SE3_rovin = Math::SE3::Exp(se3_rovin1, se3_rovin2), 1e+7);
	cout << SE3_1 << endl;
	cout << SE3_rovin << endl;


	//Vec3 so3_srLib(1, 2, 3);
	//SO3 SO3_srLib;

	//Math::so3 so3_rovin(1, 2, 3);
	//Math::SO3 SO3_rovin;

	//PERFORM_TEST(SO3_srLib = Exp(so3_srLib), 1e+7);
	//PERFORM_TEST(SO3_rovin = Math::SO3::Exp(so3_rovin), 1e+7);

	//cout << SO3_srLib(0,0) << endl;
	//cout << SO3_rovin << endl;


	//Matrix4d A, B, C, D;
	//A.setRandom();
	//B.setRandom();
	//C.setRandom();
	//D.setRandom();

	//PERFORM_TEST(A = B*C*D*D, 1e+6);
	//cout << A << endl;
	//PERFORM_TEST(A = (((B*C).eval()*D).eval()*D).eval(), 1e+6);
	//cout << A << endl;
	return 0;
}