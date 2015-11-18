#include <iostream>
#include <conio.h>
#include <cmath>
#include <time.h>
# include <omp.h>

#include <rovin/Math/LieGroup.h>
#include <rovin/Model/Assembly.h>
#include <rovin/utils/Diagnostic.h>

#include "Rmatrix/rmatrix3.h"
#include "LieGroup/LieGroup.h"

using namespace std;
using namespace Eigen;
using namespace rovin;

Math::Real cc = 0;
se3 se3_1(1, 2, 3, 1, 3, 3);
SE3 SE3_1, SE3_1_sq;
Math::so3 se3_rovin1(1, 2, 3);
Math::so3 se3_rovin2(1, 3, 3);
Math::SE3 SE3_rovin, SE3_rovin_sq;
Math::Matrix3 MA, MB;
Math::Vector3 MAV, MBV;
Eigen::Transform<double, 3, Eigen::Isometry> T1, T2;

class MYSE3
{
public:
	MYSE3()
	{
		R.setZero();
		p.setZero();
	}

	MYSE3& operator = (const MYSE3& operand)
	{
		R = operand.R;
		p = operand.p;
		return *this;
	}

	MYSE3 operator * (const MYSE3& operand) const
	{
		MYSE3 result;

		result.R.noalias() = R * operand.R;
		result.p.noalias() = R * operand.p + p;

		return result;
	}

	MYSE3& operator *= (const MYSE3& operand) 
	{
		p = R * operand.p + p;
		R = R * operand.R;

		return *this;
	}
public:
	Math::Matrix3 R;
	Math::Vector3 p;
};

MYSE3 MYA, MYB;

int afunc(int c)
{
	return c + 1;
}

void test1()
{
	SE3_1_sq = SE3_1 * SE3_1 * SE3_1 * SE3_1;
	//cc += SE3_1_sq(0, 0);
	//SE3_rovin_sq = SE3_rovin * SE3_rovin * SE3_rovin;
	//cc += SE3_rovin_sq.getRotation().matrix()(0, 0);
}
void test2()
{
	SE3_rovin_sq = SE3_rovin * SE3_rovin * SE3_rovin * SE3_rovin;
	//SE3_rovin_sq = Math::SE3::multiply(SE3_rovin, SE3_rovin, SE3_rovin);
	//MA = MB*MB;
	//MAV = MA*MBV + MBV;
	//MA = MA*MB;
	//MAV = MA*MBV + MBV;
	//MA = MA*MB;
	//MAV = MA*MBV + MBV;
	//MA = MA*MB;
	//MAV = MA*MBV + MBV;
	//SE3_rovin_sq = SE3_rovin;
	//SE3_rovin_sq.multiply(SE3_rovin, SE3_rovin);
	//cc += SE3_rovin_sq.getRotation().matrix()(0, 0);
}
void test3()
{
	SE3_rovin_sq = Math::SE3::multiply(SE3_rovin, SE3_rovin, SE3_rovin, SE3_rovin);
	cc += SE3_rovin_sq.getRotation().matrix()(0, 0);
}
void test4()
{
	//T1 = T2 * T2 * T2 * T2;
	//cc = T1(0, 0);
	MYA = MYB * MYB * MYB;
	cc += MYA.R(0, 0);
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

	omp_set_num_threads(10);
	cout << Eigen::nbThreads() << endl;
	SE3	A, B, C;
	Matrix4d AA, BB, CC;
	AA.setZero();
	BB.setZero();
	CC.setZero();
	cout << C << endl;
	cout << CC << endl;

	PERFORM_TEST(SE3_1 = Exp(se3_1), 1e+8);
	PERFORM_TEST(SE3_rovin = Math::SE3::Exp(se3_rovin1, se3_rovin2), 1e+8);
	cout << SE3_1 << endl;
	cout << SE3_rovin << endl;
	test1();
	cout << SE3_1_sq << endl;
	test3();
	cout << SE3_rovin_sq << endl;

	MYB.R = SE3_rovin.getRotation().matrix();
	MYB.p = SE3_rovin.getPosition().matrix();
	cout << "3" << endl;
	PERFORM_TEST(test3(), 1e+8);
	PERFORM_TEST(test3(), 1e+8);
	PERFORM_TEST(test3(), 1e+8);
	cout << "4" << endl;
	MB.setRandom();
	PERFORM_TEST(test4(), 1e+8);
	PERFORM_TEST(test4(), 1e+8);
	PERFORM_TEST(test4(), 1e+8);
	cout << "1" << endl;
	PERFORM_TEST(test1(), 1e+8);
	PERFORM_TEST(test1(), 1e+8);
	PERFORM_TEST(test1(), 1e+8);
	cout << "2" << endl;
	PERFORM_TEST(test2(), 1e+8);
	PERFORM_TEST(test2(), 1e+8);
	PERFORM_TEST(test2(), 1e+8);
	cout << "3" << endl;
	PERFORM_TEST(test3(), 1e+8);
	PERFORM_TEST(test3(), 1e+8);
	PERFORM_TEST(test3(), 1e+8);
	cout << "4" << endl;
	MB.setRandom();
	PERFORM_TEST(test4(), 1e+8);
	PERFORM_TEST(test4(), 1e+8);
	PERFORM_TEST(test4(), 1e+8);
	cout << "1" << endl;
	PERFORM_TEST(test1(), 1e+8);
	PERFORM_TEST(test1(), 1e+8);
	PERFORM_TEST(test1(), 1e+8);
	cout << "2" << endl;
	PERFORM_TEST(test2(), 1e+8);
	PERFORM_TEST(test2(), 1e+8);
	PERFORM_TEST(test2(), 1e+8);
	cout << "3" << endl;
	PERFORM_TEST(test3(), 1e+8);
	PERFORM_TEST(test3(), 1e+8);
	PERFORM_TEST(test3(), 1e+8);
	cout << "4" << endl;
	MB.setRandom();
	PERFORM_TEST(test4(), 1e+8);
	PERFORM_TEST(test4(), 1e+8);
	PERFORM_TEST(test4(), 1e+8);
	cout << "1" << endl;
	PERFORM_TEST(test1(), 1e+8);
	PERFORM_TEST(test1(), 1e+8);
	PERFORM_TEST(test1(), 1e+8);
	cout << "2" << endl;
	PERFORM_TEST(test2(), 1e+8);
	PERFORM_TEST(test2(), 1e+8);
	PERFORM_TEST(test2(), 1e+8);
	cout << "3" << endl;
	PERFORM_TEST(test3(), 1e+8);
	PERFORM_TEST(test3(), 1e+8);
	PERFORM_TEST(test3(), 1e+8);
	cout << "4" << endl;
	MB.setRandom();
	PERFORM_TEST(test4(), 1e+8);
	PERFORM_TEST(test4(), 1e+8);
	PERFORM_TEST(test4(), 1e+8);
	cout << "1" << endl;
	PERFORM_TEST(test1(), 1e+8);
	PERFORM_TEST(test1(), 1e+8);
	PERFORM_TEST(test1(), 1e+8);
	cout << "2" << endl;
	PERFORM_TEST(test2(), 1e+8);
	PERFORM_TEST(test2(), 1e+8);
	PERFORM_TEST(test2(), 1e+8);
	cout << "1" << endl;
	cout << SE3_1_sq << endl;
	cout << SE3_rovin_sq << endl;


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