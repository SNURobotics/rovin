#include <iostream>
#include <conio.h>
#include <cmath>
#include <time.h>
#include <omp.h>

#include <rovin/Math/Numeric.h>
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
SO3 SO3_1, SO3_2;

std::vector<Matrix3d, Eigen::aligned_allocator< Matrix3d >> R;
std::vector<Vector3d, Eigen::aligned_allocator< Vector3d >> P;
Matrix3d resultR, R31, R32;
Vector3d resultP;
Matrix3d RR;

int afunc(int c)
{
	return c + 1;
}

void test1()
{
	//SE3_1_sq = SE3_1 * SE3_1 * SE3_1 * SE3_1;
	SO3_1 = SO3_2 *SO3_2 *SO3_2 *SO3_2;
	//cc += SE3_1_sq(0, 0);
	//SE3_rovin_sq = SE3_rovin * SE3_rovin * SE3_rovin;
	//cc += SE3_rovin_sq.getRotation().matrix()(0, 0);
}
void test2()
{
	//SE3_rovin_sq = SE3_rovin * SE3_rovin * SE3_rovin * SE3_rovin;
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

Matrix4d A4, B4;
Matrix3d A3, B3;
Matrix<double, 3, 4> B34;
class mSE3 : public Matrix<double, 4, 4, RowMajor>
{
public:
	mSE3() : Matrix<double, 4, 4, RowMajor>(Matrix<double, 4, 4, RowMajor>::Identity()) {}
	mSE3(const double& T11, const double& T12, const double& T13, const double& T14,
		const double& T21, const double& T22, const double& T23, const double& T24,
		const double& T31, const double& T32, const double& T33, const double& T34)
	{
		double* T = &((*this)(0));
		(*T) = T11;
		(*(T + 1)) = T12;
		(*(T + 2)) = T13;
		(*(T + 3)) = T14;

		(*(T + 4)) = T21;
		(*(T + 5)) = T22;
		(*(T + 6)) = T23;
		(*(T + 7)) = T24;

		(*(T + 8)) = T31;
		(*(T + 9)) = T32;
		(*(T + 10)) = T33;
		(*(T + 11)) = T34;
	}

	mSE3& operator = (const mSE3& operand)
	{
		double* T = &((*this)(0));
		const double* O = &((operand)(0));
		(*T) = (*O);
		(*(T + 1)) = (*(O + 1));
		(*(T + 2)) = (*(O + 2));
		(*(T + 3)) = (*(O + 3));

		(*(T + 4)) = (*(O + 4));
		(*(T + 5)) = (*(O + 5));
		(*(T + 6)) = (*(O + 6));
		(*(T + 7)) = (*(O + 7));

		(*(T + 8)) = (*(O + 8));
		(*(T + 9)) = (*(O + 9));
		(*(T + 10)) = (*(O + 10));
		(*(T + 11)) = (*(O + 11));
		return (*this);
	}

	mSE3& operator *= (const mSE3& operand)
	{
		double tmp1, tmp2, tmp3;

		double* T = &((*this)(0));
		const double* O = &((operand)(0));

		(*(T + 3)) += (*(T + 0)) * (*(O + 3)) + (*(T + 1)) * (*(O + 7)) + (*(T + 2)) * (*(O + 11));
		(*(T + 7)) += (*(T + 4)) * (*(O + 3)) + (*(T + 5)) * (*(O + 7)) + (*(T + 6)) * (*(O + 11));
		(*(T + 11)) += (*(T + 8)) * (*(O + 3)) + (*(T + 9)) * (*(O + 7)) + (*(T + 10)) * (*(O + 11));

		tmp1 = (*(T + 0)) * (*(O + 0)) + (*(T + 1)) * (*(O + 4)) + (*(T + 2)) * (*(O + 8));
		tmp2 = (*(T + 0)) * (*(O + 1)) + (*(T + 1)) * (*(O + 5)) + (*(T + 2)) * (*(O + 9));
		tmp3 = (*(T + 0)) * (*(O + 2)) + (*(T + 1)) * (*(O + 6)) + (*(T + 2)) * (*(O + 10));
		(*(T + 0)) = tmp1;
		(*(T + 1)) = tmp2;
		(*(T + 2)) = tmp3;

		tmp1 = (*(T + 4)) * (*(O + 0)) + (*(T + 5)) * (*(O + 4)) + (*(T + 6)) * (*(O + 8));
		tmp2 = (*(T + 4)) * (*(O + 1)) + (*(T + 5)) * (*(O + 5)) + (*(T + 6)) * (*(O + 9));
		tmp3 = (*(T + 4)) * (*(O + 2)) + (*(T + 5)) * (*(O + 6)) + (*(T + 6)) * (*(O + 10));
		(*(T + 4)) = tmp1;
		(*(T + 5)) = tmp2;
		(*(T + 6)) = tmp3;

		tmp1 = (*(T + 8)) * (*(O + 0)) + (*(T + 9)) * (*(O + 4)) + (*(T + 10)) * (*(O + 8));
		tmp2 = (*(T + 8)) * (*(O + 1)) + (*(T + 9)) * (*(O + 5)) + (*(T + 10)) * (*(O + 9));
		tmp3 = (*(T + 8)) * (*(O + 2)) + (*(T + 9)) * (*(O + 6)) + (*(T + 10)) * (*(O + 10));
		(*(T + 8)) = tmp1;
		(*(T + 9)) = tmp2;
		(*(T + 10)) = tmp3;

		return *this;
	}

	mSE3 operator * (const mSE3& operand) const
	{
		const double* T = &((*this)(0));
		const double* O = &((operand)(0));

		return mSE3((*(T + 0)) * (*(O + 0)) + (*(T + 1)) * (*(O + 4)) + (*(T + 2)) * (*(O + 8)),
			(*(T + 0)) * (*(O + 1)) + (*(T + 1)) * (*(O + 5)) + (*(T + 2)) * (*(O + 9)),
			(*(T + 0)) * (*(O + 2)) + (*(T + 1)) * (*(O + 6)) + (*(T + 2)) * (*(O + 10)),
			(*(T + 0)) * (*(O + 3)) + (*(T + 1)) * (*(O + 7)) + (*(T + 2)) * (*(O + 11)) + (*(T + 3)),

			(*(T + 4)) * (*(O + 0)) + (*(T + 5)) * (*(O + 4)) + (*(T + 6)) * (*(O + 8)),
			(*(T + 4)) * (*(O + 1)) + (*(T + 5)) * (*(O + 5)) + (*(T + 6)) * (*(O + 9)),
			(*(T + 4)) * (*(O + 2)) + (*(T + 5)) * (*(O + 6)) + (*(T + 6)) * (*(O + 10)),
			(*(T + 4)) * (*(O + 3)) + (*(T + 5)) * (*(O + 7)) + (*(T + 6)) * (*(O + 11)) + (*(T + 7)),

			(*(T + 8)) * (*(O + 0)) + (*(T + 9)) * (*(O + 4)) + (*(T + 10)) * (*(O + 8)),
			(*(T + 8)) * (*(O + 1)) + (*(T + 9)) * (*(O + 5)) + (*(T + 10)) * (*(O + 9)),
			(*(T + 8)) * (*(O + 2)) + (*(T + 9)) * (*(O + 6)) + (*(T + 10)) * (*(O + 10)),
			(*(T + 8)) * (*(O + 3)) + (*(T + 9)) * (*(O + 7)) + (*(T + 10)) * (*(O + 11)) + (*(T + 11)));
	}
} A, B;

void test3()
{
	A4 = (((B4 * B4).eval() * B4).eval() * B4).eval();
}

void test4()
{
	R32.noalias() = R31;
	R32 *= R31;
	R32 *= R31;
	R32 *= R31;
	//A = B * B * B * B;
	//A = B;
	//A *= B;
	//A *= B;
	//A *= B;
}

int main()
{
	Math::Real s, c;
	Math::fsincos(Math::PI/2, s, c);
	RR << 1, 2, 3, 4, 5, 6, 7, 8, 9;
	B3 << 1, 2, 3, 4, 5, 6, 7, 8, 9;
	B4 << 1, 2, 3, 4, 5, 6, 7, 8, 9;
	resultR.setIdentity();
	R.push_back(RR);
	R.push_back(RR);
	R.push_back(RR);
	P.push_back(Vector3d::Zero());
	P.push_back(Vector3d::Zero());
	P.push_back(Vector3d::Zero());
	cout << s << " " << c << endl;
	//Math::makeSineLookUpTable();
	//ofstream output("test.txt");
	//double theta = 0.0;
	//for (unsigned int i = 0; i < 1000000; i++)
	//{
	//	output << Math::fsin(theta) - sin(theta) << endl;
	//	theta += 0.00002;
	//}
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

	//omp_set_num_threads(10);
	//cout << Eigen::nbThreads() << endl;
	//SE3	A, B, C;
	//Matrix4d AA, BB, CC;
	//AA.setZero();
	//BB.setZero();
	//CC.setZero();
	//cout << C << endl;
	//cout << CC << endl;
	//double x = -10;
	//cout << std::setprecision(15) << Math::fsin(x) << endl;
	//cout << std::setprecision(15) << sin(x) << endl;
	//cout << std::setprecision(15) << abs(Math::fsin(x) - sin(x)) << endl;

	//PERFORM_TEST(SE3_1 = Exp(se3_1), 1e+8);
	//PERFORM_TEST(SE3_rovin = Math::SE3::Exp(se3_rovin1, se3_rovin2), 1e+8);
	//cout << SE3_1 << endl;
	//cout << std::setprecision(15) << SE3_rovin << endl;
	//cout << Log(SE3_1) << endl;
	//cout << Math::SE3::Log(SE3_rovin) << endl;
	//test1();
	//cout << SE3_1_sq << endl;
	//test3();
	//cout << SE3_rovin_sq << endl;
	test4();

	cout << "3" << endl;
	PERFORM_TEST(test3(), 1e+8);
	cout << "4" << endl;
	B << 1, 2, 3, 4, 5, 6, 7, 8, 9, 10, 11, 12;
	cout << B << endl;
	PERFORM_TEST(test4(), 1e+8);
	cout << A << endl;
	MB.setRandom();
	cout << "1" << endl;
	PERFORM_TEST(test1(), 1e+8);
	cout << "2" << endl;
	PERFORM_TEST(test2(), 1e+8);
	//cout << SE3_1_sq << endl;
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