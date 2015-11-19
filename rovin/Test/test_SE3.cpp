#include "rovin/Math/SE3.h"
#include "Rmatrix/rmatrix3.h"
#include "LieGroup/LieGroup.h"
#include <rovin/utils/Diagnostic.h>
#include <iostream>

using namespace std;

rovin::Math::SE3 A, B, C, D, E, F;
SE3 SE3_1, SE3_2, SE3_3, SE3_1_sq;
//Eigen::Matrix4d AA, BB, CC, DD;
Eigen::Matrix<double, 4, 4, Eigen::RowMajor> AA, BB, CC, DD;
Eigen::Matrix<double, 4, 4, Eigen::RowMajor> AA34;

typedef Eigen::Matrix<double, 2, 2, Eigen::RowMajor> Matrix2Row;
typedef Eigen::Matrix<double, 2, 2, Eigen::ColMajor> Matrix2Col;

typedef Eigen::Matrix<double, 4, 4, Eigen::RowMajor> Matrix4Row;
typedef Eigen::Matrix<double, 4, 4, Eigen::ColMajor> Matrix4Col;
typedef Eigen::Matrix<double, 3, 4, Eigen::RowMajor> Matrix3Row;
class MyMatrix : public Matrix4Row
{
public:
	MyMatrix() : Matrix4Row() { }
	MyMatrix(const Matrix4Row& eMtr) : Matrix4Row(eMtr) { }

	template<typename OtherDerived>
	MyMatrix& operator= (const Eigen::MatrixBase<OtherDerived>& other)
	{
		this->block<3, 4>(0, 0).Base::operator=(other);
		return *this;
	}

	template<typename OtherDerived>
	MyMatrix& operator=(const Eigen::ReturnByValue<OtherDerived>& other)
	{
		this->Base::operator=(other);
		return *this;
	}

	const Eigen::ProductReturnType<Eigen::Block<const Matrix4Row, 3, 4>, Matrix4Row>::Type
		operator* (const MyMatrix& other) const
	{
		return ((*this).block<3, 4>(0, 0)).Base::operator*(other);
	}

	template<typename Derived> friend
		const typename Eigen::ProductReturnType<Derived, Matrix4Row>::Type
		operator* (const Eigen::MatrixBase<Derived>& m1, const MyMatrix& m2)
	{
		return m1 * static_cast<const Matrix4Row&>(m2);
	}
} AAA, BBB, CCC;

void test1()
{
	AAA = ((((BBB * BBB).eval() * BBB).eval() * BBB).eval() * BBB).eval();
//	A *= A;
}
void test2()
{
	AA34 = ((((BB * BB).eval() * BB).eval() * BB).eval() * BB).eval();
//	AA *= AA;
}
void test3()
{
	SE3_1_sq = SE3_1_sq * SE3_3 * SE3_2 * SE3_2 * SE3_2;
//	SE3_1_sq *= SE3_1_sq;
}

int main()
{
	//Matrix2Row SY;
	//Matrix2Col KJ;

	//SY << 1, 2, 3, 4;
	//KJ << 4, 3, 2, 1;

	//cout << SY * KJ << endl;

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

	BBB(0, 0) = SE3_1(0, 0);
	BBB(0, 1) = SE3_1(0, 1);
	BBB(0, 2) = SE3_1(0, 2);
	BBB(0, 3) = SE3_1(0, 3);

	BBB(1, 0) = SE3_1(1, 0);
	BBB(1, 1) = SE3_1(1, 1);
	BBB(1, 2) = SE3_1(1, 2);
	BBB(1, 3) = SE3_1(1, 3);

	BBB(2, 0) = SE3_1(2, 0);
	BBB(2, 1) = SE3_1(2, 1);
	BBB(2, 2) = SE3_1(2, 2);
	BBB(2, 3) = SE3_1(2, 3);

	BBB(3, 0) = SE3_1(3, 0);
	BBB(3, 1) = SE3_1(3, 1);
	BBB(3, 2) = SE3_1(3, 2);
	BBB(3, 3) = SE3_1(3, 3);

	D = C = E = F = B;
	//BBB = B;

	cout << (Matrix4Row)BBB << endl;
	cout << B << endl;
	cout << B*B*B*B*B << endl;
	PERFORM_TEST(test2(), 1e+8);
	PERFORM_TEST(test3(), 1e+8);
	PERFORM_TEST(test1(), 1e+8);
	cout << (Matrix4Row)AAA << endl;
	cout << SE3_1 * SE3_1 * SE3_1 * SE3_1 * SE3_1 << endl;

	return 0;
}