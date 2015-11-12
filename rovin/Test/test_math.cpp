#include <iostream>
#include <conio.h>
#include <cmath>
#include <time.h>

#include <rovin/Math/LieGroup.h>
#include <rovin/Model/Assembly.h>
#include <rovin/Dynamics/System.h>

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
	VectorXd a(1, 0);

	_getch();
	return 0;
}