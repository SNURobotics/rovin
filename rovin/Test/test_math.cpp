#include <iostream>
#include <conio.h>
#include <cmath>
#include <time.h>

#include <rovin/Math/LieGroup.h>
#include <rovin/Math/Constraint.h>
#include <rovin/Model/Assembly.h>

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
	_getch();
	return 0;
}