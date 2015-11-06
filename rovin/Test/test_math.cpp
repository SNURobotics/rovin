#include <iostream>
#include <conio.h>

#include <rovin/Math/LieGroup.h>

using namespace std;

int main()
{
	cout << "ROVIN È­ÀÌÆÃ!" << endl;

	rovin::Math::SE3 T(rovin::Math::SO3::RotX(3.141592));
	cout << T << endl;

	_getch();
	return 0;
}