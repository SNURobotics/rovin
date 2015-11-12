#include <iostream>
#include <rovin/Model/Joint.h>
#include <rovin/Model/RevoluteJoint.h>
#include <rovin/Model/PrismaticJoint.h>

using namespace std;
using namespace rovin::Model;
int main()
{
	//Joint A("A",3);
	ScrewJoint A("ZYX", 3);
	RevoluteJoint R("R");
	PrismaticJoint P("P");
	rovin::Math::Vector3 state;
	state << 0, rovin::Math::PI/4, 0;

	//cout<<A.getJacobian(state)<<endl;
	rovin::Math::VectorX state1(1);
	state1 << 2.3;
	cout << P.getTransform(state1) << endl;

	
	
	return 0;
}