#pragma once

#include <vector>
#include <rovin/Math/Constant.h>

using namespace rovin;
using namespace rovin::Math;
class GaussianQuadrature
{
public:
	const enum Scheme
	{
		LG,
		LGR,
		LGL,
	};

	//	Warning: Constructor has high computational cost. Reduce construction of this class as possible as you can.
	GaussianQuadrature(unsigned int num_of_points, Real initialTime, Real finalTime);

	void	setTimeInterval(Real initialTime, Real finalTime);

	const VectorX&	getQueryPoints() const { return _t; }
	const VectorX&	getWeights() const { return _w; }
	const Real	evalIntegration(const VectorX& functionVal) const;

//private:

	void	_calcCoeffs();

	unsigned int _N;
	Real		_t0, _tf;
	//	sampling points from t0 to tf.
	VectorX		_t;
	//	sampling points in [-1, 1].
	VectorX		_x;
	//	weight for integration.
	VectorX		_w;
};