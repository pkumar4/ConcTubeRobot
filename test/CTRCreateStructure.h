#include "CTRStructure.h"
#include "CTRSegment.h"

#include <cmath>
#include <limits>
#include <vector>
#include <iterator>
#include <cstdlib>
#include <iostream>
#include <string>
#include <algorithm>
#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/Dense>

using Eigen::MatrixXf;
using namespace Eigen;

class CTRCreateStructure{

	public:
		vector<CTRSegment> createStructure(std::vector<CTRStructure> ctr_vector);

	
};