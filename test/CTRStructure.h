
#include <vector>
#include <cstdlib>
#include <string>

using namespace std;
struct CTRStructure{

	std::string type;

	double u;
	double u_min;
	double u_max;

	double c_len;
	double c_len_min;
	double c_len_max;

	std::vector<double> k;//stiffness;
	double diameter;
	std::vector<double> theta;
	double phi;

};