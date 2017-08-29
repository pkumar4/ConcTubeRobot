#include <vector>
#include <cstdlib>

struct CTRSegment{
	int id;
	double u; //curvature
	double u_temp; // temporary variable for u
	double length; //arclength
	double k;//stiffness
	double k_temp; // temporary variable for k
	double diameter;//
	double theta;
	double phi;
	double start;//arc start
	double end;//arc end
	double curve;// holds the curve
};