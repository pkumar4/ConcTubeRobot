#define CATCH_CONFIG_MAIN
#include "catch.hpp"
#include "CTRCreateStructure.h"

using Eigen::MatrixXf;
using namespace Eigen;
/*
This is a test class for CTRCreateStructure.cpp
It matches the resultant CTR Structure by supplying a signature structure
and asserting the output with resultant structure of existing implementation
*/
TEST_CASE( "CTRCreateStructure", "createStructure" ) {
	REQUIRE( 1 == 1 );

	// creating a vector of CTRStructure 
	std::vector<CTRStructure> ctr_vec;

	// adding a segment of type 'balanced'
	CTRStructure ctr_struct;
	ctr_struct.type = "balanced";
	ctr_struct.u = 50.0;
	ctr_struct.u_min = 0.0;
	ctr_struct.u_max = 100.0;
	ctr_struct.c_len = 0.0550;
	ctr_struct.c_len_min = 0.0100;
	ctr_struct.c_len_max = 0.2000;
	ctr_struct.diameter = 0.0030;
	// adding an array of stiffness
	std::vector<double> k_vector;
	k_vector.push_back(5.0);
	k_vector.push_back(5.0);
	ctr_struct.k = k_vector;
	// adding an array of rotation
	std::vector<double> theta_vector;
	theta_vector.push_back(2.7274);
	theta_vector.push_back(2.7274);
	ctr_struct.theta = theta_vector;
	ctr_struct.phi = 0.0390;
	ctr_vec.push_back(ctr_struct);

	// adding a segment of type 'fixed'
	CTRStructure ctr_struct1;
	ctr_struct1.type = "fixed";
	ctr_struct1.u = 45.0;
	ctr_struct1.u_min = 0.0;
	ctr_struct1.u_max = 100.0;
	ctr_struct1.c_len = 0.0650;
	ctr_struct1.c_len_min = 0.0100;
	ctr_struct1.c_len_max = 0.2000;
	ctr_struct1.diameter = 0.0020;
	// adding an array of stiffness
	std::vector<double> k_vector1;
	k_vector1.push_back(1.0);
	ctr_struct1.k = k_vector1;
	// adding an array of rotation
	std::vector<double> theta_vector1;
	theta_vector1.push_back(-1.0);
	ctr_struct1.theta = theta_vector1;
	ctr_struct1.phi = 0.0640;
	ctr_vec.push_back(ctr_struct1);

// adding another segment of type 'fixed'
	CTRStructure ctr_struct2;
	ctr_struct2.type = "fixed";
	ctr_struct2.u = 1e-10;
	ctr_struct2.u_min = 1e-10;
	ctr_struct2.u_max = 1e-10;
	ctr_struct2.c_len = 10e-3;
	ctr_struct2.c_len_min = 10e-3;
	ctr_struct2.c_len_max = 200e-3;
	ctr_struct2.diameter = 9.0000e-04;
// adding an array of stiffness
	std::vector<double> k_vector2;
	k_vector1.push_back(0.5);
	ctr_struct2.k = k_vector2;
// adding an array of rotation
	std::vector<double> theta_vector2;
	theta_vector1.push_back(0.0);
	ctr_struct2.theta = theta_vector2;
	ctr_struct2.phi = 0.023;
	ctr_vec.push_back(ctr_struct2);

	vector<CTRSegment> ctr_seg_res;
	CTRCreateStructure ctr_create;
	// invoke the createStructure method
	ctr_seg_res = ctr_create.createStructure(ctr_vec);
	// match the first segment parameters with values of existing implementation
	REQUIRE(ctr_seg_res[0].id==1);
	REQUIRE(ctr_seg_res[0].u==0.0);
	REQUIRE(ctr_seg_res[0].length==0);
	REQUIRE(ctr_seg_res[0].k==5.0);
	REQUIRE(ctr_seg_res[0].diameter==0.003);
	REQUIRE(ctr_seg_res[0].phi==quiet_NaN());
	REQUIRE(ctr_seg_res[0].start==0);
	REQUIRE(ctr_seg_res[0].end==0);
	REQUIRE(ctr_seg_res[0].u_temp==0);
	REQUIRE(ctr_seg_res[0].k_temp==5.0);
	REQUIRE(ctr_seg_res[0].theta==2.7274);
// match the second segment parameters with values of existing implementation
	REQUIRE(ctr_seg_res[1].id==2);
	REQUIRE(ctr_seg_res[1].u==0.0);
	REQUIRE(ctr_seg_res[1].length==0);
	REQUIRE(ctr_seg_res[1].k==5.0);
	REQUIRE(ctr_seg_res[1].diameter==0.003);
	REQUIRE(ctr_seg_res[1].phi==quiet_NaN());
	REQUIRE(ctr_seg_res[1].start==0);
	REQUIRE(ctr_seg_res[1].end==0);
	REQUIRE(ctr_seg_res[1].u_temp==0);
	REQUIRE(ctr_seg_res[1].k_temp==5.0);
	REQUIRE(ctr_seg_res[1].theta==2.7274);
// match the third segment parameters with values of existing implementation.In total there will be
// 8 segments in the output with the input of 1 balanced and 2 fixed segment CTR Structure.
	REQUIRE(ctr_seg_res[2].id==3);
	REQUIRE(ctr_seg_res[2].u==50.0);
	REQUIRE(ctr_seg_res[2].length==0.055);
	REQUIRE(ctr_seg_res[2].k==5.0);
	REQUIRE(ctr_seg_res[2].diameter==0.003);
	REQUIRE(ctr_seg_res[2].phi==0.039);
	REQUIRE(ctr_seg_res[2].start==0);
	REQUIRE(ctr_seg_res[2].end==0.039);
	REQUIRE(ctr_seg_res[2].u_temp==50.0);
	REQUIRE(ctr_seg_res[2].k_temp==5.0);
	REQUIRE(ctr_seg_res[2].theta==2.7274);

	REQUIRE(ctr_seg_res[3].id==4);
	REQUIRE(ctr_seg_res[3].u==50.0);
	REQUIRE(ctr_seg_res[3].length==0.055);
	REQUIRE(ctr_seg_res[3].k==5.0);
	REQUIRE(ctr_seg_res[3].diameter==0.003);
	REQUIRE(ctr_seg_res[3].phi==0.039);
	REQUIRE(ctr_seg_res[3].start==0);
	REQUIRE(ctr_seg_res[3].end==0.039);
	REQUIRE(ctr_seg_res[3].u_temp==50.0);
	REQUIRE(ctr_seg_res[3].k_temp==5.0);
	REQUIRE(ctr_seg_res[3].theta==2.7274);

	REQUIRE(ctr_seg_res[4].id==5);
	REQUIRE(ctr_seg_res[4].u==0.0);
	REQUIRE(ctr_seg_res[4].length==0.038);
	REQUIRE(ctr_seg_res[4].k==1.0);
	REQUIRE(ctr_seg_res[4].diameter==0.002);
	REQUIRE(ctr_seg_res[4].phi==quiet_NaN());
	REQUIRE(ctr_seg_res[4].start==0);
	REQUIRE(ctr_seg_res[4].end==0.038);
	REQUIRE(ctr_seg_res[4].u_temp==0.0);
	REQUIRE(ctr_seg_res[4].k_temp==1.0);
	REQUIRE(ctr_seg_res[4].theta==-1);

	REQUIRE(ctr_seg_res[5].id==6);
	REQUIRE(ctr_seg_res[5].u==45.0);
	REQUIRE(ctr_seg_res[5].length==0.065);
	REQUIRE(ctr_seg_res[5].k==1.0);
	REQUIRE(ctr_seg_res[5].diameter==0.002);
	REQUIRE(ctr_seg_res[5].phi==0.064);
	REQUIRE(ctr_seg_res[5].start==0.038);
	REQUIRE(ctr_seg_res[5].end==0.103);
	REQUIRE(ctr_seg_res[5].u_temp==45.0);
	REQUIRE(ctr_seg_res[5].k_temp==1.0);
	REQUIRE(ctr_seg_res[5].theta==-1);

	REQUIRE(ctr_seg_res[6].id==7);
	REQUIRE(ctr_seg_res[6].u==0.0);
	REQUIRE(ctr_seg_res[6].length==0.103);
	REQUIRE(ctr_seg_res[6].k==0.05);
	REQUIRE(ctr_seg_res[6].diameter==0.002);
	REQUIRE(ctr_seg_res[6].phi==0.00009);
	REQUIRE(ctr_seg_res[6].start==0.038);
	REQUIRE(ctr_seg_res[6].end==0.103);
	REQUIRE(ctr_seg_res[6].u_temp==0.0);
	REQUIRE(ctr_seg_res[6].k_temp==0.05);
	REQUIRE(ctr_seg_res[6].theta==0);

	REQUIRE(ctr_seg_res[7].id==8);
	REQUIRE(ctr_seg_res[7].u==0.00000001);
	REQUIRE(ctr_seg_res[7].length==0.01);
	REQUIRE(ctr_seg_res[7].k==0.05);
	REQUIRE(ctr_seg_res[7].diameter==0.002);
	REQUIRE(ctr_seg_res[7].phi==0.00009);
	REQUIRE(ctr_seg_res[7].start==0.038);
	REQUIRE(ctr_seg_res[7].end==0.103);
	REQUIRE(ctr_seg_res[7].u_temp==0.0);
	REQUIRE(ctr_seg_res[7].k_temp==0.05);
	REQUIRE(ctr_seg_res[7].theta==0);


	
}