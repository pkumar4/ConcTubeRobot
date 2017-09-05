/*
CTRCreateStructure

@author - Prashant Kumar
@date - 25.07.2017

*/
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
//using namespace std;



class CTRCreateStructure{

	public:
		vector<CTRSegment> createStructure(std::vector<CTRStructure> ctr_vector);

	
};
	vector<CTRSegment> CTRCreateStructure:: createStructure(std::vector<CTRStructure> ctr_vector){
		std::vector<CTRSegment> ctr_segments (10);
		int n_segments = 0;
		int i = 0;
		int j =0;
		int all_end = 0;
		MatrixXf s = MatrixXf::Zero(1,1);
		std::vector<CTRStructure>::iterator it = ctr_vector.begin();
		// find the number of segments
		for(it =ctr_vector.begin() ; it != ctr_vector.end();it++) {
	    
		    if(it->type.compare("balanced") == 0){ 
		    	n_segments = n_segments+4;
		    }
		    if(it->type.compare("fixed") == 0){
		    	n_segments = n_segments+2;
		    }
		    	
		}
		i = 0;
		//std::vector<int>::iterator it;
		//while(i<=n_segments){
		for(i=0;i<n_segments;i++){

			if(ctr_vector[j].type.compare("balanced") == 0){
				// create a new struct object and populate it
				CTRSegment seg1,seg2,seg3,seg4;
				seg1.id = i + 2;
				seg1.u = ctr_vector[j].u;
				seg1.u_temp = ctr_vector[j].u;
				seg1.length = ctr_vector[j].c_len;
				seg1.k = ctr_vector[j].k.at(0);
				seg1.k_temp = ctr_vector[j].k.at(0);
				seg1.diameter = ctr_vector[j].diameter;
				seg1.theta = ctr_vector[j].theta.at(1);// need to check the index

				//seg1.phi = std::max_element(0.0, std::min_element(ctr_vector[j].phi, ctr_vector[j].c_len));
				std::vector<double> min_phi;
				min_phi.push_back(ctr_vector[j].phi);
				min_phi.push_back(ctr_vector[j].c_len);
				double min_phi_val = *std::min_element(min_phi.begin(),min_phi.end());

				std::vector<double> max_phi;
				max_phi.push_back(0.0);
				max_phi.push_back(min_phi_val);
				seg1.phi = *std::max_element(max_phi.begin(),max_phi.end());

				//seg1.start = std::max_element(0.0, seg1.phi + all_end - seg1.length);
				std::vector<double> max_start;
				max_start.push_back(0.0);
				max_start.push_back(seg1.phi + all_end - seg1.length);
				seg1.start = *std::max_element(max_start.begin(),max_start.end());

				seg1.end = seg1.phi + all_end;
				// insert the new struct obj
				// TODO : need to check the index here - compare with line 56 in matlab
				ctr_segments.insert(ctr_segments.begin()+2,seg1);
				
				//Curved segment #2
				// Need code review to confirm if the below corresponds to line 69 of matlab code
				seg2.id = i+3;
				seg2.u = ctr_vector[j].u;
				seg2.u_temp = ctr_vector[j].u;
				seg2.length = ctr_vector[j].c_len;
				seg2.k = ctr_vector[j].k.at(1);
				seg2.k_temp = ctr_vector[j].k.at(1);
				seg2.diameter = ctr_vector[j].diameter;
				seg2.theta = ctr_vector[j].theta.at(1);// need to check the index
				//seg2.phi = std::max_element(0.0, std::min_element(ctr_vector[j].phi, ctr_vector[j].c_len));
				seg2.phi = seg1.phi;
				seg2.start = seg1.start;
				seg2.end = seg1.end;
				
				if((ctr_vector[j].k).size() ==2){
					seg2.k = ctr_vector[j].k.at(1);
				}

				// TODO : check if i+3 is equal to ctr_segments.begin()+3 : line 69 matlab
				ctr_segments.insert(ctr_segments.begin()+3,seg2);


				//Straight segment #1
				seg3.id = i;
				seg3.u = 0;
				seg3.u_temp = 0;
				//seg3.length = std::max_element(0.0, seg1.end - seg1.length);
				std::vector<double> seg3_max_len;
				seg3_max_len.push_back(0.0);
				seg3_max_len.push_back(seg1.end - seg1.length);
				seg3.length = *std::max_element(seg3_max_len.begin(),seg3_max_len.end());

				seg3.k = ctr_vector[j].k.at(1);
				seg3.k_temp = ctr_vector[j].k.at(1);
				seg3.diameter = ctr_vector[j].diameter;
				seg3.theta = ctr_vector[j].theta.at(1);// need to check the index
				seg3.phi = std::numeric_limits<double>::quiet_NaN();
				seg3.start = 0;
				seg3.end = seg3.length;
				// TODO : check if i is equal to ctr_segments.begin() : matlab 77
				ctr_segments.insert(ctr_segments.begin(),seg3);

				//Straight segment #2
				seg4.id = i+1;
				seg4.u = 0;
				seg4.u_temp = 0;
				//seg4.length = std::max_element(0.0, seg1.end - seg1.length);
				std::vector<double> seg4_max_len;
				seg4_max_len.push_back(0.0);
				seg4_max_len.push_back(seg1.end - seg1.length);
				seg4.length = *std::max_element(seg4_max_len.begin(),seg4_max_len.end());
				seg4.k = ctr_vector[j].k.at(0);
				seg4.k_temp = ctr_vector[j].k.at(0);
				seg4.diameter = ctr_vector[j].diameter;
				seg4.theta = ctr_vector[j].theta.at(1);// need to check the index
				seg4.phi = std::numeric_limits<double>::quiet_NaN();
				seg4.start = 0;
				seg4.end = seg3.length;
				if(ctr_vector[j].k.size() ==2){
					seg4.k = ctr_vector[j].k.at(1);
					seg4.k_temp = ctr_vector[j].k.at(1);
				}
				ctr_segments.insert(ctr_segments.begin()+1,seg4); // check if i+1 = begin()+1

				/*s.conservativeResize(NoChange, s.cols()+2);
				//s.col()
				s<<s,seg4.end,seg2.end;*/
				all_end = seg2.end; // corresponding to line no 99
				i = i+4;
			}

			if(ctr_vector[j].type.compare("fixed") == 0){
				CTRSegment seg5,seg6;
				//Curved segment
				seg5.id = i + 1;
				seg5.u= ctr_vector[j].u;
				seg5.u_temp   = ctr_vector[j].u;
				seg5.length   = ctr_vector[j].c_len;
				seg5.k        = ctr_vector[j].k.at(0); // this is different to what is in matlab code
				seg5.k_temp   = ctr_vector[j].k.at(0);// this is different to what is in matlab code
				seg5.diameter = ctr_vector[j].diameter;
				seg5.theta    = ctr_vector[j].theta.at(0);// this is different to what is in matlab code
				//seg5.phi      = std::max_element(0.0, std::min_element(ctr_vector[j].phi, ctr_vector[j].c_len));
				std::vector<double> seg5_min_phi;
				seg5_min_phi.push_back(ctr_vector[j].phi);
				seg5_min_phi.push_back(ctr_vector[j].c_len);
				std::vector<double> seg5_max_phi;
				seg5_max_phi.push_back(0.0);
				seg5_max_phi.push_back(*std::min_element(seg5_min_phi.begin(),seg5_min_phi.end()));
				seg5.phi = *std::max_element(seg5_max_phi.begin(),seg5_max_phi.end());

				//seg5.start    = std::max_element(0.0, seg5.phi + all_end - seg5.length);
				std::vector<double> seg5_max_start;
				seg5_max_start.push_back(0.0);
				seg5_max_start.push_back(seg5.phi + all_end - seg5.length);
				seg5.start = *std::max_element(seg5_max_start.begin(),seg5_max_start.end());
				seg5.end      = seg5.phi + all_end;
				// check below
				ctr_segments.insert(ctr_segments.begin()+1,seg5); 

				//Straight segment
				seg6.id       = i;
				seg6.u        = 0;
				seg6.u_temp   = 0;
				//seg6.length   = std::max_element(0.0, seg5.end - seg5.length);
				std::vector<double> seg6_max_len;
				seg6_max_len.push_back(0.0);
				seg6_max_len.push_back(seg5.end - seg5.length);
				seg6.length   =*std::max_element(seg6_max_len.begin(),seg6_max_len.end());
				
				seg6.k        = ctr_vector[j].k.at(0); // this is different to what is in matlab code
				seg6.k_temp   = ctr_vector[j].k.at(0);// this is different to what is in matlab code
				seg6.diameter = ctr_vector[j].diameter;
				seg6.theta    = ctr_vector[j].theta.at(0);// this is different to what is in matlab code
				seg6.phi      = std::numeric_limits<double>::quiet_NaN();;
				seg6.start    = 0;
				seg6.end      = seg6.length;
				// check if i corresponds to begin() below
				ctr_segments.insert(ctr_segments.begin(),seg5); 

     			/*s.conservativeResize(NoChange, s.cols()+2);
				s<<s,seg6.end,seg5.end;*/
				all_end = seg5.end;
				i = i+2;

			}
			j = j+1;
		}
		// line 144 of matlab will go here
	
		return ctr_segments;
	}

int main(){
	std::vector<CTRStructure> ctr_vec;

	CTRStructure ctr_struct;
	ctr_struct.type = "balanced";
	ctr_struct.u = 50.0;
	ctr_struct.u_min = 0.0;
	ctr_struct.u_max = 100.0;
	ctr_struct.c_len = 0.0550;
	ctr_struct.c_len_min = 0.0100;
	ctr_struct.c_len_max = 0.2000;
	ctr_struct.diameter = 0.0030;

	std::vector<double> k_vector;
	k_vector.push_back(5.0);
	k_vector.push_back(5.0);
	ctr_struct.k = k_vector;

	std::vector<double> theta_vector;
	theta_vector.push_back(2.7274);
	theta_vector.push_back(2.7274);
	ctr_struct.theta = theta_vector;
	ctr_struct.phi = 0.0390;

	ctr_vec.push_back(ctr_struct);

	CTRStructure ctr_struct1;
	ctr_struct1.type = "fixed";
	ctr_struct1.u = 45.0;
	ctr_struct1.u_min = 0.0;
	ctr_struct1.u_max = 100.0;
	ctr_struct1.c_len = 0.0650;
	ctr_struct1.c_len_min = 0.0100;
	ctr_struct1.c_len_max = 0.2000;
	ctr_struct1.diameter = 0.0020;

	std::vector<double> k_vector1;
	k_vector1.push_back(1.0);
	ctr_struct1.k = k_vector1;

	std::vector<double> theta_vector1;
	theta_vector1.push_back(-1.0);
	ctr_struct1.theta = theta_vector1;
	ctr_struct1.phi = 0.0640;

	ctr_vec.push_back(ctr_struct1);


	CTRStructure ctr_struct2;
	ctr_struct2.type = "fixed";
	ctr_struct2.u = 1e-10;
	ctr_struct2.u_min = 1e-10;
	ctr_struct2.u_max = 1e-10;
	ctr_struct2.c_len = 10e-3;
	ctr_struct2.c_len_min = 10e-3;
	ctr_struct2.c_len_max = 200e-3;
	ctr_struct2.diameter = 9.0000e-04;

	std::vector<double> k_vector2;
	k_vector1.push_back(0.5);
	ctr_struct2.k = k_vector2;

	std::vector<double> theta_vector2;
	theta_vector1.push_back(0.0);
	ctr_struct2.theta = theta_vector2;
	ctr_struct2.phi = 0.023;

	ctr_vec.push_back(ctr_struct2);


	vector<CTRSegment> ctr_seg_res;
	CTRCreateStructure ctr_create;
	ctr_seg_res = ctr_create.createStructure(ctr_vec);

	std::vector<CTRSegment>::iterator it = ctr_seg_res.begin();

	std::vector<CTRStructure>::iterator it1 = ctr_vec.begin();
	for(it1 = ctr_vec.begin();it1!=ctr_vec.end();it1++){
		cout<<it1->type<<endl;
	}
	
	int i =0;
	/*cout<<"Id "<<ctr_seg_res[i].id<<endl;
	cout<<"u "<<ctr_seg_res[i].u<<endl;
	cout<<"u_temp "<<ctr_seg_res[i].u_temp<<endl;
	cout<<"k "<<ctr_seg_res[i].k<<endl;
	cout<<"k_temp "<<ctr_seg_res[i].k_temp<<endl;

	cout<<"diameter "<<ctr_seg_res[i].diameter<<endl;
	cout<<"theta "<<ctr_seg_res[i].theta<<endl;
	cout<<"length "<<ctr_seg_res[i].length<<endl;
	cout<<"phi "<<ctr_seg_res[i].phi<<endl;
	cout<<"start "<<ctr_seg_res[i].start<<endl;
	cout<<"end "<<ctr_seg_res[i].end<<endl;*/

	for(it =ctr_seg_res.begin() ; it != ctr_seg_res.end();it++ ) {
		//cout<<ctr_seg_res[i].id<<endl;
		cout<<"Id "<<it->id<<" ";
		cout<<"u "<<it->u<<" ";
		cout<<"length "<<it->length<<" ";
		cout<<"k "<<it->k<<" ";
		cout<<"diameter "<<it->diameter<<" ";
		cout<<"theta "<<it->theta<<" ";
		cout<<"phi "<<it->phi<<" ";
		cout<<"start "<<it->start<<" ";
		cout<<"end "<<it->end<<" ";
		cout<<"u_temp "<<it->u_temp<<" ";
		cout<<"k_temp "<<it->k_temp<<" ";
		
		cout<<endl;
	}
	return 1;
}
