#include <iostream>
#include <vector>
#include <unsupported/Eigen/MatrixFunctions>
#include <Eigen/Dense>

#include "CTRSegment.h"

using Eigen::MatrixXf;
using namespace Eigen;
using namespace std;

struct estimate_curve{
      // memebers that are returned by estimate_curve()

};

class CTRKinematics{
      
      public:
            void estimate_curve(const Ref<const MatrixXf> &u,double L,const Ref<const MatrixXf> &E_start, int n_divs);
            void curve2matrix(const Ref<const MatrixXf> &u, double ds);
            void curve2twist(const Ref<const MatrixXf> &u, double ds);
            void twist2matrix(const Ref<const MatrixXf> &Z, double theta);
            void processCTRKinematics();
            MatrixXf Rz(double theta);
            MatrixXf find(std::vector<CTRSegment> ctr_vector,double s_i);
    
};

// MatrixXf u , double L, MatrixXf E_start, int n_divs
void CTRKinematics::estimate_curve(const Ref<const MatrixXf> &u,double L,const Ref<const MatrixXf> &E_start, int n_divs){

      MatrixXf E_total = E_start;
      int n_pt = n_divs+1;
      MatrixXf x = MatrixXf::Zero(n_pt,1);
      MatrixXf y = MatrixXf::Zero(n_pt,1);
      MatrixXf z = MatrixXf::Zero(n_pt,1);

      MatrixXf E = MatrixXf::Random(); // will be replaced with line 94 matlab when curve2matrix is ready

      for(int i = 0; i<n_pt;i++){

            x(0,i) = E_total(1,4);
            y(0,i) = E_total(2,4);
            z(0,i) = E_total(3,4);

            E_total = E_total*E;
      }
      //line 106 matlab
      MatrixXf E_end = E_start*E.pow(n_divs);
}

void CTRKinematics::curve2matrix(const Ref<const MatrixXf> &u, double ds){


}

void CTRKinematics::curve2twist(const Ref<const MatrixXf> &u, double ds){

      double un = u.norm();
      MatrixXf w(3,1);
      MatrixXf v(3,1);
      double theta;
      if(un==0){
            w = MatrixXf::Zero(3,1);
            w = w.transpose();
            v(0,0) = 0;
            v(1,0) = 0;
            v(2,0) = 1;
            v = v.transpose();
            theta = ds;
      }
      else{
            w = u/un;
            v(0,0) = 0;
            v(1,0) = 0;
            v(2,0) = 1;
            v = v.transpose();
            theta = un*ds;
      }
      MatrixXf Z(v.rows()+w.rows(),1);// the second arg should not be fixed. Looking into it. 
                                     //  Z is composed on two 3x1 matrices
      Z<<v,w;
}

void CTRKinematics::twist2matrix(const Ref<const MatrixXf> &Z, double theta){
      MatrixXf M = MatrixXf::Identity(4,4);
      MatrixXf I3 = MatrixXf::Identity(3,3);
      MatrixXf v(3,1);
      //v = Z.head(3);
      v<<Z(0,1),Z(1,1),Z(2,1);
      MatrixXf w(3,1);
      //w = Z.tail(3);
      w<<Z(3,1),Z(4,1),Z(5,1);
      if(w.norm()==0){
            // line 145 - need to understand M(1:3,4)
      }
      else{
            MatrixXf W(3,3);
            W(0,0) = 0;
            W(0,1) = -1*w(2,0);
            W(0,2) = w(1,0);
            W(1,0) = w(2,0);
            W(1,1) = 0;
            W(1,2) = -1*w(0,0);
            W(2,0) = -1*w(1,0);
            W(2,1) = w(0,0);
            W(2,2) = 0;
            MatrixXf R = I3+ W*sin(theta)+(W.pow(2))*(1-cos(theta));
           // line 152 need to understand

            // cross product
            MatrixXf z = MatrixXf::Zero(3,1);
            z(0,0) = w(1,0)*v(2,0) - w(2,0)*v(1,0);
            z(1,0) = w(2,0)*v(0,0) - w(0,0)*v(2,0);
            z(2,0) = w(0,0)*v(1,0) - w(1,0)*v(0,0);

            MatrixXf p = (I3- R)*z +theta*(w*w.transpose())*v;

            // line 162 needs to be done


      }

}

void CTRKinematics::processCTRKinematics(){
      const int n_divs = 30;
      std::vector<CTRSegment> ctr; // will be initialised by the value returned by CTRCreateStructure
      MatrixXf s = MatrixXf::Random(1,5);             // will be initialised by the value returned by CTRCreateStructure
      int segms = s.size()-1;// line 34
      int pt_idx = 1;
      double k=0.0;

      // line 41 -43 will not be rquired as curve is already added in the struct CTRsegment.

      MatrixXf P = MatrixXf::Zero(4,(n_divs+1)*segms);
      double L;
      for(int i =0; i<segms;i++){
            //line 51: need to write the logic for find(condition) as this is not available in Eigen
            //MatrixXf idx(1,4);
            //idx<<3,4,5,7;
            const MatrixXf& idx = find(ctr,s(0,i));
            L = s(0,i+1) -s(0,i);
            for(int j =0; j<idx.size();j++){
                  k = k+ctr[j].k;
            }
            MatrixXf u = MatrixXf::Zero(2,1);
            for(int t =0;t<idx.size();t++){
                  MatrixXf temp(2,1);
                  temp<<0,ctr[idx(t)].u;
                  //u = u+ctr[idx(t)].k*Rz(ctr[idx(t)].theta)*temp; this line(60) is broken into below
                 CTRSegment a = ctr.at(idx(t));
                 double k_temp = a.k;
                 CTRSegment b = ctr.at(idx(t));
                 double theta_temp = b.theta;
                 const MatrixXf& y = Rz(theta_temp);
                 u = u+k_temp*y*temp;
            }
            u = u/k;
            
             // 65-72 goes here    
      }
      //76,78 goes here;

}

MatrixXf CTRKinematics::Rz(double theta){
      MatrixXf M(2,2);
      M(0,0) = cos(theta);
      M(0,1)= -1*sin(theta);
      M(1,0) = sin(theta);
      M(1,1) = cos(theta);
      return M;

}

MatrixXf CTRKinematics::find(std::vector<CTRSegment> ctr_vector,double s_i){
      MatrixXf idx;
      int i = 0;
      std::vector<CTRSegment>::iterator it = ctr_vector.begin();
      for(it =ctr_vector.begin() ; it < ctr_vector.end();it++,i++ ) {
            if(s_i >= ctr_vector[i].start && s_i < ctr_vector[i].end){
                  idx<<i;
            }
      }

      cout<<idx<"";
      return idx;
}
int main(){
      CTRKinematics obj;
      obj.processCTRKinematics();
      return 0;
}