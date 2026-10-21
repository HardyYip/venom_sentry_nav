#include <cmath>
#include <iostream>
#include "IKFoM/IKFoM_toolkit/esekfom/esekfom.hpp"

template<int N> struct State {
  using scalar = double;
  enum {DOF=N, DIM=N};
  Eigen::Matrix<double,N,1> value = Eigen::Matrix<double,N,1>::Zero();
  void build_S2_state() {}
  void build_SO3_state() {}
  void build_vect_state() {}
  void build_SEN_state() {}
  void boxplus(const Eigen::Matrix<double,N,1> & dx) {value += dx;}
};
Eigen::MatrixXd jacobian;
Eigen::VectorXd residual;
double variance;
template<int N> void measurement(State<N> &, Eigen::Matrix3d, Eigen::Matrix3d,
  esekfom::dyn_share_modified<double> & share) {
  share.h_x=jacobian; share.z=residual; share.M_Noise=variance; share.valid=true;
}
template<int N> bool check(int count, double noise) {
  variance=noise; jacobian.resize(count,12); residual.resize(count);
  for(int i=0;i<count;++i) {
    residual(i)=std::cos(i*.27);
    for(int j=0;j<12;++j) jacobian(i,j)=std::sin((i+1)*(j+1)*.23)+.1*(i==j);
  }
  Eigen::Matrix<double,N,N> covariance=Eigen::Matrix<double,N,N>::Identity()*.4;
  for(int i=0;i<N;++i) for(int j=0;j<N;++j)
    covariance(i,j)+=.02*std::cos(i*.2)*std::cos(j*.2);
  esekfom::esekf<State<N>,N> filter(State<N>{},covariance);
  filter.init_dyn_share_modified_2h(nullptr,nullptr,measurement<N>);
  Eigen::MatrixXd full=Eigen::MatrixXd::Zero(count,N); full.leftCols(12)=jacobian;
  Eigen::MatrixXd innovation=full*covariance*full.transpose()+noise*Eigen::MatrixXd::Identity(count,count);
  Eigen::MatrixXd gain=covariance*full.transpose()*innovation.inverse();
  Eigen::VectorXd expected=gain*residual;
  Eigen::MatrixXd expected_cov=covariance-gain*full*covariance;
  const bool valid=filter.update_iterated_dyn_share_modified();
  const double state_error=(filter.get_x().value-expected).norm();
  const double cov_error=(filter.get_P()-expected_cov).norm();
  bool ok=valid && state_error<1e-7 && cov_error<1e-7;
  std::cout<<"{\"state_dof\":"<<N<<",\"measurements\":"<<count
    <<",\"variance\":"<<noise<<",\"state_error\":"<<state_error
    <<",\"covariance_error\":"<<cov_error<<",\"passed\":"<<(ok?"true":"false")<<"}\n";
  return ok;
}
int main() {
  bool ok=true;
  for(double noise:{.001,.1,1.,4.}) {
    for(int count:{1,12,23,24,25,29,30,31,64,96}) {
      ok=check<24>(count,noise)&&ok; ok=check<30>(count,noise)&&ok;
    }
  }
  return ok?0:1;
}
