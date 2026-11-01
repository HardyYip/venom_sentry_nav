// SPDX-FileCopyrightText: Copyright 2024 Kenji Koide
// SPDX-License-Identifier: MIT
// Bounded CAD correspondence selection, based on GICPFactor.
// The residual, Jacobian, LM optimizer and convergence criteria are retained.
#pragma once
#include <small_gicp/factors/gicp_factor.hpp>
#include <Eigen/Cholesky>
#include <array>
#include <algorithm>
#include <limits>
#include <cmath>
namespace small_gicp_relocalization {
struct MetricCorrespondenceFactor : small_gicp::GICPFactor {
 struct Setting {};
 static constexpr std::size_t kNeighborCap=8;
 static constexpr double kMaxExtraDistanceM=.025;
 explicit MetricCorrespondenceFactor(const Setting& = Setting()) {}
 template <typename Target,typename Source,typename Tree,typename Rejector>
 bool linearize(const Target& target,const Source& source,const Tree& tree,
  const Eigen::Isometry3d& T,std::size_t source_id,const Rejector& rejector,
  Eigen::Matrix<double,6,6>* H,Eigen::Matrix<double,6,1>* b,double* e) {
  namespace traits=small_gicp::traits;
  source_index=source_id;target_index=std::numeric_limits<std::size_t>::max();
  const Eigen::Vector4d p=T*traits::point(source,source_id);
  std::array<std::size_t,kNeighborCap> ids{};std::array<double,kNeighborCap> d2{};
  const auto n=traits::knn_search(tree,p,kNeighborCap,ids.data(),d2.data());
  if(!n || n>kNeighborCap)return false;
  double nearest=std::numeric_limits<double>::infinity();
  for(std::size_t j=0;j<n;++j)if(std::isfinite(d2[j])&&d2[j]>=0)nearest=std::min(nearest,d2[j]);
  if(!std::isfinite(nearest))return false;
  const double max_local=std::sqrt(nearest)+kMaxExtraDistanceM;
  double best=std::numeric_limits<double>::infinity();
  for(std::size_t j=0;j<n;++j){
   if(!std::isfinite(d2[j])||d2[j]<0||rejector(target,source,T,ids[j],source_id,d2[j]))continue;
   if(std::sqrt(d2[j])>max_local)continue;
   const Eigen::Matrix4d C=traits::cov(target,ids[j])+T.matrix()*traits::cov(source,source_id)*T.matrix().transpose();
   if(!C.allFinite())continue;
   Eigen::LDLT<Eigen::Matrix3d> decomposition(C.block<3,3>(0,0));
   if(decomposition.info()!=Eigen::Success||!decomposition.isPositive()||decomposition.vectorD().minCoeff()<=0)continue;
   const Eigen::Matrix3d M=decomposition.solve(Eigen::Matrix3d::Identity());
   const Eigen::Vector4d residual=traits::point(target,ids[j])-p;
   double cost=.5*residual.head<3>().dot(M*residual.head<3>());
   if(!M.allFinite()||!std::isfinite(cost)||cost<0||cost>=best)continue;
   best=cost;target_index=ids[j];mahalanobis.setZero();mahalanobis.block<3,3>(0,0)=M;
  }
  if(target_index==std::numeric_limits<std::size_t>::max())return false;
  const Eigen::Vector4d residual=traits::point(target,target_index)-p;
  Eigen::Matrix<double,4,6> J=Eigen::Matrix<double,4,6>::Zero();
  J.block<3,3>(0,0)=T.linear()*small_gicp::skew(traits::point(source,source_id).template head<3>());
  J.block<3,3>(0,3)=-T.linear();
  *H=J.transpose()*mahalanobis*J;*b=J.transpose()*mahalanobis*residual;
  *e=.5*residual.transpose()*mahalanobis*residual;
  if(!H->allFinite() || !b->allFinite() || !std::isfinite(*e)) {
   target_index=std::numeric_limits<std::size_t>::max();return false;
  }
  return true;
 }
};
}  // namespace small_gicp_relocalization
