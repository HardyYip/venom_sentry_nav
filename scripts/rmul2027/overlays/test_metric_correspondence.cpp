// SPDX-License-Identifier: MIT
#include <gtest/gtest.h>
#include <limits>
#include <pcl/point_cloud.h>
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/ann/kdtree_omp.hpp>
#include <small_gicp/registration/rejector.hpp>
#include "small_gicp_relocalization/metric_correspondence.hpp"
using Cloud=pcl::PointCloud<pcl::PointCovariance>;
using Tree=small_gicp::KdTree<Cloud>;
using Metric=small_gicp_relocalization::MetricCorrespondenceFactor;
struct CountedTree {std::shared_ptr<Tree> tree;mutable std::size_t queries{0},requested{0};};
namespace small_gicp {namespace traits {
template<>struct Traits<CountedTree> {
 static std::size_t knn_search(const CountedTree& t,const Eigen::Vector4d& p,std::size_t k,std::size_t* ids,double* d) {
  ++t.queries;t.requested=k;return small_gicp::traits::knn_search(*t.tree,p,k,ids,d);
 }
};
}}
static pcl::PointCovariance point(float x,float y,float z) {
 pcl::PointCovariance p;p.getVector4fMap()<<x,y,z,1.f;p.cov.setZero();p.cov.block<3,3>(0,0)=Eigen::Matrix3f::Identity()*.001f;return p;
}
struct Scene {
 Cloud::Ptr target{new Cloud},source{new Cloud};std::shared_ptr<Tree> tree;
 Eigen::Matrix<double,6,6> H;Eigen::Matrix<double,6,1> b;double e{0};
 explicit Scene(float alternative_y=.02f) {
  source->push_back(point(0,0,0));target->push_back(point(.01f,0,0));
  target->push_back(point(0,alternative_y,0));(*target)[1].cov(1,1)=1.f;
  rebuild();
 }
 void rebuild(){tree=std::make_shared<Tree>(target,small_gicp::KdTreeBuilderOMP(1));}
 template<class Factor,class Search>bool run(Factor& f,const Search& search) {
  small_gicp::DistanceRejector rejector;rejector.max_dist_sq=3.;
  return f.linearize(*target,*source,search,Eigen::Isometry3d::Identity(),0,rejector,&H,&b,&e);
 }
 template<class Factor>bool run(Factor& f){return run(f,*tree);}
};
TEST(MetricCorrespondence, SelectionUsesTheSameWeightedResidualAsTheSolve) {
 Scene s;small_gicp::GICPFactor old;Metric next;
 ASSERT_TRUE(s.run(old));EXPECT_EQ(old.target_index,0u);double legacy=s.e;
 ASSERT_TRUE(s.run(next));EXPECT_EQ(next.target_index,1u);EXPECT_LT(s.e,legacy);
}
TEST(MetricCorrespondence, AdditionalCandidateDistanceIsStrictlyLocal) {
 Scene s(.05f);Metric factor;ASSERT_TRUE(s.run(factor));EXPECT_EQ(factor.target_index,0u);
 EXPECT_DOUBLE_EQ(Metric::kMaxExtraDistanceM,.025);
}
TEST(MetricCorrespondence, OriginalSquaredDistanceRejectorRemainsEffective) {
 Scene s;for(auto& p:*s.target)p.x=2.f;s.rebuild();Metric factor;
 EXPECT_FALSE(s.run(factor));EXPECT_FALSE(factor.inlier());
}
TEST(MetricCorrespondence, NonpositiveAndNonfiniteCovarianceFailClosed) {
 for(float value:{0.f,-.001f,std::numeric_limits<float>::quiet_NaN()}) {
  Scene s;for(auto& p:*s.source){p.cov.setZero();p.cov.block<3,3>(0,0)=Eigen::Matrix3f::Identity()*value;}
  for(auto& p:*s.target){p.cov.setZero();p.cov.block<3,3>(0,0)=Eigen::Matrix3f::Identity()*value;}
  Metric factor;EXPECT_FALSE(s.run(factor));EXPECT_FALSE(factor.inlier());
 }
}
TEST(MetricCorrespondence, PublishedCostIsNotReplacedByAPlaceholder) {
 Scene s;Metric factor;ASSERT_TRUE(s.run(factor));EXPECT_GT(s.e,0);
 EXPECT_NEAR(s.e,factor.error(*s.target,*s.source,Eigen::Isometry3d::Identity()),1e-12);
 EXPECT_TRUE(s.H.allFinite());EXPECT_TRUE(s.b.allFinite());EXPECT_GT(s.b.norm(),0);
}
TEST(MetricCorrespondence, EachPointHasOneBoundedEightNeighborSearch) {
 Scene s;Metric factor;CountedTree tree{s.tree};
 ASSERT_TRUE(s.run(factor,tree));EXPECT_EQ(tree.queries,1u);EXPECT_EQ(tree.requested,8u);EXPECT_EQ(Metric::kNeighborCap,8u);
}
