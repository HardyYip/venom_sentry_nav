// SPDX-License-Identifier: Apache-2.0
#include <gtest/gtest.h>
#include <pcl/point_cloud.h>
#include <small_gicp/pcl/pcl_point.hpp>
#include <small_gicp/pcl/pcl_registration.hpp>
#include <small_gicp/util/downsampling.hpp>
#include <cstring>
using Input=pcl::PointCloud<pcl::PointXYZ>;
using Output=pcl::PointCloud<pcl::PointCovariance>;
static Output::Ptr sample(const Input& p,double leaf=.02) {
 return small_gicp::voxelgrid_sampling<Input,Output>(p,leaf);
}
TEST(GlobalSampling, OneVoxelSpanningParallelBlocksHasOneFullCentroid) {
 Input p;double expected=0;
 for(int i=0;i<4097;++i){float x=.001f+float(i%10)*.001f;p.emplace_back(x,.001f,.001f);expected+=x;}
 auto out=sample(p);ASSERT_EQ(out->size(),1u);
 EXPECT_NEAR((*out)[0].x,expected/p.size(),1e-8);
}
TEST(GlobalSampling, NegativeAndPositiveVoxelsRemainDistinct) {
 Input p;for(float x:{-.041f,-.021f,-.001f,.001f,.021f,.041f})p.emplace_back(x,.001f,.001f);
 auto out=sample(p);ASSERT_EQ(out->size(),6u);
 for(std::size_t i=0;i<p.size();++i)EXPECT_FLOAT_EQ((*out)[i].x,p[i].x);
}
TEST(GlobalSampling, RosDoubleLeafPrecisionControlsBoundaryMembership) {
 Input p;p.emplace_back(-.025f,0.f,0.f);p.emplace_back(-.024f,0.f,0.f);
 EXPECT_EQ(sample(p,.025)->size(),2u);
 EXPECT_EQ(sample(p,static_cast<float>(.025))->size(),1u);
}
TEST(GlobalSampling, RepeatedFullVoxelSamplingKeepsCoordinatesIdentical) {
 Input p;for(int i=0;i<10007;++i)p.emplace_back((i%137-68)*.0031f,(i%31-15)*.0043f,(i%13)*.0017f);
 auto a=sample(p);auto b=sample(p);ASSERT_EQ(a->size(),b->size());
 for(std::size_t i=0;i<a->size();++i)EXPECT_EQ(std::memcmp((*a)[i].data,(*b)[i].data,4*sizeof(float)),0);
}
TEST(GlobalSampling, EmptyInputRemainsEmpty) {Input p;EXPECT_TRUE(sample(p)->empty());}
