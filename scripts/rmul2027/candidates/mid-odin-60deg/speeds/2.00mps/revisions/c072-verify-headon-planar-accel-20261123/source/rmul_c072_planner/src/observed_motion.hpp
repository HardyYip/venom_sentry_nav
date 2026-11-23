// Copyright 2026 HardyYip contributors
// SPDX-License-Identifier: Apache-2.0
#ifndef PB_OMNI_OBSERVED_MOTION_HPP_
#define PB_OMNI_OBSERVED_MOTION_HPP_
#include <algorithm>
#include <cmath>
#include <cstddef>
#include <limits>
#include <vector>
namespace pb_omni_pid_pursuit_controller::observed_motion
{
struct Point {double x{0.0}, y{0.0};};
inline double norm(Point p) {return std::hypot(p.x,p.y);}
struct Cluster {Point center; double radius; std::vector<Point> points;};
struct Track {int id; double stamp; Point center, velocity; double radius; bool active;};
class Tracker
{
  struct State {Track track; Point raw; std::vector<Point> velocities; Cluster observation; double motion_since{-1.}; int good{0},quiet{0};};
  std::vector<State> states_;
  double last_stamp_{-1.0};
  int next_id_{0};
  bool sparse_association_{false};
  bool retain_unconfirmed_{false};
  static double median(std::vector<double> a) {std::sort(a.begin(),a.end());return a[a.size()/2];}
  static bool translation(const std::vector<Point> & previous,const std::vector<Point> & current,Point & shift,bool sparse=false)
  {
    if(previous.size()<3 || current.size()<3) {
      if(!sparse || previous.empty() || current.empty()) {return false;}
      double best=std::numeric_limits<double>::infinity();
      for(auto p:current) {for(auto q:previous) {const double d=std::hypot(p.x-q.x,p.y-q.y);if(d<best) {best=d;shift={p.x-q.x,p.y-q.y};}}}
      return std::isfinite(best);
    }
    auto axis=[](const std::vector<Point> & points) {
      Point mean;for(auto p:points) {mean.x+=p.x;mean.y+=p.y;}mean.x/=points.size();mean.y/=points.size();
      double xx=0.,xy=0.,yy=0.;for(auto p:points) {const double x=p.x-mean.x,y=p.y-mean.y;xx+=x*x;xy+=x*y;yy+=y*y;}
      const double angle=.5*std::atan2(2*xy,xx-yy),delta=std::hypot(xx-yy,2*xy);
      return std::vector<double>{std::cos(angle),std::sin(angle),(xx+yy+delta)/(xx+yy-delta+1e-12)};
    };
    const auto a=axis(previous),b=axis(current);
    if(a[2]>4. && b[2]>4. && std::abs(a[0]*b[0]+a[1]*b[1])<.9) {return false;}
    std::vector<double> dx,dy;
    for(auto point:current) {
      double best=1e30;Point displacement;
      for(auto old:previous) {const double x=point.x-old.x,y=point.y-old.y,d=x*x+y*y;if(d<best) {best=d;displacement={x,y};}}
      dx.push_back(displacement.x);dy.push_back(displacement.y);
    }
    shift={median(dx),median(dy)};
    auto overlap=[](const std::vector<Point> & from,const std::vector<Point> & to,Point translation) {
      std::size_t inliers=0;
      for(auto p:from) {double nearest=1e30;for(auto q:to) {nearest=std::min(nearest,std::hypot(p.x-translation.x-q.x,p.y-translation.y-q.y));}if(nearest<=.05) {++inliers;}}
      return static_cast<double>(inliers)/from.size();
    };
    return overlap(current,previous,shift)>=.65 && overlap(previous,current,{-shift.x,-shift.y})>=.5;
  }
public:
  void setSparseAssociation(bool enabled) {sparse_association_=enabled;}
  void setUnconfirmedRetention(bool enabled) {retain_unconfirmed_=enabled;}
  void clear() {states_.clear();last_stamp_=-1.;}
  double lastStamp() const {return last_stamp_;}
  static std::vector<Cluster> clusters(const std::vector<Point> & points,bool sparse=false)
  {
    std::vector<Cluster> result;std::vector<bool> seen(points.size(),false);
    for(std::size_t i=0;i<points.size();++i) {
      if(seen[i]) {continue;}
      std::vector<std::size_t> group{i};seen[i]=true;
      for(std::size_t k=0;k<group.size();++k) {
        for(std::size_t j=0;j<points.size();++j) {
          if(!seen[j] && std::hypot(points[group[k]].x-points[j].x,points[group[k]].y-points[j].y)<=.15) {seen[j]=true;group.push_back(j);}
        }
      }
      if(group.size()<(sparse?1u:3u)) {continue;}
      Point low{1e30,1e30},high{-1e30,-1e30};
      for(auto j:group) {low.x=std::min(low.x,points[j].x);low.y=std::min(low.y,points[j].y);high.x=std::max(high.x,points[j].x);high.y=std::max(high.y,points[j].y);}
      if(std::max(high.x-low.x,high.y-low.y)>.8) {continue;}
      Point center{.5*(low.x+high.x),.5*(low.y+high.y)};double radius=.05;
      for(auto j:group) {radius=std::max(radius,std::hypot(points[j].x-center.x,points[j].y-center.y));}
      Cluster cluster{center,radius,{}};for(auto j:group) {cluster.points.push_back(points[j]);}
      result.push_back(std::move(cluster));
      if(result.size()>=32) {break;}
    }
    return result;
  }
  void update(double stamp,const std::vector<Cluster> & input)
  {
    if(!std::isfinite(stamp)) {return;}
    if(last_stamp_>=0 && stamp<=last_stamp_) {if(stamp<last_stamp_-.1) {clear();} else {return;}}
    if(last_stamp_>=0 && stamp-last_stamp_>1.) {clear();}
    std::vector<State> next;std::vector<bool> used(states_.size(),false);
    std::vector<std::size_t> order(input.size());for(std::size_t i=0;i<order.size();++i) {order[i]=i;}
    if(sparse_association_) {std::stable_sort(order.begin(),order.end(),[&input](std::size_t a,std::size_t b) {return input[a].points.size()>input[b].points.size();});}
    for(auto index:order) {const auto & c=input[index];
      int match=-1;double nearest=1e30;
      for(std::size_t j=0;j<states_.size();++j) {
        const auto & previous=states_[j].track;const double dt=stamp-previous.stamp;
        if(used[j] || dt<=0. || dt>.5) {continue;}
        const double distance=std::hypot(c.center.x-states_[j].observation.center.x,c.center.y-states_[j].observation.center.y);
        if(distance<.15+3.5*dt && distance<nearest) {match=static_cast<int>(j);nearest=distance;}
      }
      if(match<0) {next.push_back({Track{next_id_++,stamp,c.center,{},c.radius,false},{},{},c,-1.,0,0});continue;}
      used[match]=true;auto state=states_[match];const double dt=stamp-state.track.stamp;
      Point displacement;const bool fitted=translation(state.observation.points,c.points,displacement,sparse_association_);
      const Point raw=fitted?Point{displacement.x/dt,displacement.y/dt}:Point{};
      const double speed=norm(raw),prior=norm(state.raw);
      const bool moving=speed>.15 && speed<3.5;
      const bool consistent=moving && prior>.15 && prior<3.5 &&
        (raw.x*state.raw.x+raw.y*state.raw.y)/(speed*prior)>.85 && std::abs(speed-prior)<.40;
      if (!consistent) {state.motion_since=moving ? stamp : -1.;}
      state.good=consistent?state.good+1:(moving?1:0);
      state.quiet=consistent?0:state.quiet+1;
      if(moving) {state.velocities.push_back(raw);if(state.velocities.size()>3) {state.velocities.erase(state.velocities.begin());}}
      else if(state.quiet>=2) {state.velocities.clear();}
      if(state.good>=3 && state.motion_since>=0. && stamp-state.motion_since>=.18) {
        Point velocity=raw;
        if(state.velocities.size()==3) {std::vector<double>x,y;for(auto v:state.velocities) {x.push_back(v.x);y.push_back(v.y);}velocity={median(x),median(y)};}
        state.track.velocity=velocity;state.track.active=norm(velocity)>.15 && norm(velocity)<3.5;
      } else if(state.quiet>=2) {state.track.active=false;}
      const auto old_center=state.track.center;
      const double old_radius=state.track.radius*std::exp(-dt);
      Point center=c.center;double radius=std::max(c.radius,old_radius);
      if(state.track.active) {
        const Point movement=fitted?displacement:Point{state.track.velocity.x*dt,state.track.velocity.y*dt};
        const Point predicted{old_center.x+movement.x,old_center.y+movement.y};
        const double distance=std::hypot(c.center.x-predicted.x,c.center.y-predicted.y);
        if(old_radius>=distance+c.radius) {center=predicted;radius=old_radius;}
        else if(c.radius>=distance+old_radius) {center=c.center;radius=c.radius;}
        else if(distance>1e-12) {radius=.5*(distance+old_radius+c.radius);const double mix=(radius-old_radius)/distance;center={predicted.x+mix*(c.center.x-predicted.x),predicted.y+mix*(c.center.y-predicted.y)};}
      }
      state.raw=raw;state.observation=c;state.track.stamp=stamp;state.track.center=center;
      state.track.radius=radius;next.push_back(state);
    }
    for(std::size_t j=0;j<states_.size() && next.size()<32;++j) {
      const auto & state=states_[j];const double age=stamp-state.track.stamp;
      const bool active=state.track.active && age<1.;
      const bool unconfirmed=retain_unconfirmed_ && !state.track.active &&
        (state.quiet<2 || state.good>0) && age<=.5;
      if(!used[j] && (active || unconfirmed)) {next.push_back(state);}
    }
    states_=std::move(next);last_stamp_=stamp;
  }
  std::vector<Track> unconfirmed(double now) const
  {std::vector<Track> result;for(const auto & s:states_) {if(!s.track.active && (s.quiet<2 || s.good>0) && now-s.track.stamp<=.5 && now-s.track.stamp>=-.1) {result.push_back(s.track);}}return result;}
  std::vector<Track> tentative(double now) const
  {
    std::vector<Track> result;
    for (const auto & state : states_) {
      const auto age = now - state.track.stamp;
      if (!state.track.active && state.good >= 2 && age >= -.1 && age <= .5) {
        auto track = state.track;
        track.velocity = state.raw;
        result.push_back(track);
      }
    }
    return result;
  }
  std::vector<Track> active(double now) const
  {std::vector<Track> result;for(const auto & s:states_) {if(s.track.active && now-s.track.stamp<=1. && now-s.track.stamp>=-.1) {result.push_back(s.track);}}return result;}
};
// Extra caution is a preference for unconfirmed returns only. Original physical
// and confirmed-moving-obstacle checks remain independent hard constraints.
inline double unconfirmedCautionCost(double minimum_extra_gap)
{return 1000. * std::max(0.,-minimum_extra_gap);}
inline double recoveredCautionRadius(double full_radius,double mandatory_distance)
{return std::min(full_radius,std::max(0.,mandatory_distance-1e-6));}
inline double unconfirmedGap(const Track & track,Point robot,double now,double future,double robot_radius,double caution_horizon)
{
  const double margin=3.5*(std::max(0.,now-track.stamp)+caution_horizon);
  return std::hypot(robot.x-track.center.x,robot.y-track.center.y)-robot_radius-track.radius-.05-margin;
}
inline double predictedGap(const Track & track,Point robot,double now,double future,double robot_radius)
{
  const double delta=std::max(0.,now-track.stamp)+future;
  const Point predicted{track.center.x+track.velocity.x*delta,track.center.y+track.velocity.y*delta};
  return std::hypot(robot.x-predicted.x,robot.y-predicted.y)-robot_radius-track.radius-.05-.15*delta;
}
}
#endif
