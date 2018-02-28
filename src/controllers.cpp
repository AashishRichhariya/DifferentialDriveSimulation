#include "controllers.h"
#include <Eigen/Geometry>
#include <cmath>
using namespace std;
using namespace Eigen;

void PurePursuitController::calculateMinimumTurnRadius(){//finds turn radius in axle length scale
  min_turn_radius = (axle_length*(linear_velocity+max_velocity))/(2*(max_velocity-linear_velocity));
}

double PurePursuitController::distance(double x1,double y1,double x2,double y2){
  return sqrt(pow(x1-x2,2) + pow(y1-y2,2));
}

int PurePursuitController::findNextPointByPursuit(robot_pose &rp,vector<pt> &path){
  int n = path.size();
  if(!n) return n;
  double min = 1e15;
  int ind = -1;
  vector<double> distances(n);
  for(int i = 0;i<n;i++){
    if((distances[i]=distance(rp.x,rp.y,path[i].x,path[i].y))<min){
      min = distances[i];
      ind = i;
    }
    if(i == n-1 && distances[i]<=reach_radius)
      return n;
  }
  int next_point = ind;
  for(int i = ind;i<n;i++){
    if(distances[i]<look_ahead_distance && distances[i]>distances[next_point])
      next_point = i;
  }
  //below case occurs when look ahead is very small, so the robot would end up circling the closest point, never being able to see the next point
  if(distances[next_point]<=reach_radius)
    next_point++;
  return next_point;
}

int PurePursuitController::findNextPointByPathIndex(robot_pose &rp, vector<pt> &path){
  if(next_index == path.size())
    return next_index;
  double dis = distance(rp.x,rp.y,path[next_index].x, path[next_index].y);
  if(dis<=reach_radius)
    next_index++;
  return next_index;
}

pair<int,int> PurePursuitController::computeStimuli(robot_pose &rp,vector<pt> &path, int &next_point_index_in_path){
  int next_point;
  if(next_point_by_pursuit)
    next_point = findNextPointByPursuit(rp,path);
  else
    next_point = findNextPointByPathIndex(rp,path);
  next_point_index_in_path = next_point;
  if(next_point == path.size())
    return make_pair(0,0);
  Vector2d vec_translate(path[next_point].x-rp.x, path[next_point].y-rp.y);
  Rotation2D<double> rot(-(rp.omega-PI/2.0));
  Vector2d robot_relative_coords = rot*vec_translate;
  if(abs(robot_relative_coords(0))<eps)
    return make_pair(linear_velocity,linear_velocity);
  int flag_turn_left = 0;
  if(robot_relative_coords(0)<0){//2 quadrants to consider now
    flag_turn_left = 1;
    robot_relative_coords(0) *= -1;
  }
  if(robot_relative_coords(1)<0){
    if(flag_turn_left) return make_pair((-1)*inplace_turn_velocity,inplace_turn_velocity);
    else return make_pair(inplace_turn_velocity,(-1)*inplace_turn_velocity);
  }
  double distance_to_next = distance(rp.x,rp.y,path[next_point].x,path[next_point].y);
  double radius_of_curvature = pow(distance_to_next,2)/(2.0*robot_relative_coords(0));
  if(radius_of_curvature<min_turn_radius){
    if(flag_turn_left) return make_pair((-1)*inplace_turn_velocity,inplace_turn_velocity);
    else return make_pair(inplace_turn_velocity,(-1)*inplace_turn_velocity);
  }
  int excess_turn = (2*axle_length*linear_velocity)/(2*radius_of_curvature-axle_length);
  if(flag_turn_left) return make_pair(linear_velocity,linear_velocity+excess_turn);
  else return make_pair(linear_velocity+excess_turn,linear_velocity);
}

