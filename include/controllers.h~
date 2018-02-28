#ifndef CONTROLLER_H
#define CONTROLLER_H
#include <utility>
#include <vector>
#include "structures.h"

class PurePursuitController{
  public:
    double look_ahead_distance;
    double reach_radius;
    double axle_length;
    int linear_velocity;
    int inplace_turn_velocity;
    int max_velocity;
    double eps = 1e-9;
    double min_turn_radius;
    bool next_point_by_pursuit;
    int next_index;//used exclusively by the findNextPointByPathIndex function, don't use it for any other purpose
    //finds turn radius in axle length scale
    void calculateMinimumTurnRadius();
    PurePursuitController(double a,double b,double c, int d,int e,int f,bool g):look_ahead_distance(a),reach_radius(b),axle_length(c),linear_velocity(d),inplace_turn_velocity(e),max_velocity(f),next_point_by_pursuit(g){
      calculateMinimumTurnRadius();
      next_index = 0;
    }
    PurePursuitController(){
      next_index = 0;
    }//dummy constructor
    double distance(double x1,double y1,double x2,double y2);
    int findNextPointByPursuit(robot_pose &rp,std::vector<pt> &path);
    int findNextPointByPathIndex(robot_pose &rp, std::vector<pt> &path);
    std::pair<int,int> computeStimuli(robot_pose &rp,std::vector<pt> &path);
};
#endif
