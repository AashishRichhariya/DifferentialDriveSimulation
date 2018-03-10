#include "pathplanners.h"
#include <algorithm>
#include <cmath>
#include <queue>
using namespace cv;
using namespace std;
using namespace Eigen;

//static members have to be initialized outside class body
RNG PathPlannerGrid::rng = RNG(12345);

double PathPlannerGrid::distance(double x1,double y1,double x2,double y2){
  return sqrt(pow(x1-x2,2) + pow(y1-y2,2));
}

//for the class PathPlannerGrid
void PathPlannerGrid::initializeLocalPreferenceMatrix(){
  //moving globally right
  aj[1][2][0].first = 0, aj[1][2][0].second = 1; 
  aj[1][2][1].first = 1, aj[1][2][1].second = 0; 
  aj[1][2][2].first = -1, aj[1][2][2].second = 0; 
  aj[1][2][3].first = 0, aj[1][2][3].second = -1; 
  //moving globally left
  aj[1][0][0].first = 0, aj[1][0][0].second = -1; 
  aj[1][0][1].first = -1, aj[1][0][1].second = 0; 
  aj[1][0][2].first = 1, aj[1][0][2].second = 0; 
  aj[1][0][3].first = 0, aj[1][0][3].second = 1; 
  //moving globally down
  aj[2][1][0].first = 1, aj[2][1][0].second = 0; 
  aj[2][1][1].first = 0, aj[2][1][1].second = -1; 
  aj[2][1][2].first = 0, aj[2][1][2].second = 1; 
  aj[2][1][3].first = -1, aj[2][1][3].second = 0; 
  //moving globally up
  aj[0][1][0].first = -1, aj[0][1][0].second = 0; 
  aj[0][1][1].first = 0, aj[0][1][1].second = 1; 
  aj[0][1][2].first = 0, aj[0][1][2].second = -1; 
  aj[0][1][3].first = 1, aj[0][1][3].second = 0; 
}

void PathPlannerGrid::initializeBactrackSearchMatrix(){
  //moving globally right and check the blocked cells towards right
  blockedcellcheck[1][2][0][0].first = 0, blockedcellcheck[1][2][0][0].second = 1;//front cell 
  blockedcellcheck[1][2][0][1].first = 1, blockedcellcheck[1][2][0][1].second = 1;//front right
  blockedcellcheck[1][2][0][2].first = 1, blockedcellcheck[1][2][0][2].second = -1;//back right
  //moving globally right and check the blocked cells towards left  
  blockedcellcheck[1][2][1][0].first = 0, blockedcellcheck[1][2][1][0].second = 1; //front cell
  blockedcellcheck[1][2][1][1].first = -1, blockedcellcheck[1][2][1][1].second = 1;//front left
  blockedcellcheck[1][2][1][2].first = -1, blockedcellcheck[1][2][1][2].second = -1; //back left
  //moving globally left and check the blocked cells towards right
  blockedcellcheck[1][0][0][0].first = 0, blockedcellcheck[1][0][0][0].second = -1; //frontcell
  blockedcellcheck[1][0][0][1].first = -1, blockedcellcheck[1][0][0][1].second = -1; //frontright
  blockedcellcheck[1][0][0][2].first = -1, blockedcellcheck[1][0][0][2].second = 1;  //backright
  //moving globally left and check the blocked cells towards left
  blockedcellcheck[1][0][1][0].first = 0, blockedcellcheck[1][0][1][0].second = -1; //front
  blockedcellcheck[1][0][1][1].first = 1, blockedcellcheck[1][0][1][1].second = -1; //front left
  blockedcellcheck[1][0][1][2].first = 1, blockedcellcheck[1][0][1][2].second = 1;  //back left
  //moving globally down and check the blocked cells towards right
  blockedcellcheck[2][1][0][0].first = 1, blockedcellcheck[2][1][0][0].second = 0; //front
  blockedcellcheck[2][1][0][1].first = 1, blockedcellcheck[2][1][0][1].second = -1; //front right
  blockedcellcheck[2][1][0][2].first = -1, blockedcellcheck[2][1][0][2].second = -1;  //back right
  //moving globally down and check the blocked cells towards left
  blockedcellcheck[2][1][1][0].first = 1, blockedcellcheck[2][1][1][0].second = 0; //front
  blockedcellcheck[2][1][1][1].first = 1, blockedcellcheck[2][1][1][1].second = 1; //front left
  blockedcellcheck[2][1][1][2].first = -1, blockedcellcheck[2][1][1][2].second = 1;  //back left
  //moving globally up and check the blocked cells towards right
  blockedcellcheck[0][1][0][0].first = -1, blockedcellcheck[0][1][0][0].second = 0; //front
  blockedcellcheck[0][1][0][1].first = -1, blockedcellcheck[0][1][0][1].second = 1; //front right
  blockedcellcheck[0][1][0][2].first = 1, blockedcellcheck[0][1][0][2].second = 1;  //back right
  //moving globally up and check the blocked cells towards left
  blockedcellcheck[0][1][1][0].first = -1, blockedcellcheck[0][1][1][0].second = 0; //front
  blockedcellcheck[0][1][1][1].first = -1, blockedcellcheck[0][1][1][1].second = -1; //front left
  blockedcellcheck[0][1][1][2].first = 1, blockedcellcheck[0][1][1][2].second = -1; //back left
}

//all planners with same map must have same grid cell size in pixels
//you should not call initialize, overlay, inversion grid on a shared map, or call if you 
//know what you are doing
void PathPlannerGrid::shareMap(const PathPlannerGrid &planner){
    rcells = planner.rcells;
    ccells = planner.ccells;
    world_grid = planner.world_grid;
}
void PathPlannerGrid::gridInversion(const PathPlannerGrid &planner,int rid){//invert visitable and non visitable cells for the given rid
  rcells = planner.rcells;
  ccells = planner.ccells;
  world_grid.resize(rcells);
  for(int i = 0;i<rcells;i++) world_grid[i].resize(ccells);
  for(int i = 0;i<rcells;i++)
    for(int j = 0;j<ccells;j++)
    {
      if(planner.world_grid[i][j].voronoi_id < 1)
      {
        if(planner.world_grid[i][j].steps > 0/* && planner.world_grid[i][j].r_id > 0*//*==rid*/){//the cell was visitable by given rid
          world_grid[i][j].blacks = world_grid[i][j].whites = world_grid[i][j].steps = 0;
          world_grid[i][j].tot_x = planner.world_grid[i][j].tot_x;
          world_grid[i][j].tot_y = planner.world_grid[i][j].tot_y;
          world_grid[i][j].tot = planner.world_grid[i][j].tot;
        }
        else
          world_grid[i][j].steps = 1;
      }
      else //in case it's a vornoi partition based implementation
      {
        if(planner.world_grid[i][j].steps > 0 && planner.world_grid[i][j].voronoi_id == planner.robot_tag_id/* && planner.world_grid[i][j].r_id > 0*//*==rid*/){//the cell was visitable by given rid
          world_grid[i][j].blacks = world_grid[i][j].whites = world_grid[i][j].steps = 0;
          world_grid[i][j].tot_x = planner.world_grid[i][j].tot_x;
          world_grid[i][j].tot_y = planner.world_grid[i][j].tot_y;
          world_grid[i][j].tot = planner.world_grid[i][j].tot;
        }
        else
          world_grid[i][j].steps = 1;
      }
    }
}

void PathPlannerGrid::addPoint(int ind,int px, int py, int x, int y){
  if(total_points+1>path_points.size()){
    path_points.resize(1+total_points);
    pixel_path_points.resize(1+total_points);
  }
  path_points[ind].x = x;
  path_points[ind].y = y;
  pixel_path_points[ind].first = px;
  pixel_path_points[ind].second = py;
  total_points++;
}

bool PathPlannerGrid::isEmpty(int r,int c){//criteria based on which to decide whether cell is empty
  if(r<0 || c<0 || r> rcells-1 || c>=ccells || world_grid[r][c].blacks > world_grid[r][c].whites*0.2){//more than 20 percent
    return false;
  }
  return true;
}
//everyone except origin tag is my friend
bool PathPlannerGrid::isFellowAgent(int x,int y,vector<AprilTags::TagDetection> &detections){
  for(int i = 0;i<detections.size();i++){
    //if(i == origin_id)
      //continue;
    if(pixelIsInsideTag(x,y,detections,i))
      return true;
  }
  return false;
}

bool PathPlannerGrid::pixelIsInsideTag(int x,int y,vector<AprilTags::TagDetection> &detections,int ind){
  if(ind<0)
    return false;
  for(int i = 0;i<4;i++){
    int j = (i+1)%4;
    if((x-detections[ind].p[j].first)*(detections[ind].p[j].second-detections[ind].p[i].second) - (y-detections[ind].p[j].second)*(detections[ind].p[j].first-detections[ind].p[i].first) >= 0)
      continue;
    return false;
  }
  return true;
}
//note the different use of r,c and x,y in the context of matrix and image respectively
//check for obstacles but excludes the black pixels ouvained from apriltags
//to use this function to check if robot is in frame, you must set start_grid_x, y to -1 before processing image
int PathPlannerGrid::setRobotCellCoordinates(vector<AprilTags::TagDetection> &detections){
  start_grid_y = path_points[index_travelled].y;
  start_grid_x = path_points[index_travelled].x;
  return 1;
}

int PathPlannerGrid::setGoalCellCoordinates(vector<AprilTags::TagDetection> &detections){
  if(goal_id < 0){
    if(goal_grid_x == goal_grid_y && goal_grid_x == -1){
      cout<<"can't find goal in tags detected"<<endl;
      return -1;
    }
    else
      return 1;
  }
  goal_grid_y = detections[goal_id].cxy.first/cell_size_x;
  goal_grid_x = detections[goal_id].cxy.second/cell_size_y;
  return 1;
}

void PathPlannerGrid::drawGrid(Mat &image, vector<PathPlannerGrid> &bots){
  int channels = image.channels();
  if(channels != 1 && channels != 3){
    cout<<"can't draw the grid on the given image"<<endl;
    return;
  }
  Vec3b col(0,0,0);
  int r = image.rows, c = image.cols;
  for(int i = 0;i<r;i += cell_size_y)
    for(int j = 0;j<c;j++)
      if(channels == 1)
        image.at<uint8_t>(i,j) = 0;
      else
        image.at<Vec3b>(i,j) = col;
  for(int i = 0;i<c;i+=cell_size_x)
    for(int j = 0;j<r;j++)
      if(channels == 1)
        image.at<uint8_t>(j,i) = 0;
      else
        image.at<Vec3b>(j,i) = col;  
}

void PathPlannerGrid::initializeGrid(int r,int c){//image rows and columns are provided
  rcells = ceil((float)r/cell_size_y);
  ccells = ceil((float)c/cell_size_x);
  world_grid.resize(rcells);
  for(int i = 0;i<rcells;i++) world_grid[i].resize(ccells);
  for(int i = 0;i<rcells;i++)
    for(int j = 0;j<ccells;j++)
      world_grid[i][j].emptyCell();
}

void PathPlannerGrid::overlayGrid(vector<AprilTags::TagDetection> &detections,Mat &grayImage){
  threshold(grayImage,grayImage,threshold_value,255,0);
  int r = grayImage.rows, c = grayImage.cols;
  initializeGrid(r,c);
  for(int i = 0;i<r;i++){
    for(int j = 0;j<c;j++){
      int gr = i/cell_size_y, gc = j/cell_size_x;
      world_grid[gr][gc].tot++;
      if(grayImage.at<uint8_t>(i,j) == 255 || isFellowAgent(j+1,i+1,detections)){
        world_grid[gr][gc].whites++;
        grayImage.at<uint8_t>(i,j) = 255;
      }
      else{
        world_grid[gr][gc].blacks++;
        grayImage.at<uint8_t>(i,j) = 0;
      }
      world_grid[gr][gc].tot_x += j+1;//pixel values are indexed from 1
      world_grid[gr][gc].tot_y += i+1;
    }
  }
}

pair<int,int> PathPlannerGrid::setParentUsingOrientation(robot_pose &ps){
  int agl = ps.omega;
  if(ps.omega == 0) return pair<int,int> (start_grid_x+1,start_grid_y);
  if(ps.omega == 1) return pair<int,int> (start_grid_x,start_grid_y-1);
  if(ps.omega == 2) return pair<int,int> (start_grid_x-1,start_grid_y);
  if(ps.omega == 3) return pair<int,int> (start_grid_x,start_grid_y+1);
}

void PathPlannerGrid::addGridCellToPath(int r,int c,AprilInterfaceAndVideoCapture &testbed){
  //cout<<"adding cell "<<r<<" "<<c<<endl;
  int ax,ay;//double bx,by;
  world_grid[r][c].r_id = robot_tag_id;//adding this because I can't figure out where in the later code in bsa incremental, I'm not updating the rid of the latest point added
  ax = world_grid[r][c].tot_x/world_grid[r][c].tot;
  ay = world_grid[r][c].tot_y/world_grid[r][c].tot;
  //testbed.pixelToWorld(ax,ay,bx,by);
  addPoint(total_points,ax,ay,r,c);
}

bool PathPlannerGrid::isBlocked(int ngr, int ngc){
  if(ngr >= 0 && ngc >=0 && ngr < rcells && ngc < ccells  && world_grid[ngr][ngc].voronoi_id>=1)
  {
    if(!isEmpty(ngr,ngc) || world_grid[ngr][ngc].steps || world_grid[ngr][ngc].voronoi_id!=robot_tag_id)
    return true;
  }
  else
  {
    if(!isEmpty(ngr,ngc) || world_grid[ngr][ngc].steps)
    return true;
  }  
  return false;
}

int PathPlannerGrid::getWallReference(int r,int c,int pr, int pc){
  if(pr < 0 || pc < 0)//for global preference coverage, as parent field remains unused
    return -1;
  int ngr[4],ngc[4];
  int nx = r-pr+1, ny = c-pc+1;
  for(int i = 0;i<4;i++)
    ngr[i] = r+aj[nx][ny][i].first, ngc[i] = c+aj[nx][ny][i].second;
  if(isBlocked(ngr[1],ngc[1]))//right wall due to higher priority
    return 1;
  if(isBlocked(ngr[0],ngc[0]))//front wall, turn right, left wall reference
    return 2;
  if(isBlocked(ngr[2],ngc[2]))//left wall
    return 2;
  return -1;//
}
//find shortest traversal,populate path_points
void PathPlannerGrid::findshortest(AprilInterfaceAndVideoCapture &testbed){
  //if(setRobotCellCoordinates(testbed.detections)<0)
   //return;
  //if(setGoalCellCoordinates(testbed.detections)<0)
    //return;

  queue<pair<int,int> > q;
  if(goal_grid_x < 0 || goal_grid_y < 0 || goal_grid_x > rcells-1 || goal_grid_y >= ccells )
  {
    cout<<"Point out of scope\n";
    total_points = -1;//dummy to prevent function recall
    return;
  }
  q.push(make_pair(start_grid_x,start_grid_y));
  world_grid[start_grid_x][start_grid_y].parent.first = rcells;//just to define parent of 1st node
  world_grid[start_grid_x][start_grid_y].parent.second = ccells;//just to define parent of 1st node
  world_grid[start_grid_x][start_grid_y].steps = 1;
  vector<pair<int,int> > aj = {{-1,0},{0,1},{1,0},{0,-1}};
  int ngr,ngc;
  pair<int,int> t;
  while(!q.empty()){
    t = q.front();q.pop();
    if(t.first == goal_grid_x && t.second == goal_grid_y)
      break;
    for(int i = 0;i<4;i++){
      ngr = t.first+aj[i].first, ngc = t.second+aj[i].second;
      if(isBlocked(ngr,ngc))
        continue;
      world_grid[ngr][ngc].parent.first = t.first;
      world_grid[ngr][ngc].parent.second = t.second;
      world_grid[ngr][ngc].steps = world_grid[t.first][t.second].steps + 1;
      q.push(make_pair(ngr,ngc));
    }
  }
  if(!( t.first == goal_grid_x && t.second == goal_grid_y )){
    cout<<"no path to reach destination"<<endl;
    total_points = -1;//dummy to prevent function recall
    return;
  }
  total_points = 0;
  int cnt = world_grid[t.first][t.second].steps;
  pixel_path_points.resize(cnt);
  path_points.resize(cnt);
  for(int i = cnt-1;!(t.first == rcells && t.second == ccells);i--){
    addGridCellToPath(t.first,t.second,testbed);
    t = world_grid[t.first][t.second].parent;
  }
}
bool PathPlannerGrid::checkReachStatus(pair <int, int> t, robot_pose &ps, double reach_distance)
{
  if(t.first != start_grid_x || t.second != start_grid_y || (index_travelled +1) < path_points.size()){//ensure the robot is continuing from the last point, and that no further planning occurs until the robot reaches the required point
      return 0;
  }
  return 1;
}

void PathPlannerGrid::addBacktrackPointToStackAndPath(stack<pair<int,int> > &sk,vector<pair<int,int> > &incumbent_cells,int &ic_no,int ngr, int ngc,pair<int,int> &t,AprilInterfaceAndVideoCapture &testbed){
  if(ic_no){
    world_grid[ngr][ngc].steps = 1;//just so that our target point is marked visited and there is no seg fault while searching goal in find shortest funciton.
    incumbent_cells[ic_no] = t; 
    ic_no++;
    vector<vector<nd> > tp;//a temporary map
    PathPlannerGrid temp_planner(tp);
    temp_planner.gridInversion(*this, robot_tag_id);
    temp_planner.start_grid_x = incumbent_cells[0].first;
    temp_planner.start_grid_y = incumbent_cells[0].second;
    /*temp_planner.goal_grid_x = incumbent_cells[ic_no-1].first;
    temp_planner.goal_grid_y = incumbent_cells[ic_no-1].second;*/
    temp_planner.goal_grid_x = ngr;
    temp_planner.goal_grid_y = ngc;
    temp_planner.findshortest(testbed);
    for(int i = temp_planner.path_points.size()-1;i>=0;i--){
      addPoint(total_points,temp_planner.pixel_path_points[i].first,temp_planner.pixel_path_points[i].second,temp_planner.path_points[i].x, temp_planner.path_points[i].y);
    }
    //for(int i = 1;i<ic_no;i++){//simply revisit the previous cells till reach the branch
      //int cellrow = incumbent_cells[i].first, cellcol = incumbent_cells[i].second;
      //addGridCellToPath(cellrow,cellcol,testbed);
    //}
    ic_no = 0;//reset to zero
  }
  world_grid[ngr][ngc].wall_reference = getWallReference(t.first,t.second,world_grid[t.first][t.second].parent.first, world_grid[t.first][t.second].parent.second);
  world_grid[ngr][ngc].steps = 1;
  world_grid[ngr][ngc].parent = t;
  world_grid[ngr][ngc].r_id = robot_tag_id;
  addGridCellToPath(ngr,ngc,testbed);
  sk.push(pair<int,int>(ngr,ngc));
}

int PathPlannerGrid::backtrackSimulateBid(pair<int,int> target,AprilInterfaceAndVideoCapture &testbed){
  if(setRobotCellCoordinates(testbed.detections)<0)//set the start_grid_y, start_grid_x though we don't needto use them in this function(but is just a weak confirmation that the robot is in current view), doesn't take into account whether the robot is in the current view or not(the variables might be set from before), you need to check it before calling this function to ensure correct response
    return 10000000;
  if(sk.empty())//the robot is inactive
    return 10000000;//it can't ever reach
  if(status==1 || status==2)
    return 10000000;//the bot is in either return mode.
  stack<pair<int,int> > skc = sk;
  vector<vector<nd> > world_gridc = world_grid;//copy current grid
  vector<vector<nd> > tp;//a temporary map
  PathPlannerGrid plannerc(tp);
  plannerc.rcells = rcells;
  plannerc.ccells = ccells;
  plannerc.world_grid = world_gridc;//world_gridc won't be copied since world_grid is a reference variable
  int nx,ny,ngr,ngc,wall;//neighbor row and column
  int step_distance = 0;
  while(true){
    pair<int,int> t = skc.top();
    nx = t.first-world_gridc[t.first][t.second].parent.first+1;//add one to avoid negative index
    ny = t.second-world_gridc[t.first][t.second].parent.second+1;
    if((wall=world_gridc[t.first][t.second].wall_reference)>=0){
      ngr = t.first+aj[nx][ny][wall].first, ngc = t.second+aj[nx][ny][wall].second;
      //if(!plannerc.isBlocked(ngr,ngc)){
       if(isEmpty(ngr,ngc)&&(!world_gridc[ngr][ngc].steps)){//don't know why but isBlocked is not working here in some cases
        world_gridc[ngr][ngc].wall_reference = -1;
        world_gridc[ngr][ngc].steps = 1;
        world_gridc[ngr][ngc].parent = t;
        world_gridc[ngr][ngc].r_id = robot_tag_id;
        skc.push(pair<int,int>(ngr,ngc));
        step_distance++;
        if(ngr == target.first && ngc == target.second)//the point was covered during the spiral only
          return step_distance;//return the distance found till now
        else
          continue;
      }
    }
    bool empty_neighbor_found = false;
    for(int i = 0;i<4;i++){
      ngr = t.first+aj[nx][ny][i].first;
      ngc = t.second+aj[nx][ny][i].second;
      //if(plannerc.isBlocked(ngr,ngc))
      if(!isEmpty(ngr,ngc) || world_gridc[ngr][ngc].steps)//don't know why but isBlocked is not working here in some cases
        continue;
      empty_neighbor_found = true;
      world_gridc[ngr][ngc].steps = 1;
      world_gridc[ngr][ngc].parent = t;
      world_gridc[ngr][ngc].r_id = robot_tag_id;
      world_gridc[ngr][ngc].wall_reference = getWallReference(t.first,t.second,world_gridc[t.first][t.second].parent.first, world_gridc[t.first][t.second].parent.second);
      skc.push(pair<int,int>(ngr,ngc));
      step_distance++;
      if(ngr == target.first && ngc == target.second)//no need to go any further
        return step_distance;
      else
        break;//already added a new point in spiral
    }
    if(empty_neighbor_found) continue;
    break;//if control reaches here, it means that the spiral phase simulation is complete and the target was not visited 
  }//while
  //check whether the target is approachable from current robot
  //for(int i = 0;i<4;i++){
    //ngr = target.first+aj[0][1][i].first;//aj[0][1] gives the global preference iteration of the neighbors
    //ngc = target.second+aj[0][1][i].second;
    //if(isEmpty(ngr,ngc) && world_gridc[ngr][ngc].steps/*r_id >0 *//*== robot_tag_id*/){//robot can get to given target via [ngr][ngc], /*no need to check steps as I'm checking the r_id which implies covered*/
      vector<vector<nd> > tp2;//a temporary map
      PathPlannerGrid temp_planner(tp2);
      temp_planner.gridInversion(plannerc, robot_tag_id);
      temp_planner.start_grid_x = skc.top().first;
      temp_planner.start_grid_y = skc.top().second;
      //temp_planner.goal_grid_x = ngr;
      //temp_planner.goal_grid_y = ngc;
      temp_planner.goal_grid_x = target.first;
      temp_planner.goal_grid_y = target.second;
      temp_planner.findshortest(testbed);
      if(temp_planner.total_points==-1)
      {
        cout<<"the robot can't return to given target\n";
        return 10000000;//the robot can't return to given target
      }
      step_distance += temp_planner.total_points;
      return step_distance;//there might be a better way via some other adj cell, but difference would be atmost 1 unit cell
    //}
  //}
  //return 10000000;//the robot can't return to given target
}

void PathPlannerGrid::updateMovementinSimulation(AprilInterfaceAndVideoCapture &testbed)
{
   if(setRobotCellCoordinates(testbed.detections)<0)//set the start_grid_y, start_grid_x
    return;
    world_grid[start_grid_x][start_grid_y].bot_presence = make_pair(1, robot_tag_id); //assigning bot presence bit to current cell, //this would come to use in collision avoidance algorithm
    if(last_grid_x != start_grid_x || last_grid_y != start_grid_y)
      {
        world_grid[last_grid_x][last_grid_y].bot_presence = make_pair(0, -1);
        last_grid_x = start_grid_x;
        last_grid_y = start_grid_y;
      }
    wait_to_plan = 1;    
}
//each function call adds only the next spiral point in the path vector, which may occur after a return phase
void PathPlannerGrid::BSACoverageIncremental(AprilInterfaceAndVideoCapture &testbed, robot_pose &ps, double reach_distance, vector<PathPlannerGrid> &bots){
  if(setRobotCellCoordinates(testbed.detections)<0)//set the start_grid_y, start_grid_x
    return;

  if(!first_call){
    //cout<<"in second and subsequent calls"<<endl;
    for(int i = 0; i < bt_destinations.size(); i++)
    {
      if(!bt_destinations[i].valid || world_grid[bt_destinations[i].next_p.first][bt_destinations[i].next_p.second].steps>0){//the bt is no longer uncovered
        bt_destinations[i].valid = false;//the point should no longer be considered in future
        continue;
      }
    }

    for(int i = 0; i< uev_destinations.size();i++){
     if(!uev_destinations[i].valid || world_grid[uev_destinations[i].next_p.first][uev_destinations[i].next_p.second].steps>0 ){//the bt is no longer uncovered or backtack conditions no longer remain
          uev_destinations[i].valid = false;//the point should no longer be considered in future
          continue;
        }
    }

    if(!sk.empty()){
      pair<int,int> t = sk.top();      
      world_grid[start_grid_x][start_grid_y].bot_presence = make_pair(1, robot_tag_id); //assigning bot presence bit to current cell, //this would come to use in collision avoidance algorithm
    if(last_grid_x != start_grid_x || last_grid_y != start_grid_y)
      {
        world_grid[last_grid_x][last_grid_y].bot_presence = make_pair(0, -1);
        last_grid_x = start_grid_x;
        last_grid_y = start_grid_y;
      }  
    if(!checkReachStatus(t, ps, reach_distance)){//ensure the robot is continuing from the last point, and that no further planning occurs until the robot reaches the required point
        cout<<"the robot has not yet reached the old target"<<t.first<<" "<<t.second<<endl;
        return;
      }
    }  
    
  }
  //cout<<"after first call check"<<endl;
  vector<pair<int,int> > incumbent_cells(rcells*ccells);//make sure rcells and ccells are defined
  int ic_no = 0;

  if(first_call){
    first_call = 0;
    total_points = 0;
    sk.push(pair<int,int>(start_grid_x,start_grid_y));
    world_grid[start_grid_x][start_grid_y].parent = setParentUsingOrientation(ps);
    world_grid[start_grid_x][start_grid_y].steps = 1;//visited
    world_grid[start_grid_x][start_grid_y].r_id = robot_tag_id;
    world_grid[start_grid_x][start_grid_y].bot_presence = make_pair(1, robot_tag_id);
    last_grid_x = start_grid_x;
    last_grid_y = start_grid_y;
    start_cell_x = start_grid_x;
    start_cell_y = start_grid_y;
    target_grid_cell = make_pair(start_grid_x, start_grid_y);
    addGridCellToPath(start_grid_x,start_grid_y,testbed);//add the current robot position as target point on first call, on subsequent calls the robot position would already be on the stack from the previous call assuming the function is called only when the robot has reached the next point
    return;//added the first spiral point
  }
  int ngr,ngc,wall;//neighbor row and column

  while(!sk.empty()){//when no other explorable path remains, the robot simply keeps updating the path vector with the same grid cell(path point)
    pair<int,int> t = sk.top();
    //cout<<"top of stack is "<<t.first<<" "<<t.second<<endl;
    int nx = t.first-world_grid[t.first][t.second].parent.first+1;//add one to avoid negative index
    int ny = t.second-world_grid[t.first][t.second].parent.second+1;
    if((wall=world_grid[t.first][t.second].wall_reference)>=0){//if the current cell has a wall reference to consider
      ngr = t.first+aj[nx][ny][wall].first, ngc = t.second+aj[nx][ny][wall].second;
      if(!isBlocked(ngr,ngc)){
        if(ic_no == 0){//if you are not backtracking then only proceed, else store the point in possible destinations
          status = 0;
          addBacktrackPointToStackAndPath(sk,incumbent_cells,ic_no,ngr,ngc,t,testbed);
          world_grid[ngr][ngc].wall_reference = -1;//to prevent wall exchange to right wall when following left wall
          //cout<<"added a wall reference considered point"<<endl;
          for(int j = 0; j < 4; j++)//incremental addition of backtracking points
          {
            if(j!=wall)
            {
              ngr = t.first+aj[nx][ny][j].first;
              ngc = t.second+aj[nx][ny][j].second;
              if(!isBlocked(ngr, ngc))
              {
                //checking for presence of bt point in the vector is redundant and can be avoided
                int k;
                for(k = 0;k<bt_destinations.size();k++)
                {
                    if(bt_destinations[k].next_p.first == ngr && bt_destinations[k].next_p.second == ngc && bt_destinations[k].parent.second == t.second && bt_destinations[k].parent.first == t.first)//the point was already added before
                    {
                      cout<<"**********************Alarm#1***********************************\n";
                      cout<<"the backtrack point is: "<<ngr<<" "<<ngc<<endl;
                      //cv::waitKey(0);
                      break;
                    }
                }
                if(k == bt_destinations.size()){//this is new point
                bt_destinations.push_back(bt(t.first,t.second,ngr,ngc,sk));
                uev_destinations.push_back(uev(t.first,t.second,ngr,ngc,sk));
                cout<<"added a new backtrack point "<<ngr<<" "<<ngc<<endl;
                }
              }
            }
            
          }//end of for j
          break;// a new spiral point has been added
        }
      }
    }
    //cout<<"past the wall check"<<endl;
    bool empty_neighbor_found = false;
    for(int i = 0;i<4;i++){
      ngr = t.first+aj[nx][ny][i].first;
      ngc = t.second+aj[nx][ny][i].second;
      //cout<<"checking neighbor "<<ngr<<" "<<ngc<<endl;
      if(isBlocked(ngr,ngc)){
        //cout<<"it is blocked"<<endl;
        continue;
      }
      //cout<<"an empty neighbor is found"<<endl;
      empty_neighbor_found = true;
      if(ic_no == 0){
        status = 0;
        addBacktrackPointToStackAndPath(sk,incumbent_cells,ic_no,ngr,ngc,t,testbed);
        cout<<"added to stack: "<<i<<endl;
        for(int j = i+1; j < 4; j++)//incremental addition of backtracking points
        {
          ngr = t.first+aj[nx][ny][j].first;
          ngc = t.second+aj[nx][ny][j].second;
          if(!isBlocked(ngr, ngc))
          {
            //checking for presence of bt point in the vector is redundant and can be avoided
            int k;
            for(k = 0;k<bt_destinations.size();k++)
            {
                if(bt_destinations[k].next_p.first == ngr && bt_destinations[k].next_p.second == ngc && bt_destinations[k].parent.second == t.second && bt_destinations[k].parent.first == t.first)//the point was already added before
                {
                  cout<<"**********************Alarm#1***********************************\n";
                  cout<<"the backtrack point is: "<<ngr<<" "<<ngc<<endl;
                  //qitKey(0);
                  break;
                }
            }
            if(k == bt_destinations.size()){//this is new point
            bt_destinations.push_back(bt(t.first,t.second,ngr,ngc,sk));
            uev_destinations.push_back(uev(t.first,t.second,ngr,ngc,sk));
            cout<<"added a new backtrack point "<<ngr<<" "<<ngc<<endl;
            }
          }
        }//end of for j
        break; //for i
      }
      
      else{//the following else statement can be avoided as all the backtrackpoints have already been added, but has just been kept. It is redundant. 
      //incase we are backtracking, adding points to bots[i]'s backtracking destinations after checking if the desitation is not already in the vector.
        int i;
        for(i = 0;i<bt_destinations.size();i++)
          if(bt_destinations[i].next_p.first == ngr && bt_destinations[i].next_p.second == ngc && bt_destinations[i].parent.second == t.second && bt_destinations[i].parent.first == t.first)//the point was already added before
            break;
        if(i == bt_destinations.size()){//this is new point
           cout<<"**********************Alarm#2***********************************\n";
          bt_destinations.push_back(bt(t.first,t.second,ngr,ngc,sk));
          uev_destinations.push_back(uev(t.first,t.second,ngr,ngc,sk));
          cout<<"added a new backtrack point "<<ngr<<" "<<ngc<<endl;
          //cv::waitKey(0);
        }
      }
    }
    //cout<<"past the local neighbors check"<<endl;
    if(empty_neighbor_found && ic_no == 0){
      //cout<<"added a local reference point"<<endl;
      break;//a new spiral point has been added and this is not a backtrack iteration
    }
    //cout<<"ic_no is "<<ic_no<<endl;
    incumbent_cells[ic_no] = t;//add the point as a possible return phase point
    ic_no++;
    sk.pop();
    //cout<<"popped the top of stack"<<endl;
    if(sk.empty()) break;//no new spiral point was added, there might be some bt points available 
    pair<int,int> next_below = sk.top();
    //the lines below are obsolete(at first thought) since the shortest path is being calculated, so wall reference and parent are obsolete on already visited points
    world_grid[next_below.first][next_below.second].parent = t;
    world_grid[next_below.first][next_below.second].wall_reference = 1;//since turning 180 degrees
  } //while
  //cout<<"reached out of while loop, will now check if this is a backtrack interation or a new spiral point has already been added"<<endl;
  //if(ic_no == 0) return;//no further bt processing
  //following 6-7 lines are added to find the backtrack point in case the stack is now empty
 if(ic_no == 0 && !sk.empty()) return;//no further bt processing
 if(sk.empty())
 {
   ic_no = 5;//any number greater than 0;
   incumbent_cells[0].first = start_grid_x;
   incumbent_cells[0].second = start_grid_y;
 }
  //in case of backtracking
  status = 1;
  for(int j = 0; j < bots.size(); j++)
  {
      for(int i = 0;i<bots[j].bt_destinations.size();i++){
      //check if backtrackpoint has been yet visited or not
      if(!bots[j].bt_destinations[i].valid || world_grid[bots[j].bt_destinations[i].next_p.first][bots[j].bt_destinations[i].next_p.second].steps>0){//the bt is no longer uncovered
        bots[j].bt_destinations[i].valid = false;//the point should no longer be considered in future
        continue;
      }

      //next 30 lines are to determine if there exist a parent for this bt_point, with smaller path
        int best_parent_x, best_parent_y; 
        int min_total_points = 10000000, min_total_points_index;
        int flag = 0;
        for(int k = 0; k < 4; k++)
        {
          best_parent_x = bots[j].bt_destinations[i].next_p.first + aj[0][1][k].first;
          best_parent_y = bots[j].bt_destinations[i].next_p.second + aj[0][1][k].second;
          if(isBlocked(best_parent_x, best_parent_y))
          {
            vector<vector<nd> > tp;//a temporary map
            PathPlannerGrid temp_planner(tp);
            temp_planner.gridInversion(*this, bots[j].robot_tag_id);
            temp_planner.start_grid_x = start_grid_x;//the current robot coordinates
            temp_planner.start_grid_y = start_grid_y;
            temp_planner.goal_grid_x = best_parent_x;
            temp_planner.goal_grid_y = best_parent_y;
            temp_planner.findshortest(testbed);
            if(temp_planner.total_points< min_total_points && temp_planner.total_points >= 0)
            {
              min_total_points = temp_planner.total_points;
              min_total_points_index = k;
              flag = 1;
            }
          }
        }
        if(flag == 1)
        {
          bots[j].bt_destinations[i].parent.first = bots[j].bt_destinations[i].next_p.first + aj[0][1][min_total_points_index].first;
          bots[j].bt_destinations[i].parent.second = bots[j].bt_destinations[i].next_p.second + aj[0][1][min_total_points_index].second;
        }//best parent assigned

      vector<vector<nd> > tp;//a temporary map
      PathPlannerGrid temp_planner(tp);
      temp_planner.gridInversion(*this, bots[j].robot_tag_id);
      temp_planner.start_grid_x = start_grid_x;//the current robot coordinates
      temp_planner.start_grid_y = start_grid_y;
      temp_planner.goal_grid_x = bots[j].bt_destinations[i].parent.first;
      temp_planner.goal_grid_y = bots[j].bt_destinations[i].parent.second;
      //vector<vector<nd> > tep;
      //tep.resize(temp_planner.world_grid.size());
      //for(int i = 0;i<tep.size();i++)
        //tep[i].resize(temp_planner.world_grid[i].size());
      //for(int i = 0;i<tep.size();i++)
        //for(int j = 0;j<tep[i].size();j++)
          //tep[i][j] = temp_planner.world_grid[i][j];

      temp_planner.findshortest(testbed);
      bots[j].bt_destinations[i].manhattan_distance = temp_planner.total_points;//-1 if no path found
      }

      cout<<"sorting for robot tag id : "<<bots[j].robot_tag_id<<endl;
      sort(bots[j].bt_destinations.begin(),bots[j].bt_destinations.end(),[](const bt &a, const bt &b) -> bool{
        return a.manhattan_distance<b.manhattan_distance;
        });
      if(bots[j].bt_destinations.size()!=0)
      {
        cout<<"shortest for the set of bt_points: "<<bots[j].bt_destinations[0].next_p.first<<" "<<bots[j].bt_destinations[0].next_p.second<<" validity: "<<bots[j].bt_destinations[0].valid<<" manhattan_distance: "<<bots[j].bt_destinations[0].manhattan_distance<<endl;
      }
      else
      {
        cout<<"this robot had no bt_destination point of its own!\n";
      }
      
  }
  cout<<"Sorting for individual bt_points done!\n";
  //vector <int> it(bots.size());
  int it = 10000000;
  vector <int> mind(bots.size(),10000000);
  //vector <bool> valid_found(bots.size(), false);
  bool valid_found = false;
  int min_global_manhattan_dist = 10000000, min_j_index = 0;
  int min_valid_distance = 10000000, min_valid_index = 0;
  for(int j = 0; j < bots.size(); j++)
  {
    cout<<"checking the closest valid bt point for robot: "<<bots[j].robot_tag_id<<endl;
    for(int k = 0;k<bots[j].bt_destinations.size();k++){
    if(!bots[j].bt_destinations[k].valid || bots[j].bt_destinations[k].manhattan_distance<0)//refer line 491
      continue;
    if(k!=10000000)//almost unnecesary condition
      {
        mind[j] = k;//closest valid backtracking point
        it = k;
        if(bots[j].bt_destinations[it].manhattan_distance < min_global_manhattan_dist)
        {
          min_global_manhattan_dist = bots[j].bt_destinations[it].manhattan_distance;
          min_j_index = j;
        }
      }      
    int i;
    for(i = 0;i<bots.size();i++){
      if(bots[i].robot_id == origin_id || bots[i].robot_id == robot_id)//the tag is actually the origin or current robot itself
        continue;
      //all planners must share the same map
      int tp = bots[i].backtrackSimulateBid(bots[j].bt_destinations[it].next_p,testbed);// returns 10000000 if no path, checks if this particular backtrack point can be reached by other bot in lesser steps
      if(tp<bots[j].bt_destinations[it].manhattan_distance)//a closer bot is available
        {
          cout<<"some other bot can reach earlier!\n";
          break;
        }
      }
      if(i == bots.size()){
        cout<<"valid bt point for this found!"<<endl;
        valid_found = true;
        if(bots[j].bt_destinations[it].manhattan_distance < min_valid_distance)
        {
          min_valid_distance = bots[j].bt_destinations[it].manhattan_distance;
          min_valid_index = j;
      }
      break;
      }
    }
  }
  
  int bot_index = 0;
  if(!valid_found && mind[min_j_index] == 10000000){//no bt point left
    status = 2;
    cout<<"no bt point left for robot "<<robot_tag_id<<endl;
    return;
  }
  if(!valid_found && mind[min_j_index] != 10000000)//no point exists for which the given robot is the closest
    {
      status = 1;
      it = mind[min_j_index];
      bot_index = min_j_index;
    }
  else if(valid_found)
  {
    status = 1;
    it = mind[min_valid_index];
    bot_index = min_valid_index;
  }
  cout<<"Just found the best backtrack point. Adding it!\n";
  addBacktrackPointToStackAndPath(sk,incumbent_cells,ic_no,bots[bot_index].bt_destinations[it].next_p.first,bots[bot_index].bt_destinations[it].next_p.second,bots[bot_index].bt_destinations[it].parent,testbed);
}//end of BSA incremental fucntion

int PathPlannerGrid::checkBactrackCondition(pair<int, int> p1, pair <int, int> p2)
{
  if(!isBlocked(p1.first, p1.second) && isBlocked(p2.first, p2.second) )
  {
    return 1;    
  }
  else return 0;
}

bool PathPlannerGrid::checkBactrackValidityForBoB(pair <int, int> t)
{
  int sum = 0;
  vector <pair<int, int>> v(9);
  v[1].first = t.first + 0;//Right
  v[1].second = t.second + 1;
  v[2].first = t.first -1;//top right
  v[2].second = t.second +1; 
  v[3].first = t.first -1;//top
  v[3].second = t.second -0; 
  v[4].first = t.first -1;//top left
  v[4].second = t.second -1; 
  v[5].first = t.first -0;//left
  v[5].second = t.second -1; 
  v[6].first = t.first +1;//bottom left
  v[6].second = t.second -1; 
  v[7].first = t.first +1;//bottom
  v[7].second = t.second -0; 
  v[8].first = t.first +1;//bottom right
  v[8].second = t.second +1; 
  
  sum+= checkBactrackCondition(v[1], v[8]);//right and bottom right
  sum+= checkBactrackCondition(v[5], v[6]);//left and bottom left
  sum+= checkBactrackCondition(v[1], v[2]);//rgiht and top right
  sum+= checkBactrackCondition(v[5], v[4]);//left and top left
  sum+= checkBactrackCondition(v[7], v[6]);//bottom and bottom left
  sum+= checkBactrackCondition(v[7], v[8]);//bottom and bottom right
  sum+= checkBactrackCondition(v[3], v[2]);//top and top right
  sum+= checkBactrackCondition(v[3], v[4]);//top and top left


  if (sum>0) return 1;
  else return 0;
}

bool PathPlannerGrid::bactrackValidityForBSA_CM(pair <int, int> t, int nx, int ny, int j)
{
  int ngr, ngc;  
  for(int i = 0; i < 3; i++)
  {   
    ngr = t.first + blockedcellcheck[nx][ny][j][i].first;
    ngc = t.second + blockedcellcheck[nx][ny][j][i].second;
    if (isBlocked(ngr, ngc)) return 1;
  }
  return 0;
}
bool PathPlannerGrid::bactrackValidityForBoB(pair <int, int> t, int nx, int ny, int j)
{
  int ngr, ngc;  
  for(int i = 1; i < 3; i++)
  {   
    ngr = t.first + blockedcellcheck[nx][ny][j][i].first;
    ngc = t.second + blockedcellcheck[nx][ny][j][i].second;
    if (isBlocked(ngr, ngc)) return 1;
  }
  return 0;
}

void PathPlannerGrid::BSA_CMSearchForBTAmongstUEV(AprilInterfaceAndVideoCapture &testbed, vector<PathPlannerGrid> &bots, vector<pair<int,int> > &incumbent_cells, int ic_no, stack<pair<int,int> > &sk){

 for(int j = 0; j < bots.size(); j++)
    {
        for(int i = 0;i<bots[j].uev_destinations.size();i++){
        //check if backtrackpoint has been yet visited or not, or if that is still valid
          pair <int, int> backtrack_parent;
          backtrack_parent.first = bots[j].uev_destinations[i].parent.first;
          backtrack_parent.second = bots[j].uev_destinations[i].parent.second;

          if(!bots[j].uev_destinations[i].valid || world_grid[bots[j].uev_destinations[i].next_p.first][bots[j].uev_destinations[i].next_p.second].steps>0 /*|| !bots[j].checkBactrackValidityForBSA_CM(backtrack_parent)*/){//the uev is no longer uncovered or backtack conditions no longer remain
            bots[j].uev_destinations[i].valid = false;//the point should no longer be considered in future
            continue;
          }

          //next 30 lines are to determine if there exist a parent for this uev_point, with smaller path (needed because just after this we are calculating the distance till the parents and sorting accordingly)
          int best_parent_x, best_parent_y; 
          int min_total_points = 10000000, min_total_points_index;
          int flag = 0;
          for(int k = 0; k < 4; k++)
          {
            best_parent_x = bots[j].uev_destinations[i].next_p.first + aj[0][1][k].first;
            best_parent_y = bots[j].uev_destinations[i].next_p.second + aj[0][1][k].second;
            if(isBlocked(best_parent_x, best_parent_y))
            {
              vector<vector<nd> > tp;//a temporary map
              PathPlannerGrid temp_planner(tp);
              temp_planner.gridInversion(*this, bots[j].robot_tag_id);
              temp_planner.start_grid_x = start_grid_x;//the current robot coordinates
              temp_planner.start_grid_y = start_grid_y;
              temp_planner.goal_grid_x = best_parent_x;
              temp_planner.goal_grid_y = best_parent_y;
              temp_planner.findshortest(testbed);
              if(temp_planner.total_points< min_total_points && temp_planner.total_points >= 0)
              {
                min_total_points = temp_planner.total_points;
                min_total_points_index = k;
                flag = 1;
              }
            }
          }
          if(flag == 1)
          {
            bots[j].uev_destinations[i].parent.first = bots[j].uev_destinations[i].next_p.first + aj[0][1][min_total_points_index].first;
            bots[j].uev_destinations[i].parent.second = bots[j].uev_destinations[i].next_p.second + aj[0][1][min_total_points_index].second;
          }//best parent assigned

          vector<vector<nd> > tp;//a temporary map
          PathPlannerGrid temp_planner(tp);
          temp_planner.gridInversion(*this, bots[j].robot_tag_id);
          temp_planner.start_grid_x = start_grid_x;//the current robot coordinates
          temp_planner.start_grid_y = start_grid_y;
          temp_planner.goal_grid_x = bots[j].uev_destinations[i].parent.first;
          temp_planner.goal_grid_y = bots[j].uev_destinations[i].parent.second;
          temp_planner.findshortest(testbed);
          bots[j].uev_destinations[i].manhattan_distance = temp_planner.total_points;//-1 if no path found

        }//for i
        
        sort(bots[j].uev_destinations.begin(),bots[j].uev_destinations.end(),[](const uev &a, const uev &b) -> bool{
          return a.manhattan_distance<b.manhattan_distance;
          });
      }//for j

    int it = 10000000;
    vector <int> mind(bots.size(),10000000);
    bool valid_found = false;
    int min_global_manhattan_dist = 10000000, min_j_index = 0;
    int min_valid_distance = 10000000, min_valid_index = 0;
    for(int j = 0; j < bots.size(); j++)
    {
        for(int k = 0;k<bots[j].uev_destinations.size();k++){
        if(!bots[j].uev_destinations[k].valid || bots[j].uev_destinations[k].manhattan_distance<0)//refer line 491
          continue;
        if(k!=10000000)//almost unnecesary condition
          {
            mind[j] = k;//closest valid backtracking point
            it = k;
            if(bots[j].uev_destinations[it].manhattan_distance < min_global_manhattan_dist)
            {
              min_global_manhattan_dist = bots[j].uev_destinations[it].manhattan_distance;
              min_j_index = j;
            }
          }//if k
         
        int i;
        for(i = 0;i<bots.size();i++){
          if(bots[i].robot_id == origin_id || bots[i].robot_id == robot_id)//the tag is actually the origin or current robot itself
            continue;
          //all planners must share the same map
          cout<<bots[i].robot_tag_id<<": Going for backtrack simulation bid!"<<endl;
          int tp = bots[i].backtrackSimulateBid(bots[j].uev_destinations[it].next_p,testbed);// returns 10000000 if no path, checks if this particular backtrack point can be reached by other bot in lesser steps
          if(tp<bots[j].uev_destinations[it].manhattan_distance)//a closer bot is available
            {
              cout<<"some other bot can reach earlier!\n";
              break;
            }
          }//for i
          if(i == bots.size()){
            cout<<"valid uev point for this found!"<<endl;
            valid_found = true;
            if(bots[j].uev_destinations[it].manhattan_distance < min_valid_distance)
            {
              min_valid_distance = bots[j].uev_destinations[it].manhattan_distance;
              min_valid_index = j;
            }
          cout<<"the best uev point yet is: "<<bots[min_valid_index].uev_destinations[mind[min_valid_index]].next_p.first<<" "<<bots[min_valid_index].uev_destinations[mind[min_valid_index]].next_p.second<<endl;
          break;
          } //if(i==)
        }//for k
    }//for j

    int bot_index = 0;
    if(!valid_found && mind[min_j_index] == 10000000){//no uev point left
        status = 2;
        cout<<"no uev point left for robot "<<robot_tag_id<<endl;
        return;
    }
    if(!valid_found && mind[min_j_index] != 10000000)//no point exists for which the given robot is the closest
    {
        status = 1;
        it = mind[min_j_index];
        bot_index = min_j_index;
    }
    else if(valid_found)
    {
        status = 1;
        it = mind[min_valid_index];
        bot_index = min_valid_index;
    }
      //else it stores the index of the next uev point
      //sk = uev_destinations[it].stack_state;
    cout<<"Just found the best backtrack point. Adding it!\n";
    addBacktrackPointToStackAndPath(sk,incumbent_cells,ic_no,bots[bot_index].uev_destinations[it].next_p.first,bots[bot_index].uev_destinations[it].next_p.second,bots[bot_index].uev_destinations[it].parent,testbed);
}

void PathPlannerGrid::BSACoverageWithUpdatedBactrackSelection(AprilInterfaceAndVideoCapture &testbed, robot_pose &ps, double reach_distance, vector<PathPlannerGrid> &bots){
  if(setRobotCellCoordinates(testbed.detections)<0)//set the start_grid_y, start_grid_x
    return;

  if(!first_call){

    for(int i = 0; i< bt_destinations.size();i++){
      if(!bt_destinations[i].valid || world_grid[bt_destinations[i].next_p.first][bt_destinations[i].next_p.second].steps>0 /*|| !checkBactrackValidityForBSA_CM(backtrack_parent)*/){//the bt is no longer uncovered or backtack conditions no longer remain
            bt_destinations[i].valid = false;//the point should no longer be considered in future
            continue;
          }
    }

    //checking validity of uevpoints made as of now.
    for(int i = 0; i< uev_destinations.size();i++){
     if(!uev_destinations[i].valid || world_grid[uev_destinations[i].next_p.first][uev_destinations[i].next_p.second].steps>0 ){//the bt is no longer uncovered or backtack conditions no longer remain
          uev_destinations[i].valid = false;//the point should no longer be considered in future
          continue;
        }
    }

    //cout<<"in second and subsequent calls"<<endl;
    if(!sk.empty()){

      pair<int,int> t = sk.top();
      world_grid[start_grid_x][start_grid_y].bot_presence = make_pair(1, robot_tag_id); //assigning bot presence bit to current cell, //this would come to use in collision avoidance algorithm
      if(last_grid_x != start_grid_x || last_grid_y != start_grid_y)
      {
        world_grid[last_grid_x][last_grid_y].bot_presence = make_pair(0, -1);
        last_grid_x = start_grid_x;
        last_grid_y = start_grid_y;
      }    
    if(!checkReachStatus(t, ps, reach_distance)){//ensure the robot is continuing from the last point, and that no further planning occurs until the robot reaches the required point
        cout<<"the robot has not yet reached the old target"<<t.first<<" "<<t.second<<endl;
        return;
      }
    }
    //checking validit of backtrackpoints made as of now.
    
  }//if !first call

  vector<pair<int,int> > incumbent_cells(rcells*ccells);//make sure rcells and ccells are defined
  int ic_no = 0;

  if(first_call){
    first_call = 0;
    total_points = 0;
    sk.push(pair<int,int>(start_grid_x,start_grid_y));
    world_grid[start_grid_x][start_grid_y].parent = setParentUsingOrientation(ps);
    world_grid[start_grid_x][start_grid_y].steps = 1;//visited
    world_grid[start_grid_x][start_grid_y].r_id = robot_tag_id;
    world_grid[start_grid_x][start_grid_y].bot_presence = make_pair(1, robot_tag_id);
    last_grid_x = start_grid_x;
    last_grid_y = start_grid_y;
    start_cell_x = start_grid_x;
    start_cell_y = start_grid_y;
    target_grid_cell = make_pair(start_grid_x, start_grid_y);
    addGridCellToPath(start_grid_x,start_grid_y,testbed);//add the current robot position as target point on first call, on subsequent calls the robot position would already be on the stack from the previous call assuming the function is called only when the robot has reached the next point
    return;//added the first spiral point
  }//if first_call
  int ngr,ngc,wall;//neighbor row and column

  while(!sk.empty()){//when no other explorable path remains, the robot simply keeps updating the path vector with the same grid cell(path point)
    pair<int,int> t = sk.top();
    int nx = t.first-world_grid[t.first][t.second].parent.first+1;//add one to avoid negative index
    int ny = t.second-world_grid[t.first][t.second].parent.second+1;
    if((wall=world_grid[t.first][t.second].wall_reference)>=0){//if the current cell has a wall reference to consider
      ngr = t.first+aj[nx][ny][wall].first, ngc = t.second+aj[nx][ny][wall].second;
      if(!isBlocked(ngr,ngc)){
        if(ic_no == 0){//if you are not backtracking then only proceed, else store the point in possible destinations
            status = 0;
            
            for(int j = 1; j < 3; j++)//checking the backtrack point, as it's necessary to check it before adding point. this is because once the point is added the front cell is automatically treated as an obsatcle and the cell classifies as backtrack cell
            {
              int ngr2 = t.first+aj[nx][ny][j].first;//check the right and left points in consecutive iterations
              int ngc2 = t.second+aj[nx][ny][j].second;
                           
              if(ngr2==ngr && ngc2 == ngc) continue;
              if(!isBlocked(ngr2, ngc2))
              {
                if(bactrackValidityForBSA_CM(t, nx, ny, j-1))
                {
                  bt_destinations.push_back(bt(t.first,t.second,ngr2,ngc2,sk));
                }
              }
            }
            int nx2 = ngr-t.first+1;//add one to avoid negative index
            int ny2 = ngc-t.second+1;
            for(int j = 1; j < 3; j++)//checking the backtrack point, as it's necessary to check it before adding point. this is because once the point is added the front cell is automatically treated as an obsatcle and the cell classifies as backtrack cell
            {
              int ngr3 = t.first+aj[nx2][ny2][j].first;//check the right and left points in consecutive iterations
              int ngc3 = t.second+aj[nx2][ny2][j].second;
                           
              if(ngr3==ngr && ngc3 == ngc) continue;
              if(!isBlocked(ngr3, ngc3))
              {
                if(bactrackValidityForBSA_CM(t, nx2, ny2, j-1))
                {
                  bt_destinations.push_back(bt(t.first,t.second,ngr3,ngc3,sk));
                }
              }
            }
            addBacktrackPointToStackAndPath(sk,incumbent_cells,ic_no,ngr,ngc,t,testbed);
            world_grid[ngr][ngc].wall_reference = -1;//to prevent wall exchange to right wall when following left wall
            for(int j = 0; j < 4; j++)//incremental addition of backtracking points
            {
              if(j!=wall)
              {
                ngr = t.first+aj[nx][ny][j].first;
                ngc = t.second+aj[nx][ny][j].second;
                if(!isBlocked(ngr, ngc))
                {                  
                  uev_destinations.push_back(uev(t.first,t.second,ngr,ngc,sk));
                  cout<<"added a new uev point "<<ngr<<" "<<ngc<<endl;                  
                }
              }              
            }//end of for j
                        
          break;// a new spiral point has been added
        }//if ic_no
      }
    }//if wall

    bool empty_neighbor_found = false;
    for(int i = 0;i<4;i++){
      ngr = t.first+aj[nx][ny][i].first;
      ngc = t.second+aj[nx][ny][i].second;
      if(isBlocked(ngr,ngc))continue;
      empty_neighbor_found = true;
      if(ic_no == 0){
        status = 0;
        //int nx2 = ngr-t.first+1;//add one to avoid negative index
        //int ny2 = ngc-t.second+1;
        for(int j = 1; j < 3; j++)
        {
          int ngr2 = t.first+aj[nx][ny][j].first;//check the right and left points in consecutive iterations
          int ngc2 = t.second+aj[nx][ny][j].second;
          if(ngr2==ngr && ngc2 == ngc) continue;
          if(!isBlocked(ngr2, ngc2))
          {
            if(bactrackValidityForBSA_CM(t, nx, ny, j-1))
            {
              bt_destinations.push_back(bt(t.first,t.second,ngr2,ngc2,sk));
            }
          }
        }

        int nx2 = ngr-t.first+1;//add one to avoid negative index
        int ny2 = ngc-t.second+1;
        for(int j = 1; j < 3; j++)//checking the backtrack point, as it's necessary to check it before adding point. this is because once the point is added the front cell is automatically treated as an obsatcle and the cell classifies as backtrack cell
        {
          int ngr3 = t.first+aj[nx2][ny2][j].first;//check the right and left points in consecutive iterations
          int ngc3 = t.second+aj[nx2][ny2][j].second;
                           
          if(ngr3==ngr && ngc3 == ngc) continue;
          if(!isBlocked(ngr3, ngc3))
          {
            if(bactrackValidityForBSA_CM(t, nx2, ny2, j-1))
            {
              bt_destinations.push_back(bt(t.first,t.second,ngr3,ngc3,sk));
            }
          }
        }
        addBacktrackPointToStackAndPath(sk,incumbent_cells,ic_no,ngr,ngc,t,testbed);
        for(int j = i+1; j < 4; j++)//incremental addition of backtracking points
        {
          ngr = t.first+aj[nx][ny][j].first;
          ngc = t.second+aj[nx][ny][j].second;
          if(!isBlocked(ngr, ngc))
          {
            uev_destinations.push_back(uev(t.first,t.second,ngr,ngc,sk));
            cout<<"added a new uev point "<<ngr<<" "<<ngc<<endl;
            
          }
        }//end of for j
                
        break; //for i
      }//if ic_no
  }//for i
    if(empty_neighbor_found && ic_no == 0) break;//a new spiral point has been added and this is not a backtrack iteration
    incumbent_cells[ic_no] = t;//add the point as a possible return phase point
    ic_no++;
    sk.pop();
    //cout<<"popped the top of stack"<<endl;
    if(sk.empty()) break;//no new spiral point was added, there might be some bt points available 
    pair<int,int> next_below = sk.top();
    //the lines below are obsolete(at first thought) since the shortest path is being calculated, so wall reference and parent are obsolete on already visited points
    world_grid[next_below.first][next_below.second].parent = t;
    world_grid[next_below.first][next_below.second].wall_reference = 1;//since turning 180 degrees
 }//while

 //if(ic_no == 0) return;
 //following 6-7 lines are added to find the backtrack point in case the stack is now empty
 if(ic_no == 0 && !sk.empty()) return;
 if(sk.empty())
 {
   ic_no = 5;//any number greater than 0;
   incumbent_cells[0].first = start_grid_x;
   incumbent_cells[0].second = start_grid_y;
 }

 status = 1;
 for(int j = 0; j < bots.size(); j++)
 {
      for(int i = 0;i<bots[j].bt_destinations.size();i++){
      //check if backtrackpoint has been yet visited or not, or if that is still valid
        pair <int, int> backtrack_parent;
        backtrack_parent.first = bots[j].bt_destinations[i].parent.first;
        backtrack_parent.second = bots[j].bt_destinations[i].parent.second;

        if(!bots[j].bt_destinations[i].valid || world_grid[bots[j].bt_destinations[i].next_p.first][bots[j].bt_destinations[i].next_p.second].steps>0 /*|| !bots[j].checkBactrackValidityForBSA_CM(backtrack_parent)*/){//the bt is no longer uncovered or backtack conditions no longer remain
          bots[j].bt_destinations[i].valid = false;//the point should no longer be considered in future
          continue;
        }

        //next 30 lines are to determine if there exist a parent for this bt_point, with smaller path (needed because just after this we are calculating the distance till the parents and sorting accordingly)
        int best_parent_x, best_parent_y; 
        int min_total_points = 10000000, min_total_points_index;
        int flag = 0;
        for(int k = 0; k < 4; k++)
        {
          best_parent_x = bots[j].bt_destinations[i].next_p.first + aj[0][1][k].first;
          best_parent_y = bots[j].bt_destinations[i].next_p.second + aj[0][1][k].second;
          if(isBlocked(best_parent_x, best_parent_y))
          {
            vector<vector<nd> > tp;//a temporary map
            PathPlannerGrid temp_planner(tp);
            temp_planner.gridInversion(*this, bots[j].robot_tag_id);
            temp_planner.start_grid_x = start_grid_x;//the current robot coordinates
            temp_planner.start_grid_y = start_grid_y;
            temp_planner.goal_grid_x = best_parent_x;
            temp_planner.goal_grid_y = best_parent_y;
            temp_planner.findshortest(testbed);
            if(temp_planner.total_points< min_total_points && temp_planner.total_points >= 0)
            {
              min_total_points = temp_planner.total_points;
              min_total_points_index = k;
              flag = 1;
            }
          }
        }
        if(flag == 1)
        {
          bots[j].bt_destinations[i].parent.first = bots[j].bt_destinations[i].next_p.first + aj[0][1][min_total_points_index].first;
          bots[j].bt_destinations[i].parent.second = bots[j].bt_destinations[i].next_p.second + aj[0][1][min_total_points_index].second;
        }//best parent assigned

        vector<vector<nd> > tp;//a temporary map
        PathPlannerGrid temp_planner(tp);
        temp_planner.gridInversion(*this, bots[j].robot_tag_id);
        temp_planner.start_grid_x = start_grid_x;//the current robot coordinates
        temp_planner.start_grid_y = start_grid_y;
        temp_planner.goal_grid_x = bots[j].bt_destinations[i].parent.first;
        temp_planner.goal_grid_y = bots[j].bt_destinations[i].parent.second;
        temp_planner.findshortest(testbed);
        bots[j].bt_destinations[i].manhattan_distance = temp_planner.total_points;//-1 if no path found

      }
      
      sort(bots[j].bt_destinations.begin(),bots[j].bt_destinations.end(),[](const bt &a, const bt &b) -> bool{
        return a.manhattan_distance<b.manhattan_distance;
        });
  }//for j

  int it = 10000000;
  vector <int> mind(bots.size(),10000000);
  bool valid_found = false;
  int min_global_manhattan_dist = 10000000, min_j_index = 0;
  int min_valid_distance = 10000000, min_valid_index = 0;
  for(int j = 0; j < bots.size(); j++)
  {
    for(int k = 0;k<bots[j].bt_destinations.size();k++){
    if(!bots[j].bt_destinations[k].valid || bots[j].bt_destinations[k].manhattan_distance<0)//refer line 491
      continue;
    if(k!=10000000)//almost unnecesary condition
      {
        mind[j] = k;//closest valid backtracking point
        it = k;
        if(bots[j].bt_destinations[it].manhattan_distance < min_global_manhattan_dist)
        {
          min_global_manhattan_dist = bots[j].bt_destinations[it].manhattan_distance;
          min_j_index = j;
        }
      }//if k
     
    int i;
    for(i = 0;i<bots.size();i++){
      if(bots[i].robot_id == origin_id || bots[i].robot_id == robot_id)//the tag is actually the origin or current robot itself
        continue;

      int tp = bots[i].backtrackSimulateBid(bots[j].bt_destinations[it].next_p,testbed);// returns 10000000 if no path, checks if this particular backtrack point can be reached by other bot in lesser steps
      if(tp<bots[j].bt_destinations[it].manhattan_distance)//a closer bot is available
        {
          cout<<"some other bot can reach earlier!\n";
          break;
        }
      }//for i
      if(i == bots.size()){
        cout<<"valid bt point for this found!"<<endl;
        valid_found = true;
        if(bots[j].bt_destinations[it].manhattan_distance < min_valid_distance)
        {
          min_valid_distance = bots[j].bt_destinations[it].manhattan_distance;
          min_valid_index = j;
        }    
      break;
      } //if(i==)
    }//for k
  }//for j

  int bot_index = 0;
  if(!valid_found && mind[min_j_index] == 10000000){//no bt point left
    status = 2;
    cout<<"no bt point left for robot "<<robot_tag_id<<endl;
    BSA_CMSearchForBTAmongstUEV(testbed, bots, incumbent_cells, ic_no, sk);
    return;
  }
  if(!valid_found && mind[min_j_index] != 10000000)//no point exists for which the given robot is the closest
    {
      status = 1;
      it = mind[min_j_index];
      bot_index = min_j_index;
    }
  else if(valid_found)
  {
    status = 1;
    it = mind[min_valid_index];
    bot_index = min_valid_index;
  }
  //else it stores the index of the next bt point
  //sk = bt_destinations[it].stack_state;
  addBacktrackPointToStackAndPath(sk,incumbent_cells,ic_no,bots[bot_index].bt_destinations[it].next_p.first,bots[bot_index].bt_destinations[it].next_p.second,bots[bot_index].bt_destinations[it].parent,testbed);
}//function

int PathPlannerGrid::backtrackSimulateBidForBoustrophedonMotion(pair<int,int> target,AprilInterfaceAndVideoCapture &testbed){
  if(setRobotCellCoordinates(testbed.detections)<0)//set the start_grid_y, start_grid_x though we don't needto use them in this function(but is just a weak confirmation that the robot is in current view), doesn't take into account whether the robot is in the current view or not(the variables might be set from before), you need to check it before calling this function to ensure correct response
      return 10000000;
    if(sk.empty())//the robot is inactive
      return 10000000;//it can't ever reach
    if(status==1 || status==2)
      return 10000000;//the bot is in either return mode.
    stack<pair<int,int> > skc = sk;
  vector<vector<nd> > world_gridc = world_grid;//copy current grid
  vector<vector<nd> > tp;//a temporary map
  PathPlannerGrid plannerc(tp);
  plannerc.rcells = rcells;
  plannerc.ccells = ccells;
  plannerc.world_grid = world_gridc;//world_gridc won't be copied since world_grid is a reference variable
  int nx,ny,ngr,ngc,wall;//neighbor row and column
  int step_distance = 0;
  vector <pair<int,int>> preference(4);
    preference[0].first = -1, preference[0].second = 0; 
    preference[1].first = 1, preference[1].second = 0; 
    preference[2].first = 0, preference[2].second = 1; 
    preference[3].first = 0, preference[3].second = -1; 
  while(true){
    pair<int,int> t = skc.top();
    nx = t.first-world_gridc[t.first][t.second].parent.first+1;//add one to avoid negative index
    ny = t.second-world_gridc[t.first][t.second].parent.second+1;
    bool empty_neighbor_found = false;
    for(int i = 0;i<4;i++){
      ngr = t.first+preference[i].first;//priority in following order: up-down-right-left
      ngc = t.second+preference[i].second;
      //if(plannerc.isBlocked(ngr,ngc))
      if(!isEmpty(ngr,ngc) || world_gridc[ngr][ngc].steps)//don't know why but isBlocked is not working here in some cases
        continue;
      empty_neighbor_found = true;
      world_gridc[ngr][ngc].steps = 1;
      world_gridc[ngr][ngc].parent = t;
      world_gridc[ngr][ngc].r_id = robot_tag_id;
      skc.push(pair<int,int>(ngr,ngc));
      step_distance++;
      if(ngr == target.first && ngc == target.second)//no need to go any further
        return step_distance;
      else
        break;//already added a new point in path point
    }
    if(empty_neighbor_found) continue;
    break;//if control reaches here, it means that the spiral phase simulation is complete and the target was not visited 
  }//while

  vector<vector<nd> > tp2;//a temporary map
  PathPlannerGrid temp_planner(tp2);
  temp_planner.gridInversion(plannerc, robot_tag_id);
  temp_planner.start_grid_x = skc.top().first;
  temp_planner.start_grid_y = skc.top().second;
  temp_planner.goal_grid_x = target.first;
  temp_planner.goal_grid_y = target.second;
  temp_planner.findshortest(testbed);
  if(temp_planner.total_points==-1)
  {
    cout<<"the robot can't return to given target\n";
    return 10000000;//the robot can't return to given target
  }
  step_distance += temp_planner.total_points;
  return step_distance;//there might be a better way via some other adj cell, but difference would be atmost 1 unit cell
}//function

void PathPlannerGrid::BoustrophedonMotionSearchForBTAmongstUEV(AprilInterfaceAndVideoCapture &testbed, vector<PathPlannerGrid> &bots, vector<pair<int,int> > &incumbent_cells, int ic_no, stack<pair<int,int> > &sk){

 for(int j = 0; j < bots.size(); j++)
    {
        for(int i = 0;i<bots[j].uev_destinations.size();i++){
        //check if backtrackpoint has been yet visited or not, or if that is still valid
          pair <int, int> backtrack_parent;
          backtrack_parent.first = bots[j].uev_destinations[i].parent.first;
          backtrack_parent.second = bots[j].uev_destinations[i].parent.second;

          if(!bots[j].uev_destinations[i].valid || world_grid[bots[j].uev_destinations[i].next_p.first][bots[j].uev_destinations[i].next_p.second].steps>0 /*|| !bots[j].checkBactrackValidityForBSA_CM(backtrack_parent)*/){//the uev is no longer uncovered or backtack conditions no longer remain
            bots[j].uev_destinations[i].valid = false;//the point should no longer be considered in future
            continue;
          }

          //next 30 lines are to determine if there exist a parent for this uev_point, with smaller path (needed because just after this we are calculating the distance till the parents and sorting accordingly)
          int best_parent_x, best_parent_y; 
          int min_total_points = 10000000, min_total_points_index;
          int flag = 0;
          for(int k = 0; k < 4; k++)
          {
            best_parent_x = bots[j].uev_destinations[i].next_p.first + aj[0][1][k].first;
            best_parent_y = bots[j].uev_destinations[i].next_p.second + aj[0][1][k].second;
            if(isBlocked(best_parent_x, best_parent_y))
            {
              vector<vector<nd> > tp;//a temporary map
              PathPlannerGrid temp_planner(tp);
              temp_planner.gridInversion(*this, bots[j].robot_tag_id);
              temp_planner.start_grid_x = start_grid_x;//the current robot coordinates
              temp_planner.start_grid_y = start_grid_y;
              temp_planner.goal_grid_x = best_parent_x;
              temp_planner.goal_grid_y = best_parent_y;
              temp_planner.findshortest(testbed);
              if(temp_planner.total_points< min_total_points && temp_planner.total_points >= 0)
              {
                min_total_points = temp_planner.total_points;
                min_total_points_index = k;
                flag = 1;
              }
            }
          }
          if(flag == 1)
          {
            bots[j].uev_destinations[i].parent.first = bots[j].uev_destinations[i].next_p.first + aj[0][1][min_total_points_index].first;
            bots[j].uev_destinations[i].parent.second = bots[j].uev_destinations[i].next_p.second + aj[0][1][min_total_points_index].second;
          }//best parent assigned

          vector<vector<nd> > tp;//a temporary map
          PathPlannerGrid temp_planner(tp);
          temp_planner.gridInversion(*this, bots[j].robot_tag_id);
          temp_planner.start_grid_x = start_grid_x;//the current robot coordinates
          temp_planner.start_grid_y = start_grid_y;
          temp_planner.goal_grid_x = bots[j].uev_destinations[i].parent.first;
          temp_planner.goal_grid_y = bots[j].uev_destinations[i].parent.second;
          temp_planner.findshortest(testbed);
          bots[j].uev_destinations[i].manhattan_distance = temp_planner.total_points;//-1 if no path found

        }//for i
        
        sort(bots[j].uev_destinations.begin(),bots[j].uev_destinations.end(),[](const uev &a, const uev &b) -> bool{
          return a.manhattan_distance<b.manhattan_distance;
          });
      }//for j

    int it = 10000000;
    vector <int> mind(bots.size(),10000000);
    bool valid_found = false;
    int min_global_manhattan_dist = 10000000, min_j_index = 0;
    int min_valid_distance = 10000000, min_valid_index = 0;
    for(int j = 0; j < bots.size(); j++)
    {
        for(int k = 0;k<bots[j].uev_destinations.size();k++){
        if(!bots[j].uev_destinations[k].valid || bots[j].uev_destinations[k].manhattan_distance<0)//refer line 491
          continue;
        if(k!=10000000)//almost unnecesary condition
          {
            mind[j] = k;//closest valid backtracking point
            it = k;
            if(bots[j].uev_destinations[it].manhattan_distance < min_global_manhattan_dist)
            {
              min_global_manhattan_dist = bots[j].uev_destinations[it].manhattan_distance;
              min_j_index = j;
            }
          }//if k
         
        int i;
        for(i = 0;i<bots.size();i++){
          if(bots[i].robot_id == origin_id || bots[i].robot_id == robot_id)//the tag is actually the origin or current robot itself
            continue;
          //all planners must share the same map
          cout<<bots[i].robot_tag_id<<": Going for backtrack simulation bid!"<<endl;
          int tp = bots[i].backtrackSimulateBidForBoustrophedonMotion(bots[j].uev_destinations[it].next_p,testbed);// returns 10000000 if no path, checks if this particular backtrack point can be reached by other bot in lesser steps
          if(tp<bots[j].uev_destinations[it].manhattan_distance)//a closer bot is available
            {
              cout<<"some other bot can reach earlier!\n";
              break;
            }
          }//for i
          if(i == bots.size()){
            cout<<"valid uev point for this found!"<<endl;
            valid_found = true;
            if(bots[j].uev_destinations[it].manhattan_distance < min_valid_distance)
            {
              min_valid_distance = bots[j].uev_destinations[it].manhattan_distance;
              min_valid_index = j;
            }
          cout<<"the best uev point yet is: "<<bots[min_valid_index].uev_destinations[mind[min_valid_index]].next_p.first<<" "<<bots[min_valid_index].uev_destinations[mind[min_valid_index]].next_p.second<<endl;
          break;
          } //if(i==)
        }//for k
    }//for j

    int bot_index = 0;
    if(!valid_found && mind[min_j_index] == 10000000){//no uev point left
        status = 2;
        cout<<"no uev point left for robot "<<robot_tag_id<<endl;
        return;
    }
    if(!valid_found && mind[min_j_index] != 10000000)//no point exists for which the given robot is the closest
    {
        status = 1;
        it = mind[min_j_index];
        bot_index = min_j_index;
    }
    else if(valid_found)
    {
        status = 1;
        it = mind[min_valid_index];
        bot_index = min_valid_index;
    }
      //else it stores the index of the next uev point
      //sk = uev_destinations[it].stack_state;
    cout<<"Just found the best backtrack point. Adding it!\n";
    addBacktrackPointToStackAndPath(sk,incumbent_cells,ic_no,bots[bot_index].uev_destinations[it].next_p.first,bots[bot_index].uev_destinations[it].next_p.second,bots[bot_index].uev_destinations[it].parent,testbed);
}

void PathPlannerGrid::BoustrophedonMotionWithUpdatedBactrackSelection(AprilInterfaceAndVideoCapture &testbed, robot_pose &ps, double reach_distance, vector<PathPlannerGrid> &bots){
  if(setRobotCellCoordinates(testbed.detections)<0)//set the start_grid_y, start_grid_x
  return;
    if(!first_call){
      for(int i = 0; i< bt_destinations.size();i++){
        pair<int, int> backtrack_parent;
        if(!bt_destinations[i].valid || world_grid[bt_destinations[i].next_p.first][bt_destinations[i].next_p.second].steps>0 /*|| !checkBactrackValidityForBSA_CM(backtrack_parent)*/){//the bt is no longer uncovered or backtack conditions no longer remain
            bt_destinations[i].valid = false;//the point should no longer be considered in future
            continue;
        }
      } 

      for(int i = 0; i< uev_destinations.size();i++){
      if(!uev_destinations[i].valid || world_grid[uev_destinations[i].next_p.first][uev_destinations[i].next_p.second].steps>0 ){//the bt is no longer uncovered or backtack conditions no longer remain
          uev_destinations[i].valid = false;//the point should no longer be considered in future
          continue;
        }
      }
    //cout<<"in second and subsequent calls"<<endl;
      if(!sk.empty()){
        pair<int,int> t = sk.top();
        world_grid[start_grid_x][start_grid_y].bot_presence = make_pair(1, robot_tag_id); //assigning bot presence bit to current cell, //this would come to use in collision avoidance algorithm
        if(last_grid_x != start_grid_x || last_grid_y != start_grid_y)
        {
          world_grid[last_grid_x][last_grid_y].bot_presence = make_pair(0, -1);
          last_grid_x = start_grid_x;
          last_grid_y = start_grid_y;
        }    
      if(!checkReachStatus(t, ps, reach_distance)){//ensure the robot is continuing from the last point, and that no further planning occurs until the robot reaches the required point
          cout<<"the robot has not yet reached the old target"<<t.first<<" "<<t.second<<endl;
          return;
        }
      }
    //checking validit of backtrackpoints made as of now.
      
    }//if !first call

    vector<pair<int,int> > incumbent_cells(rcells*ccells);//make sure rcells and ccells are defined
    int ic_no = 0;

    if(first_call){
      first_call = 0;
      total_points = 0;
      sk.push(pair<int,int>(start_grid_x,start_grid_y));
      world_grid[start_grid_x][start_grid_y].parent = setParentUsingOrientation(ps);
      world_grid[start_grid_x][start_grid_y].steps = 1;//visited
      world_grid[start_grid_x][start_grid_y].r_id = robot_tag_id;
      world_grid[start_grid_x][start_grid_y].bot_presence = make_pair(1, robot_tag_id);
      last_grid_x = start_grid_x;
      last_grid_y = start_grid_y;
      start_cell_x = start_grid_x;
      start_cell_y = start_grid_y;
      target_grid_cell = make_pair(start_grid_x, start_grid_y);
      addGridCellToPath(start_grid_x,start_grid_y,testbed);//add the current robot position as target point on first call, on subsequent calls the robot position would already be on the stack from the previous call assuming the function is called only when the robot has reached the next point
      
      return;//added the first spiral point
    } 

    int ngr, ngc;

    while(!sk.empty()){
      pair<int,int> t = sk.top();
      //following two points though needed activally in BSA-CM, here they are just to add the (updated) backtrack points.
      int nx = t.first-world_grid[t.first][t.second].parent.first+1;//add one to avoid negative index
      int ny = t.second-world_grid[t.first][t.second].parent.second+1;
      
      vector <pair<int,int>> preference(4);
      preference[0].first = -1, preference[0].second = 0; 
      preference[1].first = 1, preference[1].second = 0; 
      preference[2].first = 0, preference[2].second = 1; 
      preference[3].first = 0, preference[3].second = -1; 

      bool empty_neighbor_found = false;

      for(int i = 0;i<4;i++){
        ngr = t.first+preference[i].first;//priority in following order: up-down-right-left
        ngc = t.second+preference[i].second;
        if(isBlocked(ngr,ngc))continue;
        empty_neighbor_found = true;
        if(ic_no == 0){
          status = 0;
          for(int j = 1; j < 3; j++)
            {
              int ngr2 = t.first+aj[nx][ny][j].first;//check the right and left points in consecutive iterations
              int ngc2 = t.second+aj[nx][ny][j].second;
              if(ngr2==ngr && ngc2 == ngc) continue;
              if(!isBlocked(ngr2, ngc2))
              {
                if(bactrackValidityForBSA_CM(t, nx, ny, j-1))
                {
                  bt_destinations.push_back(bt(t.first,t.second,ngr2,ngc2,sk));
                }
              }
            }
        int nx2 = ngr-t.first+1;//add one to avoid negative index
        int ny2 = ngc-t.second+1;
        for(int j = 1; j < 3; j++)//checking the backtrack point, as it's necessary to check it before adding point. this is because once the point is added the front cell is automatically treated as an obsatcle and the cell classifies as backtrack cell
        {
          int ngr3 = t.first+aj[nx2][ny2][j].first;//check the right and left points in consecutive iterations
          int ngc3 = t.second+aj[nx2][ny2][j].second;
                           
          if(ngr3==ngr && ngc3 == ngc) continue;
          if(!isBlocked(ngr3, ngc3))
          {
            if(bactrackValidityForBSA_CM(t, nx2, ny2, j-1))
            {
              bt_destinations.push_back(bt(t.first,t.second,ngr3,ngc3,sk));
            }
          }
        }
        addBacktrackPointToStackAndPath(sk,incumbent_cells,ic_no,ngr,ngc,t,testbed);
        for(int j = i+1; j < 4; j++)//incremental addition of backtracking points
        {
          ngr = t.first+preference[j].first;
          ngc = t.second+preference[j].second;
          if(!isBlocked(ngr, ngc))
          {
            uev_destinations.push_back(uev(t.first,t.second,ngr,ngc,sk));
            cout<<"added a new uev point "<<ngr<<" "<<ngc<<endl;            
          }
        }                    
        break; //for i
      }//if ic_no
    }//for i 
    if(empty_neighbor_found && ic_no == 0) break;//a new spiral point has been added and this is not a backtrack iteration
      incumbent_cells[ic_no] = t;//add the point as a possible return phase point
      ic_no++;
      sk.pop();
      //cout<<"popped the top of stack"<<endl;
      if(sk.empty()) break;//no new spiral point was added, there might be some bt points available 
      pair<int,int> next_below = sk.top();
      //the lines below are obsolete(at first thought) since the shortest path is being calculated, so wall reference and parent are obsolete on already visited points
      world_grid[next_below.first][next_below.second].parent = t;
      world_grid[next_below.first][next_below.second].wall_reference = 1;//since turning 180 degrees  
    }//while
    if(ic_no == 0 && !sk.empty()) return;
  if(sk.empty())
  {
    ic_no = 5;//any number greater than 0;
    incumbent_cells[0].first = start_grid_x;
    incumbent_cells[0].second = start_grid_y;
  }
  status = 1;
  for(int j = 0; j < bots.size(); j++)
  {
      for(int i = 0;i<bots[j].bt_destinations.size();i++){
      //check if backtrackpoint has been yet visited or not, or if that is still valid
        pair <int, int> backtrack_parent;
        backtrack_parent.first = bots[j].bt_destinations[i].parent.first;
        backtrack_parent.second = bots[j].bt_destinations[i].parent.second;

        if(!bots[j].bt_destinations[i].valid || world_grid[bots[j].bt_destinations[i].next_p.first][bots[j].bt_destinations[i].next_p.second].steps>0 /*|| !bots[j].checkBactrackValidityForBSA_CM(backtrack_parent)*/){//the bt is no longer uncovered or backtack conditions no longer remain
          bots[j].bt_destinations[i].valid = false;//the point should no longer be considered in future
          continue;
        }

        //next 30 lines are to determine if there exist a parent for this bt_point, with smaller path (needed because just after this we are calculating the distance till the parents and sorting accordingly)
        int best_parent_x, best_parent_y; 
        int min_total_points = 10000000, min_total_points_index;
        int flag = 0;
        for(int k = 0; k < 4; k++)
        {
          best_parent_x = bots[j].bt_destinations[i].next_p.first + aj[0][1][k].first;
          best_parent_y = bots[j].bt_destinations[i].next_p.second + aj[0][1][k].second;
          if(isBlocked(best_parent_x, best_parent_y))
          {
            vector<vector<nd> > tp;//a temporary map
            PathPlannerGrid temp_planner(tp);
            temp_planner.gridInversion(*this, bots[j].robot_tag_id);
            temp_planner.start_grid_x = start_grid_x;//the current robot coordinates
            temp_planner.start_grid_y = start_grid_y;
            temp_planner.goal_grid_x = best_parent_x;
            temp_planner.goal_grid_y = best_parent_y;
            temp_planner.findshortest(testbed);
            if(temp_planner.total_points< min_total_points && temp_planner.total_points >= 0)
            {
              min_total_points = temp_planner.total_points;
              min_total_points_index = k;
              flag = 1;
            }
          }
        }
        if(flag == 1)
        {
          bots[j].bt_destinations[i].parent.first = bots[j].bt_destinations[i].next_p.first + aj[0][1][min_total_points_index].first;
          bots[j].bt_destinations[i].parent.second = bots[j].bt_destinations[i].next_p.second + aj[0][1][min_total_points_index].second;
        }//best parent assigned

        vector<vector<nd> > tp;//a temporary map
        PathPlannerGrid temp_planner(tp);
        temp_planner.gridInversion(*this, bots[j].robot_tag_id);
        temp_planner.start_grid_x = start_grid_x;//the current robot coordinates
        temp_planner.start_grid_y = start_grid_y;
        temp_planner.goal_grid_x = bots[j].bt_destinations[i].parent.first;
        temp_planner.goal_grid_y = bots[j].bt_destinations[i].parent.second;
        temp_planner.findshortest(testbed);
        bots[j].bt_destinations[i].manhattan_distance = temp_planner.total_points;//-1 if no path found

      }//for i
      
      sort(bots[j].bt_destinations.begin(),bots[j].bt_destinations.end(),[](const bt &a, const bt &b) -> bool{
        return a.manhattan_distance<b.manhattan_distance;
        });
    }//for j

  int it = 10000000;
  vector <int> mind(bots.size(),10000000);
  bool valid_found = false;
  int min_global_manhattan_dist = 10000000, min_j_index = 0;
  int min_valid_distance = 10000000, min_valid_index = 0;
  for(int j = 0; j < bots.size(); j++)
  {
      for(int k = 0;k<bots[j].bt_destinations.size();k++){
      if(!bots[j].bt_destinations[k].valid || bots[j].bt_destinations[k].manhattan_distance<0)//refer line 491
        continue;
      if(k!=10000000)//almost unnecesary condition
        {
          mind[j] = k;//closest valid backtracking point
          it = k;
          if(bots[j].bt_destinations[it].manhattan_distance < min_global_manhattan_dist)
          {
            min_global_manhattan_dist = bots[j].bt_destinations[it].manhattan_distance;
            min_j_index = j;
          }
        }//if k
       
      int i;
      for(i = 0;i<bots.size();i++){
        if(bots[i].robot_id == origin_id || bots[i].robot_id == robot_id)//the tag is actually the origin or current robot itself
          continue;
        //all planners must share the same map
        int tp = bots[i].backtrackSimulateBidForBoustrophedonMotion(bots[j].bt_destinations[it].next_p,testbed);// returns 10000000 if no path, checks if this particular backtrack point can be reached by other bot in lesser steps
        if(tp<bots[j].bt_destinations[it].manhattan_distance)//a closer bot is available
          {
            cout<<"some other bot can reach earlier!\n";
            break;
          }
        }//for i
        if(i == bots.size()){
          cout<<"valid bt point for this found!"<<endl;
          valid_found = true;
          if(bots[j].bt_destinations[it].manhattan_distance < min_valid_distance)
          {
            min_valid_distance = bots[j].bt_destinations[it].manhattan_distance;
            min_valid_index = j;
          }        
        break;
        } //if(i==)
      }//for k
  }//for j

  int bot_index = 0;
  if(!valid_found && mind[min_j_index] == 10000000){//no bt point left
      status = 2;
      BoustrophedonMotionSearchForBTAmongstUEV(testbed, bots, incumbent_cells, ic_no, sk);
      return;
  }
  if(!valid_found && mind[min_j_index] != 10000000)//no point exists for which the given robot is the closest
  {
      status = 1;
      it = mind[min_j_index];
      bot_index = min_j_index;
  }
  else if(valid_found)
  {
      status = 1;
      it = mind[min_valid_index];
      bot_index = min_valid_index;
  }
    //else it stores the index of the next bt point
    //sk = bt_destinations[it].stack_state;
  cout<<"Just found the best backtrack point. Adding it!\n";
  addBacktrackPointToStackAndPath(sk,incumbent_cells,ic_no,bots[bot_index].bt_destinations[it].next_p.first,bots[bot_index].bt_destinations[it].next_p.second,bots[bot_index].bt_destinations[it].parent,testbed);
}//function

void PathPlannerGrid::BoustrophedonMotionWithBSA_CMlikeBacktracking(AprilInterfaceAndVideoCapture &testbed, robot_pose &ps, double reach_distance, vector<PathPlannerGrid> &bots){
  if(setRobotCellCoordinates(testbed.detections)<0)//set the start_grid_y, start_grid_x
  return;
    if(!first_call){
      for(int i = 0; i< bt_destinations.size();i++){
        pair<int, int> backtrack_parent;
        if(!bt_destinations[i].valid || world_grid[bt_destinations[i].next_p.first][bt_destinations[i].next_p.second].steps>0 /*|| !checkBactrackValidityForBSA_CM(backtrack_parent)*/){//the bt is no longer uncovered or backtack conditions no longer remain
            bt_destinations[i].valid = false;//the point should no longer be considered in future
            continue;
        }
      } 

      for(int i = 0; i< uev_destinations.size();i++){
      if(!uev_destinations[i].valid || world_grid[uev_destinations[i].next_p.first][uev_destinations[i].next_p.second].steps>0 ){//the bt is no longer uncovered or backtack conditions no longer remain
          uev_destinations[i].valid = false;//the point should no longer be considered in future
          continue;
        }
      }
    //cout<<"in second and subsequent calls"<<endl;
      if(!sk.empty()){
        pair<int,int> t = sk.top();
        world_grid[start_grid_x][start_grid_y].bot_presence = make_pair(1, robot_tag_id); //assigning bot presence bit to current cell, //this would come to use in collision avoidance algorithm
        if(last_grid_x != start_grid_x || last_grid_y != start_grid_y)
        {
          world_grid[last_grid_x][last_grid_y].bot_presence = make_pair(0, -1);
          last_grid_x = start_grid_x;
          last_grid_y = start_grid_y;
        }    
      if(!checkReachStatus(t, ps, reach_distance)){//ensure the robot is continuing from the last point, and that no further planning occurs until the robot reaches the required point
          cout<<"the robot has not yet reached the old target"<<t.first<<" "<<t.second<<endl;
          return;
        }
      }
    //checking validit of backtrackpoints made as of now.
      
    }//if !first call

    vector<pair<int,int> > incumbent_cells(rcells*ccells);//make sure rcells and ccells are defined
    int ic_no = 0;

    if(first_call){
      first_call = 0;
      total_points = 0;
      sk.push(pair<int,int>(start_grid_x,start_grid_y));
      world_grid[start_grid_x][start_grid_y].parent = setParentUsingOrientation(ps);
      world_grid[start_grid_x][start_grid_y].steps = 1;//visited
      world_grid[start_grid_x][start_grid_y].r_id = robot_tag_id;
      world_grid[start_grid_x][start_grid_y].bot_presence = make_pair(1, robot_tag_id);
      last_grid_x = start_grid_x;
      last_grid_y = start_grid_y;
      start_cell_x = start_grid_x;
      start_cell_y = start_grid_y;
      target_grid_cell = make_pair(start_grid_x, start_grid_y);
      addGridCellToPath(start_grid_x,start_grid_y,testbed);//add the current robot position as target point on first call, on subsequent calls the robot position would already be on the stack from the previous call assuming the function is called only when the robot has reached the next point
      
      return;//added the first spiral point
    } 

    int ngr, ngc;

    while(!sk.empty()){
      pair<int,int> t = sk.top();
      //following two points though needed activally in BSA-CM, here they are just to add the (updated) backtrack points.
      int nx = t.first-world_grid[t.first][t.second].parent.first+1;//add one to avoid negative index
      int ny = t.second-world_grid[t.first][t.second].parent.second+1;
      
      vector <pair<int,int>> preference(4);
      preference[0].first = -1, preference[0].second = 0; 
      preference[1].first = 1, preference[1].second = 0; 
      preference[2].first = 0, preference[2].second = 1; 
      preference[3].first = 0, preference[3].second = -1; 

      bool empty_neighbor_found = false;

      for(int i = 0;i<4;i++){
        ngr = t.first+preference[i].first;//priority in following order: up-down-right-left
        ngc = t.second+preference[i].second;
        if(isBlocked(ngr,ngc))continue;
        empty_neighbor_found = true;
        if(ic_no == 0){
          status = 0;          
          addBacktrackPointToStackAndPath(sk,incumbent_cells,ic_no,ngr,ngc,t,testbed);
          for(int j = i+1; j < 4; j++)//incremental addition of backtracking points
          {
          ngr = t.first+preference[j].first;
          ngc = t.second+preference[j].second;
          if(!isBlocked(ngr, ngc))
            {
              bt_destinations.push_back(bt(t.first,t.second,ngr,ngc,sk));
              uev_destinations.push_back(uev(t.first,t.second,ngr,ngc,sk));
              cout<<"added a new uev point "<<ngr<<" "<<ngc<<endl;            
            }
          }                    
          break; //for i
        }//if ic_no
    }//for i 
    if(empty_neighbor_found && ic_no == 0) break;//a new spiral point has been added and this is not a backtrack iteration
      incumbent_cells[ic_no] = t;//add the point as a possible return phase point
      ic_no++;
      sk.pop();
      //cout<<"popped the top of stack"<<endl;
      if(sk.empty()) break;//no new spiral point was added, there might be some bt points available 
      pair<int,int> next_below = sk.top();
      //the lines below are obsolete(at first thought) since the shortest path is being calculated, so wall reference and parent are obsolete on already visited points
      world_grid[next_below.first][next_below.second].parent = t;
      world_grid[next_below.first][next_below.second].wall_reference = 1;//since turning 180 degrees  
    }//while
    if(ic_no == 0 && !sk.empty()) return;
  if(sk.empty())
  {
    ic_no = 5;//any number greater than 0;
    incumbent_cells[0].first = start_grid_x;
    incumbent_cells[0].second = start_grid_y;
  }
  status = 1;
  for(int j = 0; j < bots.size(); j++)
  {
      for(int i = 0;i<bots[j].bt_destinations.size();i++){
      //check if backtrackpoint has been yet visited or not, or if that is still valid
        pair <int, int> backtrack_parent;
        backtrack_parent.first = bots[j].bt_destinations[i].parent.first;
        backtrack_parent.second = bots[j].bt_destinations[i].parent.second;

        if(!bots[j].bt_destinations[i].valid || world_grid[bots[j].bt_destinations[i].next_p.first][bots[j].bt_destinations[i].next_p.second].steps>0 /*|| !bots[j].checkBactrackValidityForBSA_CM(backtrack_parent)*/){//the bt is no longer uncovered or backtack conditions no longer remain
          bots[j].bt_destinations[i].valid = false;//the point should no longer be considered in future
          continue;
        }

        //next 30 lines are to determine if there exist a parent for this bt_point, with smaller path (needed because just after this we are calculating the distance till the parents and sorting accordingly)
        int best_parent_x, best_parent_y; 
        int min_total_points = 10000000, min_total_points_index;
        int flag = 0;
        for(int k = 0; k < 4; k++)
        {
          best_parent_x = bots[j].bt_destinations[i].next_p.first + aj[0][1][k].first;
          best_parent_y = bots[j].bt_destinations[i].next_p.second + aj[0][1][k].second;
          if(isBlocked(best_parent_x, best_parent_y))
          {
            vector<vector<nd> > tp;//a temporary map
            PathPlannerGrid temp_planner(tp);
            temp_planner.gridInversion(*this, bots[j].robot_tag_id);
            temp_planner.start_grid_x = start_grid_x;//the current robot coordinates
            temp_planner.start_grid_y = start_grid_y;
            temp_planner.goal_grid_x = best_parent_x;
            temp_planner.goal_grid_y = best_parent_y;
            temp_planner.findshortest(testbed);
            if(temp_planner.total_points< min_total_points && temp_planner.total_points >= 0)
            {
              min_total_points = temp_planner.total_points;
              min_total_points_index = k;
              flag = 1;
            }
          }
        }
        if(flag == 1)
        {
          bots[j].bt_destinations[i].parent.first = bots[j].bt_destinations[i].next_p.first + aj[0][1][min_total_points_index].first;
          bots[j].bt_destinations[i].parent.second = bots[j].bt_destinations[i].next_p.second + aj[0][1][min_total_points_index].second;
        }//best parent assigned

        vector<vector<nd> > tp;//a temporary map
        PathPlannerGrid temp_planner(tp);
        temp_planner.gridInversion(*this, bots[j].robot_tag_id);
        temp_planner.start_grid_x = start_grid_x;//the current robot coordinates
        temp_planner.start_grid_y = start_grid_y;
        temp_planner.goal_grid_x = bots[j].bt_destinations[i].parent.first;
        temp_planner.goal_grid_y = bots[j].bt_destinations[i].parent.second;
        temp_planner.findshortest(testbed);
        bots[j].bt_destinations[i].manhattan_distance = temp_planner.total_points;//-1 if no path found

      }//for i
      
      sort(bots[j].bt_destinations.begin(),bots[j].bt_destinations.end(),[](const bt &a, const bt &b) -> bool{
        return a.manhattan_distance<b.manhattan_distance;
        });
    }//for j

  int it = 10000000;
  vector <int> mind(bots.size(),10000000);
  bool valid_found = false;
  int min_global_manhattan_dist = 10000000, min_j_index = 0;
  int min_valid_distance = 10000000, min_valid_index = 0;
  for(int j = 0; j < bots.size(); j++)
  {
      for(int k = 0;k<bots[j].bt_destinations.size();k++){
      if(!bots[j].bt_destinations[k].valid || bots[j].bt_destinations[k].manhattan_distance<0)//refer line 491
        continue;
      if(k!=10000000)//almost unnecesary condition
        {
          mind[j] = k;//closest valid backtracking point
          it = k;
          if(bots[j].bt_destinations[it].manhattan_distance < min_global_manhattan_dist)
          {
            min_global_manhattan_dist = bots[j].bt_destinations[it].manhattan_distance;
            min_j_index = j;
          }
        }//if k
       
      int i;
      for(i = 0;i<bots.size();i++){
        if(bots[i].robot_id == origin_id || bots[i].robot_id == robot_id)//the tag is actually the origin or current robot itself
          continue;
        //all planners must share the same map
        cout<<bots[i].robot_tag_id<<": Going for backtrack simulation bid!"<<endl;
        int tp = bots[i].backtrackSimulateBidForBoustrophedonMotion(bots[j].bt_destinations[it].next_p,testbed);// returns 10000000 if no path, checks if this particular backtrack point can be reached by other bot in lesser steps
        if(tp<bots[j].bt_destinations[it].manhattan_distance)//a closer bot is available
          {
            cout<<"some other bot can reach earlier!\n";
            break;
          }
        }//for i
        if(i == bots.size()){
          cout<<"valid bt point for this found!"<<endl;
          valid_found = true;
          if(bots[j].bt_destinations[it].manhattan_distance < min_valid_distance)
          {
            min_valid_distance = bots[j].bt_destinations[it].manhattan_distance;
            min_valid_index = j;
          }
        cout<<"the best bt point yet is: "<<bots[min_valid_index].bt_destinations[mind[min_valid_index]].next_p.first<<" "<<bots[min_valid_index].bt_destinations[mind[min_valid_index]].next_p.second<<endl;
        break;
        } //if(i==)
      }//for k
  }//for j

  int bot_index = 0;
  if(!valid_found && mind[min_j_index] == 10000000){//no bt point left
      status = 2;
      cout<<"no bt point left for robot, Searching Amongst UEVs! "<<robot_tag_id<<endl;
      return;
  }
  if(!valid_found && mind[min_j_index] != 10000000)//no point exists for which the given robot is the closest
  {
      status = 1;
      it = mind[min_j_index];
      bot_index = min_j_index;
  }
  else if(valid_found)
  {
      status = 1;
      it = mind[min_valid_index];
      bot_index = min_valid_index;
  }
    //else it stores the index of the next bt point
    //sk = bt_destinations[it].stack_state;
  cout<<"Just found the best backtrack point. Adding it!\n";
  addBacktrackPointToStackAndPath(sk,incumbent_cells,ic_no,bots[bot_index].bt_destinations[it].next_p.first,bots[bot_index].bt_destinations[it].next_p.second,bots[bot_index].bt_destinations[it].parent,testbed);
}//function

void PathPlannerGrid::BoB(AprilInterfaceAndVideoCapture &testbed, robot_pose &ps, double reach_distance, vector<PathPlannerGrid> &bots){
  if(setRobotCellCoordinates(testbed.detections)<0)//set the start_grid_y, start_grid_x
  return;
    if(!first_call){
      for(int i = 0; i< bt_destinations.size();i++){
        pair<int, int> backtrack_parent;
        if(!bt_destinations[i].valid || world_grid[bt_destinations[i].next_p.first][bt_destinations[i].next_p.second].steps>0 /*|| !checkBactrackValidityForBSA_CM(backtrack_parent)*/){//the bt is no longer uncovered or backtack conditions no longer remain
            bt_destinations[i].valid = false;//the point should no longer be considered in future
            continue;
        }
      } 

      for(int i = 0; i< uev_destinations.size();i++){
      if(!uev_destinations[i].valid || world_grid[uev_destinations[i].next_p.first][uev_destinations[i].next_p.second].steps>0 ){//the bt is no longer uncovered or backtack conditions no longer remain
          uev_destinations[i].valid = false;//the point should no longer be considered in future
          continue;
        }
      }
    //cout<<"in second and subsequent calls"<<endl;
      if(!sk.empty()){
        pair<int,int> t = sk.top();
        world_grid[start_grid_x][start_grid_y].bot_presence = make_pair(1, robot_tag_id); //assigning bot presence bit to current cell, //this would come to use in collision avoidance algorithm
        if(last_grid_x != start_grid_x || last_grid_y != start_grid_y)
        {
          world_grid[last_grid_x][last_grid_y].bot_presence = make_pair(0, -1);
          last_grid_x = start_grid_x;
          last_grid_y = start_grid_y;
        }    
      if(!checkReachStatus(t, ps, reach_distance)){//ensure the robot is continuing from the last point, and that no further planning occurs until the robot reaches the required point
          cout<<"the robot has not yet reached the old target"<<t.first<<" "<<t.second<<endl;
          return;
        }
      }
    //checking validit of backtrackpoints made as of now.
      
    }//if !first call

    vector<pair<int,int> > incumbent_cells(rcells*ccells);//make sure rcells and ccells are defined
    int ic_no = 0;

    if(first_call){
      first_call = 0;
      total_points = 0;
      sk.push(pair<int,int>(start_grid_x,start_grid_y));
      world_grid[start_grid_x][start_grid_y].parent = setParentUsingOrientation(ps);
      world_grid[start_grid_x][start_grid_y].steps = 1;//visited
      world_grid[start_grid_x][start_grid_y].r_id = robot_tag_id;
      world_grid[start_grid_x][start_grid_y].bot_presence = make_pair(1, robot_tag_id);
      last_grid_x = start_grid_x;
      last_grid_y = start_grid_y;
      start_cell_x = start_grid_x;
      start_cell_y = start_grid_y;
      target_grid_cell = make_pair(start_grid_x, start_grid_y);
      addGridCellToPath(start_grid_x,start_grid_y,testbed);//add the current robot position as target point on first call, on subsequent calls the robot position would already be on the stack from the previous call assuming the function is called only when the robot has reached the next point
      
      return;//added the first spiral point
    } 

    int ngr, ngc;

    while(!sk.empty()){
      //coverage_completed = 0;
      pair<int,int> t = sk.top();
      //following two points though needed activally in BSA-CM, here they are just to add the (updated) backtrack points.
      int nx = t.first-world_grid[t.first][t.second].parent.first+1;//add one to avoid negative index
      int ny = t.second-world_grid[t.first][t.second].parent.second+1;
      
      vector <pair<int,int>> preference(4);
      preference[0].first = -1, preference[0].second = 0; 
      preference[1].first = 1, preference[1].second = 0; 
      preference[2].first = 0, preference[2].second = 1; 
      preference[3].first = 0, preference[3].second = -1; 

      bool empty_neighbor_found = false;

      for(int i = 0;i<4;i++){
        ngr = t.first+preference[i].first;//priority in following order: up-down-right-left
        ngc = t.second+preference[i].second;
        if(isBlocked(ngr,ngc))continue;
        empty_neighbor_found = true;
        if(ic_no == 0){
          status = 0;
          for(int j = 1; j < 3; j++)
            {
              int ngr2 = t.first+aj[nx][ny][j].first;//check the right and left points in consecutive iterations
              int ngc2 = t.second+aj[nx][ny][j].second;
              if(ngr2==ngr && ngc2 == ngc) continue;
              if(!isBlocked(ngr2, ngc2))
              {
                if(bactrackValidityForBoB(t, nx, ny, j-1))
                {
                  bt_destinations.push_back(bt(t.first,t.second,ngr2,ngc2,sk));
                }
              }
            }
        int nx2 = ngr-t.first+1;//add one to avoid negative index
        int ny2 = ngc-t.second+1;
        for(int j = 1; j < 3; j++)//checking the backtrack point, as it's necessary to check it before adding point. this is because once the point is added the front cell is automatically treated as an obsatcle and the cell classifies as backtrack cell
        {
          int ngr3 = t.first+aj[nx2][ny2][j].first;//check the right and left points in consecutive iterations
          int ngc3 = t.second+aj[nx2][ny2][j].second;
                           
          if(ngr3==ngr && ngc3 == ngc) continue;
          if(!isBlocked(ngr3, ngc3))
          {
            if(bactrackValidityForBoB(t, nx2, ny2, j-1))
            {
              bt_destinations.push_back(bt(t.first,t.second,ngr3,ngc3,sk));
            }
          }
        }  
        addBacktrackPointToStackAndPath(sk,incumbent_cells,ic_no,ngr,ngc,t,testbed);
        for(int j = i+1; j < 4; j++)//incremental addition of backtracking points
        {
          ngr = t.first+preference[j].first;
          ngc = t.second+preference[j].second;
          if(!isBlocked(ngr, ngc))
          {
            uev_destinations.push_back(uev(t.first,t.second,ngr,ngc,sk));
            cout<<"added a new uev point "<<ngr<<" "<<ngc<<endl;            
          }
        }                    
        break; //for i
      }//if ic_no
    }//for i 
    if(empty_neighbor_found && ic_no == 0) break;//a new spiral point has been added and this is not a backtrack iteration
      incumbent_cells[ic_no] = t;//add the point as a possible return phase point
      ic_no++;
      sk.pop();
      //cout<<"popped the top of stack"<<endl;
      if(sk.empty()) break;//no new spiral point was added, there might be some bt points available 
      pair<int,int> next_below = sk.top();
      //the lines below are obsolete(at first thought) since the shortest path is being calculated, so wall reference and parent are obsolete on already visited points
      world_grid[next_below.first][next_below.second].parent = t;
      world_grid[next_below.first][next_below.second].wall_reference = 1;//since turning 180 degrees  
    }//while
    if(ic_no == 0 && !sk.empty()) return;
  if(sk.empty())
  {
    ic_no = 5;//any number greater than 0;
    incumbent_cells[0].first = start_grid_x;
    incumbent_cells[0].second = start_grid_y;
  }
  status = 1;
  for(int j = 0; j < bots.size(); j++)
  {
      for(int i = 0;i<bots[j].bt_destinations.size();i++){
      //check if backtrackpoint has been yet visited or not, or if that is still valid
        pair <int, int> backtrack_parent;
        backtrack_parent.first = bots[j].bt_destinations[i].parent.first;
        backtrack_parent.second = bots[j].bt_destinations[i].parent.second;

        if(!bots[j].bt_destinations[i].valid || world_grid[bots[j].bt_destinations[i].next_p.first][bots[j].bt_destinations[i].next_p.second].steps>0 /*|| !bots[j].checkBactrackValidityForBSA_CM(backtrack_parent)*/){//the bt is no longer uncovered or backtack conditions no longer remain
          bots[j].bt_destinations[i].valid = false;//the point should no longer be considered in future
          continue;
        }

        //next 30 lines are to determine if there exist a parent for this bt_point, with smaller path (needed because just after this we are calculating the distance till the parents and sorting accordingly)
        int best_parent_x, best_parent_y; 
        int min_total_points = 10000000, min_total_points_index;
        int flag = 0;
        for(int k = 0; k < 4; k++)
        {
          best_parent_x = bots[j].bt_destinations[i].next_p.first + aj[0][1][k].first;
          best_parent_y = bots[j].bt_destinations[i].next_p.second + aj[0][1][k].second;
          if(isBlocked(best_parent_x, best_parent_y))
          {
            vector<vector<nd> > tp;//a temporary map
            PathPlannerGrid temp_planner(tp);
            temp_planner.gridInversion(*this, bots[j].robot_tag_id);
            temp_planner.start_grid_x = start_grid_x;//the current robot coordinates
            temp_planner.start_grid_y = start_grid_y;
            temp_planner.goal_grid_x = best_parent_x;
            temp_planner.goal_grid_y = best_parent_y;
            temp_planner.findshortest(testbed);
            if(temp_planner.total_points< min_total_points && temp_planner.total_points >= 0)
            {
              min_total_points = temp_planner.total_points;
              min_total_points_index = k;
              flag = 1;
            }
          }
        }
        if(flag == 1)
        {
          bots[j].bt_destinations[i].parent.first = bots[j].bt_destinations[i].next_p.first + aj[0][1][min_total_points_index].first;
          bots[j].bt_destinations[i].parent.second = bots[j].bt_destinations[i].next_p.second + aj[0][1][min_total_points_index].second;
        }//best parent assigned

        vector<vector<nd> > tp;//a temporary map
        PathPlannerGrid temp_planner(tp);
        temp_planner.gridInversion(*this, bots[j].robot_tag_id);
        temp_planner.start_grid_x = start_grid_x;//the current robot coordinates
        temp_planner.start_grid_y = start_grid_y;
        temp_planner.goal_grid_x = bots[j].bt_destinations[i].parent.first;
        temp_planner.goal_grid_y = bots[j].bt_destinations[i].parent.second;
        temp_planner.findshortest(testbed);
        bots[j].bt_destinations[i].manhattan_distance = temp_planner.total_points;//-1 if no path found

      }//for i
      
      sort(bots[j].bt_destinations.begin(),bots[j].bt_destinations.end(),[](const bt &a, const bt &b) -> bool{
        return a.manhattan_distance<b.manhattan_distance;
        });
    }//for j
  
  ic_no = 5;
  int min_bot;
  int min_bt_index;
  bool bt_found = false;
  int min_manhattan_distance = 10000000;
  for(int j = 0; j < bots.size(); j++)
  {
      for(int k = 0;k<bots[j].bt_destinations.size();k++){
        if(!bots[j].bt_destinations[k].valid || bots[j].bt_destinations[k].manhattan_distance<0)//refer line 491
          continue;      
        if(bots[j].bt_destinations[k].manhattan_distance<min_manhattan_distance)
        {
          bt_found = true;
          min_bt_index = k;
          min_bot = j;
          min_manhattan_distance = bots[j].bt_destinations[k].manhattan_distance;
        }      
      }//for k
  }//for j

  if(!bt_found)
  {
    status = 2;
    cout<<"no bt point left for robot!"<<robot_tag_id<<endl;
    return;
  }  
  cout<<"Just found the best backtrack point. Adding it!\n";
  addBacktrackPointToStackAndPath(sk,incumbent_cells,ic_no,bots[min_bot].bt_destinations[min_bt_index].next_p.first,bots[min_bot].bt_destinations[min_bt_index].next_p.second,bots[min_bot].bt_destinations[min_bt_index].parent,testbed);
}//function

void PathPlannerGrid::MDFS(AprilInterfaceAndVideoCapture &testbed, robot_pose &ps, double reach_distance, vector<PathPlannerGrid> &bots){
  if(setRobotCellCoordinates(testbed.detections)<0)//set the start_grid_y, start_grid_x
    return;
  if(!first_call){
    if(!sk.empty()){
        pair<int,int> t = sk.top();
        world_grid[start_grid_x][start_grid_y].bot_presence = make_pair(1, robot_tag_id); //assigning bot presence bit to current cell, //this would come to use in collision avoidance algorithm
        if(last_grid_x != start_grid_x || last_grid_y != start_grid_y)
        {
          world_grid[last_grid_x][last_grid_y].bot_presence = make_pair(0, -1);
          last_grid_x = start_grid_x;
          last_grid_y = start_grid_y;
        }    
      if(!checkReachStatus(t, ps, reach_distance)){//ensure the robot is continuing from the last point, and that no further planning occurs until the robot reaches the required point
          cout<<"the robot has not yet reached the old target"<<t.first<<" "<<t.second<<endl;
          int c = (pixel_path_points[next_target_index].first)/(cell_size_x);
          int r = (pixel_path_points[next_target_index].second)/(cell_size_y);
        /* if(world_grid[r][c].visited==1)
          {
            path_points[next_target_index].x = path_points[next_target_index-1].x;
            path_points[next_target_index].y = path_points[next_target_index-1].y;
            pixel_path_points[next_target_index].first = pixel_path_points[next_target_index-1].first;
            pixel_path_points[next_target_index].second = pixel_path_points[next_target_index-1].second;
          }*/
          return;
        }
      }
      if(coverage_completed) 
        {
          cout<<"coverage complete\n";
          return;
        }
  }//!first_call

     //next two are dummy variables, 
    vector<pair<int,int> > incumbent_cells(rcells*ccells);//make sure rcells and ccells are defined
    int ic_no = 0;

  if(first_call){
      first_call = 0;
      total_points = 0;
      sk.push(pair<int,int>(start_grid_x,start_grid_y));
      world_grid[start_grid_x][start_grid_y].parent = setParentUsingOrientation(ps);      
      world_grid[start_grid_x][start_grid_y].steps = 1;//visited
      world_grid[start_grid_x][start_grid_y].r_id = robot_tag_id;
      world_grid[start_grid_x][start_grid_y].tree_id = robot_tag_id;
      world_grid[start_grid_x][start_grid_y].bot_presence = make_pair(1, robot_tag_id);
      last_grid_x = start_grid_x;
      last_grid_y = start_grid_y;
      start_cell_x = start_grid_x;
      start_cell_y = start_grid_y;
      target_grid_cell = make_pair(start_grid_x, start_grid_y);
      addGridCellToPath(start_grid_x,start_grid_y,testbed);//add the current robot position as target point on first call, on subsequent calls the robot position would already be on the stack from the previous call assuming the function is called only when the robot has reached the next point
      return;//added the first spiral point
    }//first_call
 
    int ngr, ngc;
    if(!coverage_completed && !sk.empty())
    {
      pair<int,int> t = sk.top();
      //following two points though needed activally in BSA-CM, here they are just to add the (updated) backtrack points.
      int nx = t.first-world_grid[t.first][t.second].parent.first+1;//add one to avoid negative index
      int ny = t.second-world_grid[t.first][t.second].parent.second+1;

      coverage_completed = 1;
      for(int i = 0; i < 4; i++)
      {
        ngr = t.first + aj[nx][ny][i].first;
        ngc = t.second + aj[nx][ny][i].second;
        if(isEmpty(ngr, ngc) && world_grid[ngr][ngc].visited!=1)
        {
          coverage_completed = 0;
        }       
        if(!isBlocked(ngr,ngc))
        {
          addBacktrackPointToStackAndPath(sk,incumbent_cells,ic_no,ngr,ngc,t,testbed);
          world_grid[ngr][ngc].tree_parent = t;
          world_grid[ngr][ngc].tree_id = robot_tag_id;
          return;
        }
      }
      if(coverage_completed) return;

      if(world_grid[t.first][t.second].tree_id == robot_tag_id)
      {
        world_grid[t.first][t.second].visited = 1;
        if(world_grid[t.first][t.second].tree_parent.first != -1 && world_grid[t.first][t.second].tree_parent.first !=-1)
        {
          ngr = world_grid[t.first][t.second].tree_parent.first;
          ngc = world_grid[t.first][t.second].tree_parent.second;        
          addBacktrackPointToStackAndPath(sk,incumbent_cells,ic_no,ngr,ngc,t,testbed);
          return;
        }
        else//not the root, not part of psuedo code given, but part of theory, it sometimes causes it to be stuck in a loop
        {
          for(int i = 0; i < 4; i++)
          {
            ngr = t.first + aj[nx][ny][i].first;
            ngc = t.second + aj[nx][ny][i].second;
            if(isEmpty(ngr, ngc) && world_grid[ngr][ngc].tree_id!=robot_tag_id && world_grid[ngr][ngc].visited!=1)
            {
              addBacktrackPointToStackAndPath(sk,incumbent_cells,ic_no,ngr,ngc,t,testbed);
              return;
            }
          }
        }
      }
      else
      {
        for(int i = 0; i < 4; i++)
        {
          ngr = t.first + aj[nx][ny][i].first;
          ngc = t.second + aj[nx][ny][i].second;
          if(isEmpty(ngr, ngc) && world_grid[ngr][ngc].visited!=1 && world_grid[ngr][ngc].r_id!=robot_tag_id)
          {
            addBacktrackPointToStackAndPath(sk,incumbent_cells,ic_no,ngr,ngc,t,testbed);
            return;
          }
        }
        coverage_completed = 1;
      }
    }//if !coverage_completed && !sk.empty()
}//first_call

void PathPlannerGrid::ANTS(AprilInterfaceAndVideoCapture &testbed, robot_pose &ps, double reach_distance, vector<PathPlannerGrid> &bots){
  if(setRobotCellCoordinates(testbed.detections)<0)//set the start_grid_y, start_grid_x
    return;
  if(!first_call){
    if(!sk.empty()){
        pair<int,int> t = sk.top();
        world_grid[start_grid_x][start_grid_y].bot_presence = make_pair(1, robot_tag_id); //assigning bot presence bit to current cell, //this would come to use in collision avoidance algorithm
        if(last_grid_x != start_grid_x || last_grid_y != start_grid_y)
        {
          world_grid[last_grid_x][last_grid_y].bot_presence = make_pair(0, -1);
          last_grid_x = start_grid_x;
          last_grid_y = start_grid_y;
        }    
      if(!checkReachStatus(t, ps, reach_distance)){//ensure the robot is continuing from the last point, and that no further planning occurs until the robot reaches the required point
          cout<<"the robot has not yet reached the old target"<<t.first<<" "<<t.second<<endl;
          return;
        }
      }
      if(coverage_completed) 
        {
          cout<<"coverage complete\n";
          return;
        }
        coverage_completed = 1;
        for(int i = 0; i < rcells; i++)
        {
          for(int j = 0; j < ccells; j++)
          {
            if(isEmpty(i,j) && world_grid[i][j].steps!=1)
            {
              coverage_completed = 0;
              break;
            }
          }
          if(coverage_completed == 0)
          {
            break;
          }
        }
  }//!first_call

     //next two are dummy variables, 
    vector<pair<int,int> > incumbent_cells(rcells*ccells);//make sure rcells and ccells are defined
    int ic_no = 0;

  if(first_call){
      first_call = 0;
      total_points = 0;
      sk.push(pair<int,int>(start_grid_x,start_grid_y));
      world_grid[start_grid_x][start_grid_y].parent = setParentUsingOrientation(ps);      
      world_grid[start_grid_x][start_grid_y].steps = 1;//visited
      world_grid[start_grid_x][start_grid_y].r_id = robot_tag_id;
      world_grid[start_grid_x][start_grid_y].tree_id = robot_tag_id;
      world_grid[start_grid_x][start_grid_y].bot_presence = make_pair(1, robot_tag_id);
      last_grid_x = start_grid_x;
      last_grid_y = start_grid_y;
      world_grid[start_grid_x][start_grid_y].visit_cost++;
      start_cell_x = start_grid_x;
      start_cell_y = start_grid_y;
      target_grid_cell = make_pair(start_grid_x, start_grid_y);
      addGridCellToPath(start_grid_x,start_grid_y,testbed);//add the current robot position as target point on first call, on subsequent calls the robot position would already be on the stack from the previous call assuming the function is called only when the robot has reached the next point
      return;//added the first spiral point
    }//first_call
 
    int ngr, ngc;
    if(!coverage_completed && !sk.empty())
    {
      pair<int,int> t = sk.top();
      //following two points though needed activally in BSA-CM, here they are just to add the (updated) backtrack points.
      int nx = t.first-world_grid[t.first][t.second].parent.first+1;//add one to avoid negative index
      int ny = t.second-world_grid[t.first][t.second].parent.second+1;

      int min_cost = 100000000;
      int go_to_index = 0;
      for(int i = 0; i < 4; i++)
      {
        ngr = t.first + aj[nx][ny][i].first;
        ngc = t.second + aj[nx][ny][i].second;
        if(isEmpty(ngr, ngc) && world_grid[ngr][ngc].visit_cost < min_cost)
        {
          min_cost = world_grid[ngr][ngc].visit_cost;
          go_to_index = i;
        }
      }
      ngr = t.first + aj[nx][ny][go_to_index].first;
      ngc = t.second + aj[nx][ny][go_to_index].second;
      world_grid[ngr][ngc].visit_cost++;
      addBacktrackPointToStackAndPath(sk,incumbent_cells,ic_no,ngr,ngc,t,testbed);
      return;
    }//!coverage
}

void PathPlannerGrid::S_MSTC(AprilInterfaceAndVideoCapture &testbed, robot_pose &ps, double reach_distance, vector<PathPlannerGrid> &bots){
  if(setRobotCellCoordinates(testbed.detections)<0)//set the start_grid_y, start_grid_x
    return;
  if(!first_call){
    if(!sk.empty()){
        pair<int,int> t = sk.top();
        world_grid[start_grid_x][start_grid_y].bot_presence = make_pair(1, robot_tag_id); //assigning bot presence bit to current cell, //this would come to use in collision avoidance algorithm
        if(last_grid_x != start_grid_x || last_grid_y != start_grid_y)
        {
          world_grid[last_grid_x][last_grid_y].bot_presence = make_pair(0, -1);
          last_grid_x = start_grid_x;
          last_grid_y = start_grid_y;
        }    
      if(!checkReachStatus(t, ps, reach_distance)){//ensure the robot is continuing from the last point, and that no further planning occurs until the robot reaches the required point
          cout<<"the robot has not yet reached the old target"<<t.first<<" "<<t.second<<endl;
          return;
        }
      }
      if(coverage_completed) 
        {
          cout<<"coverage complete\n";
          return;
        }
  }//!first_call

     //next two are dummy variables, 
    vector<pair<int,int> > incumbent_cells(rcells*ccells);//make sure rcells and ccells are defined
    int ic_no = 0;

  if(first_call){
      first_call = 0;
      total_points = 0;
      sk.push(pair<int,int>(start_grid_x,start_grid_y));
      second_sk.push(pair<int,int>(start_grid_x,start_grid_y));
      //world_grid[start_grid_x][start_grid_y].parent = setParentUsingOrientation(ps);      
      world_grid[start_grid_x][start_grid_y].steps = 1;//visited
      world_grid[start_grid_x][start_grid_y].r_id = robot_tag_id;
      world_grid[start_grid_x][start_grid_y].tree_id = robot_tag_id;
      world_grid[start_grid_x][start_grid_y].bot_presence = make_pair(1, robot_tag_id);
      last_grid_x = start_grid_x;
      last_grid_y = start_grid_y;
      start_cell_x = start_grid_x;
      start_cell_y = start_grid_y;
      target_grid_cell = make_pair(start_grid_x, start_grid_y);
      addGridCellToPath(start_grid_x,start_grid_y,testbed);//add the current robot position as target point on first call, on subsequent calls the robot position would already be on the stack from the previous call assuming the function is called only when the robot has reached the next point
      return;//added the first spiral point
    }//first_call
 
    int ngr, ngc;
    if(!second_sk.empty())
    {
      cout<<"robot_tag_id: "<<robot_tag_id<<endl;
      cout<<"1\n";
      pair<int,int> t = sk.top();
      cout<<"2\n";     
      vector <pair<int,int>> preference(4);//up/down//left//right
      preference[0].first = -1, preference[0].second = 0; 
      preference[1].first = 1, preference[1].second = 0; 
      preference[2].first = 0, preference[2].second = -1; 
      preference[3].first = 0, preference[3].second = 1; 

      for(int i = 0; i < 4; i++)
      {
        ngr = t.first + preference[i].first;
        ngc = t.second + preference[i].second;
        if(!isBlocked(ngr, ngc))
        {
          second_sk.push(make_pair(ngr, ngc));
        }
      }
      pair<int, int> current_cell = t;
      while(!second_sk.empty())
      {
        t = second_sk.top();      
        ngr = t.first;
        ngc = t.second;
        second_sk.pop();
        if(world_grid[ngr][ngc].steps==0)
        {
          if((abs(current_cell.first- ngr) + abs(current_cell.second-ngc))>1)
          {
            ic_no = 5;//any number greater than 0;
            incumbent_cells[0].first = start_grid_x;
            incumbent_cells[0].second = start_grid_y;
          }
          addBacktrackPointToStackAndPath(sk,incumbent_cells,ic_no,ngr,ngc,current_cell,testbed);
          return;
        }
      }    
    }//if !coverage_completed && !sk.empty()
    else
    {
      cout<<"3\n";
      coverage_completed = 1;
    }
}

bool PathPlannerGrid::checkConnectivity(pair <int, int> start, pair <int, int> end){
    queue<pair<int,int> > q;    
    vector<pair<int,int> > aj = {{-1,0},{0,1},{1,0},{0,-1}};
    int ngr, ngc;
    pair<int,int> t;
    for(int r = 0; r < rcells; r++)
    {
      for(int c = 0; c < ccells; c++)
      {
        world_grid[r][c].checked = 0;
      }
    }
    q.push(make_pair(start.first,start.second));
    world_grid[start.first][start.second].checked = 1;
    while(!q.empty()){
      t = q.front();q.pop();
      if(t.first == end.first && t.second == end.second)
        break;
      for(int i = 0;i<4;i++){
        ngr = t.first+aj[i].first, ngc = t.second+aj[i].second;
        if(!isEmpty(ngr, ngc) ||  world_grid[ngr][ngc].visited ==1 || world_grid[ngr][ngc].checked==1)
          continue;
        if(isEmpty(ngr, ngc) && world_grid[ngr][ngc].steps == 0 && world_grid[ngr][ngc].observed==0)
          continue;
        world_grid[ngr][ngc].checked = 1;
        q.push(make_pair(ngr,ngc));
      }
    }
    if(!(t.first == end.first && t.second == end.second))
      return 0;
    else
    return 1;
}

void PathPlannerGrid::BrickAndMortar(AprilInterfaceAndVideoCapture &testbed, robot_pose &ps, double reach_distance, vector<PathPlannerGrid> &bots){
  if(setRobotCellCoordinates(testbed.detections)<0)//set the start_grid_y, start_grid_x
    return;
  if(!first_call){
    if(!sk.empty()){
        pair<int,int> t = sk.top();
        world_grid[start_grid_x][start_grid_y].bot_presence = make_pair(1, robot_tag_id); //assigning bot presence bit to current cell, //this would come to use in collision avoidance algorithm
        if(last_grid_x != start_grid_x || last_grid_y != start_grid_y)
        {
          world_grid[last_grid_x][last_grid_y].bot_presence = make_pair(0, -1);
          last_grid_x = start_grid_x;
          last_grid_y = start_grid_y;
        }    
      if(!checkReachStatus(t, ps, reach_distance)){//ensure the robot is continuing from the last point, and that no further planning occurs until the robot reaches the required point
          cout<<"the robot has not yet reached the old target"<<t.first<<" "<<t.second<<endl;
          return;
        }
      }
      if(coverage_completed) 
        {
          cout<<"coverage complete\n";
          return;
        }
  }//!first_call

     //next two are dummy variables, 
    vector<pair<int,int> > incumbent_cells(rcells*ccells);//make sure rcells and ccells are defined
    int ic_no = 0;

  if(first_call){
      first_call = 0;
      total_points = 0;
      sk.push(pair<int,int>(start_grid_x,start_grid_y));
      world_grid[start_grid_x][start_grid_y].parent = setParentUsingOrientation(ps);      
      world_grid[start_grid_x][start_grid_y].steps = 1;//visited
      world_grid[start_grid_x][start_grid_y].r_id = robot_tag_id;
      world_grid[start_grid_x][start_grid_y].tree_id = robot_tag_id;
      world_grid[start_grid_x][start_grid_y].bot_presence = make_pair(1, robot_tag_id);
      last_grid_x = start_grid_x;
      last_grid_y = start_grid_y;
      start_cell_x = start_grid_x;
      start_cell_y = start_grid_y;
      target_grid_cell = make_pair(start_grid_x, start_grid_y);
      addGridCellToPath(start_grid_x,start_grid_y,testbed);//add the current robot position as target point on first call, on subsequent calls the robot position would already be on the stack from the previous call assuming the function is called only when the robot has reached the next point
      return;//added the first spiral point
    }//first_call
 
    int ngr, ngc;
    if(!coverage_completed && !sk.empty())
    {
      cout<<"robot_tag_id "<<robot_tag_id<<endl;
      pair<int,int> t = sk.top();
      int nx = t.first-world_grid[t.first][t.second].parent.first+1;//add one to avoid negative index
      int ny = t.second-world_grid[t.first][t.second].parent.second+1;
      
      bool not_blocking = 1;//Marking Step starts
      world_grid[t.first][t.second].visited = 1;
      vector <pair<int, int>> ng(8);
      ng[0] = make_pair(-1, 0); //up
      ng[1] = make_pair(-1, 1); //up-right
      ng[2] = make_pair(0, 1);//right
      ng[3] = make_pair(1, 1); //bottom right
      ng[4] = make_pair(1, 0);//bottom
      ng[5] = make_pair(1, -1);//bottom left
      ng[6] = make_pair(0, -1);//left
      ng[7] = make_pair(-1, -1);//up left
      for(int i = 0; i < 8; i++)
      {
        int ngr2 = t.first + ng[i].first;
        int ngc2 = t.second + ng[i].second;
        if(isEmpty(ngr2, ngc2))
        {
          world_grid[ngr2][ngc2].observed = 1; 
        }        
      }
      /*int not_visited_ng = 0;
      for(int i = 0; i < 4; i++)
      {
        int ngr2 = t.first + ng[i].first;
        int ngc2 = t.second + ng[i].second;
        if(isEmpty(ngr2, ngc2) && world_grid[ngr2][ngc2].visited!=1)
        {
          not_visited_ng++;
        }   
      }*/
      for(int i = 0; i < 3; i++)
      {
        int ngr2 = t.first + aj[nx][ny][i].first;
        int ngc2 = t.second + aj[nx][ny][i].second;
        /*bool consider_visited_first = 1;
        if(not_visited_ng > 1 && isEmpty(ngr2, ngc2) && world_grid[ngr2][ngc2].bot_presence.first==1 && world_grid[ngr2][ngc2].visited==1)
        {
          consider_visited_first = 0;
        }
        if(consider_visited_first!=1)
        {
          world_grid[ngr2][ngc2].visited=0;
        }*/
        
        if(isEmpty(ngr2, ngc2) && world_grid[ngr2][ngc2].visited!=1)
        {          
          for(int j = i+1; j < 4; j++)
          {
            int ngr3 = t.first + aj[nx][ny][j].first;
            int ngc3 = t.second + aj[nx][ny][j].second;
            /*bool consider_visited_second = 1;
            if(not_visited_ng > 1 &&  isEmpty(ngr3, ngc3) && world_grid[ngr3][ngc3].bot_presence.first==1 && world_grid[ngr3][ngc3].visited==1)
            {
              consider_visited_second = 0;
            }
            if(consider_visited_second!=1)
            {
              world_grid[ngr3][ngc3].visited=0;
            }*/          
            if(isEmpty(ngr3, ngc3) && world_grid[ngr3][ngc3].visited!=1)
            {
              cout<<"i, j: "<<i<<" "<<j<<endl;
              cout<<"t: "<<t.first<<" "<<t.second<<endl;
              cout<<"start: "<<ngr2<<" "<<ngc2<<endl;
              cout<<"end: "<<ngr3<<" "<<ngc3<<endl;
              not_blocking = checkConnectivity(make_pair(ngr2, ngc2) , make_pair(ngr3, ngc3));
              cout<<"connectivity: "<<checkConnectivity(make_pair(ngr2, ngc2) , make_pair(ngr3, ngc3))<<endl;
              /*if(consider_visited_second!=1)
              {
                world_grid[ngr3][ngc3].visited=1;
              }*/
              if(not_blocking == 0)
              {
                break;
              }
            }//if
            /*if(consider_visited_second!=1)
            {
              world_grid[ngr3][ngc3].visited=1;
            }  */        
          }//for j 
         /*if(consider_visited_first!=1)
          {
            world_grid[ngr2][ngc2].visited=1;
          }*/
          if(not_blocking == 0)
          {
            break;
          }
        }//if        
      }//for i

      if(not_blocking == 0)
      {
        world_grid[t.first][t.second].visited = 0;
      }// marking step ends

      bool found_unexplored_cell = 0;
      int max_visited_sum = -1;
      int max_visited_index = -1;
      for(int i = 0; i < 4; i++)
      {
        ngr = t.first + aj[nx][ny][i].first;
        ngc = t.second + aj[nx][ny][i].second;
        if(!isBlocked(ngr, ngc))//unexplored cell
        {
          found_unexplored_cell = 1;
          int visited_sum = 0;
          for(int j = 0; j < 4; j++)
          {
            int ngr2 = ngr + aj[0][1][j].first;
            int ngc2 = ngc + aj[0][1][j].second;
            if(!isEmpty(ngr2, ngc2) || world_grid[ngr2][ngc2].visited==1)
            {
              visited_sum++;
            }
          }
          if(visited_sum>max_visited_sum)
          {
            max_visited_sum = visited_sum;
            max_visited_index = i;
          }
        }
      }

      if(found_unexplored_cell)
      {
        ngr = t.first + aj[nx][ny][max_visited_index].first;
        ngc = t.second + aj[nx][ny][max_visited_index].second;
        addBacktrackPointToStackAndPath(sk,incumbent_cells,ic_no,ngr,ngc,t,testbed);
        return;
      }
      else
      {
        for(int i = 0; i < 4; i++)
        {
          ngr = t.first + aj[nx][ny][i].first;
          ngc = t.second + aj[nx][ny][i].second;
          if(isEmpty(ngr, ngc) && world_grid[ngr][ngc].visited!=1)
          {
            addBacktrackPointToStackAndPath(sk,incumbent_cells,ic_no,ngr,ngc,t,testbed);
            return;
          }
        }
        coverage_completed = 1;
      }
    }//if !coverage_completed && !sk.empty()
}//function

void PathPlannerGrid::BSACoverage(AprilInterfaceAndVideoCapture &testbed,robot_pose &ps){
  if(setRobotCellCoordinates(testbed.detections)<0)
    return;
  vector<pair<int,int> > incumbent_cells(rcells*ccells);
  int ic_no = 0;
  stack<pair<int,int> > sk;
  sk.push(pair<int,int>(start_grid_x,start_grid_y));
  total_points = 0;
  world_grid[start_grid_x][start_grid_y].parent = setParentUsingOrientation(ps);
  world_grid[start_grid_x][start_grid_y].steps = 1;//visited
  world_grid[start_grid_x][start_grid_y].r_id = robot_tag_id;
  addGridCellToPath(start_grid_x,start_grid_y,testbed);
  int ngr,ngc,wall;//neighbor row and column

  while(!sk.empty()){
    pair<int,int> t = sk.top();
    int nx = t.first-world_grid[t.first][t.second].parent.first+1;//add one to avoid negative index
    int ny = t.second-world_grid[t.first][t.second].parent.second+1;
    if((wall=world_grid[t.first][t.second].wall_reference)>=0){
      ngr = t.first+aj[nx][ny][wall].first, ngc = t.second+aj[nx][ny][wall].second;
      if(!isBlocked(ngr,ngc)){
        addBacktrackPointToStackAndPath(sk,incumbent_cells,ic_no,ngr,ngc,t,testbed);
        world_grid[ngr][ngc].wall_reference = -1;//to prevent wall exchange to right wall when following left wall
        continue;
      }
    }
    bool empty_neighbor_found = false;
    for(int i = 0;i<4;i++){
      ngr = t.first+aj[nx][ny][i].first;
      ngc = t.second+aj[nx][ny][i].second;
      if(isBlocked(ngr,ngc))
        continue;
      empty_neighbor_found = true;
      addBacktrackPointToStackAndPath(sk,incumbent_cells,ic_no,ngr,ngc,t,testbed);
      break;
    }
    if(empty_neighbor_found) continue;
    incumbent_cells[ic_no] = t;
    ic_no++;
    sk.pop();
    if(sk.empty()) break;
    pair<int,int> next_below = sk.top();
    world_grid[next_below.first][next_below.second].parent = t;
    world_grid[next_below.first][next_below.second].wall_reference = 1;//since turning 180 degrees
  }
}

void PathPlannerGrid::findCoverageLocalNeighborPreference(AprilInterfaceAndVideoCapture &testbed,robot_pose &ps){
  if(setRobotCellCoordinates(testbed.detections)<0)
    return;
  vector<pair<int,int> > incumbent_cells(rcells*ccells);
  int ic_no = 0;
  stack<pair<int,int> > sk;
  sk.push(pair<int,int>(start_grid_x,start_grid_y));
  total_points = 0;
  world_grid[start_grid_x][start_grid_y].parent = setParentUsingOrientation(ps);
  world_grid[start_grid_x][start_grid_y].steps = 1;//visited
  world_grid[start_grid_x][start_grid_y].r_id = robot_tag_id;
  addGridCellToPath(start_grid_x,start_grid_y,testbed);
  int ngr,ngc;//neighbor row and column

  while(!sk.empty()){
    pair<int,int> t = sk.top();
    int nx = t.first-world_grid[t.first][t.second].parent.first+1;//add one to avoid negative index
    int ny = t.second-world_grid[t.first][t.second].parent.second+1;
    bool empty_neighbor_found = false;
    for(int i = 0;i<4;i++){
      ngr = t.first+aj[nx][ny][i].first;
      ngc = t.second+aj[nx][ny][i].second;
      if(isBlocked(ngr,ngc))
        continue;
      empty_neighbor_found = true;
      addBacktrackPointToStackAndPath(sk,incumbent_cells,ic_no,ngr,ngc,t,testbed);
      break;
    }
    if(empty_neighbor_found) continue;
    incumbent_cells[ic_no] = t;
    ic_no++;
    sk.pop();
    if(sk.empty()) break;
    pair<int,int> next_below = sk.top();
    world_grid[next_below.first][next_below.second].parent = t;
  }
}
void PathPlannerGrid::findCoverageGlobalNeighborPreference(AprilInterfaceAndVideoCapture &testbed){
  if(setRobotCellCoordinates(testbed.detections)<0)
    return;
  vector<pair<int,int> > incumbent_cells(rcells*ccells);
  int ic_no = 0;//points in above vector
  stack<pair<int,int> > sk;
  vector<pair<int,int> > aj = {{-1,0},{0,1},{0,-1},{1,0}};//adjacent cells in order of preference
  sk.push(pair<int,int>(start_grid_x,start_grid_y));
  //parent remains -1, -1
  world_grid[start_grid_x][start_grid_y].steps = 1;
  world_grid[start_grid_x][start_grid_y].r_id = robot_tag_id;
  addGridCellToPath(start_grid_x,start_grid_y,testbed);
  total_points = 0;
  while(!sk.empty()){
    pair<int,int> t = sk.top();
    int ng_no = world_grid[t.first][t.second].steps;
    if(ng_no == 5){//add yourself in possible backtrack cells
      incumbent_cells[ic_no] = t;
      ic_no++;
      sk.pop();
    }
    else{
      int ngr = t.first+aj[ng_no-1].first, ngc = t.second+aj[ng_no-1].second;
      if(isBlocked(ngr,ngc)){
        world_grid[t.first][t.second].steps = ng_no+1;
        continue;
      }
      addBacktrackPointToStackAndPath(sk,incumbent_cells,ic_no,ngr,ngc,t,testbed);
      world_grid[t.first][t.second].steps = ng_no+1;
    }
  }
}

void PathPlannerGrid::setPathColor(){
  switch(robot_tag_id)
  {
    case 0: path_color = cv::Scalar(255, 0, 0); break;
    case 1: path_color = cv::Scalar(0, 0, 255); break;
    case 2: path_color = cv::Scalar(0, 255, 255); break;
    case 3: path_color = cv::Scalar(255, 0, 255); break;
    case 4: path_color = cv::Scalar(255, 32, 123); break;
    case 5: path_color = cv::Scalar(21, 155, 85); break;
    case 6: path_color = cv::Scalar(232, 124, 64); break;
    case 7: path_color = cv::Scalar(84, 25, 237); break;
    case 8: path_color = cv::Scalar(48, 143, 225); break;
    case 9: path_color = cv::Scalar(74, 125, 232); break;
    case 10: path_color = cv::Scalar(85, 252, 64); break;
    case 11: path_color = cv::Scalar(23, 55, 25); break;
    default: path_color = cv::Scalar(204, 126, 73);
  }
}

void PathPlannerGrid::drawPath(Mat &image){
  setPathColor();
  int i;  
  for(i = 0;i<total_points-1;i++){
    if((abs((path_points[i].x)- (path_points[i+1].x)) + abs((path_points[i].y)- (path_points[i+1].y)))>1)
    {
      cout<<"first: "<<path_points[i].x<<" "<<path_points[i].y<<endl;
      cout<<"next: "<<path_points[i+1].x<<" "<<path_points[i+1].y<<endl;
      cout<<"robot_tag_id: "<<robot_tag_id<<endl;
      cout<<"i: "<<i<<endl;
      cout<<"manhattan_distance of target_grid_cell greater than 1\n";
      cv::waitKey(0);
    }
    line(image,Point(pixel_path_points[i].first,pixel_path_points[i].second),Point(pixel_path_points[i+1].first,pixel_path_points[i+1].second),path_color,2); 
  }
  /*if(total_points!=0)//to prevent segmentation faul in case no bot tag is present.
  {
    circle(image, Point(pixel_path_points[i].first,pixel_path_points[i].second), 5, path_color, CV_FILLED, 8, 0); 
  }*/
  
  for(i = 0; i < bt_destinations.size(); i++)
  {
    if(bt_destinations[i].valid)
    {
      int ax,ay;
      ax = world_grid[bt_destinations[i].next_p.first][bt_destinations[i].next_p.second].tot_x/world_grid[bt_destinations[i].next_p.first][bt_destinations[i].next_p.second].tot;
      ay = world_grid[bt_destinations[i].next_p.first][bt_destinations[i].next_p.second].tot_y/world_grid[bt_destinations[i].next_p.first][bt_destinations[i].next_p.second].tot;
      rectangle(image,Point(ax-4, ay-4),Point(ax+4,ay+4),path_color,-1);
    }
  }
  if(start_cell_x >=0 && start_cell_x >=0)
  {
    int ax, ay;
    ax = world_grid[start_cell_x][start_cell_y].tot_x/world_grid[start_cell_x][start_cell_y].tot;
    ay = world_grid[start_cell_x][start_cell_y].tot_y/world_grid[start_cell_x][start_cell_y].tot;
    rectangle(image,Point(ax-2, ay-2),Point(ax+2,ay+2),path_color,-1);
  }
  /*for(int i = 0; i < rcells; i++)
  {
    for(int j = 0; j < ccells; j++)
    {
      if(world_grid[i][j].visited==1)
      {
        int ax,ay;
        ax = world_grid[i][j].tot_x/world_grid[i][j].tot;
        ay = world_grid[i][j].tot_y/world_grid[i][j].tot;
        line(image,Point(ax-4, ay-4),Point(ax + 4, ay +4),Scalar (0, 0, 0),1);
        line(image,Point(ax+4, ay-4),Point(ax - 4, ay +4),Scalar (0, 0, 0),1); 
      }
    }
  }*/
  /*
  for(i = 0; i < uev_destinations.size(); i++)
  {
    if(uev_destinations[i].valid)
    {
      int ax,ay;
      ax = world_grid[uev_destinations[i].next_p.first][uev_destinations[i].next_p.second].tot_x/world_grid[uev_destinations[i].next_p.first][uev_destinations[i].next_p.second].tot;
      ay = world_grid[uev_destinations[i].next_p.first][uev_destinations[i].next_p.second].tot_y/world_grid[uev_destinations[i].next_p.first][uev_destinations[i].next_p.second].tot;
      line(image,Point(ax-15, ay-15),Point(ax+15, ay-15),path_color,2); 
      line(image,Point(ax+15, ay-15),Point(ax+15, ay+15),path_color,2);
      line(image,Point(ax+15, ay+15),Point(ax-15, ay+15),path_color,2);
      line(image,Point(ax-15, ay+15),Point(ax-15, ay-15),path_color,2);
    }
  }

  for(i = 0; i < uv_boundary.size(); i++)
  {
    if(uv_boundary[i].valid)
    {
      int ax,ay;
      ax = world_grid[uv_boundary[i].next_p.first][uv_boundary[i].next_p.second].tot_x/world_grid[uv_boundary[i].next_p.first][uv_boundary[i].next_p.second].tot;
      ay = world_grid[uv_boundary[i].next_p.first][uv_boundary[i].next_p.second].tot_y/world_grid[uv_boundary[i].next_p.first][uv_boundary[i].next_p.second].tot;
      line(image,Point(ax, ay-15),Point(ax+15, ay),path_color,2); 
      line(image,Point(ax+15, ay),Point(ax, ay+15),path_color,2);
      line(image,Point(ax, ay+15),Point(ax-15, ay),path_color,2);
      line(image,Point(ax-15, ay),Point(ax, ay-15),path_color,2);
    }
  }

  for(i = 0; i < patches.size(); i++)
  {
    for(int j = 0; j < patches[i].size(); j++)
    {
      int ax,ay;
      ax = world_grid[patches[i][j].first][patches[i][j].second].tot_x/world_grid[patches[i][j].first][patches[i][j].second].tot;
      ay = world_grid[patches[i][j].first][patches[i][j].second].tot_y/world_grid[patches[i][j].first][patches[i][j].second].tot;
      line(image,Point(ax-25, ay-25),Point(ax+25, ay+25),path_color,2); 
      line(image,Point(ax-25, ay+25),Point(ax+25, ay-25),path_color,2);
    }
  }*/
}
void PathPlannerGrid::drawRobot(Mat &image){
  //setPathColor(); 
  if(index_travelled>0)
  {   
    int cx = pixel_path_points[index_travelled].first;
    int cy = pixel_path_points[index_travelled].second;  
    int current_x = path_points[index_travelled].x;
    int current_y = path_points[index_travelled].y;
    int last_x, last_y;
    for(int i = index_travelled-1; i >=0; i--)
    {
      last_x = path_points[i].x;
      last_y = path_points[i].y;
      if(last_x!=current_x || last_y!=current_y)break;
    }  
      int nx = (current_x - last_x);
      int ny = (current_y - last_y);
      if(nx==0 && ny==1)
      {
        line(image,Point(cx, cy),Point(cx-4,cy+4),path_color,2); 
        line(image,Point(cx, cy),Point(cx-4,cy-4),path_color,2); 
      }
      else if(nx==0 && ny==-1)
      {
        line(image,Point(cx, cy),Point(cx+4,cy+4),path_color,2); 
        line(image,Point(cx, cy),Point(cx+4,cy-4),path_color,2); 
      }
      else if(nx==1 && ny == 0)
      {
        line(image,Point(cx, cy),Point(cx+4,cy-4),path_color,2); 
        line(image,Point(cx, cy),Point(cx-4,cy-4),path_color,2);
      }
      else if(nx==-1 && ny == 0)
      {
        line(image,Point(cx, cy),Point(cx+4,cy+4),path_color,2);
        line(image,Point(cx, cy),Point(cx-4,cy+4),path_color,2);
      }
      else
      {
        rectangle(image,Point(cx-4, cy-4),Point(cx+4,cy+4),path_color,-1); 
      }    
    }

  int ax, ay;
  if(start_cell_x >=0 && start_cell_x >=0)
  {
    ax = world_grid[start_cell_x][start_cell_y].tot_x/world_grid[start_cell_x][start_cell_y].tot;
    ay = world_grid[start_cell_x][start_cell_y].tot_y/world_grid[start_cell_x][start_cell_y].tot;  
    string s = to_string(robot_tag_id);
    putText(image, s, Point(ax,ay), FONT_HERSHEY_DUPLEX, 0.5, Scalar(255,255,255), 1);
  }
  
   
}

//for the class PathPlannerUser
void PathPlannerUser::addPoint(int px, int py, double x,double y){
  if(total_points>=path_points.size()){
    path_points.resize(100+path_points.size());//add hundred points in one go
    pixel_path_points.resize(100+pixel_path_points.size());
  }
  path_points[total_points].x = x;
  path_points[total_points].y = y;
  pixel_path_points[total_points].first = px;
  pixel_path_points[total_points].second = py;
  total_points++;
}
void PathPlannerUser::CallBackFunc(int event, int x, int y){
  static int left_clicked = 0;
  static int x_pixel_previous = -1;
  static int y_pixel_previous = -1;
  //ignore EVENT_RABUTTONDOWN, EVENT_MBUTTONDOWN 
   if(event == EVENT_LBUTTONDOWN){
      left_clicked = (left_clicked+1)%2;
      if(left_clicked){
        path_points.clear();
        pixel_path_points.clear();
        total_points = 0;
      }
      x_pixel_previous = x;
      y_pixel_previous = y;
   }
   else if(event == EVENT_MOUSEMOVE && left_clicked){
     double xd, yd;
     testbed->pixelToWorld(x,y,xd,yd);
     addPoint(x,y,xd,yd);
     x_pixel_previous = x; 
     y_pixel_previous = y;
   }
}
void PathPlannerUser::drawPath(Mat &image){
  for(int i = 0;i<total_points-1;i++){
    line(image,Point(pixel_path_points[i].first,pixel_path_points[i].second),Point(pixel_path_points[i+1].first,pixel_path_points[i+1].second),Scalar(0,0,255),2);
  }
}


void PathPlannerGrid::DeadlockReplan(AprilInterfaceAndVideoCapture &testbed, vector<PathPlannerGrid> &bots){
  if(setRobotCellCoordinates(testbed.detections)<0)//set the start_grid_y, start_grid_x
    return;
    
    int i;
    bool empty_neighbor_found = false;
    int ngr, ngc;
    for(i = 0; i < 4; i++)
    {
      ngr = start_grid_x + aj[0][1][i].first;
      ngc = start_grid_y + aj[0][1][i].second;    
      if(isEmpty(ngr,ngc) && world_grid[ngr][ngc].steps && world_grid[ngr][ngc].bot_presence.first!=1){//this mean the saio neighbouring cell is empty, has been visited and no bot is currently there
        empty_neighbor_found = true;//found an empty cell to shift to.
        break;
      }
    } 
  cout<<"paht point size: "<<path_points.size()<<endl;
  cout<<"next_target_index: "<<next_target_index<<endl;
  cout<<"total_points: "<<total_points<<endl;
  cout<<"robot_tag_id: "<<robot_tag_id<<endl;
  cout<<"start_cell_x, y: "<<start_cell_x<<" "<<start_cell_y<<endl;
  cout<<"converage_completed: "<<coverage_completed<<endl;
  cout<<"path_points.size(): "<<path_points.size()<<endl;
  cout<<"index_travelled: "<<index_travelled<<endl;
  cout<<"next_target_index: "<<next_target_index<<endl;
  cout<<"empty_neighbor_found: "<<empty_neighbor_found<<endl;
  if(status != 2 && !coverage_completed)
  {
    cout<<"*********\ncomp\n**********\n";
    total_points+=2;
    if(total_points > path_points.size())
    {
      path_points.resize(total_points);
      pixel_path_points.resize(total_points);
    }

    for(i = path_points.size()-1; i >= next_target_index; i--)
    {
      path_points[i].x = path_points[i-2].x;
      path_points[i].y = path_points[i-2].y;
      pixel_path_points[i].first = pixel_path_points[i-2].first;
      pixel_path_points[i].second = pixel_path_points[i-2].second; 
    }     
    
    if(empty_neighbor_found){       
      int ax,ay;
      double bx,by;
      ax = world_grid[ngr][ngc].tot_x/world_grid[ngr][ngc].tot;
      ay = world_grid[ngr][ngc].tot_y/world_grid[ngr][ngc].tot;
      //testbed.pixelToWorld(ax,ay,bx,by);

      path_points[next_target_index].x = ngr;//point at next target index updated
      path_points[next_target_index].y = ngc;
      pixel_path_points[next_target_index].first = ax;
      pixel_path_points[next_target_index].second = ay; 
    }
  }

  else
  {
    cout<<"*********\nelse\n**********\n";
    status = 1;
    vector<pair<int,int> > incumbent_cells(rcells*ccells);
    int ic_no=0;
    pair <int, int> t = make_pair(start_grid_x, start_grid_y);

    if(empty_neighbor_found){       
      addBacktrackPointToStackAndPath(sk,incumbent_cells,ic_no,ngr,ngc,t,testbed);
    }
    else
    {
      int c1 = pixel_path_points[total_points-2].first/cell_size_x;
      int r1 = pixel_path_points[total_points-2].second/cell_size_y;
      addBacktrackPointToStackAndPath(sk,incumbent_cells,ic_no,r1,c1,t,testbed); 
    }
  }
  cout<<"After replan!\n**********\n";
  cout<<"paht point size: "<<path_points.size()<<endl;
  cout<<"next_target_index: "<<next_target_index<<endl;
  cout<<"total_points: "<<total_points<<endl;
  cout<<"robot_tag_id: "<<robot_tag_id<<endl;
  cout<<"start_cell_x, y: "<<start_cell_x<<" "<<start_cell_y<<endl;
  cout<<"converage_completed: "<<coverage_completed<<endl;
  cout<<"path_points.size(): "<<path_points.size()<<endl;
  cout<<"index_travelled: "<<index_travelled<<endl;
  cout<<"next_target_index: "<<next_target_index<<endl;
  cout<<"empty_neighbor_found: "<<empty_neighbor_found<<endl;
}

void PathPlannerGrid::defineVoronoiPartition(AprilInterfaceAndVideoCapture &testbed, vector<PathPlannerGrid> &bots){

  vector <pair<int, int>> start_cells(bots.size());

  int bots_present_count = bots.size();
  for(int i = 0; i < bots.size(); i++)
  {
    if(bots[i].setRobotCellCoordinates(testbed.detections)<0)
    {
      start_cells[i].first = 10000000;//assigning large numbers so that the subsequent manhattan distance is large
      start_cells[i].second = 10000000;
      bots_present_count--;
      continue;
    }
    start_cells[i].first = bots[i].start_grid_x;
    start_cells[i].second = bots[i].start_grid_y;
  }
  if(bots_present_count <= 1) return;

  int min_man_dist = 10000000;
  int manhattan_dist;

  for(int r = 0; r < rcells; r++)
  {
    for(int c = 0; c < ccells; c++)
    { 
      int min_man_dist = 10000000;
      for(int k = 1; k < bots.size(); k++)
      { 
        manhattan_dist = abs(start_cells[k].first - r) + abs(start_cells[k].second - c);
        if(manhattan_dist < min_man_dist)
        {
          min_man_dist = manhattan_dist;
          world_grid[r][c].voronoi_id = k;
        }
      }
      bots[world_grid[r][c].voronoi_id].total_voronoi_cells++;
    }
  }
  for(int r = 0; r < rcells; r++)//declaring boundary cells to the points which are in direct contact with the other boundary
  {
    for(int c = 0; c < ccells; c++)
    {
      if(world_grid[r][c].isBoundaryCell) continue;
      for(int i = 0; i < 4; i++)
      {
        int ngr = r + aj[0][1][i].first;
        int ngc = c + aj[0][1][i].second;
        if(ngr < 0 || ngc < 0 || ngr > rcells-1 || ngc >=ccells )
        {
          world_grid[r][c].isBoundaryCell = 1;
          //bots[world_grid[r][c].voronoi_id].boundary_points.push_back(bp(r, c));
          break;
        }
        if(world_grid[r][c].voronoi_id != world_grid[ngr][ngc].voronoi_id)
        {
          world_grid[r][c].isBoundaryCell = 1;
          world_grid[ngr][ngc].isBoundaryCell = 1;
          //bots[world_grid[r][c].voronoi_id].boundary_points.push_back(bp(r, c));
          //bots[world_grid[ngr][ngc].voronoi_id].boundary_points.push_back(bp(ngr, ngc));
          break; 
        }        
      }
    }
  }
  vector <pair<int, int>> corner(4);
  corner[0].first = 1, corner[0].second = 1;
  corner[1].first = -1, corner[1].second = -1;
  corner[2].first = 1, corner[2].second = -1;
  corner[3].first = -1, corner[3].second = 1;

  for(int r = 0; r < rcells; r++)//declaring boundary cells, which are in contact to the other boundary through a corner
  {
    for(int c = 0; c < ccells; c++)
    {
      if(world_grid[r][c].isBoundaryCell) continue;
      for(int i = 0; i < 4; i++)
      {
        int ngr = r + corner[i].first;
        int ngc = c + corner[i].second;
        if(ngr < 0 || ngc < 0 || ngr > rcells-1 || ngc >=ccells )
        {
          continue;
        }
        
        if(world_grid[r][c].voronoi_id != world_grid[ngr][ngc].voronoi_id)
        {
          world_grid[r][c].isBoundaryCell = 1;
          bots[world_grid[r][c].voronoi_id].boundary_points.push_back(bp(r, c));
        }
        
      }
    }
  }
}//function
