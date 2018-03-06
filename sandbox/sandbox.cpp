#include "aprilvideointerface.h"
#include <unistd.h>
#include "pathplanners.h"
#include "controllers.h"
#include <iostream>
#include <fstream>
// For Arduino: serial port access class
#include "Serial.h"
#include <stdlib.h>
using namespace std;
using namespace cv;

struct bot_config{
  PathPlannerGrid plan; 
  int id;
  robot_pose pose;
  //using intializer list allows intializing variables with non trivial contructor
  //assignment would not help if there is no default contructor with no arguments
  bot_config(int cx,int cy, int thresh,vector<vector<nd> > &tp):plan(PathPlannerGrid(cx,cy,thresh,tp)){
    id = -1;//don't forget to set the id
    //below line would first call plan PathPlannerGrid constructor with no argument and then replace lhs variables with rhs ones
    //plan = PathPlannerGrid(cx,cy,thresh,tp);
  }
  void init(){
    plan.start_grid_x = plan.start_grid_y = -1;
    plan.robot_id = -1;
  }
};


int check_deadlock(vector<bot_config> &bots, int index)
{
  cout<<"\nChecking deadlock presence:\n"<<endl;
  for(int i = 0; i < bots.size(); i++)
  {
    bots[i].plan.deadlock_check_counter = 0;
  }
  int clear_flag = 0;
  int target_cell_bot_id = -1;
  while(!clear_flag)
  {
    cout<<"index: "<<index<<endl;
    cout<<"present cells: "<<bots[index].plan.start_grid_x<<" "<<bots[index].plan.start_grid_y<<endl;
    int r = bots[index].plan.target_grid_cell.first;
    int c = bots[index].plan.target_grid_cell.second;
    cout<<"r,c :"<<r<<" "<<c<<endl;
    if(bots[index].plan.world_grid[r][c].bot_presence.first == 1 && bots[index].plan.world_grid[r][c].bot_presence.second != bots[index].plan.robot_tag_id)
    {
      target_cell_bot_id = bots[index].plan.world_grid[r][c].bot_presence.second;
      bots[target_cell_bot_id].plan.deadlock_check_counter++;
      if(bots[target_cell_bot_id].plan.deadlock_check_counter > 1)
      {
        break;
      }
      else if(bots[target_cell_bot_id].plan.status == 2 || bots[target_cell_bot_id].plan.coverage_completed==1)// to check if the said target bot has covered all its point and is in no position to move
      {
        break;
      }
      index = target_cell_bot_id;
      continue;
    }
    else
    {
      clear_flag = 1;
    }

  }
  if(clear_flag == 1)
  {
    return -1;
  }
  else
  {
    return target_cell_bot_id;
  }
}

bool check_collision_possibility(AprilInterfaceAndVideoCapture &testbed, vector<PathPlannerGrid> &planners, vector<bot_config> &bots, pair<int,int> &wheel_velocities, int i)
{
  cout<<"Checking collision possibility\n";
  if(bots[i].plan.next_target_index != bots[i].plan.path_points.size()) //for collision avoidance
  {
    int c = (bots[i].plan.pixel_path_points[bots[i].plan.next_target_index].first)/(bots[i].plan.cell_size_x);
    int r = (bots[i].plan.pixel_path_points[bots[i].plan.next_target_index].second)/(bots[i].plan.cell_size_y);
    bots[i].plan.target_grid_cell = make_pair(r, c);
    /*for(int j = 0; j < bots.size(); j++)
    {
    	if(j==i)continue;
    	if(bots[j].plan.target_grid_cell.first == bots[i].plan.target_grid_cell.first && bots[j].plan.target_grid_cell.second == bots[i].plan.target_grid_cell.second)
    	{    		    		
    		cout<<"curretn bot: "<<i<<endl;
    		cout<<"target_grid_cell: "<<bots[i].plan.target_grid_cell.first<<" "<<bots[i].plan.target_grid_cell.second<<endl;
    		cout<<"same target cell bots: "<<j<<endl;
    		cout<<"target_grid_cell: "<<bots[j].plan.target_grid_cell.first<<" "<<bots[j].plan.target_grid_cell.second<<endl;
    		//return 1;
    	}
    }*/
    if(bots[i].plan.world_grid[r][c].bot_presence.first == 1 && bots[i].plan.world_grid[r][c].bot_presence.second != bots[i].plan.robot_tag_id)
    {
      int deadlocked_bot = check_deadlock(bots, i);
      if(deadlocked_bot != -1)
      {
      cout<<"\n******\n";
      cout<<"Deadlock Detected!"<<endl;
      bots[deadlocked_bot].plan.DeadlockReplan(testbed, planners);
      cout<<"Path Replanned!"<<endl;
      cout<<"******\n\n";
      }
      return 1;
    }
    /*bots[i].plan.world_grid[r][c].bot_presence.first = 1;
    bots[i].plan.world_grid[r][c].bot_presence.second = bots[i].plan.robot_tag_id;*/ 
    return 0;
  }

}

void getSimulatioResults(int number_of_maps, int number_of_trials, int number_of_algos)
{	
	vector <vector <vector <int>>> iterations (number_of_maps);
	vector <vector <vector <int>>> repetedsteps (number_of_maps);
	vector <vector <vector <int>>> cummalative_path_length (number_of_maps);
	vector <vector <vector <double>>> computationtime (number_of_maps);
	vector <vector <vector <double>>> movement_time (number_of_maps);

	vector <int> number_of_robots = {1, 2, 4, 6, 8, 10, 12}; 
	for(int a = 0; a < number_of_maps; a++)
	{
		iterations[a].resize(number_of_trials);
		repetedsteps[a].resize(number_of_trials);
		cummalative_path_length[a].resize(number_of_trials);
		computationtime[a].resize(number_of_trials);
		movement_time[a].resize(number_of_trials);

		string address;
		switch(a)
		{
			case 0: address = "../Maps/Basic.png"; break;
			case 1: address = "../Maps/Cluttered.png"; break;
			case 2: address = "../Maps/Office.png"; break;
		}			
		
		for(int b = 0; b < number_of_robots.size(); b++)			
		{	
			int trials = number_of_trials;						
			while(trials)
			{	
				bool problem_in_the_trial = 0;
				vector <int> start_r(number_of_robots[b]);
				vector <int> start_c(number_of_robots[b]);
				vector <int> start_o(number_of_robots[b]);
				vector <int> total_iterations(number_of_algos);
				vector <int> total_path_length(number_of_algos);
				vector <int> repeatedCoverage(number_of_algos);
				vector <double> time_to_compute(number_of_algos);
				vector <double> total_movement_time(number_of_algos);
				vector <double> complete_process(number_of_algos);
				int first_algo_call = 1;
				for(int c = 0; c < number_of_algos; c++)
				{
					AprilInterfaceAndVideoCapture testbed; 
                    int first_iter = 1;
					//const char *windowName = "Arena";
					//cv::namedWindow(windowName,WINDOW_NORMAL);
					cv::Mat image;
					cv::Mat image_gray;
					int robotCount = number_of_robots[b];
					vector<vector<nd>> tp;//a map that would be shared among all
					vector<bot_config> bots(robotCount,bot_config(10, 10,130,tp));
					vector<PathPlannerGrid> planners(robotCount,PathPlannerGrid(tp));
					int algo_select = c+1;
					double start_t = tic();
					double compute_time = 0;
					double movement_time = 0;
					while(true){
						cout<<"*************************\n\n";
						cout<<"Map: "<<a+1<<"/"<<number_of_maps<<endl;
						cout<<"Robots count: "<<(b+1)<<"/"<<number_of_robots.size()<<endl;
						cout<<"trial count: "<<number_of_trials-trials+1<<"/"<<number_of_trials<<endl;
						cout<<"Algo Count: "<<c+1<<"/"<<number_of_algos<<endl;
						cout<<"*************************\n\n";
						total_iterations[c]++;
						if(total_iterations[c]>60000)
						{
							problem_in_the_trial = 1;
							break;
						}						
						image = imread(address);						
						cvtColor(image, image_gray, CV_BGR2GRAY);
  						if(first_iter){
						    bots[0].plan.overlayGrid(testbed.detections,image_gray);//overlay grid completely reintialize the grid, we have to call it once at the beginning only when all robots first seen simultaneously(the surrounding is assumed to be static) not every iteration
						    for(int i = 1;i<bots.size();i++){
						    	bots[i].plan.rcells = bots[0].plan.rcells;
						   		bots[i].plan.ccells = bots[0].plan.ccells;
						    }
						    srand(time(0));
						    if(first_algo_call)
						    {
						    	first_algo_call = 0;
						    	for(int i = 0; i < bots.size(); i++)
							    {
							    	
							    	start_r[i] = rand()%bots[0].plan.rcells;
								    start_c[i] = rand()%bots[0].plan.ccells;
								    while((bots[0].plan.isBlocked(start_r[i], start_c[i])))
								    {
								      	start_r[i] = rand()%bots[0].plan.rcells;
								      	start_c[i] = rand()%bots[0].plan.ccells;
								    }			    							    
							      	start_o[i] = rand()%4;						      
							    }		
						    }
						    for(int i = 0; i < bots.size(); i++)
						    {				    	
						    	bots[i].plan.addGridCellToPath(start_r[i], start_c[i], testbed);
						      	bots[i].plan.world_grid[start_r[i]][start_c[i]].steps = 1;      	
						      	bots[i].pose.x = start_r[i];
						      	bots[i].pose.y = start_c[i];
						      	bots[i].pose.omega = start_o[i];
						      	bots[i].plan.current_orient = bots[i].pose.omega;
						      	bots[i].plan.robot_id = i;
						      	bots[i].plan.robot_tag_id = i;
						    }						    
					    }//if first_iter

					    for(int i = 0;i<bots.size();i++){      
					      planners[i] = bots[i].plan;
					    }
					    
					    double compute_start = tic();
					    for(int i = 0;i<bots.size();i++){
					      switch(algo_select)
					      {
					      case 1: bots[i].plan.BSACoverageIncremental(testbed,bots[i].pose, 2.5,planners); break;
					      case 2: bots[i].plan.BSACoverageWithUpdatedBactrackSelection(testbed,bots[i].pose, 2.5,planners); break;
					      case 3: bots[i].plan.BoustrophedonMotionWithUpdatedBactrackSelection(testbed,bots[i].pose, 2.5,planners); break;
					      case 4: bots[i].plan.BoustrophedonMotionWithBSA_CMlikeBacktracking(testbed,bots[i].pose, 2.5,planners); break;    
					      case 5: bots[i].plan.BoB(testbed,bots[i].pose, 2.5,planners); break; 
					      case 6: bots[i].plan.MDFS(testbed,bots[i].pose, 2.5,planners); break; 
					      case 7: bots[i].plan.ANTS(testbed,bots[i].pose, 2.5,planners); break;     
					      default: bots[i].plan.BSACoverageIncremental(testbed,bots[i].pose, 2.5,planners);   
					      }   
					    }
					    double compute_end  = tic();

					    time_to_compute[c] += (compute_end-compute_start);
					    double start_movement = tic();
					    pair <int, int> wheel_velocities;//dummy variable in case of simulation
					    for(int i = 0;i<bots.size();i++){    
					        bots[i].plan.next_target_index = bots[i].plan.index_travelled+1;
					        if((bots[i].plan.next_target_index) < bots[i].plan.path_points.size())
					        {
					        	if(bots[i].plan.movement_made==1 && !first_iter)
						        {
						        	bots[i].plan.last_orient = bots[i].plan.current_orient;
						        	int nx = bots[i].plan.path_points[bots[i].plan.next_target_index].x - bots[i].plan.path_points[bots[i].plan.next_target_index-1].x;
						        	int ny = bots[i].plan.path_points[bots[i].plan.next_target_index].y - bots[i].plan.path_points[bots[i].plan.next_target_index-1].y;
						        	if(nx==0 && ny==0) bots[i].plan.iter_wait = 0;
						        	else if(nx == -1 && ny == 0 )//up
						        	{
						        		bots[i].plan.current_orient = 0;	        	
						        	}
						        	else if(nx == 0 && ny == 1)//right
						        	{
						        		bots[i].plan.current_orient = 1;
						        	}
						        	else if(nx == 1 && ny == 0)//down
						        	{
						        		bots[i].plan.current_orient = 2;
						        	}
						        	else if(nx == 0 && ny == -1)//left
						        	{
						        		bots[i].plan.current_orient = 3;
						        	}
						        	if(!(nx==0 && ny==0))
						        	{
						        		if(abs(bots[i].plan.current_orient - bots[i].plan.last_orient)==0)
						        		{
						        			bots[i].plan.iter_wait = 0 + rand()%3;
						        		}
						        		else if(abs(bots[i].plan.current_orient - bots[i].plan.last_orient)%3==0)
						        		{
						        			bots[i].plan.iter_wait = 3 + rand()%3;
						        		}
						        		else if(abs(bots[i].plan.current_orient - bots[i].plan.last_orient)==1)
						        		{
						        			bots[i].plan.iter_wait = 3 + rand()%3;
						        		}
						        		else if(abs(bots[i].plan.current_orient - bots[i].plan.last_orient)==2)
						        		{
						        			bots[i].plan.iter_wait = 6 + rand()%3;
						        		}
						        	}
						        } 
					        	if(!check_collision_possibility(testbed, planners, bots, wheel_velocities, i) && bots[i].plan.iter_wait <=0) {
					        		bots[i].plan.index_travelled++;
					        		bots[i].plan.movement_made = 1;
					        	}
					        	else{
					        	bots[i].plan.iter_wait--; 
					        	bots[i].plan.movement_made = 0;
					        	}    	
					        }     
					   	}
					   	double end_movement = tic();
					   	total_movement_time[c] += (end_movement-start_movement);
				/*	   	bots[0].plan.drawGrid(image, planners);
					   	for(int i = 0;i<bots.size();i++){      	
					        bots[i].plan.drawPath(image);        
					    }
					    for(int i = 0; i < bots.size(); i++)
					    {
					    	bots[i].plan.drawRobot(image);
					    }
					    imshow(windowName,image);*/
						bool completed = 1;
					    for(int i = 0; i < bots.size(); i++)
					    {
					    	if(bots[i].plan.path_points.size()!=(bots[i].plan.next_target_index))
					    	{
					    		completed = 0;
					    		break;
					    	}
					    }
					    if(!first_iter && completed == 1)
					    {
					    	cout<<"Coverage Completed!\n";
					    	break;
					    }

					    if(first_iter)
					    {
					     	first_iter = 0;
					    }
					  /*  if (cv::waitKey(1) == 27){
					        break;//until escape is pressed
					    }*/
					}//while true
					if(problem_in_the_trial) break;
					double end_t = tic();
  					//imshow(windowName,image);
  					vector <vector<int>> coverage(bots[0].plan.rcells);
					for(int i = 0; i < bots[0].plan.rcells; i++)
					{
						coverage[i].resize(bots[0].plan.ccells);
					}
					for(int i = 0; i < bots.size(); i++)
					{
						total_path_length[c]+= bots[i].plan.path_points.size();
						for(int j = 0; j < bots[i].plan.path_points.size(); j++)
						{
							coverage[bots[i].plan.path_points[j].x][bots[i].plan.path_points[j].y]++;
							if(coverage[bots[i].plan.path_points[j].x][bots[i].plan.path_points[j].y] > 1)
							{
								repeatedCoverage[c]++;
							}
						}
					}
					/*time_to_compute[c] = compute_time;
					total_movement_time[c] = movement_time;*/
					complete_process[c] = end_t - start_t;
					cout<<"***************************\n";
					cout<<"Results: "<<endl;
					cout<<"total_iterations: "<<total_iterations[c]<<endl;
					cout<<"total_path_length: "<<total_path_length[c]<<endl;
					cout<<"repeatedCoverage: "<<repeatedCoverage[c]<<endl;
					cout<<"Total Computation Time: "<<time_to_compute[c]<<endl;
					cout<<"total_movement_time: "<<total_movement_time[c]<<endl;	
					cout<<"Complete Process (everything): "<<complete_process[c]<<endl;
					//cv::waitKey(0);
				}//for c
				if(problem_in_the_trial) continue;
				//values to be logged here
				for(int i = 0; i < number_of_algos; i++)
				{
					iterations[a][number_of_trials-trials].push_back(total_iterations[i]);
					repetedsteps[a][number_of_trials-trials].push_back(repeatedCoverage[i]);
					cummalative_path_length[a][number_of_trials-trials].push_back(total_path_length[i]);
					computationtime[a][number_of_trials-trials].push_back(time_to_compute[i]);
					movement_time[a][number_of_trials-trials].push_back(total_movement_time[i]);
				
				}
				
				trials--;
			}//while trials			
		}//number of robots
	}//number of maps

	ofstream outputFile;
	int l;
	for(int a = 0; a < number_of_maps; a++)
	{	
		string path;
		string save_address;
		switch(a)
		{
			case 0: path = "../Results/Basic/"; break;
			case 1: path = "../Results/Cluttered/"; break;
			case 2: path = "../Results/Office/"; break;
		}	
		cout<<"*************\n";
		cout<<"Map #"<<a<<endl<<endl<<endl;
		cout<<"*************\n";
		cout<<"iterations: "<<endl;
		save_address= path + "iterations.csv";
		outputFile.open(save_address);
		for(int i = 0; i < number_of_trials; i++)
		{
			
			l = 0;
			for(int j = 0; j < iterations[a][i].size(); j++)
			{
				outputFile<<iterations[a][i][j]<<",";
				cout<<iterations[a][i][j]<<" ";
				l++;
				l%=number_of_algos;
				if(l==0)
				{
					outputFile<<" "<<",";
					cout<<"** ";
				}
			}
			outputFile<<endl;
			cout<<endl;
		}
		outputFile.close();
		
		cout<<endl;
		cout<<"Redundant Coverage: "<<endl;
		save_address = path +"RedundantCoverage.csv";		
		outputFile.open(save_address);
		for(int i = 0; i < number_of_trials; i++)
		{
			l = 0;
			for(int j = 0; j < repetedsteps[a][i].size(); j++)
			{
				outputFile<<repetedsteps[a][i][j]<<",";
				cout<<repetedsteps[a][i][j]<<" ";
				l++;
				l%=number_of_algos;
				if(l==0)
				{
					outputFile<<" "<<",";
					cout<<"** ";
				}
			}
			outputFile<<endl;
			cout<<endl;
		}
		outputFile.close();	
		cout<<endl;
		cout<<"total_path_length: "<<endl;
		save_address = path + "total_path_length.csv";
		outputFile.open(save_address);
		for(int i = 0; i < number_of_trials; i++)
		{			
			l = 0;
			for(int j = 0; j < cummalative_path_length[a][i].size(); j++)
			{
				outputFile<<cummalative_path_length[a][i][j]<<",";
				cout<<cummalative_path_length[a][i][j]<<" ";
				l++;
				l%=number_of_algos;
				if(l==0)
				{
					outputFile<<" "<<",";
					cout<<"** ";
				}
			}
			outputFile<<endl;
			cout<<endl;
		}	
		outputFile.close();	

		cout<<endl;
		cout<<"Computation time: "<<endl;
		save_address = path + "ComputationTime.csv";
		outputFile.open(save_address);
		for(int i = 0; i < number_of_trials; i++)
		{
			l = 0;
			for(int j = 0; j < computationtime[a][i].size(); j++)
			{
				outputFile<<computationtime[a][i][j]<<",";
				cout<<computationtime[a][i][j]<<" ";
				l++;
				l%=number_of_algos;
				if(l==0)
				{
					outputFile<<" "<<",";
					cout<<"** ";
				}
			}
			outputFile<<endl;
			cout<<endl;
		}	
		outputFile.close();				
		cout<<endl;
		cout<<"total_movement_time: "<<endl;
		save_address = path +"total_movement_time.csv";
		outputFile.open(save_address);
		for(int i = 0; i < number_of_trials; i++)
		{
			l = 0;
			for(int j = 0; j < movement_time[a][i].size(); j++)
			{
				outputFile<<movement_time[a][i][j]<<",";
				cout<<movement_time[a][i][j]<<" ";
				l++;
				l%=number_of_algos;
				if(l==0)
				{
					outputFile<<" "<<",";
					cout<<"** ";
				}
			}
			outputFile<<endl;
			cout<<endl;
		}		
		outputFile.close();
		cout<<endl;
	}//for a
							
}




int main(int argc, char* argv[]) {
  bool get_results = true;
  if(get_results)
  {
  	getSimulatioResults(3, 25, 7);
  	return 0;
  }

  AprilInterfaceAndVideoCapture testbed;  
  int frame = 0;
  int first_iter = 1;
  double last_t = tic();
  const char *windowName = "Arena";
  cv::namedWindow(windowName,WINDOW_NORMAL);
 
  cv::Mat image;
  cv::Mat image_gray;
  
  //image = imread("../Maps/Basic.png");
  //cvtColor(image, image_gray, CV_BGR2GRAY);

  int robotCount = 4;  
  cout<<"Enter the number of robots: ";
  cin>>robotCount;
  
  //tag id should also not go beyond max_robots
  vector<vector<nd>> tp;//a map that would be shared among all
  vector<bot_config> bots(robotCount,bot_config(10, 10,130,tp));
  vector<PathPlannerGrid> planners(robotCount,PathPlannerGrid(tp));

  int algo_select;
  cout<<"\nSelect the Algorithm\n" 
  "1: BSA-CM (Basic)\n" 
  "2: BSA-CM with updated Backtrack Search\n" 
  "3: Boustrophedon Motion With Updated Bactrack Search\n"
  "4: Boustrophedon Motion With BSA_CM like Backtracking\n" 
  "5: BoB\n"
  "6: MDFS\n"
  "7: ANTS\n"
  "\nEnter here: ";
  int bots_in_same_cell = 0;
  cin>>algo_select;


  int repeatedCoverage = 0;
  int total_path_length = 0;
  int total_iterations = 0;
  double total_completion_time = 0;
  //double total_computation_time = 0;
  double time_to_compute = 0;
  double total_movement_time = 0;

  double start_t = tic();
  while (true){    
  	total_iterations++;
    image = imread("../Maps/Office.png");
  	cvtColor(image, image_gray, CV_BGR2GRAY);

    if(first_iter){
      bots[0].plan.overlayGrid(testbed.detections,image_gray);//overlay grid completely reintialize the grid, we have to call it once at the beginning only when all robots first seen simultaneously(the surrounding is assumed to be static) not every iteration
      for(int i = 1;i<bots.size();i++){
        bots[i].plan.rcells = bots[0].plan.rcells;
        bots[i].plan.ccells = bots[0].plan.ccells;
      }
     srand(time(0));
      for(int i = 0; i < bots.size(); i++)
      {
      	int r = rand()%bots[0].plan.rcells;
      	int c = rand()%bots[0].plan.ccells;
      	while((bots[0].plan.isBlocked(r, c)))
      	{
      		r = rand()%bots[0].plan.rcells;
      		c = rand()%bots[0].plan.ccells;
      	}
      	//bots[i].plan.path_points.push_back(pt(r, c));
      	bots[i].plan.addGridCellToPath(r, c, testbed);
      	bots[i].plan.world_grid[r][c].steps = 1;      	
      	bots[i].pose.x = r;
      	bots[i].pose.y = c;
      	bots[i].pose.omega = rand()%4;
      	bots[i].plan.current_orient = bots[i].pose.omega;
      	bots[i].plan.robot_id = i;
      	bots[i].plan.robot_tag_id = i;
      }
    }

    
 
    for(int i = 0;i<bots.size();i++){      
      planners[i] = bots[i].plan;
    }
    
    double compute_start = tic();
    for(int i = 0;i<bots.size();i++){
      switch(algo_select)
      {
      case 1: bots[i].plan.BSACoverageIncremental(testbed,bots[i].pose, 2.5,planners); break;
      case 2: bots[i].plan.BSACoverageWithUpdatedBactrackSelection(testbed,bots[i].pose, 2.5,planners); break;
      case 3: bots[i].plan.BoustrophedonMotionWithUpdatedBactrackSelection(testbed,bots[i].pose, 2.5,planners); break;
      case 4: bots[i].plan.BoustrophedonMotionWithBSA_CMlikeBacktracking(testbed,bots[i].pose, 2.5,planners); break;    
      case 5: bots[i].plan.BoB(testbed,bots[i].pose, 2.5,planners); break; 
      case 6: bots[i].plan.MDFS(testbed,bots[i].pose, 2.5,planners); break; 
      case 7: bots[i].plan.ANTS(testbed,bots[i].pose, 2.5,planners); break;     
      default: bots[i].plan.BSACoverageIncremental(testbed,bots[i].pose, 2.5,planners);   
      }   
    }
    double compute_end  = tic();
    time_to_compute += (compute_end-compute_start);

    double start_movement = tic();
    pair <int, int> wheel_velocities;//dummy variable in case of simulation
    for(int i = 0;i<bots.size();i++){    
        bots[i].plan.next_target_index = bots[i].plan.index_travelled+1;
        if((bots[i].plan.next_target_index) < bots[i].plan.path_points.size())
        {
        	if(bots[i].plan.movement_made==1 && !first_iter)
	        {
	        	bots[i].plan.last_orient = bots[i].plan.current_orient;
	        	int nx = bots[i].plan.path_points[bots[i].plan.next_target_index].x - bots[i].plan.path_points[bots[i].plan.next_target_index-1].x;
	        	int ny = bots[i].plan.path_points[bots[i].plan.next_target_index].y - bots[i].plan.path_points[bots[i].plan.next_target_index-1].y;
	        	if(nx==0 && ny==0) bots[i].plan.iter_wait = 0;
	        	else if(nx == -1 && ny == 0 )//up
	        	{
	        		bots[i].plan.current_orient = 0;	        	
	        	}
	        	else if(nx == 0 && ny == 1)//right
	        	{
	        		bots[i].plan.current_orient = 1;
	        	}
	        	else if(nx == 1 && ny == 0)//down
	        	{
	        		bots[i].plan.current_orient = 2;
	        	}
	        	else if(nx == 0 && ny == -1)//left
	        	{
	        		bots[i].plan.current_orient = 3;
	        	}
	        	if(!(nx==0 && ny==0))
	        	{
	        		if(abs(bots[i].plan.current_orient - bots[i].plan.last_orient)==0)
	        		{
	        			bots[i].plan.iter_wait = 0 + rand()%3;
	        		}
	        		else if(abs(bots[i].plan.current_orient - bots[i].plan.last_orient)%3==0)
	        		{
	        			bots[i].plan.iter_wait = 3 + rand()%3;
	        		}
	        		else if(abs(bots[i].plan.current_orient - bots[i].plan.last_orient)==1)
	        		{
	        			bots[i].plan.iter_wait = 3 + rand()%3;
	        		}
	        		else if(abs(bots[i].plan.current_orient - bots[i].plan.last_orient)==2)
	        		{
	        			bots[i].plan.iter_wait = 6 + rand()%3;
	        		}
	        	}
	        } 
        	if(!check_collision_possibility(testbed, planners, bots, wheel_velocities, i) && bots[i].plan.iter_wait <=0) {
        		bots[i].plan.index_travelled++;
        		bots[i].plan.movement_made = 1;
        	}
        	else{
        	bots[i].plan.iter_wait--; 
        	bots[i].plan.movement_made = 0;
        	}    	
        }     
   	}
   	double end_movement = tic();
   	total_movement_time += (end_movement-start_movement);    
    
    bots[0].plan.drawGrid(image, planners);
   	for(int i = 0;i<bots.size();i++){      	
        bots[i].plan.drawPath(image);        
    }
    for(int i = 0; i < bots.size(); i++)
    {
    	bots[i].plan.drawRobot(image);
    }
      //add a next point circle draw for visualisation
      //add a only shortest path invocation drawing function in pathplanners
      //correct next point by index to consider reach radius to determine the next point
    imshow(windowName,image);
    //cv::waitKey(0);
   /*for(int i = 0; i < bots.size()-1; i++)
    {
    	for(int j=i+1; j < bots.size(); j++)
    	{
    		if(bots[i].plan.path_points[bots[i].plan.index_travelled].x == bots[j].plan.path_points[bots[j].plan.index_travelled].x)
    		{
    			if(bots[i].plan.path_points[bots[i].plan.index_travelled].y == bots[j].plan.path_points[bots[j].plan.index_travelled].y)
    			{
    				cout<<"bots in same cell!\n";
    				cout<<"i, j: "<<i<<" "<<j<<endl;
    				cout<<"r,c: "<<bots[i].plan.path_points[bots[i].plan.index_travelled].x<<" "<<bots[i].plan.path_points[bots[i].plan.index_travelled].y<<endl;
    				bots_in_same_cell = 1;
    			}
    		}
    	}
    }
    if(bots_in_same_cell) cv::waitKey(0);*/
    bool completed = 1;
    for(int i = 0; i < bots.size(); i++)
    {
    	if(bots[i].plan.path_points.size()!=(bots[i].plan.next_target_index))
    	{
    		completed = 0;
    		break;
    	}
    }
    if(!first_iter && completed == 1)
    {
    	cout<<"Coverage Completed!\n";
    	break;
    }

    if(first_iter)
    {
     	first_iter = 0;
    }
    if (cv::waitKey(1) == 27){
        break;//until escape is pressed
    }
  }//while
  double end_t = tic();
  imshow(windowName,image);

  cout<<"***********************\n***************\n";
    	for(int i = 0; i < bots.size(); i++)
    	{
    		cout<<"id: "<<bots[i].plan.robot_tag_id<<endl;
    		cout<<"path points size(): "<<bots[i].plan.path_points.size()<<endl;
    		cout<<"index_travelled: "<<bots[i].plan.index_travelled<<endl;
    		cout<<"next_target_index: "<<bots[i].plan.next_target_index<<endl;
    		cout<<"current points: "<<bots[i].plan.path_points[bots[i].plan.index_travelled].x<<" "<<bots[i].plan.path_points[bots[i].plan.index_travelled].y<<endl;
    	}

	vector <vector<int>> coverage(bots[0].plan.rcells);
	for(int i = 0; i < bots[0].plan.rcells; i++)
	{
		coverage[i].resize(bots[0].plan.ccells);
	}
	for(int i = 0; i < bots.size(); i++)
	{
		total_path_length+= bots[i].plan.path_points.size();
		for(int j = 0; j < bots[i].plan.path_points.size(); j++)
		{
			coverage[bots[i].plan.path_points[j].x][bots[i].plan.path_points[j].y]++;
			if(coverage[bots[i].plan.path_points[j].x][bots[i].plan.path_points[j].y] > 1)
			{
				repeatedCoverage++;
			}
		}
	}
	//total_completion_time = time_to_compute + total_movement_time;
	double complete_process = end_t - start_t;
	total_movement_time = complete_process - time_to_compute;
	

	cout<<"***************************\n";
	cout<<"Results: "<<endl;
	cout<<"total_iterations: "<<total_iterations<<endl;
	cout<<"total_path_length: "<<total_path_length<<endl;
	cout<<"repeatedCoverage: "<<repeatedCoverage<<endl;
	cout<<"Total Computation Time: "<<time_to_compute<<endl;
	cout<<"total_movement_time: "<<total_movement_time<<endl;	
	cout<<"Complete Process (everything): "<<complete_process<<endl;
    cv::waitKey(0);
  return 0;
}
