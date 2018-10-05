/* Copyright (C) Brian Paden (bapaden@mit.edu) - All Rights Reserved
 * Written by Brian Paden
 * Incorporation into open source software is not permitted.
 * Use in private (closed source) projects for academic research is permitted.
 */

/* This example illustrates how to interface with the glc planner.
 * One must provide:
 * 
 * 1) A finite subset of admissible control inputs
 * parameterized by a resolution so that this finite
 * so that this finite set converges to a dense subset. 
 * 
 * 2) A goal checking subroutine that determines
 * if a trajectory opject intersects the goal set.
 * 
 * 3) An admissible heuristic that underestimates 
 * the optimal cost-to-go from every feasible state.
 * 
 * 4) A dynamic model describing the response of the 
 * system to control inputs and also a lipschitz 
 * constant for the model.
 * 
 * 5) A feasibility or collision checking function.
 * 
 * 6) A cost functional for candidate trajectories.
 */

#include <glc_planner.h>

////////////////////////////////////////////////////////
/////////Discretization of Control Inputs///////////////
////////////////////////////////////////////////////////
class CarControlInputs: public glc::Inputs{
public:
  //uniformly spaced points on a circle
  CarControlInputs(int num_steering_angles){
    //Make all pairs (forward_speed,steering_angle)
    glc::vctr car_speeds({1.0});//Pure path planning
    glc::vctr steering_angles = glc::linearSpace(-0.0625*M_PI,0.0625*M_PI,num_steering_angles);
    glc::vctr control_input(2);
    for(double& vel : car_speeds){
      for(double& ang : steering_angles ){
        control_input[0]=vel;
        control_input[1]=ang;
        addInputSample(control_input);
      }
    }
  }
};

////////////////////////////////////////////////////////
///////////////Goal Checking Interface//////////////////
////////////////////////////////////////////////////////
class Sphericalgoal: public glc::GoalRegion{
  double radius_sqr;
  glc::vctr center;
  int resolution;
public:
  Sphericalgoal(double& _goal_radius_sqr, 
                glc::vctr& _goal_center,
                int _resolution):
                resolution(_resolution),
                center(_goal_center),
                radius_sqr(_goal_radius_sqr){}

  //Returns true if traj intersects goal and sets t to the first time at which the trajectory is in the goal
  bool inGoal(const glc::splinePtr& traj, double& time) override {
    time=traj->initialTime();
    
    double dt=(traj->numberOfIntervals()*traj->intervalLength())/resolution;
    for(int i=0;i<resolution;i++){
      time+=dt;//don't need to check t0 since it was part of last traj
      glc::vctr state = traj->at(time);
        if(glc::sqr(state[0]-center[0]) + glc::sqr(state[1]-center[1]) < radius_sqr){return true;}
    }
    return false;
  }
};

////////////////////////////////////////////////////////
////////Problem Specific Admissible Heuristic///////////
////////////////////////////////////////////////////////
class EuclideanHeuristic : public glc::Heuristic{
  double radius;
public:
  EuclideanHeuristic(glc::vctr& _goal,double _radius):radius(_radius){goal=_goal;}
  double costToGo(const glc::vctr& state){
    return std::max(0.0,sqrt(glc::sqr(goal[0]-state[0])+glc::sqr(goal[1]-state[1]))-radius);//offset by goal radius
  }
};

////////////////////////////////////////////////////////
/////////////////Dynamic Model//////////////////////////
////////////////////////////////////////////////////////
class CarNonholonomicConstraint : public glc::SymplecticEuler {
public:
  CarNonholonomicConstraint(const double& _max_time_step): SymplecticEuler(_max_time_step,3) {}
  void flow(glc::vctr& dx, const glc::vctr& x, const glc::vctr& u) override {
    dx[0]=u[0]*cos(x[2]);
    dx[1]=u[0]*sin(x[2]);
    dx[2]=u[1];
  }
  double getLipschitzConstant(){return 1.0;}
};

////////////////////////////////////////////////////////
/////////////////Cost Function//////////////////////////
////////////////////////////////////////////////////////
class ArcLength: public glc::CostFunction 
{
  double sample_resolution;
public:
  ArcLength(int _sample_resolution) : glc::CostFunction(0.0),sample_resolution(double(_sample_resolution)){}
  
  double cost(const glc::splinePtr& traj, const glc::splinePtr& control, double t0, double tf) override {
    return traj->numberOfIntervals()*traj->intervalLength();
  }
};

////////////////////////////////////////////////////////
/////////////////State Constraints//////////////////////
////////////////////////////////////////////////////////
class PlanarDemoObstacles: public glc::Obstacles{
  int resolution;
  glc::vctr center1;
  glc::vctr center2;
public:        
  PlanarDemoObstacles(int _resolution):resolution(_resolution),center1({3.0,2.0}),center2({6.0,8.0}){}
  bool collisionFree(const glc::splinePtr& traj) override {
    double t=traj->initialTime();
    double dt=(traj->numberOfIntervals()*traj->intervalLength())/resolution;
    glc::vctr state;
    for(int i=0;i<resolution;i++){
      t+=dt;//don't need to check t0 since it was part of last traj
      state=traj->at(t);
      
      //Disk shaped obstacles
      if(glc::sqr(state[0]-center1[0])+glc::sqr(state[1]-center1[1]) <= 4.0 or
         glc::sqr(state[0]-center2[0])+glc::sqr(state[1]-center2[1]) <= 4.0 )
      {
           return false;
      }
    }
    return true;
  }
};

////////////////////////////////////////////////////////
///////////////Run a planning query in main/////////////
////////////////////////////////////////////////////////
int main() 
{
  //Motion planning algorithm parameters
  glc::Parameters alg_params;
  alg_params.res=21;
  alg_params.control_dim = 2;
  alg_params.state_dim = 3;
  alg_params.depth_scale = 100;
  alg_params.dt_max = 5.0;
  alg_params.max_iter = 50000;
  alg_params.time_scale = 20;
  alg_params.partition_scale = 60;
  alg_params.x0 = glc::vctr({0.0,0.0,M_PI/2.0});
  
  //Create a dynamic model
  CarNonholonomicConstraint dynamic_model(alg_params.dt_max);
  
  //Create the control inputs
  CarControlInputs controls(alg_params.res);
  
  //Create the cost function
  ArcLength performance_objective(4);
  
  //Create instance of goal region
  double goal_radius_sqr(.25);
  glc::vctr goal_center({10.0,10.0});
  Sphericalgoal goal(goal_radius_sqr,goal_center,10);
  
  //Create the obstacles
  PlanarDemoObstacles obstacles(10);
  
  //Create a heuristic for the current goal
  EuclideanHeuristic heuristic(goal_center,sqrt(goal_radius_sqr));
  glc::GLCPlanner planner(&obstacles,
                          &goal,
                          &dynamic_model,
                          &heuristic,
                          &performance_objective,
                          alg_params,
                          controls.points);
  
  //Run the planner and print solution
  glc::PlannerOutput out;
  planner.plan(out);
  if(out.solution_found){
    std::vector<glc::nodePtr> path = planner.pathToRoot(true);
    glc::splinePtr solution = planner.recoverTraj( path );
    glc::splineTensor coef(solution->coefficient_array);
    glc::printSpline( solution , 20, "Solution");
    glc::trajectoryToFile("shortest_path.txt","../plots/",solution,500);
  }
    glc::nodesToFile("shortest_path_nodes.txt","../plots/",planner.domain_labels);
  return 0;
}
