
/* This example illustrates how to interface with the glc planner.
 * One must provide:
 * 
 * 1) A finite subset of admissible control inputs
 * parameterized by a resolution so that this finite
 * so that this finite set converges to a dense subset
 * with increasing resolution. 
 * 
 * 2) A goal checking subroutine that determines
 * if a trajectory object intersects the goal set.
 * 
 * 3) An admissible heuristic that underestimates 
 * the optimal cost-to-go from every feasible state.
 * One can always use h(x)=0 for all x as a heuristic.
 * 
 * 4) A dynamic model describing the response of the 
 * system to control inputs and also a lipschitz 
 * constant for the model.
 * 
 * 5) A feasibility or collision checking function.
 * 
 * 6) A Lipschitz continuous cost running cost for
 * candidate trajectories along with a known Lipschitz
 * constant.
 */

#include <glc_planner_core.h>

////////////////////////////////////////////////////////
/////////Discretization of Control Inputs///////////////
////////////////////////////////////////////////////////
class ControlInputs2D : public glc::Inputs{

public:
  //uniformly spaced points on a circle
  ControlInputs2D(int num_inputs){
    glc::vctr u(2);
    for(int i=0;i<num_inputs;i++){
      u[0]=sin(2.0*i*M_PI/num_inputs);
      u[1]=cos(2.0*i*M_PI/num_inputs);
      addInputSample(u);
    }
  }
};

////////////////////////////////////////////////////////
///////////////Goal Checking Interface//////////////////
////////////////////////////////////////////////////////
class SphericalGoal: public glc::GoalRegion{
  double goal_radius, goal_radius_sqr;
  glc::vctr error;
  glc::vctr x_g;
  int resolution;
public:
  SphericalGoal(const int& _state_dim, 
                const double& _goal_radius,
                int _resolution):
                x_g(_state_dim,0.0), 
                resolution(_resolution),
                goal_radius(_goal_radius),
                error(_state_dim,0.0)
                {
                  goal_radius_sqr=glc::sqr(goal_radius);
                }
  //Returns true if traj intersects goal and sets t to the first time at which the trajectory is in the goal
  bool inGoal(const glc::splinePtr& traj, double& time) override {
    time=traj->initialTime();
    
    double dt=(traj->numberOfIntervals()*traj->intervalLength())/resolution;
    for(int i=0;i<resolution;i++){
      time+=dt;//don't need to check t0 since it was part of last traj
      error=x_g-traj->at(time);
      if(glc::dot(error,error) < goal_radius_sqr){
        return true;}
    }
    return false;
  }

  void setRadius(double r){
    goal_radius = r;
    goal_radius_sqr = r*r;
  }
  double getRadius(){return goal_radius;}
  void setGoal(glc::vctr& _x_g){x_g=_x_g;}
  glc::vctr getGoal(){return x_g;}
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
class SingleIntegrator : public glc::SymplecticEuler {
public:
  SingleIntegrator(const double& _max_time_step): SymplecticEuler(_max_time_step,2) {}
  void flow(glc::vctr& dx, const glc::vctr& x, const glc::vctr& u) override {dx=u;}
  double getLipschitzConstant(){return 0.0;}//TODO override for DynamicSystem
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
    double c(0);
    double t = traj->initialTime();
    double dt = (tf-t0)/sample_resolution;
    for(int i=0;i<sample_resolution;i++){
      c+=glc::norm2(traj->at(t+dt)-traj->at(t));
      t+=dt;
    }
    return c;
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
      if(glc::norm_sqr(state-center1)<=4.0 or glc::norm_sqr(state-center2)<=4.0){return false;}
    }
    return true;
  }
};

int main() 
{
  //Motion planning algorithm parameters
  glc::Parameters alg_params;
  alg_params.res=16;
  alg_params.control_dim = 2;
  alg_params.state_dim = 2;
  alg_params.depth_scale = 100;
  alg_params.dt_max = 5.0;
  alg_params.max_iter = 50000;
  alg_params.time_scale = 20;
  alg_params.partition_scale = 40;
  alg_params.x0 = glc::vctr({0.0,0.0});
  
  //Create a dynamic model
  SingleIntegrator dynamic_model(alg_params.dt_max);
  
  //Create the control inputs
  ControlInputs2D controls(alg_params.res);
  
  //Create the cost function
  ArcLength performance_objective(4);
  
  //Create instance of goal region
  glc::vctr xg({10.0,10.0});
  SphericalGoal goal(xg.size(),0.25,4);
  goal.setGoal(xg);
  
  //Create the obstacles
  PlanarDemoObstacles obstacles(4);
  
  //Create a heuristic for the current goal
  EuclideanHeuristic heuristic(xg,goal.getRadius());
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
    glc::nodesToFile("shortest_path_nodes.txt","../plots/",planner.domain_labels);
  }
  return 0;
}
