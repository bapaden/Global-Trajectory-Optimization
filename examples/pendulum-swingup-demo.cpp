#include <glc_planner_core.h>

namespace example{

  /**
   * This is an example of a discrete input set for the torque limited
   * pendulum swingup problem where the sampling of inputs from the
   * addmissible torque inputs is equal to the resolution parameter for
   * the glc algorithm.
   */
  class PendulumTorque : public glc::Inputs{
  public:
    PendulumTorque(int resolution){
      std::valarray<double> u(1);
      double max_torque=0.1;
      for(double torque=-max_torque;torque<=max_torque;torque+=2.0*max_torque/resolution){
        u[0]=torque;
        addInputSample(u);
      }
    }
  };
  
/**
 * The goal region for this example is a small circular domain in 
 * the angle-velocity space for the pendulum centered where 
 * the pendulum is in the inverted position and the velocity is zero.
 * It is required that the goal region have a nonempty interior.
 */  
 class SphericalGoal: public glc::GoalRegion{
    double goal_radius, goal_radius_sqr;
    std::valarray<double> error;
    std::valarray<double> x_g;
    int num_samples;
  public:
    SphericalGoal(const int& _state_dim, 
                  const double& _goal_radius,
                  int _num_samples):
                  x_g(_state_dim,0.0), 
                  num_samples(_num_samples),
                  goal_radius(_goal_radius),
                  error(_state_dim,0.0){
                    goal_radius_sqr=glc::sqr(goal_radius);
                  }
                  bool inGoal(const std::shared_ptr<glc::InterpolatingPolynomial>& traj, double& time) override {
                    time=traj->initialTime();
                    double dt=(traj->numberOfIntervals()*traj->intervalLength())/num_samples;
                    for(int i=0;i<num_samples;i++){
                      time+=dt;
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
                  void setGoal(std::valarray<double>& _x_g){x_g=_x_g;}
                  std::valarray<double> getGoal(){return x_g;}
  };
  
  /**
   * Since the pendulum has non-trivial dynamics, we will use the
   * zero heuristic for this example
   */
  class ZeroHeuristic : public glc::Heuristic{
  public:
    ZeroHeuristic(){}
    double costToGo(const std::valarray<double>& state) const {
      return 0.0;
    }
  };
  
  /**
   * The dynamic model for this example is a pendulum with
   * origin defined at the stable equilibrium. The input
   * is a torque applied at the pivot.
   */
  class InvertedPendulum : public glc::RungeKuttaTwo{
  public:
    //For the chosen coordinate system for the dynamic model, the Lipschitz constant is 1.0
    InvertedPendulum(const double& max_time_step_): glc::RungeKuttaTwo(1.0,max_time_step_,2) {}
    void flow(std::valarray<double>& dx, const std::valarray<double>& x, const std::valarray<double>& u) override {
      dx[0] = x[1];
      dx[1] = u[0] - sin(x[0]);
    }
    double getLipschitzConstant(){return lipschitz_constant;}
  };
  
  /**
   * We will use the minimum time performance objective.
   */
  class MinTime: public glc::CostFunction 
  {
    double sample_resolution;
  public:
    MinTime(int _sample_resolution) : glc::CostFunction(0.0),sample_resolution(double(_sample_resolution)){}
    
    double cost(const std::shared_ptr<glc::InterpolatingPolynomial>& traj, 
                const std::shared_ptr<glc::InterpolatingPolynomial>& control, 
                double t0, 
                double tf) const {
                  return tf-t0;
                }
  };
  
  /**
   * This is an unconstrained free space all states are collision free
   */
  class UnconstrainedSpace: public glc::Obstacles{
  public:        
    UnconstrainedSpace(int _resolution){}
    bool collisionFree(const std::shared_ptr<glc::InterpolatingPolynomial>& traj) override {return true;}
  };
}//namespace example

int main() 
{
  using namespace example;
  //Motion planning algorithm parameters
  glc::Parameters alg_params;
  alg_params.res=15;
  alg_params.control_dim = 1;
  alg_params.state_dim = 2;
  alg_params.depth_scale = 100;
  alg_params.dt_max = 5.0;
  alg_params.max_iter = 50000;
  alg_params.time_scale = 20;
  alg_params.partition_scale = 30;
  alg_params.x0 = std::valarray<double>({0.0,0.0});
  
  //Create a dynamic model
  InvertedPendulum dynamic_model(alg_params.dt_max);
  
  //Create the control inputs
  PendulumTorque controls(alg_params.res);
  
  //Create the cost function
  MinTime performance_objective(4);
  
  //Create instance of goal region
  std::valarray<double> xg({M_PI,0.0});
  SphericalGoal goal(xg.size(),0.25,4);
  goal.setGoal(xg);
  
  //Create the obstacles
  UnconstrainedSpace obstacles(4);
  
  //Create a heuristic for the current goal
  ZeroHeuristic heuristic;
  glc::Planner planner(&obstacles,
                       &goal,
                       &dynamic_model,
                       &heuristic,
                       &performance_objective,
                       alg_params,
                       controls.readInputs());
  
  //Run the planner and print solution
  glc::PlannerOutput out;
  planner.plan(out);
  if(out.solution_found){
    std::vector<std::shared_ptr<const glc::Node>> path = planner.pathToRoot(true);
    std::shared_ptr<glc::InterpolatingPolynomial> solution = planner.recoverTraj( path );
//     solution->printSpline(20, "Solution");
    solution->printData();
    glc::trajectoryToFile("pendulum_swingup_demo.txt","./",solution,500);
    glc::nodesToFile("pendulum_swingup_demo_nodes.txt","./",planner.partition_labels);
  }
  return 0;
}