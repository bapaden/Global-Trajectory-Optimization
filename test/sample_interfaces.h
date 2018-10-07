/* Copyright (C) Brian Paden (bapaden@mit.edu)
 * Written by Brian Paden
 * Released under the GNU General Public License v3
 */

#ifndef GLC_SAMPLE_INTERFACE_H
#define GLC_SAMPLE_INTERFACE_H

#include <glc_planner_core.h>

////////////////////////////////////////////////////////
/////////Discretization of Control Inputs///////////////
////////////////////////////////////////////////////////
class ControlInputs2D : public glc::Inputs{
  
public:
  //uniformly spaced points on a circle
  ControlInputs2D(int num_inputs){
    std::valarray<double> u(2);
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
  std::valarray<double> error;
  std::valarray<double> x_g;
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
                bool inGoal(const std::shared_ptr<glc::InterpolatingPolynomial>& traj, double& time) override {
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
                void setGoal(std::valarray<double>& _x_g){x_g=_x_g;}
                std::valarray<double> getGoal(){return x_g;}
};

////////////////////////////////////////////////////////
/////////////////Dynamic Model//////////////////////////
////////////////////////////////////////////////////////
class SingleIntegrator : public glc::RungeKuttaTwo{
public:
  SingleIntegrator(const double& max_time_step_): glc::RungeKuttaTwo(0.0,max_time_step_,2) {}
  void flow(std::valarray<double>& dx, const std::valarray<double>& x, const std::valarray<double>& u) override {dx=u;}
  double getLipschitzConstant(){return 0.0;}
};

////////////////////////////////////////////////////////
/////////////////Cost Function//////////////////////////
////////////////////////////////////////////////////////
class ArcLength: public glc::CostFunction 
{
  double sample_resolution;
public:
  ArcLength(int _sample_resolution) : glc::CostFunction(0.0),sample_resolution(double(_sample_resolution)){}
  
  double cost(const std::shared_ptr<glc::InterpolatingPolynomial>& traj, 
              const std::shared_ptr<glc::InterpolatingPolynomial>& control, 
              double t0, 
              double tf) const {
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


class CarControlInputs: public glc::Inputs{
public:
  //uniformly spaced points on a circle
  CarControlInputs(int num_steering_angles){
    //Make all pairs (forward_speed,steering_angle)
    std::valarray<double> car_speeds({1.0});//Pure path planning
    std::valarray<double> steering_angles = glc::linearSpace(-0.0625*M_PI,0.0625*M_PI,num_steering_angles);
    std::valarray<double> control_input(2);
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
  std::valarray<double> center;
  int resolution;
public:
  Sphericalgoal(double& _goal_radius_sqr, 
                std::valarray<double>& _goal_center,
                int _resolution):
                resolution(_resolution),
                center(_goal_center),
                radius_sqr(_goal_radius_sqr){}
                
                //Returns true if traj intersects goal and sets t to the first time at which the trajectory is in the goal
                bool inGoal(const std::shared_ptr<glc::InterpolatingPolynomial>& traj, double& time) override {
                  time=traj->initialTime();
                  
                  double dt=(traj->numberOfIntervals()*traj->intervalLength())/resolution;
                  for(int i=0;i<resolution;i++){
                    time+=dt;//don't need to check t0 since it was part of last traj
                    std::valarray<double> state = traj->at(time);
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
  std::valarray<double> goal;
public:
  EuclideanHeuristic(std::valarray<double>& _goal,double _radius):radius(_radius),goal(_goal){}
  double costToGo(const std::valarray<double>& state)const{
    return std::max(0.0,sqrt(glc::sqr(goal[0]-state[0])+glc::sqr(goal[1]-state[1]))-radius);//offset by goal radius
  }
};

////////////////////////////////////////////////////////
/////////////////Dynamic Model//////////////////////////
////////////////////////////////////////////////////////
class CarNonholonomicConstraint : public glc::RungeKuttaTwo {
public:
  CarNonholonomicConstraint(const double& _max_time_step): RungeKuttaTwo(1.0,_max_time_step,3) {}
  void flow(std::valarray<double>& dx, const std::valarray<double>& x, const std::valarray<double>& u) override {
    dx[0]=u[0]*cos(x[2]);
    dx[1]=u[0]*sin(x[2]);
    dx[2]=u[1];
  }
  double getLipschitzConstant(){return lipschitz_constant;}
};

////////////////////////////////////////////////////////
/////////////////State Constraints//////////////////////
////////////////////////////////////////////////////////
class PlanarDemoObstacles: public glc::Obstacles{
  int resolution;
  std::valarray<double> center1;
  std::valarray<double> center2;
public:        
  PlanarDemoObstacles(int _resolution):resolution(_resolution),center1({3.0,2.0}),center2({6.0,8.0}){}
  bool collisionFree(const std::shared_ptr<glc::InterpolatingPolynomial>& traj) override {
    double t=traj->initialTime();
    double dt=(traj->numberOfIntervals()*traj->intervalLength())/resolution;
    std::valarray<double> state;
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


#endif