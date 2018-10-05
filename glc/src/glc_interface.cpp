/* Copyright (C) Brian Paden (bapaden@mit.edu) - All Rights Reserved
 * Written by Brian Paden
 * Released under the GNU General Public License v3
 */

#include <glc_interface.h>

namespace glc{

void Inputs::addInputSample(std::valarray<double>& _input){points.push_back(_input);}
const std::vector<std::valarray<double>>& Inputs::readInputs() const {
  return points;
}

CostFunction::CostFunction(double _lipschitz_constant):lipschitz_constant(_lipschitz_constant){}

double CostFunction::getLipschitzConstant(){
  return lipschitz_constant;
}

DynamicalSystem::DynamicalSystem(double _max_time_step):max_time_step(_max_time_step){}

void DynamicalSystem::sim(std::shared_ptr<InterpolatingPolynomial>& solution, double t0, double tf, const std::valarray<double>& x0, const std::shared_ptr<InterpolatingPolynomial>& u){
  assert(tf>t0);
  double num_steps=ceil((tf-t0)/max_time_step);
  double integration_step=(tf-t0)/num_steps;
  solution = std::shared_ptr<InterpolatingPolynomial>(new InterpolatingPolynomial(integration_step,t0,x0.size(),4));
  solution->reserve(num_steps);
  //set initial state and time
  std::valarray<double> state=x0;
  double time=t0;
  std::shared_ptr<InterpolatingPolynomial> traj_segment;
  //integrate
  for(int i=0;i<num_steps;i++){
    //Use numerical integration scheme to compute a spline extending from state with input u([t,t+integration_step])
    step(traj_segment,state,u,time,time+integration_step);
    //add traj_segment to solution
    solution->concatenate(traj_segment);
    time+=integration_step;
    state=traj_segment->at(time);
  }
  sim_counter++;
  return;
}

}