/* Copyright (C) Brian Paden (bapaden@mit.edu)
 * Written by Brian Paden
 * Released under the GNU General Public License v3
 */

#include <glc_numerical_integration.h>

namespace glc{
  
  RungeKuttaTwo::RungeKuttaTwo(double lipschitz_constant_, 
                               double max_time_step_, 
                               int state_dim_): 
                               DynamicalSystem(lipschitz_constant_),
                               max_time_step(max_time_step_),
                               x1(state_dim_),
                               x2(state_dim_),
                               f0(state_dim_),
                               f1(state_dim_),
                               f2(state_dim_){}
  
  void RungeKuttaTwo::step(std::shared_ptr<InterpolatingPolynomial>& segment, const std::valarray<double>& x0, const std::shared_ptr<InterpolatingPolynomial>& u, const double& t1, const double& t2){  
    assert(t1<t2 && "[ERROR]: Integration step must be positive in RungeKuttaTwo");
    
    h=t2-t1;
    flow(f0,x0,u->at(t1));
    x1=x0+0.5*h*f0;
    flow(f1,x1,u->at(t1+0.5*h));
    x2=x0+h*f1;
    flow(f2,x2,u->at(t2));
    
    //Cubic interpolation between x0 and x2 with x'(t1)=f(x0,u(t0)) and x'(t2)=f(x2,u(t2))
    std::vector< std::valarray<double> > cubic;
    cubic.push_back(x0);//t^0 term
    cubic.push_back(f0);//t^1 term
    cubic.push_back((-2.0*f0+3.0*f1-f2)/h);//t^2 term
    cubic.push_back((f0-2.0*f1+f2)/(h*h));//t^3 term
    std::vector< std::vector< std::valarray<double> > > knot_point({cubic});
    segment=std::shared_ptr<InterpolatingPolynomial>(new InterpolatingPolynomial(knot_point,t2-t1,t1,x0.size(),4));
  }
  void RungeKuttaTwo::sim(std::shared_ptr<InterpolatingPolynomial>& solution, double t0, double tf, const std::valarray<double>& x0, const std::shared_ptr<InterpolatingPolynomial>& u){
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
