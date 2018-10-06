#ifndef GLC_NUMERICAL_INTEGRATION_H
#define GLC_NUMERICAL_INTEGRATION_H

#include <glc_interface.h>
#include <glc_interpolation.h>

namespace glc{

  //An integration scheme which has local error of O(dt^3)  
  class RungeKuttaTwo : public DynamicalSystem{
  protected:
    std::valarray<double> x1,x2,f0,f1,f2;
    double max_time_step;
    double h;
  public:
    RungeKuttaTwo(double lipschitz_constant_, double max_time_step_, int state_dim_);
    
    void sim(std::shared_ptr<InterpolatingPolynomial>& solution, double t0, double tf, const std::valarray<double>& x0, const std::shared_ptr<InterpolatingPolynomial>& u)override;
    
    //Step returns a spline between collocation points of the integrator
    void step(std::shared_ptr<InterpolatingPolynomial>& segment, const std::valarray<double>& x0, const std::shared_ptr<InterpolatingPolynomial>& u, const double& t1, const double& t2);
  };  
  
  
}//namespace
#endif