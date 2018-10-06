#include<glc_numerical_integration.h>
#include<gtest/gtest.h>

using namespace glc;

/**
 * A concrete implementation of a dynamical system is required to test the numerical integration methods
 */
class SingleIntegrator : public glc::RungeKuttaTwo{
public:
  SingleIntegrator(const double& max_time_step_): glc::RungeKuttaTwo(max_time_step_,2) {}
  void flow(std::valarray<double>& dx, const std::valarray<double>& x, const std::valarray<double>& u) override {dx=u;}
  double getLipschitzConstant(){return 0.0;}
};

/**
 * This tests that the numerical integration integrates the input correctly for the given dynamical system
 */
TEST(GlcNode,Order){
  //Create an instance of a 2D single integrator
  SingleIntegrator holonomic_model(1.0);
  //Construct an InerpolatingPolynomial for a constant control input of (1,0) for 1 time unit
  std::valarray<double> initial_state({0.0,0.0});
  std::valarray<double> control({1.0,0.0});
  std::vector< std::valarray<double> > zero_order_interp({control});
  std::vector< std::vector< std::valarray<double> > > zero_order_interp_array({zero_order_interp});
  std::shared_ptr<InterpolatingPolynomial> control_segment(new InterpolatingPolynomial(zero_order_interp_array,1.0,0.0,2,1));
  //Simulate dynamics with input control_segment
  std::shared_ptr<InterpolatingPolynomial> traj_segment;
  holonomic_model.sim(traj_segment, 0.0, 1.0, initial_state,control_segment);
  
  //Check that the resulting trajectory matches hand-calculated solution of x(t) = (t,0)
  for(double t=0;t<=1.0;t+=0.1){
    EXPECT_NEAR(traj_segment->at(t)[0],t,1e-4);
    EXPECT_NEAR(traj_segment->at(t)[1],0.0,1e-4);
  }
}

int main(int argc, char** argv){
  
  ::testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
  
}
