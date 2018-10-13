/* Copyright (C) Brian Paden (bapaden@mit.edu)
 * Written by Brian Paden
 * Released under the MIT License
 */

#include<glc_interpolation.h>
#include<gtest/gtest.h>

using namespace glc;

/**
 * This test checks that the constructor for builds the interpolating polynomial correctly
 */
TEST(GlcInterpolation,Constructor){
  
  //These are the test values for the InterpolatingPolynomial's attributes
  double initial_time = 5.0;
  double interval_length = 2.0;
  int degree = 2;
  int dimension = 3;
  int number_of_intervals = 3;
  
  //Here the coefficient array for an interpolating polynomial spline is created
  std::vector<std::vector<std::valarray<double>>> coefficient_array;
  for(int j=0;j<number_of_intervals;j++){
    //This will be the coefficients of a polynomial in R^3 with the basis c_i*t^i for i ranging from 0 to 2 
    std::vector< std::valarray<double> > polynomial_xyz;
    for(int i=0;i<degree+1;i++){
      //A trivial spline is created with f(t)=(i+i*t+i*t^2,0,0) for all t
      polynomial_xyz.push_back(std::valarray<double>({double(i),0.,0.}));
    }
    coefficient_array.push_back(polynomial_xyz);
  }
  std::shared_ptr<InterpolatingPolynomial> curve(new InterpolatingPolynomial(coefficient_array,
                                                                             interval_length,
                                                                             initial_time,
                                                                             dimension,
                                                                             degree));
  
  EXPECT_NEAR(initial_time,curve->initialTime(),1e-10);
  EXPECT_NEAR(interval_length,curve->intervalLength(),1e-10);
  EXPECT_NEAR(number_of_intervals,curve->numberOfIntervals(),1e-10);
  EXPECT_NEAR(5.0,curve->initialTime(),1e-10);
  for(double t=initial_time;t<initial_time+double(number_of_intervals)*interval_length;t+=0.2){
    std::valarray<double> value = curve->at(t); 
    EXPECT_NEAR(value[1],0.,1e-10);
    EXPECT_NEAR(value[2],0.,1e-10);
  }
}

/**
 * This tests that concatenating two splines creates a spline that has twice the domain
 */
TEST(GlcInterpolation,Concatenation){
  double initial_time = 5.0;
  double interval_length = 2.0;
  int degree = 3;
  int dimension = 3;
  int number_of_intervals = 3;
  
  //Here the coefficient array for an interpolating polynomial spline is created
  std::vector<std::vector<std::valarray<double>>> coefficient_array;
  for(int j=0;j<number_of_intervals;j++){
    //This will be the coefficients of a polynomial in R^3 with the basis c_i*t^i for i ranging from 0 to 2 
    std::vector< std::valarray<double> > polynomial_xyz;
    for(int i=0;i<degree+1;i++){
      //A trivial spline is created with f(t)=(i+i*t+i*t^2,0,0) for all t
      polynomial_xyz.push_back(std::valarray<double>({double(i),0.,0.}));
    }
    coefficient_array.push_back(polynomial_xyz);
  }
  std::shared_ptr<InterpolatingPolynomial> curve(new InterpolatingPolynomial(coefficient_array,
                                                                              interval_length,
                                                                              initial_time,
                                                                              dimension,
                                                                              degree));
  std::shared_ptr<InterpolatingPolynomial> tail(new InterpolatingPolynomial(coefficient_array,
                                                                              interval_length,
                                                                              initial_time,
                                                                              dimension,
                                                                              degree));
  curve->concatenate(tail);
  EXPECT_NEAR(initial_time,curve->initialTime(),1e-10);
  EXPECT_NEAR(interval_length,curve->intervalLength(),1e-10);
  EXPECT_EQ(number_of_intervals+number_of_intervals,curve->numberOfIntervals());
  EXPECT_NEAR(5.0,curve->initialTime(),1e-10);
}



int main(int argc, char** argv){
  
  ::testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
  
}
