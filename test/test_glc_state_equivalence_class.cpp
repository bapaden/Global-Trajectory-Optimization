/* Copyright (C) Brian Paden (bapaden@mit.edu)
 * Written by Brian Paden
 * Released under the MIT License
 */

#include<glc/glc_state_equivalence_class.h>
#include<gtest/gtest.h>

using namespace glc;

/**
 * \brief Test that the square function squares a number.
 */
TEST(GlcStateEquivalence,Order){
  double x = 2.0;
  
  EXPECT_EQ(sqr(x),4.0);
}

/**
 * \brief Tests that the size of input and output to vecFloor is the same, and that the element-wise floor operation works as expected.
 */
TEST(GlcMath,TestFloor){
  std::valarray<double> x = {1.1,2.2,3.99,4.05,-2.01};
  std::vector<int> y = vecFloor(x);
  
  EXPECT_EQ(y.size(),x.size());
  EXPECT_EQ(y[0],1);
  EXPECT_EQ(y[1],2);
  EXPECT_EQ(y[2],3);
  EXPECT_EQ(y[3],4);
  EXPECT_EQ(y[4],-3);
}

/**
 * \brief Tests that the inner product between two vectors is calculated correctly.
 */
TEST(GlcMath,TestDot){
  std::valarray<double> x = {1.,1.,1.};
  std::valarray<double> y = {1.,2.,3.};
  double inner_product_a = dot(x,y);
  
  EXPECT_EQ(inner_product_a,6.);
  
  std::valarray<double> a = {1.,1.,-1.};
  std::valarray<double> b = {1.,2.,3.};
  double inner_product_b = dot(a,b);
  
  EXPECT_EQ(inner_product_b,0.);
}

/**
 * \brief Tests that the square of the norm of a vector is calculated correctly.
 */
TEST(GlcMath,TestNormSqr){
  std::valarray<double> x = {1.,2.,3.};
  double norm_square_a = normSqr(x);
  
  EXPECT_EQ(norm_square_a,14.);
  
  std::valarray<double> y = {1.,2.,-3.};
  double norm_square_b = normSqr(x);
  
  EXPECT_EQ(norm_square_b,14.);
}

/**
 * \brief Tests that the 2-norm of a vector is calculated correctly
 */
TEST(GlcMath,TestNorm){
  std::valarray<double> a = {1.,1.,1.};
  double norm_a = norm2(a);
  EXPECT_EQ(norm_a,sqrt(3.));
  
  std::valarray<double> b = {3.,0.,0.};
  double norm_b = norm2(b);
  EXPECT_EQ(norm_b,3.);
}


/**
 * \brief Tests that the linearSpace method samples uniformly across an interval
 */
TEST(GlcMath,LinearSpace){
  std::valarray<double> points = linearSpace(0.,1.,5);
  EXPECT_NEAR(points[0],0.0,1e-8);
  EXPECT_NEAR(points[1],0.2,1e-8);
  EXPECT_NEAR(points[2],0.4,1e-8);
  EXPECT_NEAR(points[3],0.6,1e-8);
  EXPECT_NEAR(points[4],0.8,1e-8);
}


int main(int argc, char** argv){
  
  ::testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
  
}
