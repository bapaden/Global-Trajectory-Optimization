/* Copyright (C) Brian Paden (bapaden@mit.edu)
 * Written by Brian Paden
 * Released under the MIT License
 */

#include <gtest/gtest.h>
#include "sample_interfaces.h"

using namespace test;

/**
 * This test runs a 2D shortest path problem and checks
 * the solution cost against a gold standard solution
 * from a point where the implementation is believed
 * to be correct.
 */ 
TEST(Planner,TestShortestPathSolution){
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
  alg_params.x0 = std::valarray<double>({0.0,0.0});
  
  //Create a dynamic model
  SingleIntegrator dynamic_model(alg_params.dt_max);
  
  //Create the control inputs
  ControlInputs2D controls(alg_params.res);
  
  //Create the cost function
  ArcLength performance_objective(4);
  
  //Create instance of goal region 
  std::valarray<double> xg({10.0,10.0});
  SphericalGoal goal(xg.size(),0.25,4);
  goal.setGoal(xg);
  
  //Create the obstacles
  PlanarDemoObstacles obstacles(4);
  
  //Create a heuristic for the current goal
  EuclideanHeuristic heuristic(xg,goal.getRadius());
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
  
  EXPECT_NEAR(out.cost,14.8374,1e-3);
  
}

/**
 * This test runs a shortest path problem with a 
 * nonholonomic constraint. It checks that a solution
 * is found for this feasible problem, and secondly
 * that the output curve is continuous.
 */
TEST(Planner,TestNonholonomicSolutionContinuity){
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
  alg_params.x0 = std::valarray<double>({0.0,0.0,M_PI/2.0});
  
  //Create a dynamic model
  CarNonholonomicConstraint dynamic_model(alg_params.dt_max);
  
  //Create the control inputs
  CarControlInputs controls(alg_params.res);
  
  //Create the cost function
  ArcLength performance_objective(4);
  
  //Create instance of goal region
  double goal_radius_sqr(.25);
  std::valarray<double> goal_center({10.0,10.0});
  Sphericalgoal goal(goal_radius_sqr,goal_center,10);
  
  //Create the obstacles
  PlanarDemoObstacles obstacles(10);
  
  //Create a heuristic for the current goal
  EuclideanHeuristic heuristic(goal_center,sqrt(goal_radius_sqr));
  glc::Planner planner(&obstacles,
                       &goal,
                       &dynamic_model,
                       &heuristic,
                       &performance_objective,
                       alg_params,
                       controls.readInputs());
  glc::PlannerOutput out;
  planner.plan(out);
  EXPECT_TRUE(out.solution_found);
  
  std::vector<std::shared_ptr<const glc::Node>> path = planner.pathToRoot(true);
  std::shared_ptr<glc::InterpolatingPolynomial> solution = planner.recoverTraj( path );
  double t0 = solution->initialTime();
  double tf = solution->initialTime()+double(solution->numberOfIntervals())*solution->intervalLength();
  
  for(double t=t0;t<tf;t+=(tf-t0)/1000){
    std::valarray<double> x = solution->at(t);
    std::valarray<double> y = solution->at(t+(tf-t0)/1000);
    EXPECT_NEAR(glc::norm2(x-y),(tf-t0)/1000,0.001);
  }
//   glc::trajectoryToFile("nonholonomic_path_demo.txt","../examples/",solution,500);
  
}

/**
 * This test runs a minimum time pendulum swingup
 * problem and checks that a solution is found and
 * that the solution is continuous
 */
TEST(Planner,TestPendulumSolutionContinuity){
  //Motion planning algorithm parameters
  glc::Parameters alg_params;
  alg_params.res=15;
  alg_params.control_dim = 1;
  alg_params.state_dim = 2;
  alg_params.depth_scale = 100;
  alg_params.dt_max = 5.0;
  alg_params.max_iter = 50000;
  alg_params.time_scale = 20;
  alg_params.partition_scale = 40;
  alg_params.x0 = std::valarray<double>({0.0,0.0});  


  //Create a dynamic model
  InvertedPendulum dynamic_model(alg_params.dt_max);
  
  //Create the control inputs
  PendulumTorque controls(alg_params.res);
  
  //Create the cost function
  MinTime performance_objective(4);
  
  //Create instance of goal region
  std::valarray<double> xg({M_PI,0.0});
  SphericalGoal goal(xg.size(),1.0,4);
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
  EXPECT_TRUE(out.solution_found);
  
  int count=0;
  std::vector<std::shared_ptr<const glc::Node>> path = planner.pathToRoot(true);
  for(std::shared_ptr<const glc::Node>& n : path){
    if(count==0){continue;}
    
    double t0 = n->trajectory_from_parent->initialTime();
    double dt = n->trajectory_from_parent->intervalLength()*double(n->trajectory_from_parent->numberOfIntervals());
    double tf = t0+dt;
    auto xf = n->trajectory_from_parent->at(tf);
    count++;
  }
  std::shared_ptr<glc::InterpolatingPolynomial> solution = planner.recoverTraj( path );
  double t0 = solution->initialTime();
  double tf = solution->initialTime()+double(solution->numberOfIntervals())*solution->intervalLength();
  
  for(double t=t0;t<tf;t+=(tf-t0)/100){
    std::valarray<double> x = solution->at(t);
    std::valarray<double> y = solution->at(t+(tf-t0)/100);
    EXPECT_TRUE(glc::norm2(x-y)<=3.0*(tf-t0)/100);
//     std::cout << glc::norm2(x-y) << std::endl;
  }
  glc::trajectoryToFile("pendulum_swingup_demo.txt","../examples/",solution,100);
  
}

int main(int argc, char **argv) {

  ::testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
  
}