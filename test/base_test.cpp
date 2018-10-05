/* Copyright (C) Brian Paden (bapaden@mit.edu) - All Rights Reserved
 * Written by Brian Paden
 * Released under the GNU General Public License v3
 */

#include <gtest/gtest.h>
#include "sample_interface.h"


TEST(Base,Test){
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

int main(int argc, char **argv) {

  ::testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
  
}