/* Copyright (C) Brian Paden (bapaden@mit.edu)
 * Written by Brian Paden
 * Released under the GNU General Public License v3
 */

#include<glc_state_equivalence_class.h>
#include<gtest/gtest.h>

using namespace glc;

/**
 * This checks that the elements of the partition of the state space are ordered 
 * lexicographically by their multi-dimensional index.
 */
TEST(GlcEquivalenceRelation,Order){
  
  StateEquivalenceClass d0;
  StateEquivalenceClass d1;
  StateEquivalenceClass d2;
  
  d0.coordinate = std::vector<int>({1,2,3});
  d1.coordinate = std::vector<int>({1,3,3});
  d2.coordinate = std::vector<int>({1,2,3});
  
  EXPECT_TRUE(d0<d1);
  EXPECT_FALSE(d1<d0);
  
  EXPECT_TRUE(d2<d1);
  EXPECT_FALSE(d1<d2);
  
  EXPECT_FALSE(d0<d2);
  EXPECT_FALSE(d2<d0);
}

int main(int argc, char** argv){
  
  ::testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
  
}