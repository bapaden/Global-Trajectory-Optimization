#include<gtest/gtest.h>

TEST(test,simple){

	EXPECT_EQ(1.0,1.0);

}

int main(int argc, char** argv){

  ::testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();

return RUN_ALL_TESTS();
}
