#include<glc/glc_node.h>
#include<gtest/gtest.h>

using namespace glc;

TEST(GlcNode,Order){
  Node n(5,0,10.0,10.0,std::valarray<double>);
  Node(int _card_omega, 
       int _control_index, 
       double _cost, 
       double _cost_to_go, 
       const std::valarray<double>& _state, 
       double _time,
       const std::shared_ptr<Node> _parent
  );
}

int main(int argc, char** argv){
  
  ::testing::InitGoogleTest(&argc,argv);
  return RUN_ALL_TESTS();
  
}
