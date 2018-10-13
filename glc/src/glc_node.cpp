/* Copyright (C) Brian Paden (bapaden@mit.edu)
 * Written by Brian Paden
 * Released under the MIT License
 */

#include <glc_node.h>

namespace glc{

  Node::Node(int _card_omega, 
       int _control_index, 
       double _cost, 
       double _cost_to_go, 
       const std::valarray<double>& _state,  
       double _time,
       const std::shared_ptr<const Node> _parent,
       const std::shared_ptr<InterpolatingPolynomial>& _trajectory_from_parent,
       const std::shared_ptr<InterpolatingPolynomial>& _control_from_parent
            ): 
//        children(_card_omega), 
       cost(_cost), 
       merit(_cost_to_go+_cost), 
       time(_time), 
       parent(_parent), 
       state(_state), 
       u_idx(_control_index),
       control_from_parent(_control_from_parent),
       trajectory_from_parent(_trajectory_from_parent){
         if(parent != nullptr){
         assert(parent->state[0]==trajectory_from_parent->at(parent->time)[0]);
         assert(parent->state[1]==trajectory_from_parent->at(parent->time)[1]);
         }
  }
  
  bool NodeMeritOrder::operator()(const std::shared_ptr<const Node>& node1, const std::shared_ptr<const Node>& node2){
    return node1->merit>node2->merit;//negation so top of queue is min not max     
  }
  
}//namespace glc