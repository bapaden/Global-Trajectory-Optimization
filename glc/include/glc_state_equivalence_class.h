/* Copyright (C) Brian Paden (bapaden@mit.edu) - All Rights Reserved
 * Written by Brian Paden
 * Released under the GNU General Public License v3
 */

#ifndef GLC_STATE_EQUIVALENCE_CLASS
#define GLC_STATE_EQUIVALENCE_CLASS

#include <limits>

#include <glc_node.h>
#include <glc_math.h>

namespace glc{

struct StateEquivalenceClass{
  std::vector<int> coordinate;//serves as index of partition domain
  std::shared_ptr<Node> label;//pointer to node labeling region
  std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, NodeMeritOrder> candidates;//candidates for relabeling
  
  StateEquivalenceClass();
  StateEquivalenceClass(const std::shared_ptr<Node>& _label);
  bool empty();
  //Lexicographical order of integer tuple for sorting stl set of domains
  bool operator<(const StateEquivalenceClass& y)const;
};

}//namespace glc

#endif