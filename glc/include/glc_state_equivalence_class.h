/* Copyright (C) Brian Paden (bapaden@mit.edu)
 * Written by Brian Paden
 * Released under the GNU General Public License v3
 */

#ifndef GLC_STATE_EQUIVALENCE_CLASS
#define GLC_STATE_EQUIVALENCE_CLASS

#include <limits>

#include <glc_node.h>
#include <glc_math.h>

namespace glc{

/**
 * \brief This class defines the equivalence classes (i.e. partition) of the state space
 * 
 * The GLC algorithm is able to build a sparse search tree by maintaining at most one node
 * per equivalence class. The size of the equivalence class is determined by the resolution
 * parameter in the current planning query.
 */
struct StateEquivalenceClass{
  
  //! \brief Each equivalence class (a hyper-cubicle region) is uniquely identified by an integer tuple
  std::vector<int> coordinate;
  //! \brief An equivalence class has a pointer to a node whose associated state is in the cubicle region
  std::shared_ptr<Node> label;
  /**
   * \brief A priority queue of potential new nodes that could label the cell
   * 
   * Once relabeled, the subtree rooted at the old label is deleted.
   */ 
  std::priority_queue<std::shared_ptr<Node>, std::vector<std::shared_ptr<Node>>, NodeMeritOrder> candidates;//candidates for relabeling
  
  //! \brief The constructor is the default constructor
  StateEquivalenceClass();
  //! \brief If no label has been set and the label attribute is nullptr, then empty will return true -- and false otherwise
  bool empty();
  //! \brief Lexicographical ordering is used to support std::map
  bool operator<(const StateEquivalenceClass& y)const;
};

}//namespace glc

#endif