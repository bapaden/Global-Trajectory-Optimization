/* Copyright (C) Brian Paden (bapaden@mit.edu)
 * Written by Brian Paden
 * Released under the GNU General Public License v3
 */

#include <glc_state_equivalence_class.h>

namespace glc{

StateEquivalenceClass::StateEquivalenceClass():label(new Node(0, 
                                                              -1, 
                                                              std::numeric_limits<double>::max(), 
                                                              std::numeric_limits<double>::max(),
                                                              std::valarray<double>(0),
                                                              0,
                                                              nullptr,
                                                              nullptr,
                                                              nullptr
                                                             )){
}
                                                                
bool StateEquivalenceClass::empty(){
  return label->cost == std::numeric_limits<double>::max();
}

bool StateEquivalenceClass::operator<(const StateEquivalenceClass& y) const{
  assert(coordinate.size()==y.coordinate.size());
  return std::lexicographical_compare <std::vector<int>::const_iterator, std::vector<int>::const_iterator>
  (coordinate.begin(), coordinate.end(), y.coordinate.begin(), y.coordinate.end());
}

}