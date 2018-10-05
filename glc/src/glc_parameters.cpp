/* Copyright (C) Brian Paden (bapaden@mit.edu) - All Rights Reserved
 * Written by Brian Paden
 * Released under the GNU General Public License v3
 */

#include <glc_parameters.h>

namespace glc{
  
  void Parameters::printParams(){
    std::cout << "state_dim " << state_dim << std::endl;
    std::cout << "control_dim " << control_dim << std::endl;
    std::cout << "res " << res << std::endl;
    std::cout << "max_iter " << max_iter << std::endl;
    std::cout << "time_scale " << time_scale << std::endl;
    std::cout << "partition_scale " << partition_scale << std::endl;
    std::cout << "depth_scale " << depth_scale << std::endl;
    std::cout << "dt_max_scale " << partition_scale << std::endl;
    std::cout << "size of x0 " << x0.size() << std::endl;
    
    return;        
  }
}