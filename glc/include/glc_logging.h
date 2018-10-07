/* Copyright (C) Brian Paden (bapaden@mit.edu)
 * Written by Brian Paden
 * Released under the GNU General Public License v3
 */

#ifndef GLC_LOGGING_H
#define GLC_LOGGING_H

#include <fstream>
#include <set>

#include <glc_state_equivalence_class.h>
#include <glc_interpolation.h>

namespace glc{
  
/**
 * \brief Logs the states labeling each equivalence class to a nodesToFile
 * \param[in] name is the desired filename
 * \param[in] path is the desired location for the file to be saved
 * \param[in] domains is the set of labeled equivalence classes from a run of GLC
 */  
void nodesToFile(const std::string& name, const std::string& path, const std::set<StateEquivalenceClass>& domains);


/**
 * \brief Logs a finely sampled set of points along a trajectory to a file
 * \param[in] name is the desired filename
 * \param[in] path is the desired location for the file to be saved 
 * \param[in] traj an interpolating spline object that is to be logged
 * \param[in] num_points is the number of points sampled uniformly along traj
 */
void trajectoryToFile(const std::string& name, 
                      const std::string& path, 
                      const std::shared_ptr<InterpolatingPolynomial> traj, 
                      int num_points);

}//namespace atg
#endif