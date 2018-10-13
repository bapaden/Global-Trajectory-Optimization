/* Copyright (C) Brian Paden (bapaden@mit.edu)
 * Written by Brian Paden
 * Released under the MIT License
 */

#ifndef GLC_NUMERICAL_INTEGRATION_H
#define GLC_NUMERICAL_INTEGRATION_H

#include "glc_interface.h"
#include "glc_interpolation.h"

namespace glc{

  /**
   * \brief This class implements the sim method of the base class Dynamical System with a numerical integration scheme
   * 
   * Note that the user must still provide a concrete dynamical model with flow implemented
   */  
  class RungeKuttaTwo : public DynamicalSystem{
  protected:
    //! \brief These are temporary variables used in the Runge-Kutta 2 integration method
    std::valarray<double> x1,x2,f0,f1,f2;
    //! \brief This is the maximum time step that sim is allowed to use
    double max_time_step;
    //! \brief This is a temporary variable used by the integration scheme
    double h;
  public:
    /**
     * \brief The constructor sets the member parameters of this and the base class
     */
    RungeKuttaTwo(double lipschitz_constant_, double max_time_step_, int state_dim_);
    
    /**
     * \brief This method numerically integrates the dynamics 
     * \param[in] t0 is the initial time for the simulation
     * \param[in] tf is the final time for the simulation
     * \param[in] x0 is the initial state for the simulation
     * \param[in] u is the control input defined over the interval [t0,tf]
     * \param[out] solution is the trajectory satisfying the dynamic equations
     */
    void sim(std::shared_ptr<InterpolatingPolynomial>& solution, double t0, double tf, const std::valarray<double>& x0, const std::shared_ptr<InterpolatingPolynomial>& u)override;
    
    /**
     * \brief This method implements one step of the Runge-Kutta 2 numerical integration method
     * \param[in] t1 is the initial time for the integration step
     * \param[in] t2 is the final time for the integration step
     * \param[in] x0 is the initial state for the integration step
     * \param[out] segment is a cubic approximation of the trajectory from t1 to t2 
     */ 
    void step(std::shared_ptr<InterpolatingPolynomial>& segment, const std::valarray<double>& x0, const std::shared_ptr<InterpolatingPolynomial>& u, const double& t1, const double& t2);
  };  
  
  
}//namespace
#endif