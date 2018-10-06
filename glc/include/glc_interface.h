/* Copyright (C) Brian Paden (bapaden@mit.edu) - All Rights Reserved
 * Written by Brian Paden
 * Released under the GNU General Public License v3
 */

#ifndef GLC_INTERFACE_H
#define GLC_INTERFACE_H

//External dependencies
#include<vector>

//Local libraries
#include <glc_interpolation.h>

namespace glc{

/** 
 * \brief Base class which defines a finite set of control inputs from the input space that are used by the planner to forward simulate the system dynamics
 * 
 * Mathematically, the GLC method produces an out put converting to a globally 
 * optimal solution for a particular problem instance as the resolution of the 
 * algorithm is increased. The user is responsible for implementing a derived 
 * class for Inputs which is parameterized by the algorithm resolution such 
 * that with increasing resolution, the dispersion of the discrete set of 
 * control inputs within the set of admissible control inputs converges to 
 * zero. 
 */  
class Inputs{
  //! \brief points stores the finite set of control inputs 
  std::vector<std::valarray<double>> points;
public:
  /**
   * \brief inserts a control input to the set of control inputs
   * 
   * \param[in] input_ is the control input added to the set of inputs
   * 
   * Two simple ways of generating a set of control inputs meeting the requirements
   * of the algorithm are to sample randomly from the set of admissible control 
   * inputs with the number of samples increasing with resolution or to intersect a
   * uniform grid with the set of admissible control inputs and refine the grid 
   * with increasing algorithm resolution.
   */
  void addInputSample(std::valarray<double>& input_);
  /**
   * \brief returns a read-only reference to the set of control inputs
   */
  const std::vector<std::valarray<double>>& readInputs() const ;
};

/** 
 * \brief Base class for an admissible and consistent estimate of the cost-to-go from a particular state in the state space
 * 
 * The heuristic is computed at the state associated to nodes in the search tree
 * with the ordering of nodes in a prority queue determined by the cost to reach 
 * that node from the root of the tree together with the estimated cost-to-go. 
 */
class Heuristic{
public: 
  /**
   * \brief Base class for evaluates of the under-estimates the cost to go from a particular state
   * 
   * \param[in] x0_ is the state from which the cos-to-go is estimated
   * 
   * The the estimated cost-to-go should be as close as possible to the actual
   * cost-to-go without over-approximating it anywhere in the state space. 
   * Nominally, the heuristic is computationally cheap to evaluate.
   */
  virtual double costToGo(const std::valarray<double>& x0_) const = 0;
}; 

/** 
 * \brief This class defines the cost of a control signal together with the associated trajectory for a particular problem instance
 *
 * The running cost of the cost functional must be globally Lipschitz continuous on the feasible 
 * reason of the state space and the feasible set of control inputs. A Lipschitz constant must 
 * be provided and the closer it is to the smallest Lipschitz constant for the cost function, the 
 * better the performance will be.
 */
class CostFunction{
protected:
  const double lipschitz_constant;
public:
  /**
   * \brief The constructor sets the Lipschitz constant for the running cost g in c(x,u) = integral_t_0^tf g(x(t),u(t)) dt 
   */
  CostFunction(double lipschitz_constant_);
  
  /**
   * \brief The user must implement a running cost that integrates the cost along a trajectory with a given input
   */
  virtual double cost(const std::shared_ptr<InterpolatingPolynomial>& trajectory_, 
                      const std::shared_ptr<InterpolatingPolynomial>& control_, 
                      double t0_, 
                      double tf_) const = 0;
  
  /**
   * \brief This method returns the Lipschitz constant of the cost function 
   * \returns The Lipschitz constant of the running cost in the cost function
   */
  double getLipschitzConstant();
};

/**
 * \brief The user must define a goal region for the problem instance which informs the algorithm whether or not a trajecoty intersects the goal
 */
class GoalRegion{
public:
  /**
   * \brief The user must implement the inGoal method which answers whether traj_ intersects the goal and if so, the earliest time at which this happens
   * \param[in] traj_ is the state trajectory that will be checked for intersection with the goal region
   * \param[out] intersection_time_ is the the earliest instant at which the trajectory is in the goal region
   * \returns The method returns true if traj_ intersects the goal and false otherwise
   */
  virtual bool inGoal(const std::shared_ptr<InterpolatingPolynomial>& traj_, double& intersection_time_)=0;
};

/**
 * \brief The user must define the infeasible space for the problem instance to inform the algorithm whether or not a trajectory is feasible
 */
class Obstacles{
public:
  //! \brief collision_counter monitors the number of times the collisionFree method is called in a planning query
  int collision_counter=0;
  /**
   * \brief The user must implement the collisionFree method for their problem instance
   * \param[in] traj_ is the state trajectory that will be checked for intersection with the infeasible region for a particular problem
   * \returns The method returns true of the trajectory remains in the feasible region (i.e. it is collision free) and false otherwise
   */
  virtual bool collisionFree(const std::shared_ptr<InterpolatingPolynomial>& traj_)=0; 
};

/**
 * \brief The user must define a dynamical system whose differential constraints must be satisfied by a solution to a particular problem
 */
class DynamicalSystem{
public:
  //! \brief sim_counter keeps track of how many calls to the simulation method there are in a planning query
  int sim_counter=0;
  //! \brief lipschitz_constant is the lipschitz constant of the dynamics (in the state variable!!!)
  double lipschitz_constant;
  
  /**
   * \brief The base constructor sets the lipschitz_constant for the dynamic model
   * \param[in] lipschitz_constant_ is the value that lipchitz_constant is set to
   */
  DynamicalSystem(double lipschitz_constant_);
  
  /**
   * \brief This method represents the function defining the differential constraint x'(t)=f(x(t),u(t))
   * \param[in] x is a state of the dynamic system
   * \param[in] u is a control input to the dynamical system
   * \param[out] dx is the time derivative of the state at x given x and u
   */
  virtual void flow(std::valarray<double>& dx, const std::valarray<double>& x,const std::valarray<double>& u)=0;
  
  /**
   * \brief This method returns the Lipschitz constant for the dynamic system
   */
  virtual double getLipschitzConstant()=0;
  
  /**
   * \brief This method integrates the differential equation over a given time interval, with a given control signal, and with given initial state
   * \param[in] t0 is the initial time of the simulation
   * \param[in] tf is the final time of the simulation
   * \param[in] x0 is the initial state for the simulation
   * \param[in] u is the control signal for the simulation that must be defined over the interval [t0,tf]
   * \param[out] solution is the trajectory satisfying the differential equation for the input data
   *
   * If a closed form solution to the dynamics is available, it should be used here. The most general class of systems
   * where this is the case is linear systems. On the other hand, if numerical integration is required see the 
   * derived class RungeKuttaTwo which is provided with this library for derived class implementing a numerical 
   * integration scheme. 
   */
  virtual void sim(std::shared_ptr<InterpolatingPolynomial>& solution, double t0, double tf, const std::valarray<double>& x0, const std::shared_ptr<InterpolatingPolynomial>& u)=0;
};
}//namespace glc

#endif