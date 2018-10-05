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
 * The cost function must be globally Lipschitz continuous on the feasible reasion of the state space
 * and the feasible set of control inputs. A Lipschitz constant must be provided and the closer it is
 * to the smallest Lipschitz constant for the cost function, the better the performance will be.
 */
class CostFunction{
protected:
  const double lipschitz_constant;
public:
  /**
   * \brief The constructor sets the Lipschitz constant
   */
  CostFunction(double lipschitz_constant_);
  
  virtual double cost(const std::shared_ptr<InterpolatingPolynomial>& trajectory, 
                      const std::shared_ptr<InterpolatingPolynomial>& control, 
                      double t0, 
                      double tf) const = 0;
  
  double getLipschitzConstant();
};

class GoalRegion{
public:
  virtual bool inGoal(const std::shared_ptr<InterpolatingPolynomial>& traj, double& time)=0;
};

class Obstacles{
public:
  int collision_counter=0;
  virtual bool collisionFree(const std::shared_ptr<InterpolatingPolynomial>& x)=0; 
};

class DynamicalSystem{
protected:
  const double max_time_step;
public:
  int sim_counter=0;
  
  DynamicalSystem(double _max_time_step);
  
  //This is the function defining the dynamic model x'(t)=f(x(t),u(t))
  virtual void flow(std::valarray<double>& dx, const std::valarray<double>& x,const std::valarray<double>& u)=0;
  
  //Planner requires access to a Lipschitz constant on the vector field
  virtual double getLipschitzConstant()=0;
  
  //Assigns segment to a polynomial connecting x(t1)=x0 to x(t2) 
  virtual void step(std::shared_ptr<InterpolatingPolynomial>& segment, const std::valarray<double>& x0, const std::shared_ptr<InterpolatingPolynomial>& u, const double& t1, const double& t2)=0;
  
  //Calls step repeatedly to create a spline approximating the solution to an ODE 
  void sim(std::shared_ptr<InterpolatingPolynomial>& solution, double t0, double tf, const std::valarray<double>& x0, const std::shared_ptr<InterpolatingPolynomial>& u);
};
}//namespace glc

#endif