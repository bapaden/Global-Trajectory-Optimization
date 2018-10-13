/* Copyright (C) Brian Paden (bapaden@mit.edu)
 * Written by Brian Paden
 * Released under the MIT License
 */

#ifndef GLC_PLANNER_CORE_H
#define GLC_PLANNER_CORE_H

//Used for timing planning queries
#include <unistd.h>

//STL containers used for planning queries
#include <stack>
#include <queue>
#include <vector>
#include <map>

//internal linking to dependent libs
#include "glc_interface.h"
#include "glc_interpolation.h"
#include "glc_numerical_integration.h"
#include "glc_state_equivalence_class.h"
#include "glc_node.h"
#include "glc_parameters.h"
#include "glc_logging.h"
#include "glc_math.h"

namespace glc{  
    
  //! \brief Contains some of the output data
  struct PlannerOutput{
    double cost;
    double time;
    bool solution_found;
  };
  
  /**
   * \brief The core planning class for carring out trajectory optimization
   * 
   * The algorithms is simply an A* search. The innovation is in the 
   * construction of a sparse, and nearly optimal discrete abstraction
   * of the set of feasible trajectories for a problem instance. 
   */
  class Planner{
    //! \brief A leaf node in the goal region with minimum cost from the root
    std::shared_ptr<const Node> best;
    //! \brief The root of the search tree
    std::shared_ptr<const Node> root_ptr;
    //! \brief A raw pointer to the dynamical system for the problem
    DynamicalSystem* dynamics;
    //! \brief A raw pointer to the goal region for the problem
    GoalRegion* goal;
    //! \brief A raw pointer to the (topologically)closed obstacle set for the problem
    Obstacles* obs;
    //! \brief A raw pointer to the cost function for the problem
    CostFunction* cf;
    //! \brief A raw pointer to the heuristic for the problem
    Heuristic* h;
    //! \brief A comparator for measureing relative merit of nodes
    NodeMeritOrder compare;
    /**
     * \brief A priority queue of leaf nodes that are open for expansion
     * 
     * The queue is ordered by the cost plus the estimated cost to go 
     * determined by the admissible and consistent heuristic.
     */ 
    std::priority_queue<std::shared_ptr<const Node>,std::vector<std::shared_ptr<const Node>>,NodeMeritOrder> queue;
    
    /**
     * \brief A constant factor multiplying the depth limit of the search tree
     * 
     * In order to guarantee finite time termination, the algorithm is limited
     * to a finite search depth for a fixed search resolution. This limit is 
     * increased with increasing resolution at a carefully controlled rate. 
     * The user is free to tune a constant factor.
     */
    int depth_limit;
    /**
     * \brief A constant factor dilation of the equivalence class size
     *
     * The partition size is carefully controlled by the resolution 
     * parameter. It will decrease as the resolution increases, and 
     * with larger Lipschitz coefficients in the differential 
     * constraint the faster the partition must shrink to guaranteed
     * convergence. partition_scale is a constant factor multiplying
     * this function of the Lipschitz constant. 
     */
    double partition_scale;
    
    /**
     * \brief The side length of the cubicle's forming the partition of the state space
     * 
     * This value is a function of resolution determined
     * in the constructor. In the paper this value is eta(R).
     */
    double inverse_cubicle_side_length;
    /**
     * \brief When a node is expanded, each this is the duration of the trajectories to the child nodes
     */
    double expand_time;
    /**
     * \brief This is used as a flag to stop expanding nodes and clear the priority queue to find the best solution
     */
    bool found_goal=false; 
    /**
     * \brief This is a flag to stop the algorithm if some limit has been reached
     */
    bool live=true;
    /**
     * \brief a copy of the struct containing planner parameters
     */
    Parameters params;
    /**
     * \brief An array containing the discrete set of control inputs
     * 
     * When a node is expanded each of these control inputs
     * is applied to the system dynamics from the state associated
     * with the node being expanded for a duration of 
     * expand_time.
     */
    std::vector<std::valarray<double>> controls;
    //! \brief A counter for the number of calls to the sim method
    int sim_count = 0;
    //! \brief A counter for the number of calls to collisionFree
    int coll_check = 0;
    //! \brief A counter for the number of calls to the expand method
    int iter=0;
    //! \brief A counter for the number of clock cycles used in a query
    clock_t run_time, tstart;
  public:
    /**
     * \brief An ordered set of equivalence classes that have been reached by a trajectory from the initial state
     * 
     * The labels on each equivalence class is the node
     * with best known merit in that equivalence class.
     */
    std::set<StateEquivalenceClass> partition_labels;
    std::set<StateEquivalenceClass>::iterator it;
    
    /** 
     * \brief The constructor assigns its parameters to the associated member attributes
     * 
     * The essential scaling functions of the method are evaluated at the given
     * resolution within the constructor.
     */
    Planner(Obstacles* _obs, 
            GoalRegion* _goal, 
            DynamicalSystem* _dynamics, 
            Heuristic* _h, 
            CostFunction* _cf, 
            const Parameters& _params, 
            const std::vector<std::valarray<double>>& _controls);

    /**
     * \brief Assigns child as a child node of parent
     * \param[in] child is a node that will be pointed to as a child node of parent
     * \param[out] parent is a node modified to have child as a child node
     */
//     void addChild(std::shared_ptr<const Node> parent, std::shared_ptr<const Node> child);
    /**
     * \brief This method returns the sequence of nodes from the lowest cost node in the goal back to the root via edge relations stored in each node
     * \param[in] forward is a flag to indicate if the nodes should be ordered from root to leaf (foward=true) or from leaf to root (forward=false)
     */
    std::vector<std::shared_ptr<const Node>> pathToRoot(bool forward=false);
    /**
     * \brief Constructs the trajectory from the root to the leaf node in the goal
     * \param[in] path is the sequence of nodes from root to leaf connected by edge relations
     * \returns a pointer to a trajectory object representing the solution trajectory
     */
    std::shared_ptr<InterpolatingPolynomial> recoverTraj(const std::vector<std::shared_ptr<const Node>>& path);
    
    /**
     * \brief The core iteration of the algorithm that pops the top of the queue and forward integrates the dynamics with each of the controls
     */
    void expand();
    /**
     * \brief Method that calls expand until one of several flags is set for the algorithm to stop
     */
    void plan();
    /**
     * \brief Overload of plan() which provides some output on the result
     * \param[out] out is a struct containing info on whether a solution was found and what its cost is
     */
    void plan(PlannerOutput& out);
    /**
     * \brief
     */
    bool getSolution(std::shared_ptr<InterpolatingPolynomial>& traj_out);
    
  };

}//glc namespace

#endif