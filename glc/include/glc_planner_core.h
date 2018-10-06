/* Copyright (C) Brian Paden (bapaden@mit.edu)
 * Written by Brian Paden
 * Released under the GNU General Public License v3
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
#include <glc_interface.h>
#include <glc_interpolation.h>
#include <glc_numerical_integration.h>
#include <glc_state_equivalence_class.h>
#include <glc_node.h>
#include <glc_parameters.h>
#include <glc_logging.h>
#include <glc_math.h>

namespace glc{  
  
  struct PlannerOutput{
    double cost;
    double time;
    bool solution_found;
  };
  
  class Planner{
    
  public:
    //! \brief A leaf node in the goal region with minimum cost from the root
    std::shared_ptr<Node> best;
    //! \brief The root of the search tree
    std::shared_ptr<Node> root_ptr;
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
    std::priority_queue<std::shared_ptr<Node>,std::vector<std::shared_ptr<Node>>,NodeMeritOrder> queue;
    std::set<StateEquivalenceClass> domain_labels;
    std::set<StateEquivalenceClass>::iterator it;
    int depth_limit;
    double partition_scale;
    double eta;
    double expand_time;
    bool found_goal=false; 
    bool live=true;
    Parameters params;
    std::vector<std::valarray<double>> controls;
    int sim_count = 0;
    int coll_check = 0;
    int iter=0;
    clock_t run_time, tstart;
    double UPPER=std::numeric_limits<double>::max();
    
    Planner(Obstacles* _obs, 
            GoalRegion* _goal, 
            DynamicalSystem* _dynamics, 
            Heuristic* _h, 
            CostFunction* _cf, 
            const Parameters& _params, 
            const std::vector<std::valarray<double>>& _controls);

    //Planner tree handling functions
    void addChild(std::shared_ptr<Node> parent, std::shared_ptr<Node> child);//TODO move to node class
    std::vector<std::shared_ptr<Node>> pathToRoot(bool forward=false);
    std::shared_ptr<InterpolatingPolynomial> recoverTraj(const std::vector<std::shared_ptr<Node>>& path);
    
    void expand();
    void plan();
    void plan(PlannerOutput& out);
    bool getSolution(std::shared_ptr<InterpolatingPolynomial>& traj_out);
    
  };

}//glc namespace

#endif