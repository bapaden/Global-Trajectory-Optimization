/* Copyright (C) Brian Paden (bapaden@mit.edu)
 * Written by Brian Paden
 * Released under the MIT License
 */

#include <glc_planner_core.h>

namespace glc{
  
Planner::Planner(Obstacles* _obs, 
                 GoalRegion* _goal, 
                 DynamicalSystem* _dynamics, 
                 Heuristic* _h, 
                 CostFunction* _cf, 
                 const Parameters& _params, 
                 const std::vector<std::valarray<double>>& _controls):
                 params(_params), 
                 controls(_controls), 
                 dynamics(_dynamics), 
                 obs(_obs), 
                 goal(_goal), 
                 cf(_cf), 
                 h(_h){
  root_ptr = std::shared_ptr<const Node>(new Node(_controls.size(),0, 0,_h->costToGo(params.x0),params.x0,0,nullptr,nullptr,nullptr));
  best = std::shared_ptr<const Node>(new Node(0, -1, std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),std::valarray<double>(0),0,nullptr,nullptr,nullptr));
  ////////////*Scaling functions*//////////////
  // 1/R
  expand_time=params.time_scale/(double)params.res;
  //h(R)
  depth_limit=params.depth_scale*params.res*floor(log(params.res));
  //eta(R) \in \little-omega (log(R)*R^L_f)
  if(dynamics->getLipschitzConstant()==0.0){
    inverse_cubicle_side_length = params.res*log(params.res)*log(params.res)/params.partition_scale;
  }
  else{
    inverse_cubicle_side_length = pow(params.res,1+dynamics->getLipschitzConstant())/params.partition_scale;
  }
  
  StateEquivalenceClass d0;
  d0.label = root_ptr;
  d0.coordinate = vecFloor(inverse_cubicle_side_length*root_ptr->state);
  queue.push(root_ptr);
  partition_labels.insert(d0);
  
  //Print a summary of the algorithm parameters
  std::cout << "\n\n\n\nPre-search summary:\n" << std::endl;
  std::cout << "      Expand time: " << expand_time << std::endl;
  std::cout << "      Depth limit: " << depth_limit <<  std::endl;
  std::cout << "   Partition size: " << 1.0/inverse_cubicle_side_length << std::endl;
  std::cout << "   Max iterations: " << params.max_iter << std::endl;
  
  tstart = clock();
}
                   
void Planner::expand(){
  //Increment the iteration count
  iter++;
  //If the queue is empty then the problem is not feasible at the current resolution
  if(queue.empty()){
    std::cout << "---The queue is empty. Resolution too low or no solution at all.---" << std::endl;
    live=false;
    return;
  }
  
  //Pop the top of the queue for expansion
  std::shared_ptr<const Node> current_node = queue.top();
  queue.pop();
  
  //Once a goal node is found we clear the queue and keep the lowest cost node in the goal
  if(found_goal){
    double goal_time;
    if(goal->inGoal(current_node->trajectory_from_parent, goal_time)){
      double cost = current_node->parent->cost + 
                    cf->cost(current_node->trajectory_from_parent,
                             current_node->control_from_parent,
                             current_node->trajectory_from_parent->initialTime(),
                             goal_time);
      if(cost<best->cost){
        best = current_node;
        run_time = clock() - tstart;
        live=false;
        std::cout << "\n\nFound goal at iter: " << iter << std::endl;
        std::cout << "     solution cost: " << best->cost << std::endl;
        std::cout << "      running time: " << (float) run_time/ (float) CLOCKS_PER_SEC << std::endl;
        std::cout << "  Simulation count: " << dynamics->sim_counter << std::endl;
        std::cout << "  Collision checks: " << obs->collision_counter << std::endl;
        std::cout << "       Size of set: " << partition_labels.size() << std::endl;
        std::cout << "     Size of queue: " << queue.size() << std::endl;
      }
    }
  }
  
  //Stop the algorithm if the search tree reaches the depth or iteration limit
  if(current_node->depth >=depth_limit or iter>params.max_iter)
  {
    std::cout << "---exceeded depth or iteration limit---" << std::endl;
    live=false;
    return;
  }
  
  //A set of equivalence classes visited by new nodes made by expand
  std::set<StateEquivalenceClass*> domains_needing_update; 
  
  //Expand top of queue and store arcs in set of domains
  for(int i=0;i<controls.size();i++){
    std::shared_ptr<InterpolatingPolynomial> new_traj;
    std::valarray<double> c0;
    //Create a control signal spline which is a first order hold.
    //u(t)=c0+c1*t. If expanding root, just use u(t)=constant;
    if(current_node->parent==nullptr){
      c0 = controls[i];
    }
    else{
      c0 = controls[current_node->u_idx];
    }
    std::valarray<double> c1 = (controls[i]-c0)/expand_time;
    std::vector<std::valarray<double> > segment({c0,c1});
    std::vector< std::vector< std::valarray<double> > > linear_interp({segment});
    //The above parameters are used to construct new_control
    std::shared_ptr<InterpolatingPolynomial> new_control(new InterpolatingPolynomial(linear_interp,expand_time,current_node->time,controls[i].size(),2));
    //Forward simulate with new_control to get a cubic spline between collocation points
    dynamics->sim(new_traj, current_node->time, current_node->time+expand_time , current_node->state, new_control);
    std::shared_ptr<const Node> new_arc(new const Node(controls.size(),
                                           i,
                                           cf->cost(new_traj, new_control,current_node->time,current_node->time+expand_time)+current_node->cost, 
                                           h->costToGo(new_traj->at(current_node->time+expand_time)), 
                                           new_traj->at(current_node->time+expand_time), 
                                           current_node->time+expand_time,
                                           current_node,
                                           new_traj,
                                           new_control
                                          ));

    //Create a region for the new trajectory
    std::valarray<double> w = inverse_cubicle_side_length * new_arc->state;
    StateEquivalenceClass d_new;
    d_new.coordinate = vecFloor( w );
    //Get the domain for the coordinate or create it and insert into labels.
    StateEquivalenceClass& bucket = const_cast<StateEquivalenceClass&>( *(partition_labels.insert(d_new).first) );
    //Add to a queue of domains that need inspection
    domains_needing_update.insert(&bucket);
    
    if(not compare(new_arc,bucket.label)){
      bucket.candidates.push(new_arc);
    }     
  }
  //Go through the new trajectories and see if there is a possibility for relabeling before collcheck
  for(auto& open_domain : domains_needing_update){
    StateEquivalenceClass& current_domain = *open_domain;
    //We go through the queue of candidates for relabeling/pushing in each set
    bool found_best = false;
    while( (not found_best) and (not current_domain.candidates.empty()))
    {
      //If the top of the candidate queue is cheaper than the label we should coll check it
      if(not compare(current_domain.candidates.top(),current_domain.label)){
        std::shared_ptr<const Node> best_relabel_candidate = current_domain.candidates.top(); 
        std::shared_ptr<InterpolatingPolynomial> candidate_traj = best_relabel_candidate->trajectory_from_parent;//traj_from_parent[best_relabel_candidate];
        if(obs->collisionFree(candidate_traj)){
          //Flag vertex if it's in the goal
          double time;
          if( goal->inGoal(candidate_traj,time)){
            found_goal = true;
          }
          queue.push(best_relabel_candidate);//anything coll free at this point goes to queue
          if(!found_best){
            found_best = true;
            current_domain.label = best_relabel_candidate;
          }
        }
      }
      current_domain.candidates.pop();
    }
    if(current_domain.empty()){
      partition_labels.erase(current_domain);
    }
  }
  return;
}

void Planner::plan(){
  while(live){
    expand();
  }
  return;
}

void Planner::plan(PlannerOutput& out){
  Planner::plan();
  out.cost=best->cost;
  out.time=(float) run_time/ (float) CLOCKS_PER_SEC; 
  out.solution_found=found_goal;//TODO change to found_goal
  return;
}

//get the std::shared_ptr<Node> path to the root with the order specified by foward
std::vector<std::shared_ptr<const Node>> Planner::pathToRoot(bool forward){
  if(found_goal==false){
    std::vector<std::shared_ptr<const Node>> empty_vector;
    return empty_vector;
  }
  std::shared_ptr<const Node> currentNode = best;
  std::vector<std::shared_ptr<const Node>> path;
  while( not (currentNode->parent == nullptr) ){
    path.push_back(currentNode);
    currentNode=currentNode->parent;
  }
  path.push_back(currentNode);
  if(forward){std::reverse(path.begin(),path.end());}
  return path;
}
 
 //return the planned trajectory
 std::shared_ptr<InterpolatingPolynomial> Planner::recoverTraj(const std::vector<std::shared_ptr<const Node>>& path){
   if(path.size()<2){return nullptr;}
   std::shared_ptr<InterpolatingPolynomial> opt_sol=path[1]->trajectory_from_parent;
   for(int i=2;i<path.size();i++){
     opt_sol->concatenate(path[i]->trajectory_from_parent);
  }
   return opt_sol;
 } 
 
}//namespace glc
