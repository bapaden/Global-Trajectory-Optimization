/* Copyright (C) Brian Paden (bapaden@mit.edu)
 * Written by Brian Paden
 * Released under the GNU General Public License v3
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
  root_ptr = std::shared_ptr<Node>(new Node(_controls.size(),0, 0,_h->costToGo(params.x0),params.x0,0,nullptr,nullptr,nullptr));
  best = std::shared_ptr<Node>(new Node(0, -1, std::numeric_limits<double>::max(), std::numeric_limits<double>::max(),std::valarray<double>(0),0,nullptr,nullptr,nullptr));
  ////////////*Scaling functions*//////////////
  // 1/R
  expand_time=params.time_scale/(double)params.res;
  //h(R)
  depth_limit=params.depth_scale*params.res*floor(log(params.res));
  //eta(R) \in \little-omega (log(R)*R^L_f)
  if(dynamics->getLipschitzConstant()==0.0){
    eta = params.res*log(params.res)*log(params.res)/params.partition_scale;
  }
  else{
    eta = pow(params.res,1+dynamics->getLipschitzConstant())/params.partition_scale;
  }
  
  StateEquivalenceClass d0;
  d0.label = root_ptr;
  d0.coordinate = vecFloor(eta*root_ptr->state);
  queue.push(root_ptr);
  domain_labels.insert(d0);
  
  /////////*Print Parameters*/////////////
  std::cout << "\n\n\n\nPre-search summary:\n" << std::endl;
  std::cout << "      Expand time: " << expand_time << std::endl;
  std::cout << "      Depth limit: " << depth_limit <<  std::endl;
  std::cout << "   Partition size: " << 1.0/eta << std::endl;
  std::cout << "   Max iterations: " << params.max_iter << std::endl;
  
  tstart = clock();
}
  
void Planner::addChild(std::shared_ptr<Node> parent, std::shared_ptr<Node> child){
    child->parent = parent;
    child->depth = parent->depth+1;
    child->time = (parent->time+expand_time);
    parent->children[child->u_idx] = child;
}
                   
void Planner::expand(){
  iter++;
  if(queue.empty()){
    std::cout << "---The queue is empty. Resolution too low or no solution.---" << std::endl;
    live=false;//TODO return this instead
    return;
  }
  std::shared_ptr<Node> current_node = queue.top();
  queue.pop();
  
  //Goal checking
  if(current_node->in_goal and current_node->cost < best->cost){
    run_time = clock() - tstart;  
    best=current_node;
    UPPER=current_node->cost;
    found_goal=true;
    live=false;//only for best-first search
    std::cout << "\n\nFound goal at iter: " << iter << std::endl;
    std::cout << "     solution cost: " << UPPER << std::endl;
    std::cout << "      running time: " << (float) run_time/ (float) CLOCKS_PER_SEC << std::endl;
    std::cout << "  Simulation count: " << dynamics->sim_counter << std::endl;
    std::cout << "  Collision checks: " << obs->collision_counter << std::endl;
    std::cout << "       Size of set: " << domain_labels.size() << std::endl;
    std::cout << "     Size of queue: " << queue.size() << std::endl;
  }
  
  if(current_node->depth >=depth_limit or iter>params.max_iter)
  {
    std::cout << "---exceeded depth or iteration limit---" << std::endl;
    live=false;
    return;
  }
  
  //A set of domains visited by new nodes made by expand
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
    std::shared_ptr<Node> new_arc(new Node(controls.size(),
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
    std::valarray<double> w = eta * new_arc->state;
    StateEquivalenceClass d_new;
    d_new.coordinate = vecFloor( w );
    //Get the domain for the coordinate or create it and insert into labels.
    StateEquivalenceClass& bucket = const_cast<StateEquivalenceClass&>( *(domain_labels.insert(d_new).first) );
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
        const std::shared_ptr<Node>& best_relabel_candidate = current_domain.candidates.top(); 
        std::shared_ptr<InterpolatingPolynomial> candidate_traj = best_relabel_candidate->trajectory_from_parent;//traj_from_parent[best_relabel_candidate];
        if(obs->collisionFree(candidate_traj)){
          addChild(current_node, best_relabel_candidate);
          //Flag vertex if it's in the goal
          double time;
          if( goal->inGoal(candidate_traj,time)){
            best_relabel_candidate->in_goal=true;
            best_relabel_candidate->cost = best_relabel_candidate->parent->cost + cf->cost(candidate_traj,
                                                                                           best_relabel_candidate->control_from_parent,
                                                                                           candidate_traj->initialTime(),
                                                                                           time);
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
      domain_labels.erase(current_domain);
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
  out.cost=UPPER;
  out.time=(float) run_time/ (float) CLOCKS_PER_SEC; 
  out.solution_found=found_goal;//TODO change to found_goal
  return;
}

//get the std::shared_ptr<Node> path to the root with the order specified by foward
std::vector<std::shared_ptr<Node>> Planner::pathToRoot(bool forward){
  if(found_goal==false){//this function doesn't work if the planner doesn't have a solution
    std::vector<std::shared_ptr<Node>> empty_vector;
    return empty_vector;
  }
  std::shared_ptr<Node> currentNode = best;
  std::vector<std::shared_ptr<Node>> path;
  while( not (currentNode->parent == nullptr) ){
    path.push_back(currentNode);
    currentNode=currentNode->parent;
  }
  path.push_back(currentNode);
  
  if(forward){std::reverse(path.begin(),path.end());}
  
  return path;
}
 
 //return the planned trajectory
 std::shared_ptr<InterpolatingPolynomial> Planner::recoverTraj(const std::vector<std::shared_ptr<Node>>& path){
   std::shared_ptr<InterpolatingPolynomial> opt_sol=nullptr;
   if(path.size()<2){return opt_sol;}
   //recalculate arcs connecting nodes
   for(int i=0; i<path.size()-1;i++){
     //The interval for the next polynomial segment is [t0,tf]
     double t0=path[i]->time;
     double tf=t0+expand_time; 
     //A piecewise linear segment of control based on collocation points stored in std::shared_ptr<Node> path[i]
     std::valarray<double> c0;
     if(i==0){c0 = controls[path[i+1]->u_idx];}//special case for root vertex - uses control of child
     else{c0 = controls[path[i]->u_idx];}
     std::valarray<double> c1 = (controls[path[i+1]->u_idx]-c0)/expand_time;
     std::vector<std::valarray<double> > segment({c0,c1});
     std::vector< std::vector< std::valarray<double> > > linear_interp({segment});
     std::shared_ptr<InterpolatingPolynomial> control_segment(new InterpolatingPolynomial(linear_interp,expand_time,t0,controls[path[i]->u_idx].size(),2));
     //Simulate dynamics with input control_segment
     std::shared_ptr<InterpolatingPolynomial> traj_segment;
     dynamics->sim(traj_segment, t0, tf, path[i]->state,control_segment);
     if(i==0){
       opt_sol=std::shared_ptr<InterpolatingPolynomial>(new InterpolatingPolynomial(*(traj_segment.get())));//
     }
     else{
       opt_sol->concatenate(traj_segment);
     }
   }
   
   return opt_sol;
 } 
 
}//namespace glc
