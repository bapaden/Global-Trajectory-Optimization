/* Copyright (C) Brian Paden (bapaden@mit.edu)
 * Written by Brian Paden
 * Released under the GNU General Public License v3
 */

#include <glc_math.h>

namespace glc{

double sqr(const double x){
  return x*x;
}

std::vector<int> vecFloor(const std::valarray<double>& x){
  std::vector<int> floored(x.size());
  for(int i=0;i<x.size();i++){
    floored.at(i)=(int)floor(x[i]);
  }
  return floored;
}

double dot(const std::valarray<double>& x,const std::valarray<double>& y){
  assert(x.size()==y.size());
  double z=0;
  for(int i=0;i<y.size();i++){
    z += x[i]*y[i];
  }
  return z;
}

double norm2(const std::valarray<double>& x){
  double norm=0;
  for(int i=0; i<x.size(); i++){
    norm+=sqr(x[i]);
  }
  return std::sqrt(norm);
}

double normSqr(const std::valarray<double>& x){
  double norm=0;
  for(int i=0; i<x.size(); i++){
    norm+=sqr(x[i]);
  }
  return norm;
}

std::valarray<double> linearSpace(const double& start, const double& end, const int points){
  assert(end > start && "[ERROR] in linearSpace -- end is less than start");
  std::valarray<double> lin_space(points);
  double step = (end-start)/double(points);
  lin_space[0]=start;
  for(int i=1;i<points;i++){lin_space[i]=lin_space[i-1]+step;}
  return lin_space;
}

}//namespace glc