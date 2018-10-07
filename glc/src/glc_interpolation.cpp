/* Copyright (C) Brian Paden (bapaden@mit.edu)
 * Written by Brian Paden
 * Released under the GNU General Public License v3
 */

#include <glc_interpolation.h>

namespace glc{
  
  InterpolatingPolynomial::InterpolatingPolynomial(const std::vector< std::vector< std::valarray<double> > >& _coeff_array, 
                                                   const double& _collocation_interval, 
                                                   const double& _t0, 
                                                   const double& _dimension, 
                                                   const double& _degree) : 
                                                   collocation_interval(_collocation_interval),
                                                   t0(_t0),
                                                   dimension(_dimension), 
                                                   degree(_degree),
                                                   coefficient_array(_coeff_array){
                                                     
  }
 
  InterpolatingPolynomial::InterpolatingPolynomial(const double& _collocation_interval, 
                                                   const double& _t0, 
                                                   const double& _dimension, 
                                                   const double& _degree) : 
                                                   collocation_interval(_collocation_interval),
                                                   t0(_t0),
                                                   dimension(_dimension), 
                                                   degree(_degree){
    coefficient_array = std::vector< std::vector< std::valarray<double> > >();
  }

                                                   
                                                   
  void InterpolatingPolynomial::concatenate(const std::shared_ptr<InterpolatingPolynomial>& tail){
    assert(tail->dimension==dimension);
    assert(tail!=nullptr);
    assert(tail->degree==degree);
    assert(tail->collocation_interval == collocation_interval);
    coefficient_array.insert(coefficient_array.end(),
                             tail->coefficient_array.begin(),
                             tail->coefficient_array.end());
  }
  
  void InterpolatingPolynomial::push(const std::vector< std::valarray<double> >& knot){
    coefficient_array.push_back(knot);
  }
  
  std::valarray<double> InterpolatingPolynomial::at(const double& t){
    int index = std::min( (int)coefficient_array.size()-1,std::max(0,(int)std::floor((t-t0)/collocation_interval)));
    double time = (t-t0)-collocation_interval*((double)index);
    std::valarray<double> eval(0.0,dimension);
    for(int i=0;i<degree;i++){
      eval+=coefficient_array[index][i]*pow(time,i);
    }
    return eval;
  }
  
  void InterpolatingPolynomial::reserve(const int& size){
    assert(size>=0 && "Cannot reserve negative space for InterpolatingPolynomial coefficients");
    coefficient_array.reserve(size);
  }
  
  int InterpolatingPolynomial::numberOfIntervals(){return coefficient_array.size();}
  
  double InterpolatingPolynomial::intervalLength(){return collocation_interval;}
  
  double InterpolatingPolynomial::initialTime(){return t0;}
  
  void InterpolatingPolynomial::printSpline(int num_points, const std::string& msg){
    std::cout << std::endl << "*****"<< msg << "*****" << std::endl;
    double t=initialTime();
    double dt=(intervalLength()*numberOfIntervals())/(double)num_points;
    for(int i=0;i<num_points+1;i++){
      std::valarray<double> x = (at(t));
      std::cout << "(";
      for(std::size_t i=0;i<x.size()-1;i++){
        std::cout << x[i] << ",";
      }
      std::cout << x[x.size()-1] << ")" << std::endl;
      t+=dt;
    }
  }
  
}