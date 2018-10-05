/* Copyright (C) Brian Paden (bapaden@mit.edu) - All Rights Reserved
 * Written by Brian Paden
 * Released under the GNU General Public License v3
 */

#ifndef GLC_INTERPOLATION_H
#define GLC_INTERPOLATION_H

//External dependencies
#include<vector>
#include<valarray>
#include<memory>
#include<cassert>
#include<iostream>

namespace glc{

class InterpolatingPolynomial{
  int degree;//coefficients in polynomial segments
  int dimension;//number of individual splines
  double collocation_interval;//time interval between collocation points
  double t0;//initial time
  //coefficient_array[time_interval_index][polynomial_coefficient_index][polynomial_coordinate_index]
  std::vector< std::vector< std::valarray<double> > > coefficient_array;
  
public: 
  InterpolatingPolynomial(const std::vector< std::vector< std::valarray<double> > >& _coeff_array, 
         const double& _collocation_interval, 
         const double& _t0, 
         const double& _dimension, 
         const double& _degree);
         
  InterpolatingPolynomial(const double& _collocation_interval, 
                const double& _t0, 
                const double& _dimension, 
                const double& _degree);
                
  //Copy tail into the back of this InterpolatingPolynomial. Ignores t0 of the tail segment 
  void concatenate(const std::shared_ptr<InterpolatingPolynomial>& tail);
  //Push a single collocation point into the spline
  void push(const std::vector< std::valarray<double> >& knot);
  
  //Evaluate spline at time t
  std::valarray<double> at(const double& t);
  
  // Allocates memory in coefficient_array for "size" collocation points
  void reserve(const int& size);
  
  int numberOfIntervals();
  
  double intervalLength();
  
  double initialTime();
  
  //Write trajectory out on the screen
  void printSpline(int num_points, const std::string& msg);
};

}//namespace glc
#endif