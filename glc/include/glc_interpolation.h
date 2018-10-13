/* Copyright (C) Brian Paden (bapaden@mit.edu)
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

/**
 * \brief A class used to represent parametric curves in arbitrary dimensions
 * 
 * This is a very minimal class that does not enforce continuity or smoothness
 * of the curve it represents. This give maximum flexibility to the user. A 
 * series of polynomials are defined on evenly spaced time intervals using 
 * the standard monomial basis [1, t, t^2, t^3, ... ,t^(degree-1)]
 */
class InterpolatingPolynomial{
  /**
   * \brief the dimension of the polynomial vector space spanned by the basis
   * 
   * Node that degree is a slight misnomer. It represents the number of coefficients
   * required in each state dimension to define a polynomial. The highest power 
   * appearing in the basis functions is degree-1.
   */
  int degree;
  /**
   * \brief dimension is the dimension of the state space that the curve is defined in
   * 
   * There is a polynomial assigned to each dimension for a given curve
   */
  int dimension;
  /**
   * \brief This is the time interval for each polynomial segment
   * 
   * Each polynomial is defined from t=0 to t=collocation_interval.
   */
  double collocation_interval;
  /**
   * \brief This is the initial time the curve is defined at
   */
  double t0;
  /**
   * \brief The 3D array of coefficients for the muli-dimensional interpolating spline
   * 
   * The outermost index identifies the collocation interval. The middle index
   * identifies the polynomial basis monomial index. Tha innermost index identifies
   * the coordinate in the state space. In summary,
   * 
   * coefficient_array[time_interval_index][polynomial_coefficient_index][polynomial_coordinate_index]
   */
  std::vector< std::vector< std::valarray<double> > > coefficient_array;
  
public: 
  /**
   * \brief This constructor takes values to set all member attributes
   * \param[in] _coeff_array initializes coefficient_array
   * \param[in] _collocation_interval initializes _collocation_interval
   * \param[in] _t0 initialilzed t0
   * \param[in] _dimension initializes dimension
   * \param[in] _degree initializes degree
   */
  InterpolatingPolynomial(const std::vector< std::vector< std::valarray<double> > >& _coeff_array, 
         const double& _collocation_interval, 
         const double& _t0, 
         const double& _dimension, 
         const double& _degree);
  /**
   * \brief This constructor takes values to set member attributes with the exception of the coefficients which are left empty
   * \param[in] _collocation_interval initializes _collocation_interval
   * \param[in] _t0 initialilzed t0
   * \param[in] _dimension initializes dimension
   * \param[in] _degree initializes degree
   */         
  InterpolatingPolynomial(const double& _collocation_interval, 
                const double& _t0, 
                const double& _dimension, 
                const double& _degree);
                
  /**
   * \brief Extends this curve by copying the contents of tail to the back of this curve
   * 
   * The value of t0 in tail is ignored
   */
  void concatenate(const std::shared_ptr<InterpolatingPolynomial>& tail);
  /**
   * \brief appends a single polynomial segment to the back of this spline
   */ 
  void push(const std::vector< std::valarray<double> >& segment);
  
  /**
   * \brief Evaluate the curve at parameter value t
   * 
   * This method first identifies which interval I should
   * be evaluated. From there an offset equal to t-I*collocation_interval
   * is computed and this value is passed to the monomial basis on the 
   * inner array indices of coefficient_array.
   */
  std::valarray<double> at(const double& t);
  
  //! \brief Allocates memory in coefficient_array for "size" spline intervals
  void reserve(const int& size);
  
  //! \brief returns a copy of the number of intervals in this interpolating spline
  int numberOfIntervals();
  
  //! \brief returns the length of each interval -- each interval has equal length
  double intervalLength();
  
  //! \brief returns the initial time for the parametric curve
  double initialTime();
  
  /** 
   * \brief prints uniformly sampled points along the curve to the terminal
   * \param[in] num_points 
   * \param[in] msg is a message that will accompany the output to the terminal
   */
  void printSpline(int num_points, const std::string& msg);
  
  void printData(){
    for(int t_id=0;t_id<numberOfIntervals();t_id++){
      std::cout << "\n==== interval: " << t_id << "====" << std::endl;
      for(int mon_id=0;mon_id<4;mon_id++){
        std::cout << "--- t^" << mon_id << ": ";
        for(int s_id=0;s_id<2;s_id++){
          std::cout << coefficient_array[t_id][mon_id][s_id] << ",";
        }
      }
      std::cout<< "end: " << at(initialTime()+double(t_id+1)*intervalLength())[0] << "," << at(initialTime()+double(t_id+1)*intervalLength())[1];
    }
    
  }
};

}//namespace glc
#endif