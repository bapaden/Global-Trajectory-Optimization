/* Copyright (C) Brian Paden (bapaden@mit.edu)
 * Written by Brian Paden
 * Released under the MIT License
 */

#ifndef GLC_MATH_H
#define GLC_MATH_H

//External depenencies
#include <iostream>
#include <vector>
#include <cassert>
#include <sstream>
#include <fstream>
#include <string>
#include <valarray>

namespace glc{

/**
 * \brief Computes the square of a floating point number
 * \param x_ a double that is squared
 * \return The square of x
 */  
double sqr(const double x_);

/**
 * \brief Calculates the square of the L2-norm of a vector; more efficient than sqr(norm2(x))
 * \param x_ is the input vector whose norm be computed and squared.
 * \returns The square of the norm of the parameter x.
 */
double normSqr(const std::valarray<double>& x_);

/**
 * \brief Element-wise floor operation
 * \param x_ is the input vector whose entries will be rounded down
 */
std::vector<int> vecFloor(const std::valarray<double>& x_);

/**
 * \brief The dot product of two vectors x and y
 * \param[in] x_ is one of the vectors in the product
 * \param[in] y_ the other vector in the product
 * \returns The dot product obtained by x'y
 */
double dot(const std::valarray<double>& x_,const std::valarray<double>& y_);

/**
 * \brief This method computes the l2 norm of a vector x
 * \param[in] x_ is the vector whose norm will be computed
 * \returns The return value is the l2 norm of the parameter x
 */
double norm2(const std::valarray<double>& x_);

/**
 * \brief This method computes uniformly spaced points on an interval
 * \param[in] start_ is the lower bound of the interval
 * \param[in] end_ is the upper bound of the interval
 * \param[in] num_points_ is the number of points uniformly spaced on [start_,end_)
 * \returns An array of points with length num_points_ starting at start_ and ending at end_ the rightmost boundary is not included.
 */
std::valarray<double> linearSpace(const double& start_, const double& end_, const int num_points_);

}//namespace glc
#endif
