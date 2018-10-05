/* Copyright (C) Brian Paden (bapaden@mit.edu) - All Rights Reserved
 * Written by Brian Paden
 * Released under the GNU General Public License v3
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
 * \param x a double that is squared
 * \return The square of x
 */  
double sqr(const double x);

/**
 * \brief Calculates the square of the L2-norm of a vector.
 * \param x is the input vector whose norm will be squared.
 * \returns The square of the norm of the parameter x.
 */
double normSqr(const std::valarray<double>& x);

/**
 * \brief Calculates the square of the L2-norm of a vector.
 * \param x is the input vector whose norm will be squared
 * \param y is set to the square of the norm of the parameter x.
 */
std::vector<int> vecFloor(const std::valarray<double>& x);

double dot(const std::valarray<double>& x,const std::valarray<double>& y);

double norm2(const std::valarray<double>& x);

// double normSqr(const std::valarray<double>& x);

std::valarray<double> linearSpace(const double& start, const double& end, const int points);

}//namespace glc
#endif
