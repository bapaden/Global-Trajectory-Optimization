/* Copyright (C) Brian Paden (bapaden@mit.edu) - All Rights Reserved
 * Written by Brian Paden
 * Released under the GNU General Public License v3
 */

#include <glc_interface.h>

namespace glc{

void Inputs::addInputSample(std::valarray<double>& _input){points.push_back(_input);}
const std::vector<std::valarray<double>>& Inputs::readInputs() const {
  return points;
}

CostFunction::CostFunction(double _lipschitz_constant):lipschitz_constant(_lipschitz_constant){}

double CostFunction::getLipschitzConstant(){
  return lipschitz_constant;
}

DynamicalSystem::DynamicalSystem(double lipschitz_constant_):lipschitz_constant(lipschitz_constant_){}


}