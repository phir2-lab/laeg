/**
 * This file is part of LAEG
 *
 * Copyright 2022:
 * - Diego Pittol <dpittol at inf dot ufrgs dot br> (Phi Robotics Research Lab - UFRGS)
 * - Renan Maffei <rqmaffei at in dot ufrgs dot br> (Phi Robotics Research Lab - UFRGS)
 * - Vitor Jorge
 * For more information see <https://github.com/phir2-lab/laeg>
 *
 * LAEG is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * LAEG is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with LAEG If not, see <https://www.gnu.org/licenses/>.
**/

#ifndef THINNING_H
#define THINNING_H

#include <stddef.h>
#include <iostream>
#include <stddef.h>
#include <stdlib.h>
#include <string.h>
#include <stack>
#include <vector>
#include <cmath>

using std::cout;
using std::cerr;
using std::endl;
using std::stack;
using std::vector;

struct specialThinPattern
{
    unsigned char pattern[9];
};

class Thinning
{
public:
    Thinning();

    // algorithm from Zoeng and Hall 1989, also used by MATLAB, claims to result in 8-connected thinned lines
    bool Thinning_Using_GUOandHALL(const unsigned char* image, unsigned char* output, int width, int height);
    bool spur_removal(int iterations, unsigned char* image, int width, int height);

    // method used to set the borders as 0
    void _setBordersAsZero(unsigned char* image, const size_t& width, const size_t& height);

    // stuff for GUO and HALL thinning algorithm
    void _grabThinPattern2(const unsigned char *image, int width, int center, specialThinPattern &p);
    int _XH(const specialThinPattern &p); // eight-connected stuff
    int _minN1N2(const specialThinPattern &p); // eight-connected stuff
    bool _testG3_odd(const specialThinPattern &p); // odd condition
    bool _testG3_even(const specialThinPattern &p); // even condition
};

#endif // THINNING_H
