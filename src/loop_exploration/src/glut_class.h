/**
 * This file is part of LAEG
 *
 * Copyright 2022:
 * - Diego Pittol <dpittol at inf dot ufrgs dot br> (Phi Robotics Research Lab - UFRGS)
 * - Renan Maffei <rqmaffei at in dot ufrgs dot br> (Phi Robotics Research Lab - UFRGS)
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

#ifndef __GLUTCLASS_H__
#define __GLUTCLASS_H__

#include <string.h>
#include <GL/glut.h>
#include <iomanip>
#include <sstream>
#include <iostream>
#include <cmath>
#include <unistd.h>

#include <FreeImage.h>

#include "robot.h"
#include "utils.h"

#include "configuration.h"

class GlutClass
{
    public:
        static GlutClass* instance();

        void Initialize();

        void Process();

        void Terminate();

        void robot(Robot* r);

        int frame;


    private:
        GlutClass ();

        static GlutClass* instance_;

        Robot* robot_;
        Grid* grid_;

        int half_window_size_x_, half_window_size_y_;
        bool lock_camera_on_robot_;

        int id_;

        int glut_window_width_, glut_window_height_, glut_scale_aux_;
        int glut_window_half_width_, glut_window_half_height_;
        int glut_x_aux_, glut_y_aux_;

        double aspect_ratio_;

        void Render();

        static void Display();
        static void Reshape(int w, int h);
        static void Keyboard(unsigned char key, int x, int y);
        static void SpecialKeys(int key, int x, int y);
};

#endif /* __GLUT_H__ */


