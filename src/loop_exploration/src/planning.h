/**
 * This file is part of LAEG
 *
 * Copyright 2022 Diego Pittol <dpittol at inf dot ufrgs dot br> (Phi Robotics Research Lab - UFRGS)
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

#ifndef __PLANNING_H__
#define __PLANNING_H__

class Planning;

#include <pthread.h>
#include <queue>
#include <stack>
#include <math.h>
#include <iomanip>
#include <sstream>

#include "robot.h"
#include "grid.h"
#include "loop_exploration.h"
#include "thinning.h"

#include "configuration.h"
#include "logger.h"

class Planning {
	public:
        Planning(Configuration* config, Logger* logger);
        ~Planning();

        bool Run();

        void Initialize();

        void UpdateMap();

        void SetNewRobotPose(Pose p);
        void grid(Grid* g);

        void SaveMap();

        Grid* grid_;

        void RequestToFinish();

private:

        LoopExploration* exploration_;

        void ExpandObstacles();
        void UpdateCellsTypes();
        void UpdateCenterCells(bool remove_spurs, bool traverse_unexplored);

        int iteration_;

        Pose* current_pose_;
        BoundingBox update_window_;

        Pose* new_pose_;
        BoundingBox new_update_window_;

        bool requested_to_stop_;

        // Parameters
        int free_threshold_;
        int obstacle_threshold_;
        int inflate_obstacles_pad_size_;
        int external_border_;

        // Misc
        Logger* logger_;
};


#endif /* __PLANNING_H__ */
