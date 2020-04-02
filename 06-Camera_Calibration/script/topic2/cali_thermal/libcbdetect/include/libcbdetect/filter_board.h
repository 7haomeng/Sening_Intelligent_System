/**
* Copyright 2018, ftdlyc <yclu.cn@gmail.com>
*
* This library is free software; you can redistribute it and/or
* modify it under the terms of the GNU Lesser General Public
* License as published by the Free Software Foundation; either
* version 3 of the License, or (at your option) any later version.
*
* This library is distributed in the hope that it will be useful,
* but WITHOUT ANY WARRANTY; without even the implied warranty of
* MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the GNU
* Lesser General Public License for more details.
*
* You should have received a copy of the GNU Lesser General Public
* License along with this library; if not, see <http://www.gnu.org/licenses/>.
*/

#pragma once
#ifndef LIBCBDETECT_FILTER_BOARD_H
#define LIBCBDETECT_FILTER_BOARD_H

#include <vector>

#include <opencv2/opencv.hpp>

#include "libcbdetect/config.h"

namespace cbdetect {

LIBCBDETECT_DLL_DECL void filter_board(const Corner& corners, std::vector<int>& used, Board& board,
                                       std::vector<cv::Point2i>& proposal, double& energy, const Params& params);

}

#endif //LIBCBDETECT_FILTER_BOARD_H
