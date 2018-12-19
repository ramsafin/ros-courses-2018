/*
 * MIT License
 *
 * Copyright (c) 2018 Ramil Safin
 * 
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 * 
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 * 
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 * 
 * @author Ramil Safin.
 */

#include "mapping/PgmOccupancyConverter.hpp"

PgmOccupancyConverter::PgmOccupancyConverter(double occupiedThreshold, double freeThreshold, double mapResolution)
			: occupiedThreshold_{occupiedThreshold}, freeThreshold_{freeThreshold}, mapResolution_{mapResolution} {}

nav_msgs::OccupancyGrid PgmOccupancyConverter::convert(Pgm &pgm) const {
	nav_msgs::OccupancyGrid grid;

	grid.data.reserve(pgm.height() * pgm.width());

	for (int i = 0; i < pgm.height(); ++i) {
		for (int j = 0; j < pgm.width(); ++j) {
			// NOTE: row-major order
			grid.data.push_back(computeProbability(pgm[i][j]));
		}
	}

	// FIXME (Ramil Safin): Add origin pose and timestamp.
	grid.info.resolution = mapResolution_;
	grid.info.width = pgm.width();
	grid.info.height = pgm.height();
	grid.header.frame_id = "map";
	grid.info.origin.position.x = -6.899;
	grid.info.origin.position.y = -5.899;
	grid.info.origin.position.z = 0.0;

	return grid;
}

// see http://wiki.ros.org/map_server (#Value Interpretation)
double PgmOccupancyConverter::computeProbability(uint8_t pixelValue) const {
  double p = (255 - pixelValue) / (double) pixelValue;
  if (p > occupiedThreshold_) return 100.0;
  if (p < freeThreshold_) return 0.0;
  return 99 * (p - freeThreshold_) / (occupiedThreshold_ - freeThreshold_);
}
