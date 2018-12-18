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

#include <ros/ros.h>
#include <nav_msgs/OccupancyGrid.h>

#include <string>

#include "mapping/Pgm.hpp"
#include "mapping/PgmOccupancyConverter.hpp"

static const std::string PGM_FILE_PATH = "/opt/ros/kinetic/share/turtlebot_gazebo/maps/playground.pgm";

int main(int argc, char** argv) {
    ros::init(argc, argv, "pgm_map_publisher");

    ros::NodeHandle nh;

    auto pgm = Pgm::loadFrom(PGM_FILE_PATH);

    auto pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 2, true);

    PgmOccupancyConverter converter {0.9, 0.2};  // define occupied and free thresholds in [0..1]

    auto occupancyGrid = converter.convert(pgm);

    ros::Rate loopRate(1);

    while(nh.ok()) {            
        pub.publish(occupancyGrid);

        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
