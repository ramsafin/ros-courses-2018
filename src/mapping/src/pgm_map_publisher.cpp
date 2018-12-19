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

#include <opencv2/highgui.hpp>
#include <opencv2/imgproc.hpp>

#include <string>
#include <functional>

#include "mapping/Pgm.hpp"
#include "mapping/PgmOccupancyConverter.hpp"

static const std::string PGM_FILE_PATH = "/opt/ros/kinetic/share/turtlebot_gazebo/maps/playground.pgm";

using namespace std;

int thresh = 254;
int morph_elem = 0;
int morph_size = 0;
int morph_operator = 1;

int occupiedThreshold = 0.65;
int freeThreshold = 0.196;

void trackbarStub( int, void*) {};

// http://qaru.site/questions/162344/fill-the-holes-in-opencv
void fillEdgeImage(cv::Mat edgesIn, cv::Mat& filledEdgesOut) {
  using namespace cv;

  Mat edgesNeg = edgesIn.clone();
  floodFill(edgesNeg, Point(0,0), CV_RGB(255, 255, 255));
  bitwise_not(edgesNeg, edgesNeg);
  filledEdgesOut = (edgesNeg | edgesIn);
}

void fillTheGaps(cv::Mat &pgm) {
  using namespace cv;

  Mat src_thresh_bin, src_thresh_bin_morph, src_thresh_bin_morph_fill;

  threshold(pgm, src_thresh_bin, thresh, 255, THRESH_BINARY_INV);

  morphologyEx(src_thresh_bin, src_thresh_bin_morph, morph_operator, 
      getStructuringElement(morph_elem, Size(1 * morph_size + 1, 1 * morph_size + 1),
         Point(morph_size, morph_size)));

  fillEdgeImage(src_thresh_bin_morph, src_thresh_bin_morph_fill);

  for (int i = 0; i < pgm.rows; ++i) {
    for (int j = 0; j < pgm.cols; ++j) {
      if (src_thresh_bin_morph_fill.at<uint8_t>(Point(j, i)) == 255) {
        pgm.at<uint8_t>(Point(j, i)) = 0;
      }
    }
  }
}

// see http://wiki.ros.org/map_server (#Value Interpretation)
double computeProbability(uint8_t pixelValue) {
  double p = (255.0 - pixelValue) / pixelValue;
  if (p > occupiedThreshold) return 100.0;
  if (p < freeThreshold) return 0.0;
  return 99 * (p - freeThreshold) / (occupiedThreshold - freeThreshold);
}

nav_msgs::OccupancyGrid createFrom(cv::Mat const& img) {
  nav_msgs::OccupancyGrid grid;

  grid.data.reserve(img.rows * img.cols);

  for (int i = 0; i < img.rows; ++i) {
    for (int j = 0; j < img.cols; ++j) {
      grid.data.push_back(computeProbability(img.at<uint8_t>(cv::Point(j, i))));
    }
  }

  // FIXME (Ramil Safin): Add origin pose and timestamp.
  grid.info.resolution = 0.05;
  grid.info.width = img.cols;
  grid.info.height = img.rows;
  grid.header.frame_id = "map";
  grid.info.origin.position.x = -6.899;
  grid.info.origin.position.y = -5.899;
  grid.info.origin.position.z = 0.0;

  return grid;
}

int main(int argc, char** argv) {
    ros::init(argc, argv, "pgm_map_publisher");
    ros::NodeHandle nh;

    auto pgm = cv::imread("/home/ramilsafin/playground_1.pgm", cv::IMREAD_UNCHANGED);

    cv::flip(pgm, pgm, 1);

    // fillTheGaps(pgm);

    auto pub = nh.advertise<nav_msgs::OccupancyGrid>("map", 10);

    auto occupancyGrid = createFrom(pgm);

    ros::Rate loopRate(10);

    while(nh.ok()) {            
        pub.publish(occupancyGrid);

        ros::spinOnce();
        loopRate.sleep();
    }

    return 0;
}
