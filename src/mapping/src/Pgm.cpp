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

#include <fstream>
#include <sstream>
#include <cassert>

#include "mapping/Pgm.hpp"

Pgm Pgm::loadFrom(std::string const & path) {
  std::ifstream file{path, std::ios::binary};

  assert(file);

  std::string line;
  std::getline(file, line); // version: P5
  std::getline(file, line); // comment

  std::stringstream ss;
  ss << file.rdbuf();

  int width, height, gray;
  ss >> width >> height >> gray;

  Pgm pgm{width, height, gray};

  for (int rowIdx = 0; rowIdx < height; ++rowIdx) {
    for (int columnIdx = 0; columnIdx < width; ++columnIdx) {
	    ss >> pgm[rowIdx][columnIdx];
    }
  }
  
  return pgm;
}

Pgm::Pgm(int w, int h, int gray) : width_{w}, height_{h}, gray_{gray} {
  data_.resize(height_);
 
  for (auto& row : data_) {
    row.reserve(width_);
  }
}
