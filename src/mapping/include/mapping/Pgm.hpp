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

#pragma once

#include <vector>
#include <string>

class Pgm {
  public:
  	static Pgm loadFrom(std::string const &);

	int gray() const {
		return gray_;
	}

	int height() const {
		return height_;
	}

	int width() const {
		return width_;
	}

	std::vector<uint8_t> data() const {
		std::vector<uint8_t> buffer;
		buffer.reserve(width_ * height_);

		for (int i = 0; i < height_; ++i){
			for (int j = 0; j < width_; ++j) {
				buffer.push_back(data_[i][j]);
			}
		}

		return buffer;
	}

	// access to the specified row
	std::vector<uint8_t>& operator[] (int rowIdx) {
		return data_[rowIdx];
	}
  private:
	Pgm(int w, int h, int gray);

	int width_;
	int height_;
	int gray_;

	std::vector<std::vector<uint8_t>> data_;
};
