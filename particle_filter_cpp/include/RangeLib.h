

// #include "vendor/lodepng/lodepng.h"
// #include "vendor/distance_transform.h"
//#include "includes_for_pf_cpp.h"

#include <stdio.h>      /* printf */
#include <cstdlib>
#include <vector>
#include <string>
#include <iostream>
#include <cmath>
#include <algorithm>    // std::min
#include <time.h>
#include <chrono>
#include <set>
#include <iomanip>      // std::setw
#include <unistd.h>
#include <stdexcept>
#include <sstream>
// #define NDEBUG
#include <cassert>
#include <tuple>

// these flags determine whether to compile helper functions specially designed for 6.141 lab 5
#define ROS_WORLD_TO_GRID_CONVERSION 1

// No inline
#if _NO_INLINE == 1
#define ANIL __attribute__ ((noinline))
#else
#define ANIL 
#endif

// these defines are for yaml/JSON serialization
//#define T1 "  "
//#define T2 T1 T1
//#define T3 T1 T1 T1
//#define T4 T2 T2

//#define J1 "  "
//#define J2 J1 J1
//#define J3 J1 J1 J1
//#define J4 J2 J2
//#ifndef CHUNK_SIZE 
//#define CHUNK_SIZE 262144
//#define CHUNK_THREADS 256

#include "CudaRangeLib.h"



// struct OMap
// {
// 	bool has_error;
// 	unsigned width;  // x axis
// 	unsigned height; // y axis
// 	std::vector<std::vector<bool> > grid;
// 	std::vector<std::vector<float> > raw_grid;
// 	std::string fn; // filename
// 	#if _MAKE_TRACE_MAP == 1
// 	std::vector<std::vector<bool> > trace_grid;
// 	#endif
// 	// this stuff is for ROS integration, not necessary for raw usage
// 	#if ROS_WORLD_TO_GRID_CONVERSION == 1
// 	float world_scale; 
// 	float world_angle;
// 	float world_origin_x;
// 	float world_origin_y;
// 	float world_sin_angle;
// 	float world_cos_angle;
// 	#endif

// 	OMap(int w, int h) : width(w), height(h), fn(""), has_error(false) {
// 		for (int i = 0; i < w; ++i) {
// 			std::vector<bool> y_axis;
// 			for (int q = 0; q < h; ++q) y_axis.push_back(false);
// 			grid.push_back(y_axis);
// 		}
// 		#if _MAKE_TRACE_MAP == 1
// 		for (int i = 0; i < w; ++i) {
// 			std::vector<bool> y_axis;
// 			for (int q = 0; q < h; ++q) y_axis.push_back(false);
// 			trace_grid.push_back(y_axis);
// 		}
// 		#endif
// 	}

// 	OMap(std::string filename) : OMap(filename, 128) {}
// 	OMap(std::string filename, float threshold) : fn(filename), has_error(false) {
// 		unsigned error;
// 		unsigned char* image;

// 		error = lodepng_decode32_file(&image, &width, &height, filename.c_str());
// 		if(error) {
// 			printf("ERROR %u: %s\n", error, lodepng_error_text(error));
// 			has_error = true;
// 			return;
// 		}
// 		for (int i = 0; i < width; ++i) {
// 			std::vector<bool> y_axis;
// 			for (int q = 0; q < height; ++q) y_axis.push_back(false);
// 			grid.push_back(y_axis);
// 		}
// 		for (int i = 0; i < width; ++i) {
// 			std::vector<float> y_axis;
// 			for (int q = 0; q < height; ++q) y_axis.push_back(0);
// 			raw_grid.push_back(y_axis);
// 		}
// 		#if _MAKE_TRACE_MAP == 1
// 		for (int i = 0; i < width; ++i) {
// 			std::vector<bool> y_axis;
// 			for (int q = 0; q < height; ++q) y_axis.push_back(false);
// 			trace_grid.push_back(y_axis);
// 		}
// 		#endif
// 		for (int y = 0; y < height; ++y) {
// 			for (int x = 0; x < width; ++x) {
// 				unsigned idx = 4 * y * width + 4 * x;
// 				int r = image[idx + 2];
// 				int g = image[idx + 1];
// 			int b = image[idx + 0];
// 				int gray = (int) utils::rgb2gray(r,g,b);
// 				if (gray < threshold) grid[x][y] = true;
// 				raw_grid[x][y] = gray;
// 			}
// 		}
// 	}
// 	bool get(int x, int y) { return grid[x][y]; }
// 	bool isOccupied(int x, int y) { 
// 		if (x < 0 || x >= width || y < 0 || y >= height) return false;
// 		#if _MAKE_TRACE_MAP == 1
// 		trace_grid[x][y] = true;
// 		#endif
// 		return grid[x][y]; 
// 	}
// 	// query the grid without a trace
// 	bool isOccupiedNT(int x, int y) { return grid[x][y]; }
// 	#if _MAKE_TRACE_MAP == 1
// 	bool saveTrace(std::string filename) {
// 		std::vector<unsigned char> png;
// 		lodepng::State state; //optionally customize this one
// 		// char image = new char[width * height * 4] = 0;
// 		char image[width * height * 4];
// 		for (int y = 0; y < height; ++y) {
// 			for (int x = 0; x < width; ++x) {
// 				unsigned idx = 4 * y * width + 4 * x;
// 				// if (trace_grid[x][y]) {
// 				// 	image[idx + 0] = 255;
// 				// 	image[idx + 1] = 255;
// 				// 	image[idx + 2] = 255;
// 				// }
	
// 				image[idx + 2] = 255;
// 				image[idx + 1] = 255;
// 				image[idx + 0] = 255;

// 				if (trace_grid[x][y]) {
// 					image[idx + 0] = 0;
// 					image[idx + 1] = 0;
// 					image[idx + 2] = 200;
// 				}

// 				if (grid[x][y]) {
// 					image[idx + 0] = 255;
// 					image[idx + 1] = 0;
// 					image[idx + 2] = 0;
// 				}
// 				if (grid[x][y] && trace_grid[x][y]) {
// 					image[idx + 0] = 0;
// 					image[idx + 1] = 0;
// 					image[idx + 2] = 0;
// 				}
// 				image[idx + 3] = 255;
// 			}
// 		}
// 		unsigned error = lodepng::encode(png, reinterpret_cast<const unsigned char*> (image), width, height, state);
// 		if(!error) lodepng::save_file(png, filename);
// 		//if there's an error, display it
// 		if(error) std::cout << "encoder error " << error << ": "<< lodepng_error_text(error) << std::endl;
// 		return error;
// 	}
// 	#endif
// 	bool save(std::string filename) {
// 		std::vector<unsigned char> png;
// 		lodepng::State state; //optionally customize this one
// 		// char image = new char[width * height * 4] = 0;
// 		char image[width * height * 4];
// 		for (int y = 0; y < height; ++y) {
// 			for (int x = 0; x < width; ++x) {
// 				unsigned idx = 4 * y * width + 4 * x;
	
// 				image[idx + 2] = (char)255;
// 				image[idx + 1] = (char)255;
// 				image[idx + 0] = (char)255;
// 				image[idx + 3] = (char)255;
// 				if (grid[x][y]) {
// 					image[idx + 0] = 0;
// 					image[idx + 1] = 0;
// 					image[idx + 2] = 0;
// 				}
// 			}
// 		}
// 		unsigned error = lodepng::encode(png, reinterpret_cast<const unsigned char*> (image), width, height, state);
// 		if(!error) lodepng::save_file(png, filename);
// 		//if there's an error, display it
// 		if(error) std::cout << "encoder error " << error << ": "<< lodepng_error_text(error) << std::endl;
// 		return error;
// 	}
// 	OMap make_edge_map(bool count_corners) {
// 		OMap edge_map = OMap(width, height);
// 		for (int x = 0; x < width; ++x) {
// 			for (int y = 0; y < height; ++y) {
// 				if (!isOccupiedNT(x,y)) continue;
// 				std::vector<std::pair<int,int>> outline = utils::outline(x,y,count_corners);
// 				for (int i = 0; i < outline.size(); ++i) {
// 					int cx;
// 					int cy;
// 					std::tie(cx, cy) = outline[i];
// 					if (0 <= cx && 0 <= cy && cx < width && cy < height && !isOccupiedNT(cx,cy)) {
// 						edge_map.grid[x][y] = true;
// 						break;
// 					}
// 				}
// 			}
// 		}
// 		return edge_map;
// 	}
// 	bool error() {
// 		return has_error;
// 	}
// 	// returns memory usage in bytes
// 	int memory() {
// 		return sizeof(bool) * width * height;
// 	}
// };

class RayMarchingGPU
{
 protected: 
	std::vector<std::vector<double>> sensor_model;
 public:
	/*
	RayMarchingGPU(OMap m, float mr) : RangeMethod(m, mr) { 
		distImage = new DistanceTransform(&m);
		#if USE_CUDA == 1
		rmc = new RayMarchingCUDA(distImage->grid, distImage->width, distImage->height, max_range);
		#if ROS_WORLD_TO_GRID_CONVERSION == 1
   		rmc->set_conversion_params(m.world_scale,m.world_angle,m.world_origin_x, m.world_origin_y, 
			m.world_sin_angle, m.world_cos_angle);
		#endif
		#else
		throw std::string("Must compile with -DWITH_CUDA=ON to use this class.");
		#endif
	}
	~RayMarchingGPU() {
		delete distImage;
		#if USE_CUDA == 1
		delete rmc;
		#else
		throw std::string("Must compile with -DWITH_CUDA=ON to use this class.");
		#endif
	};
	*/
	float map_world_scale; //Defined as we have removed omap and used on line 312
	RayMarchingCUDA rmc;
	void numpy_calc_range_angles(float * ins, float * angles, float * outs, int num_particles, int num_angles) {
		#if USE_CUDA == 1
		#if ROS_WORLD_TO_GRID_CONVERSION == 0
		std::cout << "Cannot use GPU numpy_calc_range without ROS_WORLD_TO_GRID_CONVERSION == 1" << std::endl;
		return;
		#endif
		int particles_per_iter = std::ceil((float)CHUNK_SIZE / (float)num_angles);
		int iters = std::ceil((float)num_particles / (float) particles_per_iter);
		// must allways do the correct number of angles, can only split on the particles
		for (int i = 0; i < iters; ++i) {
			int num_in_chunk = particles_per_iter;
			if (i == iters - 1) num_in_chunk = num_particles-i*particles_per_iter;
			rmc->numpy_calc_range_angles(&ins[i*num_in_chunk*3], angles, &outs[i*num_in_chunk*num_angles],
				num_in_chunk, num_angles);
		}
		#else
		throw std::string("Must compile with -DWITH_CUDA=ON to use this class.");
		#endif
	}
	//#if SENSOR_MODEL_HELPERS == 1
	//#if USE_CUDA == 1
	void set_sensor_model(double *table, int table_width) {
		// convert the sensor model from a numpy array to a vector array
		for (int i = 0; i < table_width; ++i)
		{
			std::vector<double> table_row;
			for (int j = 0; j < table_width; ++j)
				table_row.push_back(table[table_width*i + j]);
			sensor_model.push_back(table_row);
		}
		rmc.set_sensor_table(table, table_width);
	}
	//#endif
        
        void eval_sensor_model(float * obs, float * ranges, double * outs, int rays_per_particle, int particles) {
		float inv_world_scale = 1.0 / map_world_scale;
		// do no allocations in the main loop
		double weight;
		float r;
		float d;
		int i;
		int j;
		for (i = 0; i < particles; ++i)
		{
			weight = 1.0;
			for (j = 0; j < rays_per_particle; ++j)
			{
				r = obs[j] * inv_world_scale;
				r = std::min<float>(std::max<float>(r,0.0),(float)sensor_model.size()-1.0);
				d = ranges[i*rays_per_particle+j] * inv_world_scale;
				d = std::min<float>(std::max<float>(d,0.0),(float)sensor_model.size()-1.0);
				weight *= sensor_model[(int)r][(int)d];
			}
			outs[i] = weight;
		}
	}
	//int memory() { return distImage->memory(); }
 protected:
	bool already_warned = false;
};

	
