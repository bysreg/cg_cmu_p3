/**
 * @file raytacer.hpp
 * @brief Raytracer class
 *
 * Implement these functions for project 3.
 *
 * @author H. Q. Bovik (hqbovik)
 * @bug Unimplemented
 */

#ifndef _462_RAYTRACER_HPP_
#define _462_RAYTRACER_HPP_

#include "application/options.hpp"
#include "math/color.hpp"
#include "math/random462.hpp"
#include "p3/photon.hpp"
#include "p3/neighbor.hpp"
#include "application/opengl.hpp"
#include "p3/photonmap.hpp"
#include "p3/util.hpp"
namespace _462 {

class Scene;
class Ray;
struct Intersection;

class Raytracer
{
public:
    PhotonMap photonMap;

    Raytracer();

    ~Raytracer();

    bool initialize(Scene* scene, size_t num_samples,
                    size_t width, size_t height);

	bool initialize(Scene* scene, size_t num_samples,
					size_t width, size_t height, Options opt);

	bool shoot_ray(const Ray& ray, Intersection& intersection, float t_max);
    Color3 trace_ray(const Ray &ray);
	Color3 trace_ray(const Ray& ray, int depth);
    
    bool raytrace(unsigned char* buffer, real_t* max_time);
    
    Color3 trace_pixel(size_t x,
               size_t y,
               size_t width,
               size_t height);

	//dof parameters
	bool dof_active;
	int dof_aperture_size;
	float dof_focal_length;
	int dof_number_of_rays;

private:
    // the scene to trace
    Scene* scene;
    Projector projector;
    
    // the dimensions of the image to trace
    size_t width, height;

    // the next row to raytrace
    size_t current_row;

    unsigned int num_samples;
    
};

} /* _462 */

#endif /* _462_RAYTRACER_HPP_ */
