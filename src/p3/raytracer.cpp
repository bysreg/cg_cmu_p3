/**
* @file raytacer.cpp
* @brief Raytracer class
*
* Implement these functions for project 4.
*
* @author H. Q. Bovik (hqbovik)
* @bug Unimplemented
*/

#include "raytracer.hpp"
#include "scene/scene.hpp"
#include "math/quickselect.hpp"
#include "p3/randomgeo.hpp"
#include <SDL_timer.h>
namespace _462 {


	//number of rows to render before updating the result
	static const unsigned STEP_SIZE = 1;
	static const unsigned CHUNK_SIZE = 1;

	Raytracer::Raytracer() {
		scene = 0;
		width = 0;
		height = 0;
	}

	Raytracer::~Raytracer() { }

	/**
	* Initializes the raytracer for the given scene. Overrides any previous
	* initializations. May be invoked before a previous raytrace completes.
	* @param scene The scene to raytrace.
	* @param width The width of the image being raytraced.
	* @param height The height of the image being raytraced.
	* @return true on success, false on error. The raytrace will abort if
	*  false is returned.
	*/
	bool Raytracer::initialize(Scene* scene, size_t num_samples,
		size_t width, size_t height)
	{
		this->scene = scene;
		this->num_samples = num_samples;
		this->width = width;
		this->height = height;

		current_row = 0;

		projector.init(scene->camera);
		scene->initialize();
		photonMap.initialize(scene);
		return true;
	}

	//will always make intersection.t to max of float first whatever the result is
	bool Raytracer::shoot_ray(const Ray& ray, Intersection& intersection, float t_max)
	{
		Geometry* const* geometries = scene->get_geometries();
		bool ret = false;
		intersection.t = t_max;
		for (size_t i = 0; i < scene->num_geometries(); ++i)
		{
			geometries[i]->is_intersect_with_ray(ray, intersection);
		}

		return intersection.geometry != nullptr;
	}

	Color3 Raytracer::trace_ray(const Ray &ray){
		return trace_ray(ray, MAX_DEPTH);
	}

	Color3 Raytracer::trace_ray(const Ray& ray, int depth)
	{
		Intersection intersection;

		if (shoot_ray(ray, intersection, std::numeric_limits<float>::max()))
		{
			Color3 total_shadow_color;
			Color3 diffuse_color = intersection.geometry->get_diffuse_color(intersection);

			for (int i = 0; i < scene->num_lights(); i++)
			{
				Color3 color;
				const SphereLight& light = scene->get_lights()[i];

				//is this light blocked ?
				for (int j = 0; j < DIRECT_SAMPLE_COUNT; j++)
				{
					Vector3 light_pos(random_gaussian(), random_gaussian(), random_gaussian());
					light_pos = (normalize(light_pos) * light.radius) + light.position;
					Vector3 light_dir = normalize(light_pos - intersection.position);

					Ray shadow_ray(intersection.position, light_dir);
					Intersection shadow_intersection;
					float t_max = dot(light_pos - shadow_ray.e, light_dir);
					if (!shoot_ray(shadow_ray, shadow_intersection, t_max))
					{
						//calculate the attenuation
						Color3 light_color_at_d = light.compute_light_color_at_d(t_max);

						color += light_color_at_d * diffuse_color * std::max((real_t)0, dot(intersection.normal, light_dir)); // diffuse		
					}
				}

				total_shadow_color += color;
			}

			total_shadow_color = total_shadow_color / DIRECT_SAMPLE_COUNT;

			Color3 geom_specular_color = intersection.geometry->get_specular_color(intersection);
			real_t geom_refractive_index = intersection.geometry->get_refractive_index(intersection);
			Color3 geom_texture_color = intersection.geometry->get_texture_color(intersection);

			if (depth - 1 >= 0 && geom_refractive_index > 0)
			{
				Vector3 refraction_dir;
				float cos_val = -dot(ray.d, intersection.normal);
				if (dot(ray.d, intersection.normal) < 0)
				{
					refraction_dir = normalize(refract(intersection.normal, ray.d, 1.0f / geom_refractive_index)); // no need to check for total internal reflection here. because air is the highest refractive index
					cos_val = -dot(ray.d, intersection.normal);
				}
				else
				{
					refraction_dir = refract(-intersection.normal, ray.d, geom_refractive_index / 1.0f); // not normalized yet, we will normalize inside the if
					if (refraction_dir != Vector3::Zero())
					{
						refraction_dir = normalize(refraction_dir);
						cos_val = dot(refraction_dir, intersection.normal);
					}
					else
					{
						//too bad, there is a total internal reflection here, so no refraction

						Color3 total_specular_color;
						if (depth - 1 >= 0 && geom_specular_color != Color3::Black())
						{
							Ray reflection_ray(intersection.position, normalize(reflect(ray.d, intersection.normal)));
							total_specular_color += geom_specular_color * trace_ray(reflection_ray, depth - 1);
						}

						return geom_texture_color*(total_shadow_color + total_specular_color); // no ambient color if there is refraction // fixme
					}
				}

				real_t R0 = ((geom_refractive_index - 1) * (geom_refractive_index - 1)) / ((geom_refractive_index + 1) * (geom_refractive_index + 1));
				real_t R = R0 + ((1 - R0) * (1 - cos_val) * (1 - cos_val) * (1 - cos_val) * (1 - cos_val) * (1 - cos_val));

#define USE_RAND 0
#if USE_RAND
				float random = static_cast <float> (rand()) / static_cast <float> (RAND_MAX);
				if (random < R)
				{
#endif
					Color3 total_specular_color;
					if (depth - 1 >= 0 && geom_specular_color != Color3::Black())
					{
						Ray reflection_ray(intersection.position, normalize(reflect(ray.d, intersection.normal)));
						total_specular_color += geom_specular_color * trace_ray(reflection_ray, depth - 1);
					}
#if USE_RAND
					return total_shadow_color + (R * total_specular_color); // no ambient color if there is refraction
				}
				else
				{
#endif
					Ray refraction_ray(intersection.position, refraction_dir);
#if USE_RAND
					return total_shadow_color + ((1 - R) * trace_ray(refraction_ray, depth - 1)); // no ambient color if there is refraction
				}
#endif

#if !USE_RAND
					return geom_texture_color*(total_shadow_color + ((1 - R) * trace_ray(refraction_ray, depth - 1)) + (R * total_specular_color)); // no ambient color if there is refraction
#endif
			}
			else
			{
				Color3 total_specular_color;
				if (depth - 1 >= 0 && geom_specular_color != Color3::Black())
				{
					Ray reflection_ray(intersection.position, normalize(reflect(ray.d, intersection.normal)));
					total_specular_color += geom_specular_color * trace_ray(reflection_ray, depth - 1);
				}
				return geom_texture_color*(total_shadow_color + total_specular_color + (scene->ambient_light * intersection.geometry->get_ambient_color(intersection)));
			}
		}
		else
		{
			return scene->background_color;
		}
	}

	/**
	* Performs a raytrace on the given pixel on the current scene.
	* The pixel is relative to the bottom-left corner of the image.
	* @param scene The scene to trace.
	* @param x The x-coordinate of the pixel to trace.
	* @param y The y-coordinate of the pixel to trace.
	* @param width The width of the screen in pixels.
	* @param height The height of the screen in pixels.
	* @return The color of that pixel in the final image.
	*/
	Color3 Raytracer::trace_pixel(size_t x,
		size_t y,
		size_t width,
		size_t height)
	{
		assert(x < width);
		assert(y < height);

		real_t dx = real_t(1) / width;
		real_t dy = real_t(1) / height;

		Color3 res = Color3::Black();
		unsigned int iter;
		for (iter = 0; iter < num_samples; iter++)
		{
			// pick a point within the pixel boundaries to fire our
			// ray through.
			real_t i = real_t(2)*(real_t(x) + random_uniform())*dx - real_t(1);
			real_t j = real_t(2)*(real_t(y) + random_uniform())*dy - real_t(1);

			Ray r = Ray(scene->camera.get_position(), projector.get_pixel_dir(i, j));

			res += trace_ray(r);
		}
		return res*(real_t(1) / num_samples);
	}

	/**
	* Raytraces some portion of the scene. Should raytrace for about
	* max_time duration and then return, even if the raytrace is not copmlete.
	* The results should be placed in the given buffer.
	* @param buffer The buffer into which to place the color data. It is
	*  32-bit RGBA (4 bytes per pixel), in row-major order.
	* @param max_time, If non-null, the maximum suggested time this
	*  function raytrace before returning, in seconds. If null, the raytrace
	*  should run to completion.
	* @return true if the raytrace is complete, false if there is more
	*  work to be done.
	*/
	bool Raytracer::raytrace(unsigned char* buffer, real_t* max_time)
	{
		static const size_t PRINT_INTERVAL = 64;

		// the time in milliseconds that we should stop
		unsigned int end_time = 0;
		bool is_done;

		if (max_time)
		{
			// convert duration to milliseconds
			unsigned int duration = (unsigned int)(*max_time * 1000);
			end_time = SDL_GetTicks() + duration;
		}

		// until time is up, run the raytrace. we render an entire group of
		// rows at once for simplicity and efficiency.
		for (; !max_time || end_time > SDL_GetTicks(); current_row += STEP_SIZE)
		{
			// we're done if we finish the last row
			is_done = current_row >= height;
			// break if we finish
			if (is_done) break;

			int loop_upper = std::min(current_row + STEP_SIZE, height);

			for (int c_row = current_row; c_row < loop_upper; c_row++)
			{
				/*
				* This defines a critical region of code that should be
				* executed sequentially.
				*/
#pragma omp critical
			{
				if (c_row % PRINT_INTERVAL == 0)
					printf("Raytracing (Row %d)\n", c_row);
			}

			// This tells OpenMP that this loop can be parallelized.
#pragma omp parallel for schedule(dynamic, CHUNK_SIZE)
#ifdef WIN32
			for (long x = 0; x < width; x++)
#else
			for (size_t x = 0; x < width; x++)
#endif
			{
				// trace a pixel
				Color3 color = trace_pixel(x, c_row, width, height);
				// write the result to the buffer, always use 1.0 as the alpha
				color.to_array4(&buffer[4 * (c_row * width + x)]);
			}
#pragma omp barrier

			}
		}

		if (is_done) printf("Done raytracing!\n");

		return is_done;
	}

} /* _462 */
