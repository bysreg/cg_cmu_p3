/**
 * @file sphere.hpp
 * @brief Class defnition for Sphere.
 *
 * @author Kristin Siu (kasiu)
 * @author Eric Butler (edbutler)
 */

#ifndef _462_SCENE_SPHERE_HPP_
#define _462_SCENE_SPHERE_HPP_

#include "scene/scene.hpp"

namespace _462 {

/**
 * A sphere, centered on its position with a certain radius.
 */
class Sphere : public Geometry
{
public:

    real_t radius;
    const Material* material;

    Sphere();
    virtual ~Sphere();
    virtual void render() const;
	virtual bool is_intersect_with_ray(const Ray& ray, Intersection& intersection) const;
	virtual Color3 compute_color(Raytracer* raytracer, const Intersection& intersection) const;

	void update_intersection(const Ray& ray, float t, Intersection& intersection) const;
};

} /* _462 */

#endif /* _462_SCENE_SPHERE_HPP_ */

