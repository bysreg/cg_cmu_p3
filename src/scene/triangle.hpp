/**
 * @file triangle.hpp
 * @brief Class definition for Triangle.
 *
 * @author Eric Butler (edbutler)
 */

#ifndef _462_SCENE_TRIANGLE_HPP_
#define _462_SCENE_TRIANGLE_HPP_

#include "scene/scene.hpp"

namespace _462 {

/**
 * a triangle geometry.
 * Triangles consist of 3 vertices. Each vertex has its own position, normal,
 * texture coordinate, and material. These should all be interpolated to get
 * values in the middle of the triangle.
 * These values are all in local space, so it must still be transformed by
 * the Geometry's position, orientation, and scale.
 */
class Triangle : public Geometry
{
public:

    struct Vertex
    {
        // note that position and normal are in local space
        Vector3 position;
        Vector3 normal;
        Vector2 tex_coord;
        const Material* material;
    };

    // the triangle's vertices, in CCW order
    Vertex vertices[3];

	Vector3 world_position[3]; // the position of the vertices in world coordinate
	Vector3 world_normal[3]; // the normal of the vertices in world coordinate

    Triangle();
    virtual ~Triangle();
    virtual void render() const;
	virtual bool initialize();
	virtual bool is_intersect_with_ray(const Ray& ray, Intersection& intersection) const;
	virtual Color3 compute_color(Raytracer* raytracer, const Intersection& intersection) const;

	static bool is_ray_triangle_intersect(const Ray& ray, const Vector3& p1, const Vector3& p2, const Vector3& p3, float& t_max, float& alpha, float& beta, float& gamma);
};


} /* _462 */

#endif /* _462_SCENE_TRIANGLE_HPP_ */
