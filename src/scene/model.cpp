/**
 * @file model.cpp
 * @brief Model class
 *
 * @author Eric Butler (edbutler)
 * @author Zeyang Li (zeyangl)
 */

#include "scene/model.hpp"
#include "scene/material.hpp"
#include "application/opengl.hpp"
#include "scene/triangle.hpp"
#include <iostream>
#include <cstring>
#include <string>
#include <fstream>
#include <sstream>


namespace _462 {

Model::Model() : mesh( 0 ), material( 0 ) { }
Model::~Model() { }

void Model::render() const
{
    if ( !mesh )
        return;
    if ( material )
        material->set_gl_state();
    mesh->render();
    if ( material )
        material->reset_gl_state();
}
bool Model::initialize(){
    Geometry::initialize();
    return true;
}

bool Model::is_intersect_with_ray(const Ray& ray, Intersection& intersection) const
{
	Ray local_ray(invMat.transform_point(ray.e), invMat.transform_vector(ray.d));

	for (int i = 0; i < mesh->num_triangles(); i++)
	{
		const Vector3& p1 = mesh->vertices[mesh->triangles[i].vertices[0]].position;
		const Vector3& p2 = mesh->vertices[mesh->triangles[i].vertices[1]].position;
		const Vector3& p3 = mesh->vertices[mesh->triangles[i].vertices[2]].position;
		float t_max = intersection.t;
		float alpha, beta, gamma;
		if (Triangle::is_ray_triangle_intersect(local_ray, p1, p2, p3, t_max, alpha, beta, gamma))
		{ 
			intersection.t = t_max;
			return true;
		}		
	}

	return false;
}

} /* _462 */
