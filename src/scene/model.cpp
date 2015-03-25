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

Model::Model() : mesh( 0 ), material( 0 ), world_normals(nullptr) { }
Model::~Model() 
{ 
	if (world_normals != nullptr)
	{
		delete[] world_normals;
		world_normals = nullptr;
	}
}

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

	world_normals = new Vector3[mesh->vertices.size()];

	for (int i = 0; i < mesh->vertices.size(); i++)
	{
		world_normals[i] = normMat * mesh->vertices[i].normal;
		if (world_normals[i] != Vector3::Ones())
		{
			world_normals[i] = normalize(world_normals[i]);
		}
	}

    return true;
}

bool Model::is_intersect_with_ray(const Ray& ray, Intersection& intersection) const
{
	Ray local_ray(invMat.transform_point(ray.e), invMat.transform_vector(ray.d));
	bool found = false;

	for (int i = 0; i < mesh->num_triangles(); i++)
	{
		const Vector3& p1 = mesh->vertices[mesh->triangles[i].vertices[0]].position;
		const Vector3& p2 = mesh->vertices[mesh->triangles[i].vertices[1]].position;
		const Vector3& p3 = mesh->vertices[mesh->triangles[i].vertices[2]].position;
		float t_max = intersection.t;
		float alpha, beta, gamma;
		if (Triangle::is_ray_triangle_intersect(local_ray, p1, p2, p3, t_max, alpha, beta, gamma) && t_max > 0 && t_max < intersection.t)
		{ 
			intersection.t = t_max;
			intersection.normal = alpha * world_normals[mesh->triangles[i].vertices[0]] + beta * world_normals[mesh->triangles[i].vertices[1]] + gamma * world_normals[mesh->triangles[i].vertices[2]];
			intersection.position = ray.at_time(intersection.t);
			intersection.geometry = this;
			found = true;
		}		
	}

	return found;
}

Color3 Model::compute_color(Intersection intersection) const
{
	Vector3 intersect_pos = intersection.position;
	Vector3 intersect_normal = intersection.normal;
	Color3 ret = scene->ambient_light * material->ambient;

	for (int i = 0; i < scene->num_lights(); i++)
	{
		const SphereLight& light = scene->get_lights()[i];
		Vector3 light_dir = normalize(light.position - intersect_pos);
		ret += light.color * material->diffuse * std::max((real_t)0, dot(intersect_normal, light_dir)); // diffuse		
	}

	return ret;
}

} /* _462 */
