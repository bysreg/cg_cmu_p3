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
#include "p3/raytracer.hpp"
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
		if (Triangle::is_ray_triangle_intersect(local_ray, p1, p2, p3, t_max, alpha, beta, gamma) && t_max > EPS && t_max < intersection.t)
		{ 
			intersection.t = t_max;
			intersection.normal = alpha * world_normals[mesh->triangles[i].vertices[0]] + beta * world_normals[mesh->triangles[i].vertices[1]] + gamma * world_normals[mesh->triangles[i].vertices[2]];
			intersection.position = ray.at_time(intersection.t);
			intersection.geometry = this;
			intersection.tex_coord = alpha * mesh->vertices[mesh->triangles[i].vertices[0]].tex_coord + beta * mesh->vertices[mesh->triangles[i].vertices[1]].tex_coord + gamma * mesh->vertices[mesh->triangles[i].vertices[2]].tex_coord;
			found = true;
		}		
	}

	return found;
}

Color3 Model::get_ambient_color(const Intersection& intersection) const
{
	return material->ambient;
}

Color3 Model::get_diffuse_color(const Intersection& intersection) const
{
	return material->diffuse;
}

Color3 Model::get_specular_color(const Intersection& intersection) const
{
	return material->specular;
}

real_t Model::get_refractive_index(const Intersection& intersection) const
{
	return material->refractive_index;
}

Color3 Model::get_texture_color(const Intersection& intersection) const
{
	int width, height;
	material->texture.get_texture_size(&width, &height);
	int pix_x = (int)fmod(width*intersection.tex_coord.x, width);
	int pix_y = (int)fmod(height*intersection.tex_coord.y, height);

	return material->texture.get_texture_pixel(pix_x, pix_y);
}

} /* _462 */
