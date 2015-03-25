/**
 * @file triangle.cpp
 * @brief Function definitions for the Triangle class.
 *
 * @author Eric Butler (edbutler)
 */

#include "scene/triangle.hpp"
#include "application/opengl.hpp"
#include "math/math.hpp"
#include "p3/raytracer.hpp"

namespace _462 {

Triangle::Triangle()
{
    vertices[0].material = 0;
    vertices[1].material = 0;
    vertices[2].material = 0;
    isBig=true;
}

Triangle::~Triangle() { }

bool Triangle::initialize() 
{
	Geometry::initialize();

	for (int i = 0; i < 3; i++)
	{
		world_position[i] = mat.transform_point(vertices[i].position);
		world_normal[i] = normMat * vertices[i].normal;
		if (world_normal[i] != Vector3::Ones())
		{
			world_normal[i] = normalize(world_normal[i]); // this normal has been scaled, so we have to normalize it back
		}
	}

	return true;
}

void Triangle::render() const
{
    bool materials_nonnull = true;
    for ( int i = 0; i < 3; ++i )
        materials_nonnull = materials_nonnull && vertices[i].material;

    // this doesn't interpolate materials. Ah well.
    if ( materials_nonnull )
        vertices[0].material->set_gl_state();

    glBegin(GL_TRIANGLES);

#if REAL_FLOAT
    glNormal3fv( &vertices[0].normal.x );
    glTexCoord2fv( &vertices[0].tex_coord.x );
    glVertex3fv( &vertices[0].position.x );

    glNormal3fv( &vertices[1].normal.x );
    glTexCoord2fv( &vertices[1].tex_coord.x );
    glVertex3fv( &vertices[1].position.x);

    glNormal3fv( &vertices[2].normal.x );
    glTexCoord2fv( &vertices[2].tex_coord.x );
    glVertex3fv( &vertices[2].position.x);
#else
    glNormal3dv( &vertices[0].normal.x );
    glTexCoord2dv( &vertices[0].tex_coord.x );
    glVertex3dv( &vertices[0].position.x );

    glNormal3dv( &vertices[1].normal.x );
    glTexCoord2dv( &vertices[1].tex_coord.x );
    glVertex3dv( &vertices[1].position.x);

    glNormal3dv( &vertices[2].normal.x );
    glTexCoord2dv( &vertices[2].tex_coord.x );
    glVertex3dv( &vertices[2].position.x);
#endif

    glEnd();

    if ( materials_nonnull )
        vertices[0].material->reset_gl_state();
}

bool Triangle::is_ray_triangle_intersect(const Ray& ray, const Vector3& p1, const Vector3& p2, const Vector3& p3, float& t_max, float& alpha, float& beta, float& gamma)
{
	float a = p1.x - p2.x;
	float b = p1.y - p2.y;
	float c = p1.z - p2.z;
	float d = p1.x - p3.x;
	float e = p1.y - p3.y;
	float f = p1.z - p3.z;;
	float g = ray.d.x;
	float h = ray.d.y;
	float i = ray.d.z;
	float j = p1.x - ray.e.x;
	float k = p1.y - ray.e.y;
	float l = p1.z - ray.e.z;
	float ei_min_hf = e*i - h*f;
	float gf_min_di = g*f - d*i;
	float dh_min_eg = d*h - e*g;

	float M = a*(ei_min_hf)+b*(gf_min_di)+c*(dh_min_eg);

	//compute t
	float ak_min_jb = a*k - j*b;
	float jc_min_al = j*c - a*l;
	float bl_min_kc = b*l - k*c;
	float t = (f*(ak_min_jb)+e*(jc_min_al)+d*(bl_min_kc)) / -M;

	if (t < EPS || t > t_max)
		return false;

	//compute gamma
	gamma = (i*(ak_min_jb)+h*(jc_min_al)+g*(bl_min_kc)) / M;

	if (gamma < 0 || gamma > 1)
		return false;

	//compute beta
	beta = (j*(ei_min_hf)+k*(gf_min_di)+l*(dh_min_eg)) / M;

	if (beta < 0 || beta + gamma > 1)
		return false;

	t_max = t;
	alpha = 1 - beta - gamma;
	return true;
}

bool Triangle::is_intersect_with_ray(const Ray& ray, Intersection& intersection) const
{
	float t_max = intersection.t;
	float alpha, beta, gamma;
	if (is_ray_triangle_intersect(ray, world_position[0], world_position[1], world_position[2], t_max, alpha, beta, gamma))
	{
		intersection.t = t_max;
		intersection.normal = alpha * world_normal[0] + beta * world_normal[1] + gamma * world_normal[2];
		intersection.position = ray.at_time(intersection.t);
		intersection.geometry = this;
		intersection.alpha = alpha;
		intersection.beta = beta;
		intersection.gamma = gamma;
		return true;
	}
	return false;
}

template<class T> T interpolate_value(float alpha, float beta, float gamma, T v1, T v2, T v3)
{
	return alpha * v1 + beta * v2 + gamma * v3;
}

Color3 Triangle::compute_color(Raytracer* raytracer, Intersection intersection) const
{
	//TODO : does not support refraction
	Vector3 intersect_pos = intersection.position;
	Vector3 intersect_normal = intersection.normal;
	Color3 ret;

	Color3 ambient = interpolate_value(intersection.alpha, intersection.beta, intersection.gamma, vertices[0].material->ambient, vertices[1].material->ambient, vertices[2].material->ambient);
	Color3 diffuse = interpolate_value(intersection.alpha, intersection.beta, intersection.gamma, vertices[0].material->diffuse, vertices[1].material->diffuse, vertices[2].material->diffuse);

	for (int i = 0; i < scene->num_lights(); i++)
	{
		const SphereLight& light = scene->get_lights()[i];
		Vector3 light_dir = normalize(light.position - intersect_pos);		

		//is this light blocked ?
		Ray shadow_ray(intersection.position, light_dir);
		Intersection shadow_intersection;
		float t_max = dot(light.position - shadow_ray.e, light_dir);
		if (!raytracer->shoot_ray(shadow_ray, shadow_intersection, t_max))
		{
			ret += light.color * diffuse * std::max((real_t)0, dot(intersect_normal, light_dir)); // diffuse		
		}
	}

	ret += (scene->ambient_light * ambient);

	return ret;
}

} /* _462 */
