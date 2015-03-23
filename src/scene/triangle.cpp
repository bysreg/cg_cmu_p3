/**
 * @file triangle.cpp
 * @brief Function definitions for the Triangle class.
 *
 * @author Eric Butler (edbutler)
 */

#include "scene/triangle.hpp"
#include "application/opengl.hpp"
#include "math/math.hpp"

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

	world_position[0] = mat.transform_point(vertices[0].position);
	world_position[1] = mat.transform_point(vertices[1].position);
	world_position[2] = mat.transform_point(vertices[2].position);

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

	if (t < 0 || t > t_max) // should be t < to || t > t1
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
		return true;
	}
	return false;
}

} /* _462 */
