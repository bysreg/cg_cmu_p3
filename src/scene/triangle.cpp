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

bool Triangle::is_intersect_with_ray(const Ray& ray) const
{
	float a = world_position[0].x - world_position[1].x;
	float b = world_position[0].y - world_position[1].y;
	float c = world_position[0].z - world_position[1].z;
	float d = world_position[0].x - world_position[2].x;
	float e = world_position[0].y - world_position[2].y;
	float f = world_position[0].z - world_position[2].z;;
	float g = ray.d.x;
	float h = ray.d.y;
	float i = ray.d.z;
	float j = world_position[0].x - ray.e.x;
	float k = world_position[0].y - ray.e.y;
	float l = world_position[0].z - ray.e.z;
	float ei_min_hf = e*i - h*f;
	float gf_min_di = g*f - d*i;
	float dh_min_eg = d*h - e*g;
	
	float M = a*(ei_min_hf)+b*(gf_min_di)+c*(dh_min_eg);
	
	//compute t
	float ak_min_jb = a*k - j*b;
	float jc_min_al = j*c - a*l;
	float bl_min_kc = b*l - k*c;
	float t = (f*(ak_min_jb)+e*(jc_min_al)+d*(bl_min_kc)) / -M;
	
	if (t < 0) // should be t < to || t > t1
		return false;

	//compute gamma
	float gamma = (i*(ak_min_jb)+h*(jc_min_al)+g*(bl_min_kc)) / M;

	if (gamma < 0 || gamma > 1)
		return false;

	//compute beta
	float beta = (j*(ei_min_hf)+k*(gf_min_di)+l*(dh_min_eg)) / M;

	if (beta < 0 || beta + gamma > 1)
		return false;

	return true;
}

} /* _462 */
