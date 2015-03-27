/**
 * @file sphere.cpp
 * @brief Function defnitions for the Sphere class.
 *
 * @author Kristin Siu (kasiu)
 * @author Eric Butler (edbutler)
 */

#include "scene/sphere.hpp"
#include "application/opengl.hpp"
#include "p3/raytracer.hpp"
#include <algorithm>

namespace _462 {

#define SPHERE_NUM_LAT 80
#define SPHERE_NUM_LON 100

#define SPHERE_NUM_VERTICES ( ( SPHERE_NUM_LAT + 1 ) * ( SPHERE_NUM_LON + 1 ) )
#define SPHERE_NUM_INDICES ( 6 * SPHERE_NUM_LAT * SPHERE_NUM_LON )
// index of the x,y sphere where x is lat and y is lon
#define SINDEX(x,y) ((x) * (SPHERE_NUM_LON + 1) + (y))
#define VERTEX_SIZE 8
#define TCOORD_OFFSET 0
#define NORMAL_OFFSET 2
#define VERTEX_OFFSET 5
#define BUMP_FACTOR 1

static unsigned int Indices[SPHERE_NUM_INDICES];
static float Vertices[VERTEX_SIZE * SPHERE_NUM_VERTICES];

static void init_sphere()
{
    static bool initialized = false;
    if ( initialized )
        return;

    for ( int i = 0; i <= SPHERE_NUM_LAT; i++ ) {
        for ( int j = 0; j <= SPHERE_NUM_LON; j++ ) {
            real_t lat = real_t( i ) / SPHERE_NUM_LAT;
            real_t lon = real_t( j ) / SPHERE_NUM_LON;
            float* vptr = &Vertices[VERTEX_SIZE * SINDEX(i,j)];

            vptr[TCOORD_OFFSET + 0] = lon;
            vptr[TCOORD_OFFSET + 1] = 1-lat;

            lat *= PI;
            lon *= 2 * PI;
            real_t sinlat = sin( lat );

            vptr[NORMAL_OFFSET + 0] = vptr[VERTEX_OFFSET + 0] = sinlat * sin( lon );
            vptr[NORMAL_OFFSET + 1] = vptr[VERTEX_OFFSET + 1] = cos( lat ),
            vptr[NORMAL_OFFSET + 2] = vptr[VERTEX_OFFSET + 2] = sinlat * cos( lon );
        }
    }

    for ( int i = 0; i < SPHERE_NUM_LAT; i++ ) {
        for ( int j = 0; j < SPHERE_NUM_LON; j++ ) {
            unsigned int* iptr = &Indices[6 * ( SPHERE_NUM_LON * i + j )];

            unsigned int i00 = SINDEX(i,  j  );
            unsigned int i10 = SINDEX(i+1,j  );
            unsigned int i11 = SINDEX(i+1,j+1);
            unsigned int i01 = SINDEX(i,  j+1);

            iptr[0] = i00;
            iptr[1] = i10;
            iptr[2] = i11;
            iptr[3] = i11;
            iptr[4] = i01;
            iptr[5] = i00;
        }
    }

    initialized = true;
}

Sphere::Sphere()
    : radius(0), material(0) {}

Sphere::~Sphere() {}

void Sphere::render() const
{
    // create geometry if we haven't already
    init_sphere();

    if ( material )
        material->set_gl_state();

    // just scale by radius and draw unit sphere
    glPushMatrix();
    glScaled( radius, radius, radius );
    glInterleavedArrays( GL_T2F_N3F_V3F, VERTEX_SIZE * sizeof Vertices[0], Vertices );
    glDrawElements( GL_TRIANGLES, SPHERE_NUM_INDICES, GL_UNSIGNED_INT, Indices );
    glPopMatrix();

    if ( material )
        material->reset_gl_state();
}

//quadratic formula
//If a solution exists, returns answers in x1 and x2, and returns true.
//Otherwise, returns false
bool solve_quadratic(real_t *x1,real_t *x2, real_t a, real_t b, real_t c){
    real_t b24ac = b*b-4*a*c;
    if(b24ac<0){
        return false;
    }else{
        real_t sb24ac=sqrt(b24ac);
        *x1=(-b+sb24ac)/(2*a);
        *x2=(-b-sb24ac)/(2*a);
        return true;
    }
}

//solve a quadratic equation, and then return the smallest solution larger than EPS
//if there is no solution, return -1
real_t solve_time(real_t a,real_t b,real_t c){
    real_t x1;
    real_t x2;
    if(solve_quadratic(&x1,&x2,a,b,c)){
        if(x1>EPS && x2>EPS){
            return std::min(x1,x2);
        }else if(x1>EPS){
            return x1;
        }else if(x2>EPS){
            return x2;
        }
    }

    return -1;
}

bool Sphere::is_intersect_with_ray(const Ray& ray, Intersection& intersection) const
{
	Ray local_ray(invMat.transform_point(ray.e), invMat.transform_vector(ray.d));	

	Vector3 e_min_c = local_ray.e/* - position*/; // no need to substract with position, because we test the intersection in local space
	real_t B = 2*dot(local_ray.d, e_min_c);
	real_t A = dot(local_ray.d, local_ray.d);
	real_t C = dot(e_min_c, e_min_c) - (radius * radius);

	float determinant = (B*B - (4 * A*C));

	if (determinant < 0)
		return false;
	
	//determinant is >= 0

	float t = solve_time(A, B, C);

	if (t > EPS && t < intersection.t)
	{
		update_intersection(local_ray, t, intersection);
		return true;
	}

	return false;
}

void Sphere::update_intersection(const Ray& local_ray, float t, Intersection& intersection) const
{
	intersection.t = t;	
	intersection.position = mat.transform_point(local_ray.at_time(t));
	intersection.normal = normalize(normMat * normalize(local_ray.at_time(t)));
	intersection.geometry = this;
}

Color3 Sphere::get_ambient_color(const Intersection& intersection) const
{
	return material->ambient;
}

Color3 Sphere::get_diffuse_color(const Intersection& intersection) const
{
	return material->diffuse;
}

Color3 Sphere::get_specular_color(const Intersection& intersection) const
{
	return material->specular;
}

real_t Sphere::get_refractive_index(const Intersection& intersection) const
{
	return material->refractive_index;
}

Color3 Sphere::get_texture_color(const Intersection& intersection) const
{
	//convert to local space
	Vector3 local_inter_pos = invMat.transform_point(intersection.position);
	real_t theta = std::acosf(local_inter_pos.y / radius);
	real_t phi = std::atan2(local_inter_pos.x, local_inter_pos.z);
	real_t u = phi / (2 * PI);
	real_t v = (PI - theta) / PI;
	int width;
	int height;
	material->texture.get_texture_size(&width, &height);

	return material->texture.get_texture_pixel(u * width, v * height);
}

} /* _462 */

