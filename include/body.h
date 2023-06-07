#ifndef __BODY_H__
#define __BODY_H__

#include "color.h"
#include "list.h"
#include "vector.h"
#include <stdbool.h>

/**
 * A rigid body constrained to the plane.
 * Implemented as a polygon with uniform density.
 * Bodies can accumulate forces and impulses during each tick.
 * Angular physics (i.e. torques) are not currently implemented.
 */
typedef struct body body_t;

/**
 * Initializes a body without any info.
 * Acts like body_init_with_info() where info and info_freer are NULL.
 */
body_t *body_init(list_t *shape, double mass, rgb_color_t color);

/**
 * Allocates memory for a body with the given parameters.
 * The body is initially at rest.
 * Asserts that the mass is positive and that the required memory is allocated.
 *
 * @param shape a list of vectors describing the initial shape of the body
 * @param mass the mass of the body (if INFINITY, stops the body from moving)
 * @param color the color of the body, used to draw it on the screen
 * @param info additional information to associate with the body,
 *   e.g. its type if the scene has multiple types of bodies
 * @param info_freer if non-NULL, a function call on the info to free it
 * @return a pointer to the newly allocated body
 */
body_t *body_init_with_info(list_t *shape, double mass, rgb_color_t color,
                            void *info, free_func_t info_freer);

/**
 * Releases the memory allocated for a body.
 *
 * @param body a pointer to a body returned from body_init()
 */
void body_free(body_t *body);

/**
 * Gets the current shape of a body.
 * Returns a newly allocated vector list, which must be list_free()d.
 *
 * @param body a pointer to a body returned from body_init()
 * @return the polygon describing the body's current position
 */
list_t *body_get_shape(body_t *body);

/**
 * Gets the current center of mass of a body.
 * While this could be calculated with polygon_centroid(), that becomes too slow
 * when this function is called thousands of times every tick.
 * Instead, the body should store its current centroid.
 *
 * @param body a pointer to a body returned from body_init()
 * @return the body's center of mass
 */
vector_t body_get_centroid(body_t *body);

/**
 * Gets the current velocity of a body.
 *
 * @param body a pointer to a body returned from body_init()
 * @return the body's velocity vector
 */
vector_t body_get_velocity(body_t *body);

/**
 * Gets the mass of a body.
 *
 * @param body a pointer to a body returned from body_init()
 * @return the mass passed to body_init(), which must be greater than 0
 */
double body_get_mass(body_t *body);

/**
 * Returns the moment of inertia of a desired body
 *
 * @param body the body to get the moment of inertia for
 */
double body_get_moment_of_inertia(body_t *body);

/**
 * Gets the display color of a body.
 *
 * @param body a pointer to a body returned from body_init()
 * @return the color passed to body_init(), as an (R, G, B) tuple
 */
rgb_color_t body_get_color(body_t *body);

/**
 * Gets the information associated with a body.
 *
 * @param body a pointer to a body returned from body_init()
 * @return the info passed to body_init()
 */
void *body_get_info(body_t *body);

/**
 * Translates a body to a new position.
 * The position is specified by the position of the body's center of mass.
 *
 * @param body a pointer to a body returned from body_init()
 * @param x the body's new centroid
 */
void body_set_centroid(body_t *body, vector_t x);

/**
 * Changes a body's velocity (the time-derivative of its position).
 *
 * @param body a pointer to a body returned from body_init()
 * @param v the body's new velocity
 */
void body_set_velocity(body_t *body, vector_t v);

/**
 * Changes a body's orientation in the plane.
 * The body is rotated about its center of mass.
 * Note that the angle is *absolute*, not relative to the current orientation.
 *
 * @param body a pointer to a body returned from body_init()
 * @param angle the body's new angle in radians. Positive is counterclockwise.
 */
void body_set_rotation(body_t *body, double angle);

/**
 * Applies a force to a body over the current tick.
 * If multiple forces are applied in the same tick, they should be added.
 * Should not change the body's position or velocity; see body_tick().
 *
 * @param body a pointer to a body returned from body_init()
 * @param force the force vector to apply
 */
void body_add_force(body_t *body, vector_t force);

/**
 * Applies an impulse to a body.
 * An impulse causes an instantaneous change in velocity,
 * which is useful for modeling collisions.
 * If multiple impulses are applied in the same tick, they should be added.
 * Should not change the body's position or velocity; see body_tick().
 *
 * @param body a pointer to a body returned from body_init()
 * @param impulse the impulse vector to apply
 */
void body_add_impulse(body_t *body, vector_t impulse);

/**
 * Gets the final angular velocity of a body after a tick of time dt
 *
 * @param body a pointer to a body to find the final angular velocity of
 * @param dt the time separation between ticks
 */
double body_get_final_angular_velocity(body_t *body, double dt);

/**
 * Gets the change in angle of a body after a tick of time dt
 *
 * @param curr_angular_vel current angular velocity
 * @param new_angular_vel the final angular velocity after the tick
 * @param dt the time separation between ticks
 */
double body_find_delta_angle(double curr_angular_vel, double new_angular_vel,
                             double dt);

/**
 * Sets the moment of inertia of a body (about the centorid)
 *
 * @param body a pointer to athe body to change the moment of inertia of
 * @param moment the body's new moment of inertia
 */
void body_set_normal_moment_of_inertia(body_t *body, double moment);

/**
 * Sets the current angular velocity of a given body
 *
 * @param body the body to set the angular velocity of
 * @param angular_velocity the angular velocity to set for the given body
 */
void body_set_angular_velocity(body_t *body, double angular_velocity);

/**
 * Sets the current angular acceleration of a given body
 *
 * @param body the pointer to the body to set the angular acceleration of
 * @param angular_acceleration the angular acceleration to set for the given
 * body
 */
void body_set_angular_acceleration(body_t *body, double angular_acceleration);

/**
 * Increments the angular velocity of a given body in a certain direction by a
 * certain amount
 *
 * @param body a pointer to the body to increment the angular velocity of
 * @param increment the amount by which to increment the angular velocity
 * (clockwise is positive and counterclockwise is negative)
 */
void body_increment_angular_velocity(body_t *body, double increment);

/**
 * Applies a torque to a body over the current tick.
 * If multiple torques are applied in the same tick, they are added.
 * Does not change the body's position or velocity; see body_tick().
 *
 * @param body the body to add the torque onto
 * @param torque the torque to apply
 */
void body_add_torque(body_t *body, double torque);

/**
 * Adds an angular impulse to a body over the current tick
 *
 * @param body a pointer to the body to apply the impulse to
 * @param angular_impulse the impulse to apply
 */
void body_add_angular_impulse(body_t *body, double angular_impulse);

/**
 * Sets the pivot point for rotation for a given body (away from its centroid)
 * Also changes the moment of inertia accordingly
 *
 * @param body a pointer to the body to change the current pivot of
 * @param pivot the point to instantiate as the new pivot point
 */

void body_set_pivot(body_t *body, vector_t pivot);

/**
 * Resets the pivot point of rotation back to the centroid of a body
 * Also resets the moment of inertia to its default (through the centroid)
 *
 * @param body a pointer to the body to reset the pivot point for
 */
void body_reset_pivot(body_t *body);

/**
 * Updates the body after a given time interval has elapsed.
 * Sets acceleration and velocity according to the forces and impulses
 * applied to the body during the tick.
 * The body should be translated at the *average* of the velocities before
 * and after the tick.
 * Resets the forces and impulses accumulated on the body.
 *
 * @param body the body to tick
 * @param dt the number of seconds elapsed since the last tick
 */
void body_tick(body_t *body, double dt);

/**
 * Marks a body for removal--future calls to body_is_removed() will return true.
 * Does not free the body.
 * If the body is already marked for removal, does nothing.
 *
 * @param body a pointer to the body to mark for removal
 */
void body_remove(body_t *body);

/**
 * Returns whether a body has been marked for removal.
 * This function returns false until body_remove() is called on the body,
 * and returns true afterwards.
 *
 * @param body a pointer to the body to check
 * @return whether body_remove() has been called on the body
 */
bool body_is_removed(body_t *body);

#endif // #ifndef __BODY_H__
