#ifndef __SDL_WRAPPER_H__
#define __SDL_WRAPPER_H__

#include "color.h"
#include "list.h"
#include "scene.h"
#include "state.h"
#include "vector.h"
#include <SDL2/SDL.h>
#include <SDL2/SDL2_gfxPrimitives.h>
#include <SDL2/SDL_ttf.h>
#include <stdbool.h>

// Values passed to a key handler when the given arrow key is pressed
typedef enum {
  LEFT_ARROW = 1,
  UP_ARROW = 2,
  RIGHT_ARROW = 3,
  DOWN_ARROW = 4,
  SPACE = 5
} arrow_key_t;

typedef enum { LEFT_CLICK = 1, RIGHT_CLICK = 2 } mouse_button_t;

/**
 * The possible types of key events.
 * Enum types in C are much more primitive than in Java; this is equivalent to:
 * typedef unsigned int KeyEventType;
 * #define KEY_PRESSED 0
 * #define KEY_RELEASED 1
 */
typedef enum { KEY_PRESSED, KEY_RELEASED } key_event_type_t;

/**
 * The possible types of mouse events.
 */
typedef enum { MOUSE_BUTTON_PRESSED, MOUSE_BUTTON_RELEASED } mouse_event_type_t;

/**
 * Input data type used for sdl_write_text
 * @param string text to be shown
 * @param position location of top left corner
 * @param dim dimensions of text box
 * @param color color of text
 */
typedef struct text_input {
  char *string;
  size_t font_size;
  vector_t position;
  vector_t dim;
  rgb_color_t color;
} text_input_t;

/**
 * Data type used to render text
 * @param string text to be shown
 * All other parameters are produced from sdl_write_text
 */
typedef struct text {
  char *string;
  SDL_Rect *message_rect;
  SDL_Surface *surface_message;
  SDL_Texture *message;
} text_t;

typedef struct image {
  SDL_Texture *img;
  SDL_Rect *texr;
} image_t;

/**
 * A keypress handler.
 * When a key is pressed or released, the handler is passed its char value.
 * Most keys are passed as their char value, e.g. 'a', '1', or '\r'.
 * Arrow keys have the special values listed above.
 *
 * @param key a character indicating which key was pressed
 * @param type the type of key event (KEY_PRESSED or KEY_RELEASED)
 * @param held_time if a press event, the time the key has been held in seconds
 */
typedef void (*key_handler_t)(state_t *state, char key, key_event_type_t type,
                              double held_time);

/**
 * A mouse button handler.
 * When a mouse button is pressed or released, the handler is passed its char
 * value.
 *
 * @param mouse_button a character indicating which mouse button was pressed
 * @param type the type of mouse button event (MOUSE_BUTTON_PRESSED or
 * MOUSE_BUTTON_RELEASED)
 * @param x x coordinate of mouse click
 * @param y y coordinate of mouse click
 */
typedef void (*mouse_handler_t)(state_t *state, char mouse_button,
                                mouse_event_type_t type, double x, double y);

/**
 * Initializes the SDL window and renderer.
 * Must be called once before any of the other SDL functions.
 *
 * @param min the x and y coordinates of the bottom left of the scene
 * @param max the x and y coordinates of the top right of the scene
 */
void sdl_init(vector_t min, vector_t max);

/**
 * Moves center of window and simulates camera.
 * @param position new center of window
 */
void sdl_move_window(vector_t position);

/**
 * Processes all SDL events and returns whether the window has been closed.
 * This function must be called in order to handle keypresses.
 *
 * @return true if the window was closed, false otherwise
 */
bool sdl_is_done(state_t *state);

/**
 * Adds text to list of texts to be printed in sdl_render_scene.
 */
void sdl_write_text(text_input_t text_input, char *font_style, char *font_type);

/**
 * Removes text from said list.
 */
void sdl_remove_text(text_input_t text_input);

void sdl_add_image(const char *image_path, vector_t position);

/**
 * Clears the screen. Should be called before drawing polygons in each frame.
 */
void sdl_clear(void);

/**
 * Draws a polygon from the given list of vertices and a color.
 *
 * @param points the list of vertices of the polygon
 * @param color the color used to fill in the polygon
 */
void sdl_draw_polygon(list_t *points, rgb_color_t color);

/**
 * Displays the rendered frame on the SDL window.
 * Must be called after drawing the polygons in order to show them.
 */
void sdl_show(void);

/**
 * Draws all bodies in a scene.
 * This internally calls sdl_clear(), sdl_draw_polygon(), and sdl_show(),
 * so those functions should not be called directly.
 *
 * @param scene the scene to draw
 */
void sdl_render_scene(scene_t *scene);

/**
 * Registers a function to be called every time a key is pressed.
 * Overwrites any existing handler.
 *
 * Example:
 * ```
 * void on_key(state_t *state, char key, key_event_type_t type, double
 * held_time) { if (type == KEY_PRESSED) { switch (key) { case 'a': printf("A
 * pressed\n"); break; case UP_ARROW: printf("UP pressed\n"); break;
 *         }
 *     }
 * }
 * int main(void) {
 *     sdl_on_key(on_key);
 *     while (!sdl_is_done());
 * }
 * ```
 *
 * @param handler the function to call with each key press
 */
void sdl_on_key(key_handler_t handler);

/**
 * Registers a function to be called every time a mouse button is pressed.
 * Overwrites any existing handler.
 *
 * Example:
 * ```
 * void on_mouse(state_t *state, char mouse_button, mouse_event_type_t type,
 * double x, double y) { if (type == MOUSE_BUTTON_PRESSED) { switch
 * (mouse_button) { case LEFT_CLICK: printf("left button pressed\n"); break;
 *             case RIGHT_CLICK:
 *                 printf("right button pressed\n");
 *                 break;
 *         }
 *     }
 * }
 * int main(void) {
 *     sdl_on_key(on_mouse);
 *     while (!sdl_is_done());
 * }
 * ```
 *
 * @param handler the function to call with each key press
 */
void sdl_on_mouse(mouse_handler_t handler);

/**
 * Gets the amount of time that has passed since the last time
 * this function was called, in seconds.
 *
 * @return the number of seconds that have elapsed
 */
double time_since_last_tick(void);

#endif // #ifndef __SDL_WRAPPER_H__
