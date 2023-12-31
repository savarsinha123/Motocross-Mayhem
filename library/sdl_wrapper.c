#include "sdl_wrapper.h"
#include <SDL.h>
#include <SDL2/SDL2_gfxPrimitives.h>
#include <SDL2/SDL_image.h>
#include <SDL2/SDL_mixer.h>
#include <SDL2/SDL_ttf.h>
#include <assert.h>
#include <dirent.h>
#include <math.h>
#include <stdlib.h>
#include <time.h>
#include <unistd.h>

const char WINDOW_TITLE[] = "CS 3";
const int WINDOW_WIDTH = 1000;
const int WINDOW_HEIGHT = 500;
const double MS_PER_S = 1e3;

Mix_Chunk *idle;
Mix_Chunk *acc;
Mix_Chunk *dec;
Mix_Chunk *full_throttle;

/**
 * The coordinate at the center of the screen.
 */
vector_t center;
/**
 * The coordinate difference from the center to the top right corner.
 */
vector_t max_diff;
/**
 * The SDL window where the scene is rendered.
 */
SDL_Window *window;
/**
 * The renderer used to draw the scene.
 */
SDL_Renderer *renderer;
/**
 * The keypress handler, or NULL if none has been configured.
 */
key_handler_t key_handler = NULL;
/**
 * The keypress handler, or NULL if none has been configured.
 */
mouse_handler_t mouse_handler = NULL;
/**
 * SDL's timestamp when a key was last pressed or released.
 * Used to mesasure how long a key has been held.
 */
uint32_t key_start_timestamp;
/**
 * The value of clock() when time_since_last_tick() was last called.
 * Initially 0.
 */
clock_t last_clock = 0;

list_t *text_list;
list_t *image_list;
#define FONT_PATH_SIZE 60

/** Computes the center of the window in pixel coordinates */
vector_t get_window_center(void) {
  int *width = malloc(sizeof(*width)), *height = malloc(sizeof(*height));
  assert(width != NULL);
  assert(height != NULL);
  SDL_GetWindowSize(window, width, height);
  vector_t dimensions = {.x = *width, .y = *height};
  free(width);
  free(height);
  return vec_multiply(0.5, dimensions);
}

/**
 * Computes the scaling factor between scene coordinates and pixel coordinates.
 * The scene is scaled by the same factor in the x and y dimensions,
 * chosen to maximize the size of the scene while keeping it in the window.
 */
double get_scene_scale(vector_t window_center) {
  // Scale scene so it fits entirely in the window
  double x_scale = window_center.x / max_diff.x,
         y_scale = window_center.y / max_diff.y;
  return x_scale < y_scale ? x_scale : y_scale;
}

/** Maps a scene coordinate to a window coordinate */
vector_t get_window_position(vector_t scene_pos, vector_t window_center) {
  // Scale scene coordinates by the scaling factor
  // and map the center of the scene to the center of the window
  vector_t scene_center_offset = vec_subtract(scene_pos, center);
  double scale = get_scene_scale(window_center);
  vector_t pixel_center_offset = vec_multiply(scale, scene_center_offset);
  vector_t pixel = {.x = round(window_center.x + pixel_center_offset.x),
                    // Flip y axis since positive y is down on the screen
                    .y = round(window_center.y - pixel_center_offset.y)};
  return pixel;
}

vector_t get_scene_position(vector_t pixel, vector_t window_center) {
  vector_t pixel_center_offset = {.x = window_center.x - pixel.x,
                                  .y = window_center.y - pixel.y};
  double scale = get_scene_scale(window_center);
  vector_t scene_center_offset = vec_multiply(1 / scale, pixel_center_offset);
  vector_t scene_pos = vec_add(scene_center_offset, center);
  return scene_pos;
}

/**
 * Converts an SDL key code to a char.
 * 7-bit ASCII characters are just returned
 * and arrow keys are given special character codes.
 */
char get_keycode(SDL_Keycode key) {
  switch (key) {
  case SDLK_LEFT:
    return LEFT_ARROW;
  case SDLK_UP:
    return UP_ARROW;
  case SDLK_RIGHT:
    return RIGHT_ARROW;
  case SDLK_DOWN:
    return DOWN_ARROW;
  case SDLK_SPACE:
    return SPACE;
  default:
    // Only process 7-bit ASCII characters
    return key == (SDL_Keycode)(char)key ? key : '\0';
  }
}

char get_mousecode(uint8_t button) {
  switch (button) {
  case SDL_BUTTON_LEFT:
    return LEFT_CLICK;
  case SDL_BUTTON_RIGHT:
    return RIGHT_CLICK;
  default:
    return 0;
  }
}

void free_text(text_t *text_args) {
  free(text_args->message_rect);
  SDL_DestroyTexture(text_args->message);
  SDL_FreeSurface(text_args->surface_message);
  free(text_args);
}

void free_image(image_t *image) {
  SDL_DestroyTexture(image->img);
  free(image->texr);
  free(image);
}

void sdl_init(vector_t min, vector_t max) {
  // Check parameters
  assert(min.x < max.x);
  assert(min.y < max.y);

  center = vec_multiply(0.5, vec_add(min, max));
  max_diff = vec_subtract(max, center);
  SDL_Init(SDL_INIT_EVERYTHING);
  TTF_Init();
  window = SDL_CreateWindow(WINDOW_TITLE, SDL_WINDOWPOS_CENTERED,
                            SDL_WINDOWPOS_CENTERED, WINDOW_WIDTH, WINDOW_HEIGHT,
                            SDL_WINDOW_RESIZABLE);
  renderer = SDL_CreateRenderer(window, -1, SDL_RENDERER_PRESENTVSYNC);
  text_list = list_init(1, (free_func_t)free_text);
  image_list = list_init(1, (free_func_t)free_image);
}

void sdl_move_window(vector_t position) { center = position; }

bool sdl_is_done(state_t *state) {
  SDL_Event *event = malloc(sizeof(*event));
  assert(event != NULL);
  while (SDL_PollEvent(event)) {
    switch (event->type) {
    case SDL_QUIT:
      free(event);
      return true;
    case SDL_KEYDOWN:
    case SDL_KEYUP:
      // Skip the keypress if no handler is configured
      // or an unrecognized key was pressed
      if (key_handler == NULL)
        break;
      char key = get_keycode(event->key.keysym.sym);
      if (key == '\0')
        break;

      uint32_t timestamp = event->key.timestamp;
      if (!event->key.repeat) {
        key_start_timestamp = timestamp;
      }
      key_event_type_t type =
          event->type == SDL_KEYDOWN ? KEY_PRESSED : KEY_RELEASED;
      double held_time = (timestamp - key_start_timestamp) / MS_PER_S;
      key_handler(state, key, type, held_time);
      break;
    case SDL_MOUSEBUTTONDOWN:
    case SDL_MOUSEBUTTONUP:
      if (mouse_handler == NULL)
        break;
      char mouse_button = get_mousecode(event->button.button);
      if (mouse_button == 0)
        break;
      mouse_event_type_t mouse_type = event->type == SDL_MOUSEBUTTONDOWN
                                          ? MOUSE_BUTTON_PRESSED
                                          : MOUSE_BUTTON_RELEASED;
      vector_t window_center = get_window_center();
      vector_t pixel = (vector_t){event->button.x, event->button.y};
      vector_t scene_pos = get_scene_position(pixel, window_center);
      double x_scale = window_center.x / max_diff.x;
      mouse_handler(state, mouse_button, mouse_type,
                    WINDOW_WIDTH / x_scale - scene_pos.x, scene_pos.y);
      break;
    }
  }
  free(event);
  return false;
}

SDL_Rect *create_rect(vector_t position, vector_t dim) {
  SDL_Rect *rect = malloc(sizeof(SDL_Rect)); // create a rect
  vector_t window_center = get_window_center();
  vector_t pixel = get_window_position(position, window_center);
  rect->x = pixel.x; // controls the rect's x coordinate
  rect->y = pixel.y; // controls the rect's y coordinte
  double x_scale = window_center.x / max_diff.x,
         y_scale = window_center.y / max_diff.y;
  rect->w = round(dim.x * x_scale); // controls the width of the rect
  rect->h = round(dim.y * y_scale); // controls the height of the rect
  return rect;
}

void sdl_write_text(text_input_t text_input, char *font_style,
                    char *font_type) {
  TTF_Init();
  // font style
  char font_path[FONT_PATH_SIZE] = "assets/";
  strcat(font_path, font_style);
  strcat(font_path, "-");
  strcat(font_path, font_type);
  strcat(font_path, ".otf");
  TTF_Font *font = TTF_OpenFont(font_path, text_input.font_size);
  assert(font != NULL);

  // color of text
  Uint8 r = text_input.color.r * 255;
  Uint8 g = text_input.color.g * 255;
  Uint8 b = text_input.color.b * 255;
  SDL_Color sdl_color = {r, g, b};

  // create surface
  SDL_Surface *surface_message =
      TTF_RenderText_Solid(font, text_input.string, sdl_color);

  // convert to texture
  SDL_Texture *message =
      SDL_CreateTextureFromSurface(renderer, surface_message);
  SDL_Rect *message_rect = create_rect(text_input.position, text_input.dim);

  text_t *text_args = malloc(sizeof(text_t));
  *text_args = (text_t){.string = text_input.string,
                        .message_rect = message_rect,
                        .surface_message = surface_message,
                        .message = message};

  // add to list of text boxes
  list_add(text_list, text_args);
  TTF_CloseFont(font);
}

void sdl_remove_text(text_input_t text_input) {
  for (int16_t i = list_size(text_list) - 1; i >= 0; i--) {
    text_t *text_args = list_get(text_list, i);
    if (!strcmp(text_input.string, text_args->string)) {
      text_t *removed_arg = list_remove(text_list, i);
      free_text(removed_arg);
      break;
    }
  }
}

void sdl_clear_text() {
  for (int16_t i = list_size(text_list) - 1; i >= 0; i--) {
    text_t *removed_arg = list_remove(text_list, i);
    free_text(removed_arg);
  }
}

void sdl_add_image(const char *image_path, vector_t position) {
  int w, h;
  SDL_Texture *img = IMG_LoadTexture(renderer, image_path);
  assert(img != NULL);
  SDL_QueryTexture(img, NULL, NULL, &w,
                   &h); // get the width and height of the texture
  // put the location where we want the texture to be drawn into a rectangle
  // I'm also scaling the texture 2x simply by setting the width and height
  vector_t dim = {2500, 1000};
  SDL_Rect *texr = create_rect(position, dim);
  image_t *image = malloc(sizeof(image_t));
  image->img = img;
  image->texr = texr;
  list_add(image_list, image);
}

void sdl_clear_images() {
  int len = list_size(image_list);
  for (int i = len - 1; i >= 0; i--) {
    image_t *image = list_remove(image_list, i);
    free_image(image);
  }
}

const char *idle_path = "assets/MX Idle.wav";
const char *acc_path = "assets/MX Acceleration.wav";
const char *dec_path = "assets/MX Deceleration.wav";
const char *full_throttle_path = "assets/MX Full Throttle.wav";

void sound_init() {
  const int frequency = 44100;
  const int channels = 2;
  const int chunk_size = 8192;
  Mix_OpenAudio(frequency, MIX_DEFAULT_FORMAT, channels, chunk_size);
  idle = Mix_LoadWAV(idle_path);
  acc = Mix_LoadWAV(acc_path);
  dec = Mix_LoadWAV(dec_path);
  full_throttle = Mix_LoadWAV(full_throttle_path);
}

void sound_play(sound_t sound) {
  Mix_Chunk *to_play;
  switch (sound) {
  case IDLE:
    to_play = idle;
    break;
  case ACC:
    to_play = acc;
    break;
  case DEC:
    to_play = dec;
    break;
  case FULL_THROTTLE:
    to_play = full_throttle;
    break;
  }
  Mix_HaltChannel(-1);
  Mix_PlayChannel(-1, to_play, 0);
}

void sdl_clear(void) {
  SDL_SetRenderDrawColor(renderer, 255, 255, 255, 255);
  SDL_RenderClear(renderer);
}

void sdl_draw_polygon(list_t *points, rgb_color_t color) {
  // Check parameters
  size_t n = list_size(points);
  assert(n >= 3);
  assert(0 <= color.r && color.r <= 1);
  assert(0 <= color.g && color.g <= 1);
  assert(0 <= color.b && color.b <= 1);

  vector_t window_center = get_window_center();

  // Convert each vertex to a point on screen
  int16_t *x_points = malloc(sizeof(*x_points) * n),
          *y_points = malloc(sizeof(*y_points) * n);
  assert(x_points != NULL);
  assert(y_points != NULL);
  for (size_t i = 0; i < n; i++) {
    vector_t *vertex = list_get(points, i);
    vector_t pixel = get_window_position(*vertex, window_center);
    x_points[i] = pixel.x;
    y_points[i] = pixel.y;
  }

  // Draw polygon with the given color
  filledPolygonRGBA(renderer, x_points, y_points, n, color.r * 255,
                    color.g * 255, color.b * 255, 255);
  free(x_points);
  free(y_points);
}

void sdl_show(void) {
  // Draw boundary lines
  vector_t window_center = get_window_center();
  vector_t max = vec_add(center, max_diff),
           min = vec_subtract(center, max_diff);
  vector_t max_pixel = get_window_position(max, window_center),
           min_pixel = get_window_position(min, window_center);
  SDL_Rect *boundary = malloc(sizeof(*boundary));
  boundary->x = min_pixel.x;
  boundary->y = max_pixel.y;
  boundary->w = max_pixel.x - min_pixel.x;
  boundary->h = min_pixel.y - max_pixel.y;
  SDL_SetRenderDrawColor(renderer, 0, 0, 0, 255);
  SDL_RenderDrawRect(renderer, boundary);
  free(boundary);

  SDL_RenderPresent(renderer);
}

void sdl_render_scene(scene_t *scene) {
  sdl_clear();
  size_t body_count = scene_bodies(scene);
  for (size_t i = 0; i < list_size(image_list); i++) {
    image_t *image = list_get(image_list, i);
    SDL_RenderCopy(renderer, image->img, NULL, image->texr);
  }
  for (size_t i = 0; i < body_count; i++) {
    body_t *body = scene_get_body(scene, i);
    list_t *shape = body_get_shape(body);
    sdl_draw_polygon(shape, body_get_color(body));
    list_free(shape);
  }
  for (size_t i = 0; i < list_size(text_list); i++) {
    text_t *text = list_get(text_list, i);
    SDL_RenderCopy(renderer, text->message, NULL, text->message_rect);
  }
  sdl_show();
}

void sdl_on_key(key_handler_t handler) { key_handler = handler; }

void sdl_on_mouse(mouse_handler_t handler) { mouse_handler = handler; }

double time_since_last_tick(void) {
  clock_t now = clock();
  double difference = last_clock
                          ? (double)(now - last_clock) / CLOCKS_PER_SEC
                          : 0.0; // return 0 the first time this is called
  last_clock = now;
  return difference;
}
