#include <librealsense2/rs.h>
#include <librealsense2/rs_advanced_mode.h>
#include <librealsense2/h/rs_pipeline.h>
#include <librealsense2/h/rs_option.h>
#include <librealsense2/h/rs_frame.h>

#define SDL_MAIN_HANDLED
#ifdef WIN32
#include <SDL.h>
#else
#include <SDL2/SDL.h>
#endif

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

const int cDepthW = 1280;
const int cDepthH = 720;
const int cColorW = 1920;
const int cColorH = 1080;

// Disable to render color
#define RENDER_DEPTH

#define PRESET_COUNT 3
const char* presets[PRESET_COUNT] = {
    "High Accuracy",
    "High Density",
    "Hand"
};

struct RGBA
{
    uint8_t r;
    uint8_t g;
    uint8_t b;
    uint8_t a;
};

struct RS_State
{
    rs2_context* ctx;
    rs2_device_list* device_list;
    int32_t dev_count;
    rs2_device* dev;
    rs2_sensor_list* sensor_list;
    int32_t sensor_list_count;
    int32_t advanced_enabled;
    rs2_pipeline* pipe;
    rs2_pipeline_profile* selection;
    rs2_stream_profile_list* stream_list;
    int32_t stream_list_count;
    rs2_config* config;
    rs2_processing_block* temporal_filter;
    rs2_frame_queue* frame_queue;
};

int8_t check_error(rs2_error* e)
{
    if (e)
    {
        fprintf(stderr, "rs_error was raised when calling %s(%s): \n",
               rs2_get_failed_function(e), rs2_get_failed_args(e));
        fprintf(stderr, "%s\n", rs2_get_error_message(e));
        return 1;
    }
    return 0;
}

int8_t clear_state(struct RS_State* s)
{
    if (s == NULL) {
        fprintf(stderr, "Cannot clear state: given pointer is null\n");
        return 1;
    }

    if(s->temporal_filter) {
        rs2_delete_processing_block(s->temporal_filter);
    }

    if (s->frame_queue) {
        rs2_delete_frame_queue(s->frame_queue);
    }

    if (s->dev) {
        rs2_delete_device(s->dev);
    }

    if (s->sensor_list) {
        rs2_delete_sensor_list(s->sensor_list);
    }

    if (s->device_list) {
        rs2_delete_device_list(s->device_list);
    }

    if (s->config) {
        rs2_delete_config(s->config);
    }

    if (s->selection) {
        rs2_delete_pipeline_profile(s->selection);
    }

    if (s->pipe) {
        rs2_pipeline_stop(s->pipe, NULL);
        rs2_delete_pipeline(s->pipe);
    }

    if (s->ctx) {
        rs2_delete_context(s->ctx);
    }

    memset(s, 0, sizeof(struct RS_State));
    return 0;
}

int8_t init_state(struct RS_State* s, int rs_dev_index)
{
    if (s == NULL) {
        fprintf(stderr, "Cannot init state: given pointer is null\n");
        return 1;
    }

    rs2_error* e = NULL;
    if (s->ctx != NULL) {
        rs2_delete_context(s->ctx);
        s->ctx = NULL;
    }

    s->ctx = rs2_create_context(RS2_API_VERSION, &e);
    if (check_error(e) != 0) {
        s->ctx = NULL;
        return 1;
    }

    s->device_list = rs2_query_devices(s->ctx, &e);
    if (check_error(e) != 0) {
        s->device_list = NULL;
        return 1;
    }

    s->dev_count = rs2_get_device_count(s->device_list, &e);
    if (check_error(e) != 0) {
        s->dev_count = 0;
        return 1;
    }

    fprintf(stderr, "There are %d connected RealSense devices.\n", s->dev_count);
    if (0 == s->dev_count)
        return 1;

    fprintf(stderr, "Creating device\n");
    s->dev = rs2_create_device(s->device_list, rs_dev_index, &e);
    if (check_error(e) != 0) {
        s->dev = NULL;
        return 1;
    }

    return 0;
}

int8_t init_streaming(struct RS_State* s)
{
    if (s == NULL) {
        fprintf(stderr, "Cannot init streaming: given pointer is null\n");
        return 1;
    }

    if (s->dev == NULL) {
        fprintf(stderr, "No device for initting streaming\n");
        return 1;
    }

    rs2_error* e = NULL;
    s->pipe = rs2_create_pipeline(s->ctx, &e);
    if (check_error(e) != 0) {
        s->pipe = NULL;
        return 1;
    }

    s->config = rs2_create_config(&e);
    if (check_error(e) != 0) {
        s->config = NULL;
        return 1;
    }

    rs2_config_enable_stream(s->config, RS2_STREAM_DEPTH, -1, cDepthW, cDepthH, RS2_FORMAT_Z16, 30, &e);
    if (check_error(e) != 0) {
        fprintf(stderr, "Failed initting depth streaming\n");
        return 1;
    }

    rs2_config_enable_stream(s->config, RS2_STREAM_COLOR, -1, cColorW, cColorH, RS2_FORMAT_RGB8, 30, &e);
    if (check_error(e) != 0) {
        fprintf(stderr, "Failed initting color streaming\n");
        return 1;
    }

    s->selection = rs2_pipeline_start_with_config(s->pipe, s->config, &e);
    if (check_error(e) != 0) {
        fprintf(stderr, "Failed starting pipeline\n");
        s->selection = NULL;
        return 1;
    }

    s->stream_list = rs2_pipeline_profile_get_streams(s->selection, &e);
    if (check_error(e) != 0) {
        fprintf(stderr, "Failed getting pipeline profile streams\n");
        s->stream_list = NULL;
        return 1;
    }

    s->stream_list_count = rs2_get_stream_profiles_count(s->stream_list, &e);
    if (check_error(e) != 0) {
        fprintf(stderr, "Failed getting pipeline profile stream count\n");
        s->stream_list_count = 0;
        return 1;
    }

    s->sensor_list = rs2_query_sensors(s->dev, &e);
    if (check_error(e) != 0) {
        fprintf(stderr, "Failed getting sensors for device\n");
        s->sensor_list = NULL;
        return 1;
    }

    s->sensor_list_count = rs2_get_sensors_count(s->sensor_list, &e);
    if (check_error(e) != 0) {
        fprintf(stderr, "Failed getting sensor list count\n");
        s->sensor_list_count = 0;
        return 1;
    }

    int sensor;
    for (sensor = 0; sensor < s->sensor_list_count; sensor++)
    {
        rs2_sensor* sen = rs2_create_sensor(s->sensor_list, sensor, &e);

        if (check_error(e) != 0) {
            fprintf(stderr, "Failed creating temp sensor object %d / %d\n", sensor, s->sensor_list_count);
            return 1;
        }

        // This cast is done as the types are opaque and defined in the C++ API
        // Apparently simply doing this cast is the expected way to do this, says Intel
        int supports = rs2_supports_option((const rs2_options*)sen, RS2_OPTION_LASER_POWER, &e);

        if (check_error(e) != 0) {
            fprintf(stderr, "Failed asking if sensor supports LASER POWER\n");
            rs2_delete_sensor(sen);
            return 1;
        }

        if (supports == 1) {
            float min, max, step, def;
            rs2_get_option_range((const rs2_options*)sen, RS2_OPTION_LASER_POWER, &min, &max, &step, &def, &e);
            if (check_error(e) != 0) {
                fprintf(stderr, "Failed getting RS2_OPTION_LASER_POWER ranges\n");
                rs2_delete_sensor(sen);
                return 1;
            }

            rs2_set_option((const rs2_options*)sen, RS2_OPTION_LASER_POWER, max, &e);
            if (check_error(e) != 0) {
                fprintf(stderr, "Failed setting RS2_OPTION_LASER_POWER to %f\n", max);
                rs2_delete_sensor(sen);
                return 1;
            }
        }

        supports = rs2_supports_option((const rs2_options*)sen, RS2_OPTION_FRAMES_QUEUE_SIZE, &e);
        if (check_error(e) != 0) {
            fprintf(stderr, "Failed asking if sensor supports LASER RS2_OPTION_FRAMES_QUEUE_SIZE\n");
            rs2_delete_sensor(sen);
            return 1;
        }

        if (supports == 1)
        {
            rs2_set_option((const rs2_options*)sen, RS2_OPTION_FRAMES_QUEUE_SIZE, 0, &e);
            if (check_error(e) != 0) {
                fprintf(stderr, "Failed setting RS2_OPTION_FRAMES_QUEUE_SIZE to 0\n");
                rs2_delete_sensor(sen);
                return 1;
            }
        }

        rs2_delete_sensor(sen);
    }

    return 0;
}

int8_t init_reset(struct RS_State* s, int rs_dev_index)
{
    if (s == NULL) {
        fprintf(stderr, "Cannot init-reset state: given pointer is null\n");
        return 1;
    }

    rs2_error* e = NULL;

    if (clear_state(s) != 0) {
        fprintf(stderr, "Failed clearing state while resetting\n");
        return 1;
    }

    s->ctx = rs2_create_context(RS2_API_VERSION, &e);
    if (check_error(e) != 0) {
        fprintf(stderr, "create context failed when init resetting\n");
        s->ctx = NULL;
        return 1;
    }

    s->device_list = rs2_query_devices(s->ctx, &e);
    if (check_error(e) != 0) {
        fprintf(stderr, "Failed querying for devices in init reset\n");
        s->device_list = NULL;
        return 1;
    }

    s->dev_count = rs2_get_device_count(s->device_list, &e);
    if (check_error(e) != 0) {
        fprintf(stderr, "failed getting device count in init reset\n");
        s->dev_count = 0;
        return 1;
    }

    fprintf(stderr, "There are %d connected RealSense devices.\n", s->dev_count);
    if (s->dev_count == 0)
        return 1;

    fprintf(stderr, "Creating device %d\n", rs_dev_index);
    s->dev = rs2_create_device(s->device_list, rs_dev_index, &e);
    if (check_error(e) != 0) {
        fprintf(stderr, "Failed creating device in init reset\n");
        s->dev = NULL;
        return 1;
    }

    rs2_is_enabled(s->dev, &s->advanced_enabled, &e);
    if (check_error(e) != 0) {
        fprintf(stderr, "failed testing advanced mode flag in init reset\n");
        return 1;
    }

    if (s->advanced_enabled == 0)  {
        fprintf(stderr, "Device starts with advanced mode disabled\n");

        fprintf(stderr, "Resetting device\n");
        rs2_hardware_reset(s->dev, &e);
        if (check_error(e) != 0) {
            fprintf(stderr, "hardware reset failed in init reset\n");
            return 1;
        }

    } else {
        fprintf(stderr, "Device starts with advanced mode enabled, not resetting\n");
    }

    return 0;
}

int8_t set_advanced(struct RS_State* s, int val)
{
    if (s == NULL) {
        fprintf(stderr, "Cannot set advanced: given pointer is null\n");
        return 1;
    }

    rs2_error* e = NULL;

    if (val == 0) {
        fprintf(stderr, "disabling advanced mode\n");
    } else {
        fprintf(stderr, "enabling advanced mode\n");
    }

    rs2_toggle_advanced_mode(s->dev, val, &e);
    if (check_error(e) != 0) {
        // This thing spits out errors if the device is not ready for this mode yet
        return 0;
    }

    rs2_is_enabled(s->dev, &s->advanced_enabled, &e);
    if (check_error(e) != 0) {
        // This thing spits out errors if the device is not ready for this mode yet
        return 0;
    }

    return 0;
}


int8_t ensureAdvanced(struct RS_State* rs_state)
{
    if (init_reset(rs_state, 0) != 0) {
        fprintf(stderr, "Failed init-reset while waiting for advanced mode\n");
        return 1;
    }

    while (rs_state->advanced_enabled == 0)
    {
        fprintf(stderr, "Waiting for advanced mode\n");
        if (clear_state(rs_state) != 0) {
            fprintf(stderr, "Failed clearing state while waiting for advanced mode\n");
            return 1;
        }

        if (init_state(rs_state, 0) != 0) {
            fprintf(stderr, "Failed initting state while waining for advanced mode\n");
            return 1;
        }

        if (set_advanced(rs_state, 1) != 0) {
            fprintf(stderr, "failed setting advanced mode\n");
            return 1;
        }

#ifdef WIN32
        Sleep(1000);
#else
        usleep(1000 * 1000);
#endif
    }

    fprintf(stderr, "advanced mode enabled\n");

    return 0;
}

int8_t initialize(struct RS_State* rs_state)
{
    rs2_error* e = NULL;

    rs_state->ctx = rs2_create_context(RS2_API_VERSION, &e);
    if (check_error(e) != 0) {
        rs_state->ctx = NULL;
        fprintf(stderr, "Failed creating rs context\n");
        return 1;
    }

    return 0;
}

int8_t startSensor(struct RS_State* rs_state)
{
    rs2_error* e = NULL;

    if (rs_state->device_list != NULL)
    {
        if (clear_state(rs_state) != 0)
            return 1;
    }

    rs_state->device_list = rs2_query_devices(rs_state->ctx, &e);
    if (check_error(e) != 0) {
        rs_state->device_list = NULL;
        return 1;
    }

    rs_state->dev_count = rs2_get_device_count(rs_state->device_list, &e);
    if (check_error(e) != 0) {
        rs_state->dev_count = 0;
        return 1;
    }

    int i;

    for (i = 0; i < rs_state->dev_count; i++)
    {
        rs_state->dev = rs2_create_device(rs_state->device_list, i, &e);
        if (check_error(e) != 0) {
            return 1;
        }
    }

    if (rs_state->dev == NULL)
    {
        fprintf(stderr, "Failed getting device\n");
        return 1;
    }

    if (ensureAdvanced(rs_state) != 0)
    {
        fprintf(stderr, "Ensuring advanced mode failed\n");
        return 1;
    }

    if (init_streaming(rs_state) != 0)
    {
        fprintf(stderr, "Failed initting streams\n");
        return 1;
    }

    return 0;
}

inline uint16_t lerp(uint16_t a, uint16_t b, float alpha)
{
    return a * (1.0f - alpha) + alpha * b;
}

int8_t update(struct RS_State* rs_state, uint16_t* dep, struct RGBA* dep_rgb, struct RGBA* col,
              int8_t* got_dep, int8_t* got_col)
{
    rs2_frame* frames;
    rs2_error* e = NULL;
    const int dep_bytes = cDepthW * cDepthH * sizeof(uint16_t);
    const int col_bytes = cColorW * cColorH * sizeof(struct RGBA);

    frames = rs2_pipeline_wait_for_frames(rs_state->pipe, 5000, &e);
    if (check_error(e) != 0) {
        fprintf(stderr, "Failed waiting for frames\n");
        return 1;
    }

    int num_frames = rs2_embedded_frames_count(frames, &e);
    if (check_error(e) != 0) {
        fprintf(stderr, "Failed getting framelist size\n");
        return 1;
    }

    uint16_t* dbuf = NULL;
    uint8_t* col_buf;

    int f;
    for (f = 0; f < num_frames; f++)
    {
        rs2_frame* fr = rs2_extract_frame(frames, f, &e);
        if (check_error(e) != 0) {
            fprintf(stderr, "Failed extracting frame %d / %d\n", f, num_frames);
            rs2_release_frame(frames);
            return 1;
        }

        if (rs2_is_frame_extendable_to(fr, RS2_EXTENSION_DEPTH_FRAME, &e) == 1)
        {
            if (check_error(e) != 0) {
                fprintf(stderr, "Failed waiting for frame after frame queue\n");
                rs2_release_frame(fr);
                rs2_release_frame(frames);
                return 1;
            }

            dbuf = (uint16_t*)(rs2_get_frame_data(fr, &e));
            if (check_error(e) != 0) {
                fprintf(stderr, "Failed getting depth frame\n");
                rs2_release_frame(fr);
                rs2_release_frame(frames);
                return 1;
            }

            memcpy(dep, dbuf, dep_bytes);

            int y = 0;
            int x = 0;
            uint16_t depval;
            for (y = 0; y < cDepthH; y++)
            {
                for (x = 0; x < cDepthW; x++)
                {
                    depval = lerp(0, 255, dep[y * cDepthW + x] / 10000.f);

                    dep_rgb[y * cDepthW + x].r = (uint8_t)depval;
                    dep_rgb[y * cDepthW + x].g = (uint8_t)depval;
                    dep_rgb[y * cDepthW + x].b = (uint8_t)depval;
                    dep_rgb[y * cDepthW + x].a = 255;
                }
            }
            *got_dep = 1;
        }
        else
        {
            col_buf = (uint8_t*)(rs2_get_frame_data(fr, &e));
            if (check_error(e) != 0) {
                fprintf(stderr, "Failed getting color frame\n");
                return 1;
            }

            int x, y;
            for (y = 0; y < cColorH; y++)
            {
                for (x = 0; x < cColorW; x++)
                {
                    col[y * cColorW + x].r = col_buf[3 * (y * cColorW + x) + 0];
                    col[y * cColorW + x].g = col_buf[3 * (y * cColorW + x) + 1];
                    col[y * cColorW + x].b = col_buf[3 * (y * cColorW + x) + 2];
                    col[y * cColorW + x].a = 255;
                }
            }

            *got_col = 1;
        }

        rs2_release_frame(fr);
    }

    rs2_release_frame(frames);
    return 0;
}

int8_t setPreset(const char* new_preset, struct RS_State* rs_state)
{
    rs2_error* e = NULL;

    int sensor;
    int8_t done = 0;

    for (sensor = 0; sensor < rs_state->sensor_list_count; sensor++)
    {
        if (done == 1)
            break;

        rs2_sensor* sen = rs2_create_sensor(rs_state->sensor_list, sensor, &e);

        if (check_error(e) != 0) {
            fprintf(stderr, "Failed creating temp sensor object %d / %d\n", sensor, rs_state->sensor_list_count);
            return 1;
        }

        int supports = rs2_supports_option((const rs2_options*)sen, RS2_OPTION_VISUAL_PRESET, &e);

        if (check_error(e) != 0) {
            fprintf(stderr, "Failed asking if sensor supports RS2_OPTION_VISUAL_PRESET\n");
            rs2_delete_sensor(sen);
            return 1;
        }

        if (supports == 1)
        {
            float pres = rs2_get_option((const rs2_options*)sen, RS2_OPTION_VISUAL_PRESET, &e);
            if (check_error(e) != 0) {
                fprintf(stderr, "Failed getting RS2_OPTION_VISUAL_PRESET\n");
                rs2_delete_sensor(sen);
                return 1;
            }

            const char* preset_desc = rs2_get_option_value_description((const rs2_options*)sen, RS2_OPTION_VISUAL_PRESET, pres, &e);
            if (check_error(e) != 0) {
                fprintf(stderr, "Failed getting RS2_OPTION_VISUAL_PRESET description\n");
                rs2_delete_sensor(sen);
                return 1;
            }

            if (strcmp(preset_desc, new_preset) == 0)
            {
                fprintf(stderr, "already using preset: %s\n", preset_desc);
                done = 1;
                continue;
            }

            float min, max, step, def;
            rs2_get_option_range((const rs2_options*)sen, RS2_OPTION_VISUAL_PRESET, &min, &max, &step, &def, &e);
            if (check_error(e) != 0) {
                fprintf(stderr, "Failed getting RS2_OPTION_VISUAL_PRESET ranges\n");
                rs2_delete_sensor(sen);
                return 1;
            }

            int r;
            for (r = (int)min; r < (int)max; r++)
            {
                if (done == 1)
                    break;

                preset_desc = rs2_get_option_value_description((const rs2_options*)sen, RS2_OPTION_VISUAL_PRESET, r, &e);
                if (check_error(e) != 0) {
                    fprintf(stderr, "Failed getting RS2_OPTION_VISUAL_PRESET description\n");
                    rs2_delete_sensor(sen);
                    return 1;
                }

                if (strcmp(preset_desc, new_preset) == 0)
                {
                    fprintf(stderr, "Changing preset to %s\n", preset_desc);
                    rs2_set_option((const rs2_options*)sen, RS2_OPTION_VISUAL_PRESET, r, &e);
                    if (check_error(e) != 0) {
                        fprintf(stderr, "Failed setting RS2_OPTION_VISUAL_PRESET\n");
                        rs2_delete_sensor(sen);
                        return 1;
                    }

                    pres = rs2_get_option((const rs2_options*)sen, RS2_OPTION_VISUAL_PRESET, &e);
                    if (check_error(e) != 0) {
                        fprintf(stderr, "Failed getting RS2_OPTION_VISUAL_PRESET\n");
                        rs2_delete_sensor(sen);
                        return 1;
                    }

                    if ((int)pres != (int)r) {
                        fprintf(stderr, "Setting RS2_OPTION_VISUAL_PRESET did not change preset\n");
                        rs2_delete_sensor(sen);
                        return 1;
                    }

                    done = 1;
                }
            }
        }

        rs2_delete_sensor(sen);
    }

    if (done != 1)
    {
        fprintf(stderr, "Did not find preset to use %s\n", new_preset);
        return 1;
    }

    return 0;
}

int main(int argc,  char** argv)
{
    SDL_SetMainReady();

    if (SDL_Init(SDL_INIT_VIDEO) != 0) {
        fprintf(stderr, "Failed initting SDLL: %s\n", SDL_GetError());
        return 1;
    }

    SDL_Window* sdlwin = SDL_CreateWindow("rs2", 510, 510, cDepthW, cDepthH, SDL_WINDOW_SHOWN);
    if (sdlwin == NULL)
    {
        fprintf(stderr, "failed creating SDL window\n");
        SDL_Quit();
        return 1;
    }

    SDL_Renderer* sdlren = SDL_CreateRenderer(sdlwin, -1, SDL_RENDERER_ACCELERATED | SDL_RENDERER_PRESENTVSYNC);
    if (sdlren == NULL)
    {
        fprintf(stderr, "Failed creating SDL renderer: %s\n", SDL_GetError());
        SDL_DestroyWindow(sdlwin);
        SDL_Quit();
        return 1;
    }

    struct RS_State rs_state;
    memset(&rs_state, 0, sizeof(rs_state));

    if (initialize(&rs_state) != 0)
        return 1;

    if (startSensor(&rs_state) != 0)
        return 1;

    uint16_t* dep;
    struct RGBA* dep_rgb;
    struct RGBA* col;

    const int dep_bytes = cDepthW * cDepthH * sizeof(uint16_t);
    const int dep_bytes_rgb = cDepthW * cDepthH * sizeof(struct RGBA);
    const int col_bytes = cColorW * cColorH * sizeof(struct RGBA);

    dep = (uint16_t*)malloc(dep_bytes);
    dep_rgb = (struct RGBA*)malloc(dep_bytes_rgb);
    col = (struct RGBA*)malloc(col_bytes);

    memset(dep, 0, dep_bytes);
    memset(col, 0, col_bytes);

#ifdef RENDER_DEPTH
    SDL_Surface* surf = SDL_CreateRGBSurfaceFrom((void*)dep_rgb, cDepthW, cDepthH, 32, cDepthW*3,
                             0x000000FF, 0x0000FF00, 0x00FF0000, 0xFF000000);
#else
    SDL_Surface* surf = SDL_CreateRGBSurfaceFrom((void*)col, cColorW, cColorH, 32, cColorW*3,
                             0x000000FF, 0x0000FF00, 0x00FF0000, 0xFF000000);
#endif

    if (surf == NULL)
    {
        fprintf(stderr, "Failed creating surface: %s\n", SDL_GetError());
        SDL_DestroyRenderer(sdlren);
        SDL_DestroyWindow(sdlwin);
        SDL_Quit();
        return 1;
    }

    SDL_Texture* tex = SDL_CreateTextureFromSurface(sdlren, surf);

    if (tex == NULL)
    {
        fprintf(stderr, "Failed creating sdl texture: %s\n", SDL_GetError());
        SDL_FreeSurface(surf);
        SDL_DestroyRenderer(sdlren);
        SDL_DestroyWindow(sdlwin);
        SDL_Quit();
        return 1;
    }


    uint32_t format;
    int access;
    int w;
    int h;
    if (SDL_QueryTexture(tex, &format, &access, &w, &h) != 0)
    {
        fprintf(stderr, "query tex failed: %s\n", SDL_GetError());
        SDL_DestroyTexture(tex);
        SDL_FreeSurface(surf);
        SDL_DestroyRenderer(sdlren);
        SDL_DestroyWindow(sdlwin);
        SDL_Quit();
        return 1;
    }

    // This should be ARGB
    fprintf(stderr, "format: %s\n", SDL_GetPixelFormatName(format));

    int count = 0;
    int preset_index = 0;

    int8_t got_dep = 0;
    int8_t got_col = 0;

    int8_t running = 1;

    while (running == 1)
    {
        if (update(&rs_state, dep, dep_rgb, col, &got_dep, &got_col) != 0) {
            running = 0;
            continue;
        }

        count++;
        if (count % 15 == 0)
            fprintf(stderr, "%d\n", count);

        if (count % 100 == 0)
        {
            preset_index++;
            if (preset_index >= PRESET_COUNT)
                preset_index = 0;
            if (setPreset(presets[preset_index], &rs_state) != 0)
            {
                running = 0;
            }
        }

#ifdef RENDER_DEPTH
        if (SDL_UpdateTexture(tex, NULL, (void*)dep_rgb, cDepthW * 4) != 0)
#else
        if (SDL_UpdateTexture(tex, NULL, (void*)col, cColorW * 4) != 0)
#endif
        {
            fprintf(stderr, "Failed updating texture: %s\n", SDL_GetError());
            running = 0;
            continue;
        }

        SDL_RenderClear(sdlren);
        SDL_RenderCopy(sdlren, tex, NULL, NULL);
        SDL_RenderPresent(sdlren);
    }

    clear_state(&rs_state);

    free(dep);
    free(dep_rgb);
    free(col);

    SDL_DestroyTexture(tex);
    SDL_FreeSurface(surf);
    SDL_DestroyRenderer(sdlren);
    SDL_DestroyWindow(sdlwin);
    SDL_Quit();
    return 0;
}
