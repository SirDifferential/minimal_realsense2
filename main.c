#include <librealsense2/rs.h>
#include <librealsense2/rs_advanced_mode.h>
#include <librealsense2/h/rs_pipeline.h>
#include <librealsense2/h/rs_option.h>
#include <librealsense2/h/rs_frame.h>
#include <librealsense2/rsutil.h>

#define SDL_MAIN_HANDLED
#ifdef WIN32
#include <SDL.h>
#include <Windows.h>
#else
#include <SDL2/SDL.h>
#include <unistd.h>
#include <signal.h>
#endif

#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

int8_t got_sigint = 0;

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

#define RS_STATE_SENSORS_MAX 20
struct RS_State
{
    rs2_context* ctx;
    rs2_device_list* device_list;
    int32_t dev_count;
    rs2_device* dev;
    rs2_sensor_list* sensor_list;
    int32_t sensor_list_count;
    rs2_sensor* sensors[RS_STATE_SENSORS_MAX];
    int32_t sensors_created;
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


int8_t create_context(struct RS_State* rs_state)
{
    rs2_error* e = NULL;

    fprintf(stderr, "creating context\n");

    rs_state->ctx = rs2_create_context(RS2_API_VERSION, &e);
    if (check_error(e) != 0) {
        rs_state->ctx = NULL;
        fprintf(stderr, "Failed creating rs context\n");
        return 1;
    }

    fprintf(stderr, "context created\n");

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

    int32_t sen;
    for (sen = 0; sen < s->sensors_created; sen++) {
        rs2_delete_sensor(s->sensors[sen]);
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

int8_t ensure_device(struct RS_State* s, int rs_dev_index)
{
    if (s == NULL) {
        fprintf(stderr, "Cannot init state: given pointer is null\n");
        return 1;
    }

    rs2_error* e = NULL;
    if (s->ctx == NULL) {
        fprintf(stderr, "Cannot ensure device: context is null\n");
        return 1;
    }

    if (s->device_list != NULL) {
        rs2_delete_device_list(s->device_list);
        s->device_list = NULL;
    }

    s->dev_count = 0;

    if (s->dev != NULL) {
        rs2_delete_device(s->dev);
        s->dev = NULL;
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

int8_t set_preset(struct RS_State* s, const char* new_preset)
{
    int done = 0;
    int sensor;
    rs2_error* e = NULL;

    for (sensor = 0; sensor < s->sensor_list_count && sensor < s->sensors_created; sensor++)
    {
        rs2_sensor* sen = s->sensors[sensor];
        int supports = rs2_supports_option((const rs2_options*)sen, RS2_OPTION_VISUAL_PRESET, &e);

        if (check_error(e) != 0) {
            fprintf(stderr, "Failed asking if sensor supports RS2_OPTION_VISUAL_PRESET\n");
            return 1;
        }

        if (supports == 1)
        {
            float pres = rs2_get_option((const rs2_options*)sen, RS2_OPTION_VISUAL_PRESET, &e);
            if (check_error(e) != 0) {
                fprintf(stderr, "Failed getting RS2_OPTION_VISUAL_PRESET\n");
                return 1;
            }

            const char* preset_desc = rs2_get_option_value_description((const rs2_options*)sen, RS2_OPTION_VISUAL_PRESET, pres, &e);
            if (check_error(e) != 0) {
                fprintf(stderr, "Failed getting RS2_OPTION_VISUAL_PRESET description\n");
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
                        return 1;
                    }

                    done = 1;
                }
            }
        }
    }

    if (done == 0) {
        fprintf(stderr, "Did not find preset: %s\n", new_preset);
        return 1;
    }

    return 0;
}

int8_t create_streams(struct RS_State* s)
{
    if (s == NULL) {
        fprintf(stderr, "Cannot init streaming: given pointer is null\n");
        return 1;
    }

    if (s->pipe != NULL) {
        rs2_delete_pipeline(s->pipe);
        s->pipe = NULL;
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

    fprintf(stderr, "Depth stream created\n");

    rs2_config_enable_stream(s->config, RS2_STREAM_COLOR, -1, cColorW, cColorH, RS2_FORMAT_RGB8, 30, &e);
    if (check_error(e) != 0) {
        fprintf(stderr, "Failed initting color streaming\n");
        return 1;
    }

    fprintf(stderr, "Color stream created\n");

    return 0;
}

int8_t start_stream(struct RS_State* s, int preset_index)
{
    rs2_error* e = NULL;

    if (s == NULL) {
        fprintf(stderr, "Cannot star t stream: given pointer is null\n");
        return 1;
    }

    if (s->selection) {
        rs2_delete_pipeline_profile(s->selection);
        s->selection = NULL;
    }

    s->selection = rs2_config_resolve(s->config, s->pipe, &e);
    if (check_error(e) != 0) {
        fprintf(stderr, "Failed resolving config\n");
        s->selection = NULL;
        return 1;
    }

    if (s->device_list != NULL) {
        rs2_delete_device_list(s->device_list);
        s->device_list = 0;
    }

    s->dev_count = 0;

    if (s->dev) {
        rs2_delete_device(s->dev);
        s->dev = NULL;
    }

    s->dev = rs2_pipeline_profile_get_device(s->selection, &e);
    if (check_error(e) != 0) {
        fprintf(stderr, "Failed getting device for pipeline profile\n");
        s->dev = NULL;
        return 1;
    }

    if (s->sensor_list) {
        rs2_delete_sensor_list(s->sensor_list);
        s->sensor_list = NULL;
    }

    if (s->sensor_list_count > 0) {
        int sensor;
        for (sensor = 0; sensor < s->sensor_list_count; sensor++) {
            rs2_delete_sensor(s->sensors[sensor]);
            s->sensors[sensor] = NULL;
        }
        s->sensor_list_count = 0;
    }

    s->sensor_list = rs2_query_sensors(s->dev, &e);
    if (check_error(e) != 0) {
        fprintf(stderr, "Failed querying for sensors\n");
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
    for (sensor = 0; sensor < s->sensor_list_count; sensor++) {
        s->sensors[sensor] = rs2_create_sensor(s->sensor_list, sensor, &e);
        if (check_error(e) != 0) {
            fprintf(stderr, "Failed creating sensor %d / %d\n", sensor, s->sensor_list_count);
            s->sensors[sensor] = NULL;
            return 1;
        }

        s->sensors_created++;
    }

    if (set_preset(s, presets[preset_index]) != 0)
        return 1;

    fprintf(stderr, "Preset changed");

    rs2_pipeline_start_with_config(s->pipe, s->config, &e);
    if (check_error(e) != 0) {
        fprintf(stderr, "Failed starting pipeline\n");
        return 1;
    }

    fprintf(stderr, "pipeline started\n");


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

    fprintf(stderr, "stream list count: %d\n", s->stream_list_count);

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

int8_t ensure_advanced(struct RS_State* s)
{
    while (s->advanced_enabled == 0)
    {
        fprintf(stderr, "Waiting for advanced mode\n");
        if (clear_state(s) != 0) {
            fprintf(stderr, "Failed clearing state while waiting for advanced mode\n");
            return 1;
        }

        if (create_context(s) != 0) {
            fprintf(stderr, "Failed initializing context while waiting for advanced mode\n");
            return 1;
        }

        if (ensure_device(s, 0) != 0) {
            fprintf(stderr, "Failed creating device when waiting for advanced mode\n");
            return 1;
        }

        if (set_advanced(s, 1) != 0) {
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

    // Once the device is properly in advanced mode, clear everything
    // The device can then be re-opened with whatever parameters are required
    if (clear_state(s) != 0)
        return 1;

    // Apparently the device needs a while between clearing the state and starting devices again
#ifdef WIN32
        Sleep(1000);
#else
        usleep(1000 * 1000);
#endif

    return 0;
}

int8_t start_sensor(struct RS_State* rs_state, int dev_index, int preset_index)
{
    rs2_error* e = NULL;

    if (rs_state->ctx == NULL) {
        if (create_context(rs_state) != 0)  {
            fprintf(stderr, "Failed creating context when starting sensor\n");
            return 1;
        }
    }

    if (create_streams(rs_state) != 0)
    {
        fprintf(stderr, "Failed initting streams\n");
        return 1;
    }

    fprintf(stderr, "streams created\n");

    if (start_stream(rs_state, preset_index) != 0)
    {
        fprintf(stderr, "Failed starting streams\n");
        return 1;
    }

    fprintf(stderr, "streams started\n");

    int stream;
    float fov[2];
    float rgb_fov[2];
    for (stream = 0; stream < rs_state->stream_list_count; stream++)
    {
        rs2_stream str;
        rs2_format format;
        int index;
        int id;
        int fps;

        const rs2_stream_profile* prof = rs2_get_stream_profile(rs_state->stream_list, stream, &e);
        if (check_error(e) != 0) {
            fprintf(stderr, "Failed getting stream profile: %d / %d\n", stream, rs_state->stream_list_count);
            return 1;
        }

        rs2_get_stream_profile_data(prof, &str, &format, &index, &id, &fps, &e);

        if (check_error(e) != 0) {
            fprintf(stderr, "Failed getting stream profile data for stream: %d / %d\n", stream, rs_state->stream_list_count);
            return 1;
        }

        rs2_intrinsics intrinsics;
        if (str == RS2_STREAM_DEPTH)
        {
            rs2_get_video_stream_intrinsics(prof, &intrinsics, &e);
            if (check_error(e) != 0) {
                fprintf(stdout, "Failed getting depth stream intrinsics\n");
                return 1;
            }

            rs2_fov(&intrinsics, fov);
            fprintf(stderr, "Started depth stream, fov %f, %f\n", fov[0], fov[1]);
        }
        else if (str == RS2_STREAM_COLOR)
        {
            rs2_get_video_stream_intrinsics(prof, &intrinsics, &e);
            if (check_error(e) != 0) {
                fprintf(stderr, "Failed getting color stream intrinsics\n");
                return 1;
            }

            rs2_fov(&intrinsics, rgb_fov);
            fprintf(stderr, "Started color stream, fov %f, %f\n", rgb_fov[0], rgb_fov[1]);
        }
    }

    return 0;
}

uint16_t lerp(uint16_t a, uint16_t b, float alpha)
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

#ifdef WIN32
int8_t sigint_handler(DWORD fdwCtrlType) {
    if(fdwCtrlType == CTRL_C_EVENT) {
        fprintf(stderr, "got signal: %d\n", fdwCtrlType);
        got_sigint = 1;
        return 0;
    }

    return 1;
}
#else
void sigint_handler(int sig)
{
    fprintf(stderr, "got sigint\n");
    got_sigint = 1;
}
#endif

int main(int argc,  char** argv)
{
#ifdef WIN32
    SetConsoleCtrlHandler( (PHANDLER_ROUTINE) sigint_handler, TRUE );
#else
    signal(SIGINT, sigint_handler);
#endif

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

    fprintf(stderr, "Ensuring advanced mode is enabled\n");

    if (ensure_advanced(&rs_state) != 0)
    {
        fprintf(stderr, "Ensuring advanced mode failed\n");
        return 1;
    }

    fprintf(stderr, "Starting sensor\n");

    if (start_sensor(&rs_state, 0, 0) != 0)
        return 1;

    fprintf(stderr, "Sensor started\n");

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
    memset(dep_rgb, 0, dep_bytes_rgb);
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
            fprintf(stderr, "sensor update failed\n");
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

            if (clear_state(&rs_state)) {
                running = 0;
                continue;
            }

            if (start_sensor(&rs_state, 0, preset_index) != 0) {
                running = 0;
                continue;
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

        if (got_sigint != 0)
            running = 0;
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
