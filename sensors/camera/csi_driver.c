/*******************************************************************************
 * Cule OS - CSI Camera Driver (IMX477/IMX219)
 * High-performance camera driver using GStreamer for Jetson platforms
 * Supports: 30FPS @ 1080p, stereo vision, hardware-accelerated encoding
 ******************************************************************************/

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <sys/ioctl.h>
#include <sys/mman.h>
#include <linux/videodev2.h>
#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <pthread.h>
#include <time.h>

#include "../../include/cule/kernel.h"
#include "csi_driver.h"

/* Camera sensor types */
typedef enum {
    SENSOR_IMX477 = 0,
    SENSOR_IMX219 = 1,
    SENSOR_UNKNOWN = 2
} csi_sensor_t;

/* Camera configuration */
typedef struct {
    uint32_t width;
    uint32_t height;
    uint32_t fps;
    uint32_t format;  /* V4L2_PIX_FMT_* */
    csi_sensor_t sensor;
    int camera_id;
    gboolean stereo_mode;
    gboolean hardware_encode;
} csi_config_t;

/* GStreamer pipeline state */
typedef struct {
    GstElement *pipeline;
    GstElement *source;
    GstElement *capsfilter;
    GstElement *convert;
    GstElement *sink;
    GstBus *bus;
    GMainLoop *loop;
    
    /* Appsink for raw frame access */
    GstElement *appsink;
    GstSample *last_sample;
    
    /* Frame buffers */
    uint8_t *frame_buffer;
    uint32_t buffer_size;
    pthread_mutex_t buffer_mutex;
    
    /* Timing */
    struct timespec last_frame_time;
    uint64_t frame_count;
    float actual_fps;
    
    /* State */
    gboolean running;
    gboolean paused;
    
    /* Calibration */
    camera_calibration_t calibration;
} csi_pipeline_t;

/* CSI camera device */
typedef struct {
    int fd;
    char device_path[32];
    csi_config_t config;
    csi_pipeline_t pipeline;
    
    /* V4L2 controls */
    struct v4l2_capability cap;
    struct v4l2_format fmt;
    struct v4l2_requestbuffers req;
    
    /* Threading */
    pthread_t capture_thread;
    pthread_mutex_t state_mutex;
    
    /* Callbacks */
    frame_callback_t frame_callback;
    void *user_data;
} csi_camera_t;

/* Stereo camera pair */
typedef struct {
    csi_camera_t left;
    csi_camera_t right;
    stereo_calibration_t stereo_calib;
    pthread_mutex_t sync_mutex;
    frame_pair_t last_pair;
    uint64_t sync_frame_count;
} stereo_camera_t;

/* Module state */
static struct {
    csi_camera_t cameras[MAX_CSI_CAMERAS];
    stereo_camera_t stereo_pairs[MAX_STEREO_PAIRS];
    int camera_count;
    int stereo_count;
    pthread_mutex_t module_mutex;
    gboolean gst_initialized;
} csi_module = {0};

/* IMX477 native modes */
static const csi_mode_t imx477_modes[] = {
    /* 12MP full resolution */
    {4056, 3040, 10, 10.0f, "12MP"},
    /* 4K UHD cropped */
    {3840, 2160, 10, 60.0f, "4K"},
    /* 1080p for 30FPS target */
    {1920, 1080, 10, 60.0f, "1080p"},
    /* 720p high speed */
    {1280, 720, 10, 120.0f, "720p"},
};

/* IMX219 native modes */
static const csi_mode_t imx219_modes[] = {
    /* 8MP full resolution */
    {3280, 2464, 10, 21.0f, "8MP"},
    /* 1080p */
    {1920, 1080, 10, 30.0f, "1080p"},
    /* 720p high speed */
    {1280, 720, 10, 60.0f, "720p"},
    /* VGA high speed */
    {640, 480, 10, 90.0f, "VGA"},
};

/*******************************************************************************
 * GStreamer Pipeline Construction
 ******************************************************************************/

static gchar* build_sensor_string(csi_sensor_t sensor, int camera_id) {
    const char *sensor_name;
    
    switch (sensor) {
        case SENSOR_IMX477:
            sensor_name = "imx477";
            break;
        case SENSOR_IMX219:
            sensor_name = "imx219";
            break;
        default:
            sensor_name = "imx477";
            break;
    }
    
    /* nvarguscamerasrc for Jetson with specific sensor */
    return g_strdup_printf(
        "nvarguscamerasrc sensor-id=%d "
        "! video/x-raw(memory:NVMM), width=%d, height=%d, "
        "format=NV12, framerate=%d/1",
        camera_id,
        1920, 1080, 30
    );
}

static gchar* build_v4l2_string(const char *device, csi_config_t *config) {
    return g_strdup_printf(
        "v4l2src device=%s io-mode=2 "
        "! video/x-raw, width=%d, height=%d, "
        "format=UYVY, framerate=%d/1",
        device, config->width, config->height, config->fps
    );
}

static int create_gstreamer_pipeline(csi_camera_t *cam) {
    csi_pipeline_t *pipe = &cam->pipeline;
    gchar *pipeline_str = NULL;
    GError *error = NULL;
    
    /* Initialize GStreamer if needed */
    if (!csi_module.gst_initialized) {
        gst_init(NULL, NULL);
        csi_module.gst_initialized = TRUE;
    }
    
    /* Build pipeline string based on camera type */
    if (cam->config.sensor == SENSOR_IMX477 || cam->config.sensor == SENSOR_IMX219) {
        /* Jetson native CSI via nvarguscamerasrc */
        gchar *src = build_sensor_string(cam->config.sensor, cam->config.camera_id);
        
        if (cam->config.hardware_encode) {
            /* Hardware accelerated H.264 encoding */
            pipeline_str = g_strdup_printf(
                "%s ! nvvidconv ! nvv4l2h264enc bitrate=8000000 "
                "! h264parse ! appsink name=sink max-buffers=1 drop=true",
                src
            );
        } else {
            /* Raw output via appsink for CV processing */
            pipeline_str = g_strdup_printf(
                "%s ! nvvidconv "
                "! video/x-raw, format=BGRx, width=%d, height=%d "
                "! videoconvert ! video/x-raw, format=BGR "
                "! appsink name=sink max-buffers=2 drop=true emit-signals=true",
                src, cam->config.width, cam->config.height
            );
        }
        g_free(src);
    } else {
        /* Fallback to V4L2 */
        gchar *src = build_v4l2_string(cam->device_path, &cam->config);
        pipeline_str = g_strdup_printf(
            "%s ! videoconvert ! video/x-raw, format=BGR "
            "! appsink name=sink max-buffers=2 drop=true emit-signals=true",
            src
        );
        g_free(src);
    }
    
    g_print("CSI Pipeline: %s\n", pipeline_str);
    
    /* Parse and create pipeline */
    pipe->pipeline = gst_parse_launch(pipeline_str, &error);
    g_free(pipeline_str);
    
    if (error != NULL) {
        g_printerr("Pipeline creation error: %s\n", error->message);
        g_error_free(error);
        return -1;
    }
    
    /* Get appsink */
    pipe->appsink = gst_bin_get_by_name(GST_BIN(pipe->pipeline), "sink");
    if (!pipe->appsink) {
        g_printerr("Failed to get appsink\n");
        gst_object_unref(pipe->pipeline);
        return -1;
    }
    
    /* Configure appsink */
    g_object_set(pipe->appsink,
        "emit-signals", TRUE,
        "max-buffers", 2,
        "drop", TRUE,
        NULL);
    
    /* Allocate frame buffer */
    pipe->buffer_size = cam->config.width * cam->config.height * 3; /* BGR */
    pipe->frame_buffer = (uint8_t*)aligned_alloc(64, pipe->buffer_size);
    if (!pipe->frame_buffer) {
        g_printerr("Failed to allocate frame buffer\n");
        return -1;
    }
    
    pthread_mutex_init(&pipe->buffer_mutex, NULL);
    
    return 0;
}

/*******************************************************************************
 * Frame Capture Thread
 ******************************************************************************/

static GstFlowReturn on_new_sample(GstAppSink *sink, gpointer user_data) {
    csi_camera_t *cam = (csi_camera_t*)user_data;
    csi_pipeline_t *pipe = &cam->pipeline;
    GstSample *sample;
    GstBuffer *buffer;
    GstMapInfo map;
    
    sample = gst_app_sink_pull_sample(sink);
    if (!sample) {
        return GST_FLOW_ERROR;
    }
    
    buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }
    
    if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        pthread_mutex_lock(&pipe->buffer_mutex);
        
        /* Copy frame data */
        size_t copy_size = MIN(map.size, pipe->buffer_size);
        memcpy(pipe->frame_buffer, map.data, copy_size);
        
        /* Update timing */
        clock_gettime(CLOCK_MONOTONIC, &pipe->last_frame_time);
        pipe->frame_count++;
        
        /* Calculate actual FPS */
        static struct timespec last_calc = {0};
        if (last_calc.tv_sec > 0) {
            struct timespec now;
            clock_gettime(CLOCK_MONOTONIC, &now);
            float dt = (now.tv_sec - last_calc.tv_sec) +
                      (now.tv_nsec - last_calc.tv_nsec) / 1e9f;
            if (dt >= 1.0f) {
                pipe->actual_fps = pipe->frame_count / dt;
                last_calc = now;
                pipe->frame_count = 0;
            }
        } else {
            last_calc = pipe->last_frame_time;
        }
        
        pthread_mutex_unlock(&pipe->buffer_mutex);
        
        /* Call user callback if registered */
        if (cam->frame_callback) {
            frame_data_t frame = {
                .data = pipe->frame_buffer,
                .width = cam->config.width,
                .height = cam->config.height,
                .format = FRAME_FORMAT_BGR,
                .timestamp = pipe->last_frame_time.tv_sec * 1000000000ULL + 
                            pipe->last_frame_time.tv_nsec,
                .frame_number = pipe->frame_count
            };
            cam->frame_callback(&frame, cam->user_data);
        }
        
        gst_buffer_unmap(buffer, &map);
    }
    
    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

static GstAppSinkCallbacks appsink_callbacks = {
    .eos = NULL,
    .new_preroll = NULL,
    .new_sample = on_new_sample
};

static void* capture_thread(void *arg) {
    csi_camera_t *cam = (csi_camera_t*)arg;
    csi_pipeline_t *pipe = &cam->pipeline;
    GstBus *bus;
    GstMessage *msg;
    
    /* Set up appsink callbacks */
    gst_app_sink_set_callbacks(GST_APP_SINK(pipe->appsink), 
                               &appsink_callbacks, cam, NULL);
    
    /* Set pipeline to playing */
    gst_element_set_state(pipe->pipeline, GST_STATE_PLAYING);
    pipe->running = TRUE;
    
    /* Message loop */
    bus = gst_element_get_bus(pipe->pipeline);
    
    while (pipe->running) {
        msg = gst_bus_timed_pop_filtered(bus, 100 * GST_MSECOND,
            GST_MESSAGE_ERROR | GST_MESSAGE_EOS | GST_MESSAGE_STATE_CHANGED);
        
        if (msg) {
            switch (GST_MESSAGE_TYPE(msg)) {
                case GST_MESSAGE_ERROR: {
                    GError *err = NULL;
                    gchar *debug = NULL;
                    gst_message_parse_error(msg, &err, &debug);
                    g_printerr("CSI Camera error: %s\n", err->message);
                    g_error_free(err);
                    g_free(debug);
                    pipe->running = FALSE;
                    break;
                }
                case GST_MESSAGE_EOS:
                    g_print("CSI Camera: End of stream\n");
                    pipe->running = FALSE;
                    break;
                default:
                    break;
            }
            gst_message_unref(msg);
        }
    }
    
    gst_object_unref(bus);
    return NULL;
}

/*******************************************************************************
 * Public API Implementation
 ******************************************************************************/

int csi_camera_init(void) {
    pthread_mutex_init(&csi_module.module_mutex, NULL);
    
    /* Initialize GStreamer */
    if (!gst_is_initialized()) {
        gst_init(NULL, NULL);
    }
    csi_module.gst_initialized = TRUE;
    
    g_print("CSI Camera module initialized\n");
    return 0;
}

void csi_camera_cleanup(void) {
    int i;
    
    pthread_mutex_lock(&csi_module.module_mutex);
    
    /* Stop all cameras */
    for (i = 0; i < csi_module.camera_count; i++) {
        csi_camera_stop(&csi_module.cameras[i]);
    }
    
    pthread_mutex_unlock(&csi_module.module_mutex);
    pthread_mutex_destroy(&csi_module.module_mutex);
    
    if (csi_module.gst_initialized) {
        gst_deinit();
    }
    
    g_print("CSI Camera module cleaned up\n");
}

csi_camera_t* csi_camera_open(int camera_id, csi_config_t *config) {
    csi_camera_t *cam = NULL;
    char device_path[32];
    int fd;
    
    pthread_mutex_lock(&csi_module.module_mutex);
    
    if (csi_module.camera_count >= MAX_CSI_CAMERAS) {
        pthread_mutex_unlock(&csi_module.module_mutex);
        return NULL;
    }
    
    cam = &csi_module.cameras[csi_module.camera_count++];
    memset(cam, 0, sizeof(csi_camera_t));
    
    pthread_mutex_unlock(&csi_module.module_mutex);
    
    /* Setup camera */
    cam->config = *config;
    cam->config.camera_id = camera_id;
    snprintf(cam->device_path, sizeof(cam->device_path), 
             "/dev/video%d", camera_id);
    
    pthread_mutex_init(&cam->state_mutex, NULL);
    
    /* Try to open V4L2 device for control access */
    fd = open(cam->device_path, O_RDWR | O_NONBLOCK);
    if (fd < 0) {
        g_print("Warning: Could not open %s: %s\n", 
                cam->device_path, strerror(errno));
        /* Continue anyway - GStreamer might still work */
    } else {
        /* Query capabilities */
        if (ioctl(fd, VIDIOC_QUERYCAP, &cam->cap) < 0) {
            g_print("Warning: VIDIOC_QUERYCAP failed: %s\n", strerror(errno));
        } else {
            g_print("CSI Camera: %s\n", cam->cap.card);
        }
        close(fd);
    }
    
    /* Create GStreamer pipeline */
    if (create_gstreamer_pipeline(cam) < 0) {
        pthread_mutex_destroy(&cam->state_mutex);
        pthread_mutex_lock(&csi_module.module_mutex);
        csi_module.camera_count--;
        pthread_mutex_unlock(&csi_module.module_mutex);
        return NULL;
    }
    
    g_print("CSI Camera %d opened: %dx%d@%dfps\n",
            camera_id, config->width, config->height, config->fps);
    
    return cam;
}

int csi_camera_start(csi_camera_t *cam) {
    if (!cam || cam->pipeline.running) {
        return -1;
    }
    
    pthread_mutex_lock(&cam->state_mutex);
    
    /* Start capture thread */
    if (pthread_create(&cam->capture_thread, NULL, capture_thread, cam) != 0) {
        pthread_mutex_unlock(&cam->state_mutex);
        return -1;
    }
    
    pthread_mutex_unlock(&cam->state_mutex);
    
    g_print("CSI Camera %d started\n", cam->config.camera_id);
    return 0;
}

int csi_camera_stop(csi_camera_t *cam) {
    if (!cam || !cam->pipeline.running) {
        return -1;
    }
    
    pthread_mutex_lock(&cam->state_mutex);
    
    cam->pipeline.running = FALSE;
    
    /* Stop pipeline */
    if (cam->pipeline.pipeline) {
        gst_element_set_state(cam->pipeline.pipeline, GST_STATE_NULL);
    }
    
    pthread_mutex_unlock(&cam->state_mutex);
    
    /* Wait for capture thread */
    pthread_join(cam->capture_thread, NULL);
    
    g_print("CSI Camera %d stopped\n", cam->config.camera_id);
    return 0;
}

int csi_camera_close(csi_camera_t *cam) {
    csi_pipeline_t *pipe;
    
    if (!cam) {
        return -1;
    }
    
    /* Stop if running */
    if (cam->pipeline.running) {
        csi_camera_stop(cam);
    }
    
    pipe = &cam->pipeline;
    
    /* Clean up GStreamer */
    if (pipe->appsink) {
        gst_object_unref(pipe->appsink);
    }
    if (pipe->pipeline) {
        gst_object_unref(pipe->pipeline);
    }
    
    /* Free buffer */
    if (pipe->frame_buffer) {
        free(pipe->frame_buffer);
    }
    
    pthread_mutex_destroy(&pipe->buffer_mutex);
    pthread_mutex_destroy(&cam->state_mutex);
    
    g_print("CSI Camera %d closed\n", cam->config.camera_id);
    return 0;
}

int csi_camera_get_frame(csi_camera_t *cam, frame_data_t *frame, uint32_t timeout_ms) {
    csi_pipeline_t *pipe;
    struct timespec start, now;
    int ret = -1;
    
    if (!cam || !frame) {
        return -1;
    }
    
    pipe = &cam->pipeline;
    
    clock_gettime(CLOCK_MONOTONIC, &start);
    
    while (1) {
        pthread_mutex_lock(&pipe->buffer_mutex);
        
        if (pipe->frame_count > 0) {
            /* Copy frame data */
            frame->width = cam->config.width;
            frame->height = cam->config.height;
            frame->format = FRAME_FORMAT_BGR;
            frame->timestamp = pipe->last_frame_time.tv_sec * 1000000000ULL + 
                              pipe->last_frame_time.tv_nsec;
            frame->frame_number = pipe->frame_count;
            
            /* User must provide buffer or use callback */
            frame->data = pipe->frame_buffer;
            
            pthread_mutex_unlock(&pipe->buffer_mutex);
            ret = 0;
            break;
        }
        
        pthread_mutex_unlock(&pipe->buffer_mutex);
        
        /* Check timeout */
        clock_gettime(CLOCK_MONOTONIC, &now);
        uint64_t elapsed_ms = (now.tv_sec - start.tv_sec) * 1000 +
                             (now.tv_nsec - start.tv_nsec) / 1000000;
        
        if (elapsed_ms >= timeout_ms) {
            break;
        }
        
        usleep(1000); /* 1ms */
    }
    
    return ret;
}

int csi_camera_register_callback(csi_camera_t *cam, 
                                  frame_callback_t callback, 
                                  void *user_data) {
    if (!cam) {
        return -1;
    }
    
    pthread_mutex_lock(&cam->state_mutex);
    cam->frame_callback = callback;
    cam->user_data = user_data;
    pthread_mutex_unlock(&cam->state_mutex);
    
    return 0;
}

float csi_camera_get_fps(csi_camera_t *cam) {
    float fps = 0.0f;
    
    if (cam) {
        pthread_mutex_lock(&cam->pipeline.buffer_mutex);
        fps = cam->pipeline.actual_fps;
        pthread_mutex_unlock(&cam->pipeline.buffer_mutex);
    }
    
    return fps;
}

/*******************************************************************************
 * Stereo Vision Support
 ******************************************************************************/

stereo_camera_t* csi_stereo_open(int left_id, int right_id, 
                                  csi_config_t *config) {
    stereo_camera_t *stereo = NULL;
    
    pthread_mutex_lock(&csi_module.module_mutex);
    
    if (csi_module.stereo_count >= MAX_STEREO_PAIRS) {
        pthread_mutex_unlock(&csi_module.module_mutex);
        return NULL;
    }
    
    stereo = &csi_module.stereo_pairs[csi_module.stereo_count++];
    memset(stereo, 0, sizeof(stereo_camera_t));
    
    pthread_mutex_unlock(&csi_module.module_mutex);
    
    /* Open left camera */
    stereo->left = *csi_camera_open(left_id, config);
    if (!stereo->left.pipeline.pipeline) {
        pthread_mutex_lock(&csi_module.module_mutex);
        csi_module.stereo_count--;
        pthread_mutex_unlock(&csi_module.module_mutex);
        return NULL;
    }
    
    /* Open right camera */
    stereo->right = *csi_camera_open(right_id, config);
    if (!stereo->right.pipeline.pipeline) {
        csi_camera_close(&stereo->left);
        pthread_mutex_lock(&csi_module.module_mutex);
        csi_module.stereo_count--;
        pthread_mutex_unlock(&csi_module.module_mutex);
        return NULL;
    }
    
    pthread_mutex_init(&stereo->sync_mutex, NULL);
    
    /* Enable stereo mode */
    config->stereo_mode = TRUE;
    
    g_print("Stereo camera pair opened: %d (L) / %d (R)\n", left_id, right_id);
    return stereo;
}

int csi_stereo_start(stereo_camera_t *stereo) {
    int ret;
    
    if (!stereo) {
        return -1;
    }
    
    ret = csi_camera_start(&stereo->left);
    if (ret < 0) {
        return ret;
    }
    
    ret = csi_camera_start(&stereo->right);
    if (ret < 0) {
        csi_camera_stop(&stereo->left);
        return ret;
    }
    
    return 0;
}

int csi_stereo_get_frame_pair(stereo_camera_t *stereo, frame_pair_t *pair, 
                               uint32_t timeout_ms) {
    frame_data_t left_frame, right_frame;
    int ret;
    
    if (!stereo || !pair) {
        return -1;
    }
    
    /* Get synchronized frames */
    ret = csi_camera_get_frame(&stereo->left, &left_frame, timeout_ms);
    if (ret < 0) {
        return ret;
    }
    
    ret = csi_camera_get_frame(&stereo->right, &right_frame, timeout_ms);
    if (ret < 0) {
        return ret;
    }
    
    /* Check sync (frames within 5ms) */
    int64_t time_diff = (int64_t)left_frame.timestamp - (int64_t)right_frame.timestamp;
    if (time_diff < 0) time_diff = -time_diff;
    
    if (time_diff > 5000000) { /* 5ms in nanoseconds */
        g_print("Warning: Stereo frame sync mismatch: %ld us\n", time_diff / 1000);
    }
    
    pthread_mutex_lock(&stereo->sync_mutex);
    
    pair->left = left_frame;
    pair->right = right_frame;
    pair->timestamp = (left_frame.timestamp + right_frame.timestamp) / 2;
    pair->sync_quality = (time_diff < 5000000) ? 1.0f : 
                         (time_diff < 10000000) ? 0.5f : 0.0f;
    
    stereo->last_pair = *pair;
    stereo->sync_frame_count++;
    
    pthread_mutex_unlock(&stereo->sync_mutex);
    
    return 0;
}

int csi_stereo_stop(stereo_camera_t *stereo) {
    if (!stereo) {
        return -1;
    }
    
    csi_camera_stop(&stereo->right);
    csi_camera_stop(&stereo->left);
    
    return 0;
}

int csi_stereo_close(stereo_camera_t *stereo) {
    if (!stereo) {
        return -1;
    }
    
    csi_camera_close(&stereo->right);
    csi_camera_close(&stereo->left);
    
    pthread_mutex_destroy(&stereo->sync_mutex);
    
    pthread_mutex_lock(&csi_module.module_mutex);
    csi_module.stereo_count--;
    pthread_mutex_unlock(&csi_module.module_mutex);
    
    return 0;
}

/*******************************************************************************
 * Camera Calibration
 ******************************************************************************/

int csi_camera_load_calibration(csi_camera_t *cam, const char *calib_file) {
    FILE *fp;
    camera_calibration_t *calib = &cam->pipeline.calibration;
    
    fp = fopen(calib_file, "r");
    if (!fp) {
        return -1;
    }
    
    /* Read calibration data (JSON format) */
    char line[256];
    while (fgets(line, sizeof(line), fp)) {
        if (strstr(line, "\"camera_matrix\"")) {
            /* Parse 3x3 matrix */
            for (int i = 0; i < 3 && fgets(line, sizeof(line), fp); i++) {
                sscanf(line, "[%f, %f, %f]",
                       &calib->camera_matrix[i][0],
                       &calib->camera_matrix[i][1],
                       &calib->camera_matrix[i][2]);
            }
        } else if (strstr(line, "\"distortion\"")) {
            /* Parse distortion coefficients */
            fgets(line, sizeof(line), fp);
            sscanf(line, "[%f, %f, %f, %f, %f]",
                   &calib->distortion[0], &calib->distortion[1],
                   &calib->distortion[2], &calib->distortion[3],
                   &calib->distortion[4]);
        }
    }
    
    fclose(fp);
    calib->valid = TRUE;
    
    g_print("Loaded camera calibration from %s\n", calib_file);
    return 0;
}

int csi_stereo_load_calibration(stereo_camera_t *stereo, const char *calib_file) {
    FILE *fp;
    stereo_calibration_t *calib = &stereo->stereo_calib;
    
    fp = fopen(calib_file, "r");
    if (!fp) {
        return -1;
    }
    
    /* Read stereo calibration data */
    char line[256];
    while (fgets(line, sizeof(line), fp)) {
        if (strstr(line, "\"R\"")) {
            /* Parse rotation matrix */
            for (int i = 0; i < 3 && fgets(line, sizeof(line), fp); i++) {
                sscanf(line, "[%f, %f, %f]",
                       &calib->rotation[i][0],
                       &calib->rotation[i][1],
                       &calib->rotation[i][2]);
            }
        } else if (strstr(line, "\"T\"")) {
            /* Parse translation vector */
            fgets(line, sizeof(line), fp);
            sscanf(line, "[%f, %f, %f]",
                   &calib->translation[0],
                   &calib->translation[1],
                   &calib->translation[2]);
        } else if (strstr(line, "\"baseline\"")) {
            /* Baseline in meters */
            sscanf(line, "\"baseline\": %f", &calib->baseline);
        }
    }
    
    fclose(fp);
    calib->valid = TRUE;
    
    g_print("Loaded stereo calibration from %s\n", calib_file);
    return 0;
}

/*******************************************************************************
 * Utility Functions
 ******************************************************************************/

const csi_mode_t* csi_get_native_modes(csi_sensor_t sensor, int *count) {
    switch (sensor) {
        case SENSOR_IMX477:
            *count = sizeof(imx477_modes) / sizeof(imx477_modes[0]);
            return imx477_modes;
        case SENSOR_IMX219:
            *count = sizeof(imx219_modes) / sizeof(imx219_modes[0]);
            return imx219_modes;
        default:
            *count = 0;
            return NULL;
    }
}

void csi_print_camera_info(csi_camera_t *cam) {
    if (!cam) {
        return;
    }
    
    g_print("\n=== CSI Camera %d Info ===\n", cam->config.camera_id);
    g_print("Device: %s\n", cam->device_path);
    g_print("Resolution: %dx%d\n", cam->config.width, cam->config.height);
    g_print("Target FPS: %d\n", cam->config.fps);
    g_print("Actual FPS: %.1f\n", csi_camera_get_fps(cam));
    g_print("Sensor: %s\n", 
            cam->config.sensor == SENSOR_IMX477 ? "IMX477" :
            cam->config.sensor == SENSOR_IMX219 ? "IMX219" : "Unknown");
    g_print("Hardware Encode: %s\n", cam->config.hardware_encode ? "Yes" : "No");
    g_print("Calibration: %s\n", 
            cam->pipeline.calibration.valid ? "Valid" : "Not loaded");
    g_print("========================\n\n");
}

/*******************************************************************************
 * Test Main
 ******************************************************************************/
#ifdef CSI_DRIVER_TEST

static void test_frame_callback(frame_data_t *frame, void *user_data) {
    static uint64_t last_frame = 0;
    uint64_t frame_num = (uint64_t)user_data;
    
    if (frame->frame_number != last_frame + 1 && last_frame != 0) {
        g_print("Frame drop detected: %lu -> %lu\n", last_frame, frame->frame_number);
    }
    last_frame = frame->frame_number;
    
    if (frame->frame_number % 30 == 0) {
        g_print("Frame %lu: %dx%d @ %lu\n", 
                frame->frame_number, frame->width, frame->height, frame->timestamp);
    }
}

int main(int argc, char *argv[]) {
    csi_camera_t *cam;
    csi_config_t config = {
        .width = 1920,
        .height = 1080,
        .fps = 30,
        .format = V4L2_PIX_FMT_UYVY,
        .sensor = SENSOR_IMX477,
        .hardware_encode = FALSE,
        .stereo_mode = FALSE
    };
    
    g_print("Cule OS CSI Camera Driver Test\n");
    g_print("==============================\n\n");
    
    /* Initialize */
    if (csi_camera_init() < 0) {
        g_printerr("Failed to initialize CSI module\n");
        return 1;
    }
    
    /* Open camera */
    cam = csi_camera_open(0, &config);
    if (!cam) {
        g_printerr("Failed to open camera\n");
        csi_camera_cleanup();
        return 1;
    }
    
    /* Load calibration if available */
    csi_camera_load_calibration(cam, "/opt/cule/calibration/camera_0.json");
    
    /* Register callback */
    csi_camera_register_callback(cam, test_frame_callback, NULL);
    
    /* Print info */
    csi_print_camera_info(cam);
    
    /* Start capture */
    if (csi_camera_start(cam) < 0) {
        g_printerr("Failed to start camera\n");
        csi_camera_close(cam);
        csi_camera_cleanup();
        return 1;
    }
    
    /* Run for 10 seconds */
    g_print("Capturing for 10 seconds...\n");
    sleep(10);
    
    /* Print final stats */
    csi_print_camera_info(cam);
    
    /* Cleanup */
    csi_camera_stop(cam);
    csi_camera_close(cam);
    csi_camera_cleanup();
    
    g_print("Test completed successfully\n");
    return 0;
}

#endif /* CSI_DRIVER_TEST */