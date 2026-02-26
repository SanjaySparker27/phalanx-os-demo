/*
 * GStreamer Pipeline Implementation
 * Optimized for NVIDIA Jetson and embedded platforms
 */

#include "gstreamer_pipeline.h"
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <pthread.h>

static void *gst_loop_thread(void *arg) {
    gst_pipeline_t *pipe = (gst_pipeline_t *)arg;
    g_main_loop_run(pipe->loop);
    return NULL;
}

static gboolean bus_callback(GstBus *bus, GstMessage *msg, gpointer data) {
    gst_pipeline_t *pipe = (gst_pipeline_t *)data;
    
    switch (GST_MESSAGE_TYPE(msg)) {
        case GST_MESSAGE_ERROR: {
            GError *err = NULL;
            gchar *debug = NULL;
            gst_message_parse_error(msg, &err, &debug);
            
            if (pipe->on_error) {
                pipe->on_error(err->message, pipe->userdata);
            }
            
            g_error_free(err);
            g_free(debug);
            g_main_loop_quit(pipe->loop);
            break;
        }
        case GST_MESSAGE_EOS:
            g_main_loop_quit(pipe->loop);
            break;
        default:
            break;
    }
    return TRUE;
}

static GstFlowReturn on_new_sample(GstAppSink *sink, gpointer user_data) {
    gst_pipeline_t *pipe = (gst_pipeline_t *)user_data;
    GstSample *sample = gst_app_sink_pull_sample(sink);
    
    if (sample) {
        GstBuffer *buffer = gst_sample_get_buffer(sample);
        GstMapInfo map;
        
        if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
            if (pipe->on_frame) {
                pipe->on_frame(map.data, map.size, pipe->userdata);
            }
            gst_buffer_unmap(buffer, &map);
        }
        gst_sample_unref(sample);
    }
    
    return GST_FLOW_OK;
}

gst_pipeline_t *gst_pipeline_create(const char *name) {
    gst_pipeline_t *pipe = calloc(1, sizeof(gst_pipeline_t));
    if (!pipe) {
        return NULL;
    }
    
    strncpy(pipe->sensor_name, name, sizeof(pipe->sensor_name) - 1);
    
    /* Initialize GStreamer if not already done */
    if (!gst_is_initialized()) {
        gst_init(NULL, NULL);
    }
    
    return pipe;
}

void gst_pipeline_destroy(gst_pipeline_t *pipe) {
    if (!pipe) {
        return;
    }
    
    gst_pipeline_stop(pipe);
    
    if (pipe->loop) {
        g_main_loop_unref(pipe->loop);
    }
    
    free(pipe);
}

sensor_error_t gst_pipeline_configure(gst_pipeline_t *pipe, 
                                       const char *sensor,
                                       uint32_t width, 
                                       uint32_t height, 
                                       uint32_t fps,
                                       gst_video_format_t format) {
    if (!pipe) {
        return SENSOR_ERROR_INVALID_PARAM;
    }
    
    strncpy(pipe->sensor_name, sensor, sizeof(pipe->sensor_name) - 1);
    pipe->width = width;
    pipe->height = height;
    pipe->fps = fps;
    pipe->format = format;
    
    return SENSOR_OK;
}

/* IMX477 Pipeline for Jetson */
char *gst_pipeline_imx477_create(gst_pipeline_t *pipe, uint32_t width, uint32_t height, uint32_t fps) {
    static char pipeline_str[1024];
    
    /* Jetson-optimized IMX477 pipeline with ISP */
    snprintf(pipeline_str, sizeof(pipeline_str),
        "nvarguscamerasrc sensor-id=0 ! "
        "video/x-raw(memory:NVMM), width=(int)%u, height=(int)%u, "
        "format=(string)NV12, framerate=(fraction)%u/1 ! "
        "nvvidconv flip-method=0 ! "
        "video/x-raw, width=(int)%u, height=(int)%u, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! "
        "appsink name=sink max-buffers=1 drop=true",
        width, height, fps, width, height);
    
    return pipeline_str;
}

/* IMX219 Pipeline for Jetson */
char *gst_pipeline_imx219_create(gst_pipeline_t *pipe, uint32_t width, uint32_t height, uint32_t fps) {
    static char pipeline_str[1024];
    
    /* Jetson-optimized IMX219 pipeline */
    snprintf(pipeline_str, sizeof(pipeline_str),
        "nvarguscamerasrc sensor-id=1 ! "
        "video/x-raw(memory:NVMM), width=(int)%u, height=(int)%u, "
        "format=(string)NV12, framerate=(fraction)%u/1 ! "
        "nvvidconv flip-method=0 ! "
        "video/x-raw, width=(int)%u, height=(int)%u, format=(string)BGRx ! "
        "videoconvert ! "
        "video/x-raw, format=(string)BGR ! "
        "appsink name=sink max-buffers=1 drop=true",
        width, height, fps, width, height);
    
    return pipeline_str;
}

/* Generic Jetson pipeline */
char *gst_pipeline_jetson_create(gst_pipeline_t *pipe, const char *sensor, 
                                  uint32_t width, uint32_t height, uint32_t fps) {
    if (strcmp(sensor, "imx477") == 0) {
        return gst_pipeline_imx477_create(pipe, width, height, fps);
    } else if (strcmp(sensor, "imx219") == 0) {
        return gst_pipeline_imx219_create(pipe, width, height, fps);
    }
    return NULL;
}

sensor_error_t gst_pipeline_start(gst_pipeline_t *pipe) {
    if (!pipe || pipe->is_running) {
        return SENSOR_ERROR_INVALID_PARAM;
    }
    
    GError *error = NULL;
    char *pipeline_desc;
    
    /* Build pipeline string */
    pipeline_desc = gst_pipeline_jetson_create(pipe, pipe->sensor_name, 
                                                pipe->width, pipe->height, pipe->fps);
    if (!pipeline_desc) {
        return SENSOR_ERROR_INIT;
    }
    
    /* Parse pipeline */
    pipe->pipeline = gst_parse_launch(pipeline_desc, &error);
    if (error) {
        fprintf(stderr, "Pipeline error: %s\n", error->message);
        g_error_free(error);
        return SENSOR_ERROR_INIT;
    }
    
    /* Get app sink */
    pipe->sink = gst_bin_get_by_name(GST_BIN(pipe->pipeline), "sink");
    if (!pipe->sink) {
        gst_object_unref(pipe->pipeline);
        return SENSOR_ERROR_INIT;
    }
    
    /* Configure app sink callbacks */
    GstAppSinkCallbacks callbacks = {
        .eos = NULL,
        .new_preroll = NULL,
        .new_sample = on_new_sample
    };
    gst_app_sink_set_callbacks(GST_APP_SINK(pipe->sink), &callbacks, pipe, NULL);
    
    /* Set up bus */
    pipe->bus = gst_element_get_bus(pipe->pipeline);
    gst_bus_add_watch(pipe->bus, bus_callback, pipe);
    
    /* Create main loop */
    pipe->loop = g_main_loop_new(NULL, FALSE);
    
    /* Start pipeline */
    GstStateChangeReturn ret = gst_element_set_state(pipe->pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE) {
        gst_object_unref(pipe->pipeline);
        return SENSOR_ERROR_INIT;
    }
    
    /* Start GLib main loop in thread */
    pipe->is_running = true;
    pthread_create(&pipe->gst_thread, NULL, gst_loop_thread, pipe);
    
    return SENSOR_OK;
}

sensor_error_t gst_pipeline_stop(gst_pipeline_t *pipe) {
    if (!pipe || !pipe->is_running) {
        return SENSOR_ERROR_INVALID_PARAM;
    }
    
    pipe->is_running = false;
    
    if (pipe->loop) {
        g_main_loop_quit(pipe->loop);
    }
    
    pthread_join(pipe->gst_thread, NULL);
    
    if (pipe->pipeline) {
        gst_element_set_state(pipe->pipeline, GST_STATE_NULL);
        gst_object_unref(pipe->pipeline);
        pipe->pipeline = NULL;
    }
    
    if (pipe->bus) {
        gst_object_unref(pipe->bus);
        pipe->bus = NULL;
    }
    
    return SENSOR_OK;
}

sensor_error_t gst_pipeline_pause(gst_pipeline_t *pipe) {
    if (!pipe || !pipe->pipeline) {
        return SENSOR_ERROR_INVALID_PARAM;
    }
    
    gst_element_set_state(pipe->pipeline, GST_STATE_PAUSED);
    return SENSOR_OK;
}
