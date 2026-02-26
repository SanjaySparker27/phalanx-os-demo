/*
 * GStreamer Pipeline Manager
 * Unified camera capture using GStreamer
 */

#ifndef GSTREAMER_PIPELINE_H
#define GSTREAMER_PIPELINE_H

#include <gst/gst.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C" {
#endif

typedef enum {
    GST_FORMAT_NV12,
    GST_FORMAT_I420,
    GST_FORMAT_YUY2,
    GST_FORMAT_UYVY,
    GST_FORMAT_RGB,
    GST_FORMAT_BGR,
    GST_FORMAT_GRAY8,
    GST_FORMAT_MJPEG
} gst_video_format_t;

typedef struct {
    GstElement *pipeline;
    GstElement *source;
    GstElement *capsfilter;
    GstElement *converter;
    GstElement *sink;
    GstBus *bus;
    
    /* Configuration */
    char sensor_name[32];
    uint32_t width;
    uint32_t height;
    uint32_t fps;
    gst_video_format_t format;
    char device[64];
    
    /* State */
    bool is_running;
    GMainLoop *loop;
    pthread_t gst_thread;
    
    /* Callbacks */
    void (*on_frame)(void *buffer, size_t size, void *userdata);
    void (*on_error)(const char *error, void *userdata);
    void *userdata;
} gst_pipeline_t;

/* Pipeline management */
gst_pipeline_t *gst_pipeline_create(const char *name);
void gst_pipeline_destroy(gst_pipeline_t *pipe);

/* Configuration */
sensor_error_t gst_pipeline_configure(gst_pipeline_t *pipe, 
                                       const char *sensor,
                                       uint32_t width, 
                                       uint32_t height, 
                                       uint32_t fps,
                                       gst_video_format_t format);

/* Control */
sensor_error_t gst_pipeline_start(gst_pipeline_t *pipe);
sensor_error_t gst_pipeline_stop(gst_pipeline_t *pipe);
sensor_error_t gst_pipeline_pause(gst_pipeline_t *pipe);

/* IMX477 specific pipeline */
char *gst_pipeline_imx477_create(gst_pipeline_t *pipe, uint32_t width, uint32_t height, uint32_t fps);

/* IMX219 specific pipeline */
char *gst_pipeline_imx219_create(gst_pipeline_t *pipe, uint32_t width, uint32_t height, uint32_t fps);

/* NVIDIA Jetson optimized pipeline */
char *gst_pipeline_jetson_create(gst_pipeline_t *pipe, const char *sensor, 
                                  uint32_t width, uint32_t height, uint32_t fps);

#ifdef __cplusplus
}
#endif

#endif /* GSTREAMER_PIPELINE_H */
