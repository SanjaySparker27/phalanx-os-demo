#ifndef _CSI_CAMERA_H
#define _CSI_CAMERA_H

#include "linux/types.h"

#define CSI_MAX_CAMERAS     6
#define CSI_LANES           4
#define CSI_CLK_FREQ_MHZ    1500

#define NVCSI_BASE          0x15a00000
#define VI_BASE             0x150c0000

struct csi_camera {
    uint32_t id;
    uint32_t lanes;
    uint32_t format;
    uint32_t width;
    uint32_t height;
    uint32_t fps;
    void *buffer_base;
    uint32_t buffer_size;
    bool streaming;
};

enum csi_pixel_format {
    CSI_FMT_RAW8    = 0x2A,
    CSI_FMT_RAW10   = 0x2B,
    CSI_FMT_RAW12   = 0x2C,
    CSI_FMT_RAW14   = 0x2D,
    CSI_FMT_RAW16   = 0x2E,
    CSI_FMT_RGB888  = 0x24,
    CSI_FMT_UYVY    = 0x1E,
    CSI_FMT_YUYV    = 0x16,
};

int csi_camera_init(void);
int csi_camera_register(struct csi_camera *cam);
int csi_camera_start_stream(struct csi_camera *cam);
int csi_camera_stop_stream(struct csi_camera *cam);
int csi_camera_capture(struct csi_camera *cam, void **frame, uint32_t *size);

#endif