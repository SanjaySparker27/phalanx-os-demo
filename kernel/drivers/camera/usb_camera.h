#ifndef _USB_CAMERA_H
#define _USB_CAMERA_H

#include "linux/types.h"

#define USB_CAMERA_MAX      4
#define USB_MAX_PACKET      3072

struct usb_camera {
    uint32_t id;
    uint16_t vendor_id;
    uint16_t product_id;
    uint8_t interface;
    uint8_t endpoint;
    uint32_t format;
    uint32_t width;
    uint32_t height;
    uint32_t fps;
    void *frame_buffer;
    uint32_t buffer_size;
    bool streaming;
};

int usb_camera_init(void);
int usb_camera_probe(uint16_t vid, uint16_t pid);
int usb_camera_start(struct usb_camera *cam);
int usb_camera_stop(struct usb_camera *cam);
int usb_camera_capture(struct usb_camera *cam, void **frame, uint32_t *size);

#endif