#include "usb_camera.h"
#include "linux/hal.h"
#include "linux/list.h"
#include <string.h>

#define USB_XHCI_BASE       0x3530000

#define UVC_VS_PROBE_CONTROL    0x01
#define UVC_VS_COMMIT_CONTROL   0x02
#define UVC_SET_CUR             0x01
#define UVC_GET_MIN             0x82
#define UVC_GET_MAX             0x83

struct uvc_streaming_control {
    uint16_t bmHint;
    uint8_t  bFormatIndex;
    uint8_t  bFrameIndex;
    uint32_t dwFrameInterval;
    uint16_t wKeyFrameRate;
    uint16_t wPFrameRate;
    uint16_t wCompQuality;
    uint16_t wCompWindowSize;
    uint16_t wDelay;
    uint32_t dwMaxVideoFrameSize;
    uint32_t dwMaxPayloadTransferSize;
} __attribute__((packed));

static struct list_head usb_cameras;
static void __iomem *usb_base;

int usb_camera_init(void)
{
    INIT_LIST_HEAD(&usb_cameras);
    usb_base = ioremap(USB_XHCI_BASE, 0x10000);
    return usb_base ? 0 : -ENOMEM;
}

int usb_camera_probe(uint16_t vid, uint16_t pid)
{
    struct usb_camera *cam;
    
    cam = kzalloc(sizeof(*cam));
    if (!cam)
        return -ENOMEM;
    
    cam->id = list_empty(&usb_cameras) ? 0 : 
              list_entry(usb_cameras.prev, struct usb_camera, list)->id + 1;
    cam->vendor_id = vid;
    cam->product_id = pid;
    cam->interface = 1;
    cam->endpoint = 0x81;
    
    cam->buffer_size = 1920 * 1080 * 2;
    cam->frame_buffer = kzalloc(cam->buffer_size);
    if (!cam->frame_buffer) {
        kfree(cam);
        return -ENOMEM;
    }
    
    list_add_tail(&cam->list, &usb_cameras);
    
    return cam->id;
}

static int uvc_send_control(struct usb_camera *cam, uint8_t request, 
                             uint8_t query, void *data, uint16_t len)
{
    return 0;
}

int usb_camera_start(struct usb_camera *cam)
{
    struct uvc_streaming_control ctrl;
    
    memset(&ctrl, 0, sizeof(ctrl));
    ctrl.bmHint = 1;
    ctrl.bFormatIndex = 1;
    ctrl.bFrameIndex = 1;
    ctrl.dwFrameInterval = 333333;
    ctrl.dwMaxVideoFrameSize = cam->width * cam->height * 2;
    ctrl.dwMaxPayloadTransferSize = USB_MAX_PACKET;
    
    uvc_send_control(cam, UVC_VS_PROBE_CONTROL, UVC_SET_CUR, &ctrl, sizeof(ctrl));
    uvc_send_control(cam, UVC_VS_COMMIT_CONTROL, UVC_SET_CUR, &ctrl, sizeof(ctrl));
    
    cam->streaming = true;
    return 0;
}

int usb_camera_stop(struct usb_camera *cam)
{
    cam->streaming = false;
    return 0;
}

int usb_camera_capture(struct usb_camera *cam, void **frame, uint32_t *size)
{
    if (!cam->streaming)
        return -EIO;
    
    *frame = cam->frame_buffer;
    *size = cam->width * cam->height * 2;
    
    return 0;
}
