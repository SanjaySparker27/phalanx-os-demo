#include "csi_camera.h"
#include "linux/hal.h"
#include "linux/list.h"
#include <string.h>

#define NVCSI_CIL_A_BASE    0x0000
#define NVCSI_CIL_B_BASE    0x0800
#define NVCSI_PHY_BASE      0x1000

#define CIL_CTRL            0x00
#define CIL_STATUS          0x04
#define PHY_CTRL            0x00
#define PHY_STATUS          0x04

static struct list_head csi_devices;
static void __iomem *nvcsi_base;
static void __iomem *vi_base;
static spinlock_t csi_lock;

struct csi_device {
    struct csi_camera camera;
    struct list_head list;
};

int csi_camera_init(void)
{
    INIT_LIST_HEAD(&csi_devices);
    spin_lock_init(&csi_lock);
    
    nvcsi_base = ioremap(NVCSI_BASE, 0x10000);
    vi_base = ioremap(VI_BASE, 0x10000);
    
    if (!nvcsi_base || !vi_base)
        return -ENOMEM;
    
    writel(0x1, nvcsi_base + NVCSI_PHY_BASE + PHY_CTRL);
    writel(0x1, nvcsi_base + NVCSI_CIL_A_BASE + CIL_CTRL);
    writel(0x1, nvcsi_base + NVCSI_CIL_B_BASE + CIL_CTRL);
    
    return 0;
}

int csi_camera_register(struct csi_camera *cam)
{
    struct csi_device *dev;
    
    dev = kzalloc(sizeof(*dev));
    if (!dev)
        return -ENOMEM;
    
    memcpy(&dev->camera, cam, sizeof(*cam));
    
    dev->camera.buffer_size = cam->width * cam->height * 2;
    dev->camera.buffer_base = kzalloc(dev->camera.buffer_size);
    if (!dev->camera.buffer_base) {
        kfree(dev);
        return -ENOMEM;
    }
    
    spin_lock(&csi_lock);
    list_add_tail(&dev->list, &csi_devices);
    spin_unlock(&csi_lock);
    
    return 0;
}

static void csi_configure_phy(uint32_t lanes, uint32_t data_rate)
{
    uint32_t val;
    
    val = (lanes << 16) | (1 << 4) | 1;
    writel(val, nvcsi_base + NVCSI_PHY_BASE + PHY_CTRL);
    
    val = (data_rate / 1000000) << 16;
    writel(val, nvcsi_base + NVCSI_CIL_A_BASE + 0x10);
    writel(val, nvcsi_base + NVCSI_CIL_B_BASE + 0x10);
}

int csi_camera_start_stream(struct csi_camera *cam)
{
    struct csi_device *dev;
    uint32_t val;
    
    list_for_each_entry(dev, &csi_devices, list) {
        if (dev->camera.id == cam->id) {
            csi_configure_phy(cam->lanes, 1500000000);
            
            val = (cam->format << 24) | (cam->width << 12) | cam->height;
            writel(val, vi_base + 0x100 + (cam->id * 0x100));
            
            writel((uint64_t)dev->camera.buffer_base, vi_base + 0x108 + (cam->id * 0x100));
            writel(dev->camera.buffer_size, vi_base + 0x110 + (cam->id * 0x100));
            
            writel(0x1, vi_base + 0x104 + (cam->id * 0x100));
            
            dev->camera.streaming = true;
            return 0;
        }
    }
    
    return -ENODEV;
}

int csi_camera_stop_stream(struct csi_camera *cam)
{
    struct csi_device *dev;
    
    list_for_each_entry(dev, &csi_devices, list) {
        if (dev->camera.id == cam->id) {
            writel(0x0, vi_base + 0x104 + (cam->id * 0x100));
            dev->camera.streaming = false;
            return 0;
        }
    }
    
    return -ENODEV;
}

int csi_camera_capture(struct csi_camera *cam, void **frame, uint32_t *size)
{
    struct csi_device *dev;
    uint32_t status;
    int retries = 1000;
    
    list_for_each_entry(dev, &csi_devices, list) {
        if (dev->camera.id == cam->id) {
            while (retries--) {
                status = readl(vi_base + 0x114 + (cam->id * 0x100));
                if (status & 0x1)
                    break;
                udelay(10);
            }
            
            if (retries <= 0)
                return -ETIMEDOUT;
            
            *frame = dev->camera.buffer_base;
            *size = dev->camera.buffer_size;
            
            writel(status, vi_base + 0x114 + (cam->id * 0x100));
            
            return 0;
        }
    }
    
    return -ENODEV;
}
