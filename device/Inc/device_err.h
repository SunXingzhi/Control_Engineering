#ifndef DEVICE_ERR_H
#define DEVICE_ERR_H

typedef enum device_err {
    DRV_OK = 0,
    DRV_ERR_NULL,
    DRV_ERR_INIT,
    DRV_ERR_TIMEOUT,
    DRV_ERR_PARAM,
    DRV_ERR_BUSY,
    DRV_ERR_IO,
} device_err_t;

#endif /* DEVICE_ERR_H */
