#ifndef __DECACALLBACK_H
#define __DECACALLBACK_H
#include "deca_device_api.h"
#include "deca_regs.h"
typedef unsigned long long uint64;
void tx_conf_cb(const dwt_cb_data_t *cb_data);
void rx_ok_cb(const dwt_cb_data_t *cb_data);
void rx_to_cb(const dwt_cb_data_t *cb_data);
void rx_err_cb(const dwt_cb_data_t *cb_data);


#endif
