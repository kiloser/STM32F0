#include "deca_callback.h"
/* Buffer to store received messages.
 * Its size is adjusted to longest frame that this example code is supposed to handle. */
#define RX_BUF_LEN 12
static uint8 rx_buffer[RX_BUF_LEN];

void tx_conf_cb(const dwt_cb_data_t *cb_data)
{
    /* This callback has been defined so that a breakpoint can be put here to check it is correctly called but there is actually nothing specific to
     * do on transmission confirmation in this example. Typically, we could activate reception for the response here but this is automatically handled
     * by DW1000 using DWT_RESPONSE_EXPECTED parameter when calling dwt_starttx().
     * An actual application that would not need this callback could simply not define it and set the corresponding field to NULL when calling
     * dwt_setcallbacks(). The ISR will not call it which will allow to save some interrupt processing time. */

    /* TESTING BREAKPOINT LOCATION #4 */

}
void rx_ok_cb(const dwt_cb_data_t *cb_data)
{
					dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

void rx_err_cb(const dwt_cb_data_t *cb_data)
{
					printf("rx_err int\r\n");				
					dwt_rxenable(DWT_START_RX_IMMEDIATE);
}

void rx_to_cb(const dwt_cb_data_t *cb_data)
{
				printf("rx_to int\r\n");		
				dwt_rxenable(DWT_START_RX_IMMEDIATE);
}


