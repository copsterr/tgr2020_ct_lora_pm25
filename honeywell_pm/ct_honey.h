//#include "main.h"
//#include "stm32l0xx_hal.h"
#include "hw_conf.h"

/* Honey Command Response Enumerations */
typedef enum {
    CMD_RESP_IDLE = 0x00,
    CMD_RESP_SUCCESS,
    CMD_RESP_TIMEOUT,
    CMD_RESP_BAD,           // invalid parameters
    CMD_RESP_ERR = 0xFF
} honey_cmd_resp_t;


/* Honey Structure */
typedef struct __honey_t {
	UART_HandleTypeDef  huart;
    uint16_t            pm2_5;
    uint16_t            pm10_0;
    uint8_t             customer_coef;
} honey_t;


/* Prototypes */
void honey_init(UART_HandleTypeDef huart, honey_t* honey);
honey_cmd_resp_t honey_start(honey_t* honey);
honey_cmd_resp_t honey_stop(honey_t* honey);
honey_cmd_resp_t honey_read(honey_t *honey);
honey_cmd_resp_t honey_autosend(honey_t *honey, uint8_t mode);
honey_cmd_resp_t honey_set_coef(honey_t *honey, uint8_t coef);
honey_cmd_resp_t honey_read_coef(honey_t* honey);
uint8_t calc_cs(uint8_t* CMD, uint8_t cmd_len);

