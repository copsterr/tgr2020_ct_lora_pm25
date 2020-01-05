#include "ct_honey.h"


/* Command List */
uint8_t CMD_STARTMEAS[4] = { 0x68, 0x01, 0x01, 0x96 };
uint8_t CMD_STOPMEAS[4]  = { 0x68, 0x01, 0x02, 0x95 };
uint8_t CMD_READMEAS[4]  = { 0x68, 0x01, 0x04, 0x93 };
uint8_t CMD_AUTOEN[4]    = { 0x68, 0x01, 0x40, 0x57 };
uint8_t CMD_AUTOSTOP[4]  = { 0x68, 0x01, 0x20, 0x77 };
uint8_t CMD_READCOEF[4]  = { 0x68, 0x01, 0x10, 0x87 };
uint8_t CMD_SETCOEF[5]   = { 0x68, 0x02, 0x08, 0x64, 0x2A }; // default coef is 0x64


/* APIs ----------------------------------------------------------------------*/
honey_cmd_resp_t honey_init(UART_HandleTypeDef huart, honey_t* honey) {
    /* 
        Init Honeywell Module
        params
            huart: UART port to use
            honey: honey_t structure (must be declared in main.c)

        How to init:
        > honey_t honey;
        > honey_init(huart1, &honey);
    */
    honey->huart  = huart;
    honey->pm2_5  = 0;
    honey->pm10_0 = 0;
    honey->customer_coef = 100; // default is 100

    //startup routine
    if (honey_stop(honey) != CMD_RESP_SUCCESS) return CMD_RESP_ERR;
    if (honey_autosend(honey, 0) != CMD_RESP_SUCCESS) return CMD_RESP_ERR;
    if (honey_read_coef(honey) != CMD_RESP_SUCCESS) return CMD_RESP_ERR;

    return CMD_RESP_SUCCESS;
}

honey_cmd_resp_t honey_start(honey_t* honey) {
    /*
        Start the fan for measuring
        params
            *honey: pointer type of honey_t variable
        return
            command response
    */
    uint8_t resp[2] = {0};

	HAL_UART_Transmit(&honey->huart, (uint8_t*) CMD_STARTMEAS, 4, 100);
    HAL_UART_Receive(&honey->huart, (uint8_t*) resp, 2, 100);

    if (resp[0] == 0xA5 && resp[1] == 0xA5) { // success is 0xA5A5
        return CMD_RESP_SUCCESS;
    }
    
    return CMD_RESP_ERR;
}

honey_cmd_resp_t honey_stop(honey_t* honey) {
    /* 
        Stop the fan for stopping measuring
    */
    uint8_t resp[2] = {0};

    HAL_UART_Transmit(&honey->huart, (uint8_t*) CMD_STOPMEAS, 4, 100);
    HAL_UART_Receive(&honey->huart, (uint8_t*) resp, 2, 100);

    if (resp[0] == 0xA5 && resp[1] == 0xA5) {
        return CMD_RESP_SUCCESS;
    }
    
//    // stop interrupts
//    HAL_UART_Abort(&honey->huart);

    return CMD_RESP_ERR;
}

honey_cmd_resp_t honey_read(honey_t *honey) {
    /*
        Read measurement. Values are stored in the honey_t structure
    */
    uint8_t resp[8] = {0};

    HAL_UART_Transmit(&honey->huart, (uint8_t*) CMD_READMEAS, 4, 100);
    HAL_UART_Receive(&honey->huart, (uint8_t*) resp, 8, 100);

    if (resp[0] == 0x40 && resp[1] == 0x05 && resp[2] == 0x04) {
        honey->pm2_5 = resp[3] * 256 + resp[4];
        honey->pm10_0 = resp[5] * 256 + resp[6];

        return CMD_RESP_SUCCESS;
    }

    return CMD_RESP_ERR;
}

honey_cmd_resp_t honey_autosend(honey_t *honey, uint8_t mode) {
    /*
        Enable or Disable Autosend
        params
            *honey: pointer type of honey_t variable
            mode: can be either 0 or 1, 0 for disable, 1 for enable
        return
            command response
    */
    
    uint8_t resp[2] = {0};

    if (mode == 0) { // stop autosend
        HAL_UART_Transmit(&honey->huart, (uint8_t*) CMD_AUTOSTOP, 4, 100);
    } else if (mode == 1) { // enable autosend
        HAL_UART_Transmit(&honey->huart, (uint8_t*) CMD_AUTOEN, 4, 100);
    } else {
        return CMD_RESP_BAD;
    }
    
    // receiving response
    HAL_UART_Receive(&honey->huart, (uint8_t*) resp, 2, 100);

    if (resp[0] == 0xA5 && resp[1] == 0xA5) {
        return CMD_RESP_SUCCESS;
    }
    
    return CMD_RESP_ERR;
}

honey_cmd_resp_t honey_set_coef(honey_t *honey, uint8_t coef) {
    /*
        Set customer coefficient
        params
            *honey: pointer type of honey_t variable
            coef: integer ranging from 30 to 200
        return
            command response
    */
    uint8_t resp[2] = {0};
    uint8_t cs      = 0;

    // if coef is out of range
    if (coef < 30 || coef > 200) {
        honey->customer_coef = 100;
    }
    
    // set coef command
    CMD_SETCOEF[3] = honey->customer_coef = coef;

    // calculate and set Check Sum Byte
    cs = calc_cs(CMD_SETCOEF, 5);
    CMD_SETCOEF[4] = cs;

    HAL_UART_Transmit(&honey->huart, (uint8_t*) CMD_SETCOEF, 5, 100);
    HAL_UART_Receive(&honey->huart, (uint8_t*) resp, 2, 100);

    if (resp[0] == 0xA5 && resp[1] == 0xA5) {
        return CMD_RESP_SUCCESS;
    }

    return CMD_RESP_ERR;
}

honey_cmd_resp_t honey_read_coef(honey_t* honey) {
    /*
        Read customer coefficient from the sensor and automatically set
        customer coefficient constructor to the value that's been read
    */
    uint8_t resp[5] = {0};

    HAL_UART_Transmit(&honey->huart, (uint8_t*) CMD_READCOEF, 4, 100);
    HAL_UART_Receive(&honey->huart, (uint8_t*) resp, 5, 100);

    if (resp[0] == 0x40 && resp[1] == 0x02 && resp[2] == 0x10) {
        // this function automatically set the honey.customer_coef to what it reads
        honey->customer_coef = resp[3];

        return CMD_RESP_SUCCESS;
    }

    return CMD_RESP_ERR;
}

uint8_t calc_cs(uint8_t* CMD, uint8_t cmd_len) {
    /*
        Calculate Check Sum of a command using the following formula
        cs = MOD((65536-(HEAD+LEN+CMD+DATA)), 256)

        params
            CMD: Command array
            cmd_len: the length of a command
        return
            Check Sum value
    */
    uint8_t cs   = 0;
    uint8_t temp = 0;
    uint8_t i    = 0;

    // sum of HEAD+LEN+CMD+DATA values
    for (i = 0; i < (cmd_len - 1); ++i) {
        temp += *(CMD + i);
    }

    // calculate Check Sum
    cs = (65536 - temp) % 256;
    return cs;
}
