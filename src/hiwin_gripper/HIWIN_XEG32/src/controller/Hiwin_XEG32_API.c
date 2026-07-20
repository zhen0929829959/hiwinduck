/* 
    HIWIN XEG-32 電動夾爪控制器
    版本 : 2022/09/22

    電爪規格(XEG-32):
        移動行程    移動速度      夾持行程    夾持速度      夾持力量
        0~32(mm)   0~80(mm/s)  0~32(mm)   0~20(mm/s)   40~100%
*/


#include <stdio.h>
#include <unistd.h>
#include <string.h>
#include <stdint.h>
#include <stdlib.h>
#include <time.h>
#include <sys/time.h>
#include <errno.h>
 
#include <modbus.h>



/* Modbus RTU default values */
#define MODBUS_SERIAL_DEV           "/dev/ttyUSB0"
#define MODBUS_SERIAL_BAUDRATE      115200
#define MODBUS_SERIAL_PARITY        'N'
#define MODBUS_SERIAL_DATABITS      8
#define MODBUS_SERIAL_STOPBITS      1
#define MODBUS_DEVICE_ID            1
#define MODBUS_TIMEOUT_SEC          3
#define MODBUS_TIMEOUT_USEC         0
#define MODBUS_DEBUG                ON
#define MODBUS_RO_BITS              32
#define MODBUS_RW_BITS              32
#define MODBUS_RO_REGISTERS         64
#define MODBUS_RW_REGISTERS         64

/* REGISTERS */
#define TYPE_REGISTERS_ADDRESS                      0x0600      // 型號
#define REST_REGISTERS_ADDRESS                      0x0610      // reset
#define STOP_REGISTERS_ADDRESS                      0x0620      // Stop
#define MOVE_REGISTERS_ADDRESS                      0x0630      /* Move mode reg */
#define EXPERT_REGISTERS_ADDRESS                    0x0640      /* Expert mode reg */
#define STATE_REGISTERS_ADDRESS                     0x0301


int         wrt;
int         ret;
modbus_t    *ctx;
uint16_t    type[1]         =   {2592};         // 夾爪型號(EXG-32)
uint16_t    reset_signal[1] =   {1};            // 復位訊號
uint16_t    stop_signal[1]  =   {1};            // 停止訊號
uint16_t    state[1]         =   {0};


 

/* 建立 Modbus RTU 連線 */
int libModbus_Connect(int ID){

    /***** set IP and Port *****/
    ctx = modbus_new_rtu(MODBUS_SERIAL_DEV,
                         MODBUS_SERIAL_BAUDRATE,
                         MODBUS_SERIAL_PARITY,
                         MODBUS_SERIAL_DATABITS,
                         MODBUS_SERIAL_STOPBITS);

    /* setting */
    modbus_set_slave(ctx, ID);                              /* setting slave ID */
    modbus_set_debug(ctx, TRUE);                            /* setting debug */
    modbus_set_response_timeout(ctx, 1, 0);                 /* setting timeout */
    modbus_rtu_set_serial_mode(ctx, MODBUS_RTU_RS485);      /* MODBUS_RTU_RS232 or MODBUS_RTU_RS485 */
    
    /* Connect */
    ret = modbus_connect(ctx);
    if (ret==0){
        printf("\n\tModbus Connect success\n");
    }else{
        printf("\n\tModbus Connect Error!!\n");
    }
    
    /* 寫入手臂型號 */
    modbus_write_registers(ctx, TYPE_REGISTERS_ADDRESS, 1, type);

    if(ctx == NULL)
    {
        fprintf(stderr, "Unable to allocate libmodbus context\n");
    }

    if (modbus_connect(ctx) == -1) {
        fprintf(stderr, "Unable to create the libmodbus context\n");
        printf("modbus_new_rtu = -1\n");
        return -1;
    }
    return -1;
}



/* 關閉 Modbus RTU 連線 */
void Modbus_Close(){ 
    modbus_free(ctx);
    modbus_close(ctx);
}



/*  Control mode : Reset 
        功能 : 初始化電爪
        備註 : 僅重啟時需執行此命令
*/
void reset(void){
    modbus_write_registers(ctx, REST_REGISTERS_ADDRESS, 1, reset_signal);
}


/*  Control mode : Stop 
        功能 : 強置停止並結束動作
        備註 : 無
*/
void stop(void){
    modbus_write_registers(ctx, STOP_REGISTERS_ADDRESS, 1, stop_signal);
}


/*  Control mode : Move 
        功能 : 強置停止並結束動作
        備註 : 無
*/
void move(int dis ,int speed ,int flag){
    uint16_t move_command[3] = {dis ,speed ,flag};
    modbus_write_registers(ctx, MOVE_REGISTERS_ADDRESS, 3, move_command);
}

/*  Control mode : Expert 
        功能 : 強置停止並結束動作
        備註 : 無
*/
void expert(int dir, int dis ,int speed ,int clamp_dis ,int clamp_speed ,int clamp_power ,int flag){
    uint16_t expert_command[7] = {dir, dis, speed, clamp_dis, clamp_speed, clamp_power, flag};
    modbus_write_registers(ctx, EXPERT_REGISTERS_ADDRESS, 7, expert_command);
}


uint16_t read_mode(){
    ret = modbus_read_input_registers(ctx, STATE_REGISTERS_ADDRESS, 1, state);
    return state[0];
}

int main(){
    return
}