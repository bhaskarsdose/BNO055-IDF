/* i2c - Example

   For other examples please check:
   https://github.com/espressif/esp-idf/tree/master/examples

   See README.md file to get detailed usage of this example.

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include "esp_log.h"
#include "driver/i2c.h"

//#include "sdkconfig.h"

/*User code
 * for
 * adding
 * BNO055 support
 */
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include "bno055.h"

#define SDA_PIN CONFIG_I2C_MASTER_SDA//GPIO_NUM_18
#define SCL_PIN CONFIG_I2C_MASTER_SCL//GPIO_NUM_19

#define I2C_MASTER_ACK 0
#define I2C_MASTER_NACK 1

#define I2C_MASTER_FREQ_HZ 10000//CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */

void i2c_master_init(){
	i2c_config_t i2c_config = { .mode = I2C_MODE_MASTER, .sda_io_num = SDA_PIN,
			.scl_io_num = SCL_PIN, .sda_pullup_en = GPIO_PULLUP_ENABLE,
			.scl_pullup_en = GPIO_PULLUP_ENABLE, .master.clk_speed = I2C_MASTER_FREQ_HZ
	};
	i2c_param_config(I2C_NUM_0, &i2c_config);
	i2c_driver_install(I2C_NUM_0, I2C_MODE_MASTER,0,0,0);


}

s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt){
	u8 iError = BNO055_INIT_VALUE;
	esp_err_t espRc;
	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1) | I2C_MASTER_WRITE,1);

	i2c_master_write_byte(cmd,reg_addr,1);
	i2c_master_write(cmd,reg_data,cnt,1);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
	if (espRc == ESP_OK) {
		iError = BNO055_SUCCESS;
	} else{
		iError = BNO055_ERROR;
	}
	i2c_cmd_link_delete(cmd);

	return (s8) iError;
}

s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt) {
	u8 iError = BNO055_INIT_VALUE;
	esp_err_t espRc;

	i2c_cmd_handle_t cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1)| I2C_MASTER_WRITE, 1);
	i2c_master_write_byte(cmd, reg_addr,1);
	
	// i2c_master_stop(cmd);
	// i2c_cmd_link_delete(cmd);
	// cmd = i2c_cmd_link_create();

	i2c_master_start(cmd);
	i2c_master_write_byte(cmd, (dev_addr << 1)| I2C_MASTER_READ, 1);

	if (cnt >1){
	 i2c_master_read(cmd,reg_data,cnt-1,I2C_MASTER_ACK);
	}
	i2c_master_read_byte(cmd, reg_data+cnt-1 , I2C_MASTER_NACK);
	i2c_master_stop(cmd);

	espRc = i2c_master_cmd_begin(I2C_NUM_0, cmd, 10 / portTICK_PERIOD_MS);
	if(espRc == ESP_OK){
		iError = BNO055_SUCCESS;
	}else{
		iError = BNO055_ERROR;
	}
	i2c_cmd_link_delete(cmd);
	return(s8) iError;
}


// esp_err_t bno055_read_data(i2c_number_t i2c_num, bno055_reg_t start_reg, uint8_t *buffer, uint8_t n_bytes){

//     if( !x_bno_dev[i2c_num].bno_is_open) {
//         ESP_LOGE(TAG, "bno055_read_data(): device is not open");
//         return BNO_ERR_NOT_OPEN;
//     }

//     if( n_bytes < 2 || n_bytes > 0x7F ) {
//         ESP_LOGE(TAG, "bno055_read_data(): invalid number of bytes: %d", n_bytes);
//         return BNO_ERR_NOT_IN_RANGE;
//     }

//     i2c_cmd_handle_t cmd = i2c_cmd_link_create();
    
//     // making the command - begin 
//     i2c_master_start(cmd);  // start condition
//     // device address with write bit
//     i2c_master_write_byte(cmd, (x_bno_dev[i2c_num].i2c_address << 1) | WRITE_BIT, ACK_CHECK_EN); 
//     // send the register address
//     i2c_master_write_byte(cmd, start_reg, ACK_CHECK_EN);   
//     i2c_master_start(cmd);  // start condition again
//     // device address with read bit
//     i2c_master_write_byte(cmd, (x_bno_dev[i2c_num].i2c_address << 1) | READ_BIT, ACK_CHECK_EN);
//     // read n_bytes-1, issue ACK
//     i2c_master_read(cmd, buffer, n_bytes - 1, ACK_VAL);
//     // read the last byte, issue NACK
//     i2c_master_read_byte(cmd, buffer+n_bytes-1, NACK_VAL); 
//     i2c_master_stop(cmd);  // stop condition
//     // making the command - end 

//     // Now execute the command
//     esp_err_t err = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
    
//     i2c_cmd_link_delete(cmd);
    
//     switch (err) {
//         case ESP_OK: 
//         	break;
//         case ESP_ERR_TIMEOUT:
//             ESP_LOGE(TAG, "bno055_read_data(): i2c timeout");
//             break;
//         default: 
//             ESP_LOGE(TAG, "bno055_read_data(): failed");
//     }
    
//     return err;   
// }



void BNO055_delay_msek(u32 msek) {
	vTaskDelay(msek / portTICK_PERIOD_MS);
}

void task_bno055_normal_mode(void *ignore){
	//i2c bus initialization
	i2c_master_init();
	
	//One second delay
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	

	//Structure for initializing the BNO055
	struct bno055_t myBNO = { .bus_write = BNO055_I2C_bus_write, .bus_read =
			BNO055_I2C_bus_read, .dev_addr = BNO055_I2C_ADDR1, 
			.delay_msec =BNO055_delay_msek };
	
	//BNO055 initialization
	bno055_init(&myBNO);
	
	//BNO55 configuartion to NDOF mode
	bno055_set_operation_mode(BNO055_OPERATION_MODE_NDOF);

	//One second delay
	vTaskDelay(1000 / portTICK_PERIOD_MS);
	
	
	// struct bno055_quaternion_t quaternion_wxyz;
	// bno055_read_quaternion_wxyz(&quaternion_wxyz);

	s16 quaternion_data_w = BNO055_INIT_VALUE;
    s16 quaternion_data_x = BNO055_INIT_VALUE;
    s16 quaternion_data_y = BNO055_INIT_VALUE;
    s16 quaternion_data_z = BNO055_INIT_VALUE;

	// struct bno055_euler myEulerData;

	struct bno055_euler_float_t eulerData;
	bno055_convert_float_euler_hpr_deg(&eulerData);
	
	unsigned char accel_calib_status = 0;
	unsigned char gyro_calib_status = 0;
	unsigned char mag_calib_status = 0;
	unsigned char sys_calib_status = 0;

	bno055_get_accel_calib_stat(&accel_calib_status);
	bno055_get_mag_calib_stat(&mag_calib_status);
	bno055_get_gyro_calib_stat(&gyro_calib_status);
	bno055_get_sys_calib_stat(&sys_calib_status);
	s16 heading;
	s16 pitch;
	s16 roll;

	float rollDeg;
	float pitchDeg;
	float yawDeg;

	const double scale = (1.0 / (1 << 14));

	while(1){
		
		
		// vTaskDelay(1000/portTICK_PERIOD_MS);
		// bno055_read_euler_h(&heading);
		// printf("%d\n",heading);
		bno055_read_quaternion_w(&quaternion_data_w);
		bno055_read_quaternion_x(&quaternion_data_x);
		bno055_read_quaternion_y(&quaternion_data_y);
		bno055_read_quaternion_z(&quaternion_data_z);
		// bno055_read_euler_hrp(&myEulerData);

		printf("\n w:%f x:%f y:%f z:%f",scale*quaternion_data_w,scale*quaternion_data_x,scale*quaternion_data_y,scale*quaternion_data_z);

		// printf("%d\n",quaternion_data_w);
		//  if(bno055_read_euler_h(&heading)==BNO055_SUCCESS&&bno055_read_euler_p(&pitch)==BNO055_SUCCESS&&bno055_read_euler_r(&roll)==BNO055_SUCCESS){
		// 	rollDeg = roll / 16;
		// 	pitchDeg = pitch / 16;
		// 	yawDeg = heading / 16;
		// 	printf("\nhead: %.2f pitch: %.2f roll: %.2f",yawDeg,pitchDeg,rollDeg);
		//  }
		//  else{
		// 	 printf("\nFailed");
		//  }
		// printf("\nhello world");
		vTaskDelay(20/portTICK_PERIOD_MS);
	}
	vTaskDelete(NULL);
}



//#ifdef BNO055_API
//#define BNO055_I2C_BUS_WRITE_ARRAY_INDEX((u8)1)
//
//s8 BNO055_I2C_bus_read(u8 dev_addr, u8 reg_addr, u8 *reg_data,u8 cnt);
//
//s8 BNO055_I2C_bus_write(u8 dev_addr, u8 reg_addr, u8 *reg_data, u8 cnt);
//
//s8 I2C_routine(void);
//
//void BNO055_delay_msek(u32 msek);
//
//#endif
//
//
//static const char *TAG = "i2c-example";

//#define _I2C_NUMBER(num) I2C_NUM_##num
//#define I2C_NUMBER(num) _I2C_NUMBER(num)
//
//#define DATA_LENGTH 512                  /*!< Data buffer length of test buffer */
//#define RW_TEST_LENGTH 128               /*!< Data length for r/w test, [0,DATA_LENGTH] */
//#define DELAY_TIME_BETWEEN_ITEMS_MS 1000 /*!< delay time between different test items */
//
//#define I2C_SLAVE_SCL_IO CONFIG_I2C_SLAVE_SCL               /*!< gpio number for i2c slave clock */
//#define I2C_SLAVE_SDA_IO CONFIG_I2C_SLAVE_SDA               /*!< gpio number for i2c slave data */
//#define I2C_SLAVE_NUM I2C_NUMBER(CONFIG_I2C_SLAVE_PORT_NUM) /*!< I2C port number for slave dev */
//#define I2C_SLAVE_TX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave tx buffer size */
//#define I2C_SLAVE_RX_BUF_LEN (2 * DATA_LENGTH)              /*!< I2C slave rx buffer size */
//
//#define I2C_MASTER_SCL_IO CONFIG_I2C_MASTER_SCL               /*!< gpio number for I2C master clock */
//#define I2C_MASTER_SDA_IO CONFIG_I2C_MASTER_SDA               /*!< gpio number for I2C master data  */
//#define I2C_MASTER_NUM I2C_NUMBER(CONFIG_I2C_MASTER_PORT_NUM) /*!< I2C port number for master dev */
//#define I2C_MASTER_FREQ_HZ CONFIG_I2C_MASTER_FREQUENCY        /*!< I2C master clock frequency */
//#define I2C_MASTER_TX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
//#define I2C_MASTER_RX_BUF_DISABLE 0                           /*!< I2C master doesn't need buffer */
//
//#define BH1750_SENSOR_ADDR CONFIG_BH1750_ADDR   /*!< slave address for BH1750 sensor */
//#define BH1750_CMD_START CONFIG_BH1750_OPMODE   /*!< Operation mode */
//#define ESP_SLAVE_ADDR CONFIG_I2C_SLAVE_ADDRESS /*!< ESP32 slave address, you can set any 7bit value */
//#define WRITE_BIT I2C_MASTER_WRITE              /*!< I2C master write */
//#define READ_BIT I2C_MASTER_READ                /*!< I2C master read */
//#define ACK_CHECK_EN 0x1                        /*!< I2C master will check ack from slave*/
//#define ACK_CHECK_DIS 0x0                       /*!< I2C master will not check ack from slave */
//#define ACK_VAL 0x0                             /*!< I2C ack value */
//#define NACK_VAL 0x1                            /*!< I2C nack value */
//
//SemaphoreHandle_t print_mux = NULL;

/**
 * @brief test code to read esp-i2c-slave
 *        We need to fill the buffer of esp slave device, then master can read them out.
 *
 * _______________________________________________________________________________________
 * | start | slave_addr + rd_bit +ack | read n-1 bytes + ack | read 1 byte + nack | stop |
 * --------|--------------------------|----------------------|--------------------|------|
 *
 */

//static esp_err_t i2c_master_read_slave(i2c_port_t i2c_num, uint8_t *data_rd, size_t size)
//{
//    if (size == 0) {
//        return ESP_OK;
//    }
//    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//    i2c_master_start(cmd);
//    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | READ_BIT, ACK_CHECK_EN);
//    if (size > 1) {
//        i2c_master_read(cmd, data_rd, size - 1, ACK_VAL);
//    }
//    i2c_master_read_byte(cmd, data_rd + size - 1, NACK_VAL);
//    i2c_master_stop(cmd);
//    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
//    i2c_cmd_link_delete(cmd);
//    return ret;
//}
//
///**
// * @brief Test code to write esp-i2c-slave
// *        Master device write data to slave(both esp32),
// *        the data will be stored in slave buffer.
// *        We can read them out from slave buffer.
// *
// * ___________________________________________________________________
// * | start | slave_addr + wr_bit + ack | write n bytes + ack  | stop |
// * --------|---------------------------|----------------------|------|
// *
// */
//static esp_err_t i2c_master_write_slave(i2c_port_t i2c_num, uint8_t *data_wr, size_t size)
//{
//    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//    i2c_master_start(cmd);
//    i2c_master_write_byte(cmd, (ESP_SLAVE_ADDR << 1) | WRITE_BIT, ACK_CHECK_EN);
//    i2c_master_write(cmd, data_wr, size, ACK_CHECK_EN);
//    i2c_master_stop(cmd);
//    esp_err_t ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
//    i2c_cmd_link_delete(cmd);
//    return ret;
//}
//
///**
// * @brief test code to operate on BH1750 sensor
// *
// * 1. set operation mode(e.g One time L-resolution mode)
// * _________________________________________________________________
// * | start | slave_addr + wr_bit + ack | write 1 byte + ack  | stop |
// * --------|---------------------------|---------------------|------|
// * 2. wait more than 24 ms
// * 3. read data
// * ______________________________________________________________________________________
// * | start | slave_addr + rd_bit + ack | read 1 byte + ack  | read 1 byte + nack | stop |
// * --------|---------------------------|--------------------|--------------------|------|
// */
//static esp_err_t i2c_master_sensor_test(i2c_port_t i2c_num, uint8_t *data_h, uint8_t *data_l)
//{
//    int ret;
//    i2c_cmd_handle_t cmd = i2c_cmd_link_create();
//    i2c_master_start(cmd);
//    i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR << 1 | WRITE_BIT, ACK_CHECK_EN);
//    i2c_master_write_byte(cmd, BH1750_CMD_START, ACK_CHECK_EN);
//    i2c_master_stop(cmd);
//    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
//    i2c_cmd_link_delete(cmd);
//    if (ret != ESP_OK) {
//        return ret;
//    }
//    vTaskDelay(30 / portTICK_RATE_MS);
//    cmd = i2c_cmd_link_create();
//    i2c_master_start(cmd);
//    i2c_master_write_byte(cmd, BH1750_SENSOR_ADDR << 1 | READ_BIT, ACK_CHECK_EN);
//    i2c_master_read_byte(cmd, data_h, ACK_VAL);
//    i2c_master_read_byte(cmd, data_l, NACK_VAL);
//    i2c_master_stop(cmd);
//    ret = i2c_master_cmd_begin(i2c_num, cmd, 1000 / portTICK_RATE_MS);
//    i2c_cmd_link_delete(cmd);
//    return ret;
//}
//
///**
// * @brief i2c master initialization
// */
//static esp_err_t i2c_master_init(void)
//{
//    int i2c_master_port = I2C_MASTER_NUM;
//    i2c_config_t conf;
//    conf.mode = I2C_MODE_MASTER;
//    conf.sda_io_num = I2C_MASTER_SDA_IO;
//    conf.sda_pullup_en = GPIO_PULLUP_ENABLE;
//    conf.scl_io_num = I2C_MASTER_SCL_IO;
//    conf.scl_pullup_en = GPIO_PULLUP_ENABLE;
//    conf.master.clk_speed = I2C_MASTER_FREQ_HZ;
//    i2c_param_config(i2c_master_port, &conf);
//    return i2c_driver_install(i2c_master_port, conf.mode, I2C_MASTER_RX_BUF_DISABLE, I2C_MASTER_TX_BUF_DISABLE, 0);
//}
//
///**
// * @brief i2c slave initialization
// */
//static esp_err_t i2c_slave_init(void)
//{
//    int i2c_slave_port = I2C_SLAVE_NUM;
//    i2c_config_t conf_slave;
//    conf_slave.sda_io_num = I2C_SLAVE_SDA_IO;
//    conf_slave.sda_pullup_en = GPIO_PULLUP_ENABLE;
//    conf_slave.scl_io_num = I2C_SLAVE_SCL_IO;
//    conf_slave.scl_pullup_en = GPIO_PULLUP_ENABLE;
//    conf_slave.mode = I2C_MODE_SLAVE;
//    conf_slave.slave.addr_10bit_en = 0;
//    conf_slave.slave.slave_addr = ESP_SLAVE_ADDR;
//    i2c_param_config(i2c_slave_port, &conf_slave);
//    return i2c_driver_install(i2c_slave_port, conf_slave.mode, I2C_SLAVE_RX_BUF_LEN, I2C_SLAVE_TX_BUF_LEN, 0);
//}
//
///**
// * @brief test function to show buffer
// */
//static void disp_buf(uint8_t *buf, int len)
//{
//    int i;
//    for (i = 0; i < len; i++) {
//        printf("%02x ", buf[i]);
//        if ((i + 1) % 16 == 0) {
//            printf("\n");
//        }
//    }
//    printf("\n");
//}
//
//static void i2c_test_task(void *arg)
//{
//    int i = 0;
//    int ret;
//    uint32_t task_idx = (uint32_t)arg;
//    uint8_t *data = (uint8_t *)malloc(DATA_LENGTH);
//    uint8_t *data_wr = (uint8_t *)malloc(DATA_LENGTH);
//    uint8_t *data_rd = (uint8_t *)malloc(DATA_LENGTH);
//    uint8_t sensor_data_h, sensor_data_l;
//    int cnt = 0;
//    while (1) {
//        ESP_LOGI(TAG, "TASK[%d] test cnt: %d", task_idx, cnt++);
//        ret = i2c_master_sensor_test(I2C_MASTER_NUM, &sensor_data_h, &sensor_data_l);
//        xSemaphoreTake(print_mux, portMAX_DELAY);
//        if (ret == ESP_ERR_TIMEOUT) {
//            ESP_LOGE(TAG, "I2C Timeout");
//        } else if (ret == ESP_OK) {
//            printf("*******************\n");
//            printf("TASK[%d]  MASTER READ SENSOR( BH1750 )\n", task_idx);
//            printf("*******************\n");
//            printf("data_h: %02x\n", sensor_data_h);
//            printf("data_l: %02x\n", sensor_data_l);
//            printf("sensor val: %.02f [Lux]\n", (sensor_data_h << 8 | sensor_data_l) / 1.2);
//        } else {
//            ESP_LOGW(TAG, "%s: No ack, sensor not connected...skip...", esp_err_to_name(ret));
//        }
//        xSemaphoreGive(print_mux);
//        vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS * (task_idx + 1)) / portTICK_RATE_MS);
//        //---------------------------------------------------
//        for (i = 0; i < DATA_LENGTH; i++) {
//            data[i] = i;
//        }
//        xSemaphoreTake(print_mux, portMAX_DELAY);
//        size_t d_size = i2c_slave_write_buffer(I2C_SLAVE_NUM, data, RW_TEST_LENGTH, 1000 / portTICK_RATE_MS);
//        if (d_size == 0) {
//            ESP_LOGW(TAG, "i2c slave tx buffer full");
//            ret = i2c_master_read_slave(I2C_MASTER_NUM, data_rd, DATA_LENGTH);
//        } else {
//            ret = i2c_master_read_slave(I2C_MASTER_NUM, data_rd, RW_TEST_LENGTH);
//        }
//
//        if (ret == ESP_ERR_TIMEOUT) {
//            ESP_LOGE(TAG, "I2C Timeout");
//        } else if (ret == ESP_OK) {
//            printf("*******************\n");
//            printf("TASK[%d]  MASTER READ FROM SLAVE\n", task_idx);
//            printf("*******************\n");
//            printf("====TASK[%d] Slave buffer data ====\n", task_idx);
//            disp_buf(data, d_size);
//            printf("====TASK[%d] Master read ====\n", task_idx);
//            disp_buf(data_rd, d_size);
//        } else {
//            ESP_LOGW(TAG, "TASK[%d] %s: Master read slave error, IO not connected...\n",
//                     task_idx, esp_err_to_name(ret));
//        }
//        xSemaphoreGive(print_mux);
//        vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS * (task_idx + 1)) / portTICK_RATE_MS);
//        //---------------------------------------------------
//        int size;
//        for (i = 0; i < DATA_LENGTH; i++) {
//            data_wr[i] = i + 10;
//        }
//        xSemaphoreTake(print_mux, portMAX_DELAY);
//        //we need to fill the slave buffer so that master can read later
//        ret = i2c_master_write_slave(I2C_MASTER_NUM, data_wr, RW_TEST_LENGTH);
//        if (ret == ESP_OK) {
//            size = i2c_slave_read_buffer(I2C_SLAVE_NUM, data, RW_TEST_LENGTH, 1000 / portTICK_RATE_MS);
//        }
//        if (ret == ESP_ERR_TIMEOUT) {
//            ESP_LOGE(TAG, "I2C Timeout");
//        } else if (ret == ESP_OK) {
//            printf("*******************\n");
//            printf("TASK[%d]  MASTER WRITE TO SLAVE\n", task_idx);
//            printf("*******************\n");
//            printf("----TASK[%d] Master write ----\n", task_idx);
//            disp_buf(data_wr, RW_TEST_LENGTH);
//            printf("----TASK[%d] Slave read: [%d] bytes ----\n", task_idx, size);
//            disp_buf(data, size);
//        } else {
//            ESP_LOGW(TAG, "TASK[%d] %s: Master write slave error, IO not connected....\n",
//                     task_idx, esp_err_to_name(ret));
//        }
//        xSemaphoreGive(print_mux);
//        vTaskDelay((DELAY_TIME_BETWEEN_ITEMS_MS * (task_idx + 1)) / portTICK_RATE_MS);
//    }
//    vSemaphoreDelete(print_mux);
//    vTaskDelete(NULL);
//}

//void app_main(void)
//{
//	xTaskCreate(task_bno055_normal_mode, "i2c_bno055_task", 1024 * 4,
//			(void*) 0, 10, NULL);
//    print_mux = xSemaphoreCreateMutex();
//    ESP_ERROR_CHECK(i2c_slave_init());
//    ESP_ERROR_CHECK(i2c_master_init());
//    xTaskCreate(i2c_test_task, "i2c_test_task_0", 1024 * 2, (void *)0, 10, NULL);
//    xTaskCreate(i2c_test_task, "i2c_test_task_1", 1024 * 2, (void *)1, 10, NULL);
//}
