#include "driver/i2c.h"
#include "esp_log.h"
#include <math.h>

#define I2C_PORT I2C_NUM_0
#define SDA_GPIO 10
#define SCL_GPIO 11

void sensors_init() {
    i2c_config_t conf = {
        .mode = I2C_MODE_MASTER,
        .sda_io_num = SDA_GPIO,
        .scl_io_num = SCL_GPIO,
        .master.clk_speed = 100000,
    };
    i2c_param_config(I2C_PORT, &conf);
    i2c_driver_install(I2C_PORT, conf.mode, 0, 0, 0);
}

float calc_taupunkt(float T, float RH) {
    const float a = 17.27f, b = 237.7f;
    float alpha = (a * T / (b + T)) + logf(RH / 100.0f);
    return (b * alpha) / (a - alpha);
}

// Hier: Funktionen bme_read1() und bme_read2(): Rückgabe von Temperatur/Feuchte.
bool bme_read1(float *T, float *RH);
bool bme_read2(float *T, float *RH);
