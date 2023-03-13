/*
 * MIT License
 *
 * Copyright (c) 2023 Jan H. Voigt
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

#include <stdlib.h>
#include <string.h>
#include "barometer_readout_face.h"
//#include "thermistor_driver.h"
#include "watch.h"
#include "bme180.h"
#include "watch_utility.h"
#include "watch_i2c.h"
#include "math.h"

static long temperature;
static long pressure;
static CalibrationData Cal;
static int display_mode = 0;
static int oss_delay;//if oss=0: min 4.5ms. oss=1: min. 7.5ms. oss=2: min 13.5ms. oss=3: min. 25.5ms.



static void draw(void) {
    delay_ms(10);
    char buf[14];
    //watch_enable_i2c();
    //uint8_t data;
    //data = watch_i2c_read8(BMP180_ADDRESS, BMP180_REGISTER_CHIPID);
    //sprintf(buf, "TE  %6d", data);
    //watch_display_string(buf, 0);


    switch (display_mode) {
        case 0:
            int temp_C = temperature;///10.0;
            sprintf(buf, "TE C %4d", temp_C);//"TE#C%4.1f#C", temp_C);
            break;
        case 1:
            double pressure_hPa = pressure;
            sprintf(buf, "PR  %5.0f", pressure_hPa);//pressure is somehow in 10Pa instead of Pa. Drawn like this looks like hPa with 1 decimal.
            break;
        case 2:
            double altitude = 44330*(1-pow((pressure/10132.5), (1/5.255)));//pressure is somehow in 10Pa instead of Pa
            sprintf(buf, "M   %4.0f", altitude);
            break;
    }
    watch_display_string(buf, 0);
}

static void get_calibration(void) {
                watch_set_indicator(WATCH_INDICATOR_BELL);
    watch_enable_i2c();
    Cal.AC1 = watch_i2c_read8(BMP180_ADDRESS, BMP180_REGISTER_AC1);
    Cal.AC1 = Cal.AC1 << 8;
    Cal.AC1 = Cal.AC1 + watch_i2c_read8(BMP180_ADDRESS, (BMP180_REGISTER_AC1+1));
                watch_clear_indicator(WATCH_INDICATOR_BELL);
    Cal.AC2 = watch_i2c_read8(BMP180_ADDRESS, BMP180_REGISTER_AC2);
    Cal.AC2 = Cal.AC2 << 8;
    Cal.AC2 = Cal.AC2 + watch_i2c_read8(BMP180_ADDRESS, (BMP180_REGISTER_AC2+1));
                watch_set_indicator(WATCH_INDICATOR_BELL);
    Cal.AC3 = watch_i2c_read8(BMP180_ADDRESS, BMP180_REGISTER_AC3);
    Cal.AC3 = Cal.AC3 << 8;
    Cal.AC3 = Cal.AC3 + watch_i2c_read8(BMP180_ADDRESS, (BMP180_REGISTER_AC3+1));
                watch_clear_indicator(WATCH_INDICATOR_BELL);
    Cal.AC4 = watch_i2c_read8(BMP180_ADDRESS, BMP180_REGISTER_AC4);
    Cal.AC4 = Cal.AC4 << 8;
    Cal.AC4 = Cal.AC4 + watch_i2c_read8(BMP180_ADDRESS, (BMP180_REGISTER_AC4+1));
                watch_set_indicator(WATCH_INDICATOR_BELL);
    Cal.AC5 = watch_i2c_read8(BMP180_ADDRESS, BMP180_REGISTER_AC5);
    Cal.AC5 = Cal.AC5 << 8;
    Cal.AC5 = Cal.AC5 + watch_i2c_read8(BMP180_ADDRESS, (BMP180_REGISTER_AC5+1));
                watch_clear_indicator(WATCH_INDICATOR_BELL);
    Cal.AC6 = watch_i2c_read8(BMP180_ADDRESS, BMP180_REGISTER_AC6);
    Cal.AC6 = Cal.AC6 << 8;
    Cal.AC6 = Cal.AC6 + watch_i2c_read8(BMP180_ADDRESS, (BMP180_REGISTER_AC6+1));
                watch_set_indicator(WATCH_INDICATOR_BELL);
    Cal.B1 = watch_i2c_read8(BMP180_ADDRESS, BMP180_REGISTER_B1);
    Cal.B1 = Cal.B1 << 8;
    Cal.B1 = Cal.B1 + watch_i2c_read8(BMP180_ADDRESS, (BMP180_REGISTER_B1+1));
                watch_clear_indicator(WATCH_INDICATOR_BELL);
    Cal.B2 = watch_i2c_read8(BMP180_ADDRESS, BMP180_REGISTER_B2);
    Cal.B2 = Cal.B2 << 8;
    Cal.B2 = Cal.B2 + watch_i2c_read8(BMP180_ADDRESS, (BMP180_REGISTER_B2+1));
                watch_set_indicator(WATCH_INDICATOR_BELL);
    Cal.MB = watch_i2c_read8(BMP180_ADDRESS, BMP180_REGISTER_MB);
    Cal.MB = Cal.MB << 8;
    Cal.MB = Cal.MB + watch_i2c_read8(BMP180_ADDRESS, (BMP180_REGISTER_MB+1));
                watch_clear_indicator(WATCH_INDICATOR_BELL);
    Cal.MC = watch_i2c_read8(BMP180_ADDRESS, BMP180_REGISTER_MC);
    Cal.MC = Cal.MC << 8;
    Cal.MC = Cal.MC + watch_i2c_read8(BMP180_ADDRESS, (BMP180_REGISTER_MC+1));
                watch_set_indicator(WATCH_INDICATOR_BELL);
    Cal.MD = watch_i2c_read8(BMP180_ADDRESS, BMP180_REGISTER_MD);
    Cal.MD = Cal.MD << 8;
    Cal.MD = Cal.MD + watch_i2c_read8(BMP180_ADDRESS, (BMP180_REGISTER_MD+1));
                watch_clear_indicator(WATCH_INDICATOR_BELL);
}

static void get_measurement(void) {
    watch_enable_i2c();
    short oss = 0; //oversampling (0..3), 0 saves power.
    switch (oss) {
        case 0:
            oss_delay = 5;
            break;
        case 1:
            oss_delay = 8;
            break;
        case 2:
            oss_delay = 14;
            break;
        case 3:
            oss_delay = 26;
            break;
    }
    delay_ms(10);
    //get raw data
    watch_i2c_write8(BMP180_ADDRESS, BMP180_REGISTER_CONTROL, 0x2E);
    delay_ms(5);
    long UT = watch_i2c_read8(BMP180_ADDRESS, 0xF6); //uncompensated temperature: read reg 0xF6 (MSB), 0xF7 (LSB), UT=MSB<<8+LSB
    UT = UT << 8;
    UT = UT + watch_i2c_read8(BMP180_ADDRESS, 0xF7);
    watch_i2c_write8(BMP180_ADDRESS, BMP180_REGISTER_CONTROL, (0x34+(oss<<6)));
    delay_ms(oss_delay); //if oss=0: min 4.5ms. oss=1: min. 7.5ms. oss=2: min 13.5ms. oss=3: min. 25.5ms.
    long UP = watch_i2c_read8(BMP180_ADDRESS, 0xF6); //uncompensated pressure: read reg 0xF6 (MSB), 0xF7 (LSB), 0xF8 (XLSB) UT=MSB<<16+LSB<<8+XLSB
    UP = UP << 8;
    UP = UP + watch_i2c_read8(BMP180_ADDRESS, 0xF7);
    UP = UP << 8;
    UP = UP + watch_i2c_read8(BMP180_ADDRESS, 0xF8);
    UP = UP >> (8-oss);
    //calculate true T & p
    long X1 = ((UT-Cal.AC6)*Cal.AC5)>>15; //(1<<n)=pow(2, n)
    long X2 = (Cal.MC<<11)/(X1+Cal.MD);
    long B5 = X1+X2;
    temperature = (B5+8)>>4;
    long B6 = B5-4000;
    X1 = Cal.B2*(B6*B6/(1<<12))/(1<<11);
    X2 = Cal.AC2*B6/(1<<11);
    long X3 = X1+X2;
    long B3 = (((Cal.AC1*4+X3)<<oss)+2)/4;
    X1 = Cal.AC3*B6/(1<<13);
    X2 = (Cal.B1*(B6*B6/(1<<12)))/(1<<16);
    X3 = ((X1+X2)+2)/4;
    unsigned long B4 = Cal.AC4*(unsigned long)(X3+32768)/(1<<15);
    unsigned long B7 = ((unsigned long)UP-B3)*(5000>>oss);
    if (B7<0x80000000){pressure = (B7*2)/B4;}
    else {pressure = (B7/B4)*2;}
    X1 = (pressure/(1<<8))*(pressure/(1<<8));
    X1 = (X1*3038)/(1<<16);
    X2 = (-7357*pressure)/(1<<16);
    pressure = pressure+(X1+X2+3791)/16;
    watch_disable_i2c();
}

void barometer_readout_face_setup(movement_settings_t *settings, uint8_t watch_face_index, void ** context_ptr) {
    (void) settings;
    (void) watch_face_index;
    (void) context_ptr;
}

void barometer_readout_face_activate(movement_settings_t *settings, void *context) {
    (void) settings;
    (void) context;
}

bool barometer_readout_face_loop(movement_event_t event, movement_settings_t *settings, void *context) {
    (void) context;
    watch_date_time date_time = watch_rtc_get_date_time();
    switch (event.event_type) {
        case EVENT_ALARM_BUTTON_DOWN:
            display_mode++;
            if (display_mode>2) {display_mode = 0;}
            draw();
            break;
        case EVENT_ACTIVATE:
            // force a measurement to be taken immediately.
            date_time.unit.second = 0;
            watch_enable_digital_output(A1);//enable pin A1 to power sensor
            delay_ms(10);//10ms startup time according to datasheet
            //get chip id from register D0h: 0x55=BMP180/BMP085, 0x58=BMP280, 0x60=BME280, 0x61=BME680/BME688. From register 00h: 0x50=BMP380/BMP388, 0x60=BMP390. From register 01h: 0x50=BMP580/BMP581
            get_calibration();
            // fall through
        case EVENT_TICK:
            if (date_time.unit.second % 4 == 0){
                get_measurement();
                watch_set_indicator(WATCH_INDICATOR_SIGNAL);
            }
            else watch_clear_indicator(WATCH_INDICATOR_SIGNAL);
            draw();
            break;
        case EVENT_TIMEOUT:
            movement_move_to_face(0);
            break;
        default:
            movement_default_loop_handler(event, settings);
            break;
    }

    return true;
}

void barometer_readout_face_resign(movement_settings_t *settings, void *context) {
    (void) settings;
    (void) context;
    watch_disable_i2c();
    watch_disable_digital_output(A1);
}
