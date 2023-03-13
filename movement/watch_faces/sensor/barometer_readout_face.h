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

#ifndef BAROMETER_READOUT_FACE_H_
#define BAROMETER_READOUT_FACE_H_

#include "movement.h"

void barometer_readout_face_setup(movement_settings_t *settings, uint8_t watch_face_index, void ** context_ptr);
void barometer_readout_face_activate(movement_settings_t *settings, void *context);
bool barometer_readout_face_loop(movement_event_t event, movement_settings_t *settings, void *context);
void barometer_readout_face_resign(movement_settings_t *settings, void *context);

typedef struct CalibrationData {
    short AC1;
    short AC2;
    short AC3;
    unsigned short AC4;
    unsigned short AC5;
    unsigned short AC6;
    short B1;
    short B2;
    short MB;
    short MC;
    short MD;
} CalibrationData;

#define barometer_readout_face ((const watch_face_t){ \
    barometer_readout_face_setup, \
    barometer_readout_face_activate, \
    barometer_readout_face_loop, \
    barometer_readout_face_resign, \
    NULL, \
})

#endif // BAROMETER_READOUT_FACE_H_
