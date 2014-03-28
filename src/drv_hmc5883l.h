#pragma once

bool hmc5883lDetect(void);
void hmc5883lInit(float *calGain);
void hmc5883lRead(int16_t *magData);
