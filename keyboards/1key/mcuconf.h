#pragma once

#include_next <mcuconf.h>

// static_assert(1, RP_I2C_USE_I2C0);   // => FALSE
// static_assert(1, RP_I2C_USE_I2C1);   // => TRUE

#undef RP_I2C_USE_I2C0
#define RP_I2C_USE_I2C0 FALSE
#undef RP_I2C_USE_I2C1
#define RP_I2C_USE_I2C1 TRUE
