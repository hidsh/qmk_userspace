#pragma once

// choose second stage bootloader for xiao rp2040 (GD25Q16=2MB)
// -------------------------------------------------------------------------------------------
// CAUTION!!
// DO NOT USE "RP2040_FLASH_GD25Q64CS" BELOW BECAUSE IT HALTS XIAO RP2040, JUST LEAVE DEFAULT.
// It seems that the boot area is crashed due to conflicts with wear-leveling I guess.
// -------------------------------------------------------------------------------------------
// #define RP2040_FLASH_GD25Q64CS      // nothing but this value (=8MB) though

// for io expander
#define I2C1_SDA_PIN GP6    // XIAO RP2040
#define I2C1_SCL_PIN GP7    // XIAO RP2040
#define I2C_DRIVER I2CD1    // I2C1
#define F_SCL 100000

// matrix via mcu's gpio
// -------------------------------------------------------------------------------------------
// NOTE
// no need lines "MATRIX_ROWS", and "_COLS" below nowadays.
// -------------------------------------------------------------------------------------------
// #define MATRIX_ROWS 4
// #define MATRIX_COLS (1+8)
