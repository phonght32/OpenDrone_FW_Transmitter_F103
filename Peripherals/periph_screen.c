#include "stdio.h"
#include "hw.h"
#include "hw_intf.h"
#include "periph_screen.h"
#include "hd44780.h"

#ifdef USE_HD44780_2004
hd44780_handle_t hd44780_handle;
#endif

err_code_t periph_screen_init(void)
{
#ifdef USE_HD44780_2004
	hd44780_handle = hd44780_init();

	hd44780_cfg_t hd44780_cfg = {
		.size = HD44780_SIZE_20_4,
		.comm_mode = HD44780_COMM_MODE_I2C,
		.i2c_send = hw_intf_hd44780_i2c_send,
		.delay = HAL_Delay
	};
	hd44780_set_config(hd44780_handle, hd44780_cfg);
	hd44780_config(hd44780_handle);

	hd44780_clear(hd44780_handle);
	hd44780_gotoxy(hd44780_handle, 0, 6);
	hd44780_write_string(hd44780_handle, (uint8_t *)"Welcome");
	hd44780_gotoxy(hd44780_handle, 1, 0);
	hd44780_write_string(hd44780_handle, (uint8_t *)"OpenDroneTransmitter");
#endif
	return ERR_CODE_SUCCESS;
}

err_code_t periph_screen_show_operator_data(uint16_t left_joystick_x, uint16_t left_joystick_y, uint16_t right_joystick_x, uint16_t right_joystick_y)
{
#ifdef USE_HD44780_2004
	uint8_t row_data[20];

	hd44780_clear(hd44780_handle);

	sprintf((char *)row_data, "X: %04d Y: %04d", left_joystick_x, left_joystick_y);
	hd44780_gotoxy(hd44780_handle, 0, 0);
	hd44780_write_string(hd44780_handle, (uint8_t *)row_data);

	sprintf((char *)row_data, "X: %04d Y: %04d", right_joystick_x, right_joystick_y);
	hd44780_gotoxy(hd44780_handle, 1, 0);
	hd44780_write_string(hd44780_handle, (uint8_t *)row_data);
#endif

	return ERR_CODE_SUCCESS;
}

err_code_t periph_screen_show_stabilizer_message(uint8_t throttle, uint8_t roll, uint8_t pitch, uint8_t yaw)
{
#ifdef USE_HD44780_2004
	uint8_t row0_data[20];
	uint8_t row1_data[20];
	uint8_t row2_data[20];
	uint8_t row3_data[20];

	hd44780_clear(hd44780_handle);

	sprintf((char *)row0_data, "throttle: %03d", throttle);
	sprintf((char *)row1_data, "roll    : %03d", roll);
	sprintf((char *)row2_data, "pitch   : %03d", pitch);
	sprintf((char *)row3_data, "yaw     : %03d", yaw);

	hd44780_gotoxy(hd44780_handle, 0, 0);
	hd44780_write_string(hd44780_handle, (uint8_t *)row0_data);

	hd44780_gotoxy(hd44780_handle, 1, 0);
	hd44780_write_string(hd44780_handle, (uint8_t *)row1_data);

	hd44780_gotoxy(hd44780_handle, 2, 0);
	hd44780_write_string(hd44780_handle, (uint8_t *)row2_data);

	hd44780_gotoxy(hd44780_handle, 3, 0);
	hd44780_write_string(hd44780_handle, (uint8_t *)row3_data);
#endif

	return ERR_CODE_SUCCESS;
}
