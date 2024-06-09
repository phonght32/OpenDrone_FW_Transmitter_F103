#include "hw.h"
#include "hw_intf.h"
#include "joystick.h"
#include "periph_operator.h"

#ifdef USE_JOYSTICK_MODULE
#define JOYSTICK_IDLE_VALUE 		(4096/2)
#define JOYSTICK_MAX_VALUE 			4000
#define JOYSTICK_MIN_VALUE 			100
#define JOYSTICK_FULL_SCALE			255

joystick_handle_t left_joystick_handle, right_joystick_handle;
#endif

#ifdef USE_JOYSTICK_MODULE
static int16_t joystick_constrain(int16_t value, int16_t max, int16_t min)
{
	int16_t temp_data;
	if (value > max)
	{
		temp_data = max;
	}
	else if (value < min)
	{
		temp_data = min;
	}
	else {
		temp_data = value;
	}
	return temp_data;
}
#endif

err_code_t periph_operator_init(void)
{
#ifdef USE_JOYSTICK_MODULE
	uint8_t num_sample = 20;
	uint16_t pos_value = 0;
	int16_t bias_x = 0, bias_y = 0;
	uint64_t temp_data = 0;
	uint8_t i = 0;

	left_joystick_handle = joystick_init();
	joystick_cfg_t left_joystick_cfg = {
		.bias_x = 0,
		.bias_y = 0,
		.get_pos_x = hw_intf_left_joystick_get_pos_x,
		.get_pos_y = hw_intf_left_joystick_get_pos_y,
		.get_button = hw_intf_left_joystick_get_bt_status,
	};
	joystick_set_config(left_joystick_handle, left_joystick_cfg);
	joystick_config(left_joystick_handle);

	right_joystick_handle = joystick_init();
	joystick_cfg_t right_joystick_cfg = {
		.bias_x = 0,
		.bias_y = 0,
		.get_pos_x = hw_intf_right_joystick_get_pos_x,
		.get_pos_y = hw_intf_right_joystick_get_pos_y,
		.get_button = hw_intf_right_joystick_get_bt_status,
	};
	joystick_set_config(right_joystick_handle, right_joystick_cfg);
	joystick_config(right_joystick_handle);

	/* Calculate left joystick bias */
	temp_data = 0;
	for (i = 0; i < num_sample; i++)
	{
		joystick_get_pos_x_raw(left_joystick_handle, &pos_value);
		temp_data += pos_value;
	}
	bias_x = (temp_data / num_sample) - JOYSTICK_IDLE_VALUE;

	temp_data = 0;
	for (i = 0; i < num_sample; i++)
	{
		joystick_get_pos_y_raw(left_joystick_handle, &pos_value);
		temp_data += pos_value;
	}
	bias_y = (temp_data / num_sample) - JOYSTICK_IDLE_VALUE;

	joystick_set_bias(left_joystick_handle, bias_x, bias_y);

	/* Calculate right joystick bias */
	temp_data = 0;
	for (i = 0; i < num_sample; i++)
	{
		joystick_get_pos_x_raw(right_joystick_handle, &pos_value);
		temp_data += pos_value;
	}
	bias_x = (temp_data / num_sample) - JOYSTICK_IDLE_VALUE;

	temp_data = 0;
	for (i = 0; i < num_sample; i++)
	{
		joystick_get_pos_y_raw(right_joystick_handle, &pos_value);
		temp_data += pos_value;
	}
	bias_y = (temp_data / num_sample) - JOYSTICK_IDLE_VALUE;

	joystick_set_bias(right_joystick_handle, bias_x, bias_y);

#endif

	return ERR_CODE_SUCCESS;
}

err_code_t periph_operator_get_joystick_scale(periph_operator_data_t *data)
{
	int64_t sum_left_joystick_x = 0, sum_left_joystick_y = 0;
	int64_t sum_right_joystick_x = 0, sum_right_joystick_y = 0;
	int64_t left_joystick_x = 0, left_joystick_y = 0;
	int64_t right_joystick_x = 0, right_joystick_y = 0;

#ifdef USE_JOYSTICK_MODULE
	uint8_t num_conversion = 10;
	int16_t temp_data = 0;
	uint8_t i = 0;

	for (i = 0; i < num_conversion; i++)
	{
		joystick_get_pos_x_calib(left_joystick_handle, &temp_data);
		sum_left_joystick_x += temp_data;
	}
	left_joystick_x = joystick_constrain(sum_left_joystick_x / num_conversion, JOYSTICK_MAX_VALUE, JOYSTICK_MIN_VALUE);

	for (i = 0; i < num_conversion; i++)
	{
		joystick_get_pos_y_calib(left_joystick_handle, &temp_data);
		sum_left_joystick_y += temp_data;
	}
	left_joystick_y = joystick_constrain(sum_left_joystick_y / num_conversion, JOYSTICK_MAX_VALUE, JOYSTICK_MIN_VALUE);

	for (i = 0; i < num_conversion; i++)
	{
		joystick_get_pos_x_calib(right_joystick_handle, &temp_data);
		sum_right_joystick_x += temp_data;
	}
	right_joystick_x = joystick_constrain(sum_right_joystick_x / num_conversion, JOYSTICK_MAX_VALUE, JOYSTICK_MIN_VALUE);

	for (i = 0; i < num_conversion; i++)
	{
		joystick_get_pos_y_calib(right_joystick_handle, &temp_data);
		sum_right_joystick_y += temp_data;
	}
	right_joystick_y = joystick_constrain(sum_right_joystick_y / num_conversion, JOYSTICK_MAX_VALUE, JOYSTICK_MIN_VALUE);


	data->left_joystick_x = (float)(left_joystick_x - JOYSTICK_MIN_VALUE) / (float)(JOYSTICK_MAX_VALUE - JOYSTICK_MIN_VALUE) * JOYSTICK_FULL_SCALE;
	data->left_joystick_y = (float)(left_joystick_y - JOYSTICK_MIN_VALUE) / (float)(JOYSTICK_MAX_VALUE - JOYSTICK_MIN_VALUE) * JOYSTICK_FULL_SCALE;
	data->right_joystick_x = (float)(right_joystick_x - JOYSTICK_MIN_VALUE) / (float)(JOYSTICK_MAX_VALUE - JOYSTICK_MIN_VALUE) * JOYSTICK_FULL_SCALE;
	data->right_joystick_y = (float)(right_joystick_y - JOYSTICK_MIN_VALUE) / (float)(JOYSTICK_MAX_VALUE - JOYSTICK_MIN_VALUE) * JOYSTICK_FULL_SCALE;
#endif

	return ERR_CODE_SUCCESS;
}
