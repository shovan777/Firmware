/****************************************************************************
 *
 *   Copyright (c) 2018 PX4 Development Team. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 *
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in
 *    the documentation and/or other materials provided with the
 *    distribution.
 * 3. Neither the name PX4 nor the names of its contributors may be
 *    used to endorse or promote products derived from this software
 *    without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 * COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 * BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
 * OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED
 * AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 * LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 * ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 *
 ****************************************************************************/


#include "PX4Gyroscope.hpp"

#include <lib/drivers/device/Device.hpp>

PX4Gyroscope::PX4Gyroscope(uint32_t device_id, uint8_t priority, enum Rotation rotation) :
	ModuleParams(nullptr),
	_sensor_gyro_pub{ORB_ID(sensor_gyro), priority},
	_sensor_gyro_control_pub{ORB_ID(sensor_gyro_control), priority},
	_device_id(device_id),
	_rotation{rotation}
{
	_sensor_gyro_pub.get().device_id = device_id;
	_sensor_gyro_pub.get().scaling = 1.0f;
	_sensor_gyro_control_pub.get().device_id = device_id;

	// set software low pass filter for controllers
	updateParams();
	UpdateCalibration();
	ConfigureFilter(_param_imu_gyro_cutoff.get());
}

void
PX4Gyroscope::set_device_type(uint8_t devtype)
{
	// current DeviceStructure
	union device::Device::DeviceId device_id;
	device_id.devid = _sensor_gyro_pub.get().device_id;

	// update to new device type
	device_id.devid_s.devtype = devtype;

	// copy back to report
	_device_id = device_id.devid;
	_sensor_gyro_pub.get().device_id = device_id.devid;
	_sensor_gyro_control_pub.get().device_id = device_id.devid;
}

void
PX4Gyroscope::set_sample_rate(unsigned rate)
{
	_sample_rate = rate;
	_filter.set_cutoff_frequency(_sample_rate, _filter.get_cutoff_freq());
}

void
PX4Gyroscope::UpdateCalibration()
{
	for (unsigned i = 0; i < 3; ++i) {
		char str[30] {};
		sprintf(str, "CAL_GYRO%u_ID", i);
		int32_t device_id = -1;

		if (param_get(param_find(str), &device_id) != OK) {
			PX4_ERR("Could not access param %s", str);
			continue;
		}

		if ((uint32_t)device_id != _device_id) {
			continue;
		}

		// scale factors (x, y, z)
		float scale[3] {};

		sprintf(str, "CAL_GYRO%u_XSCALE", i);

		if (param_get(param_find(str), &scale[0]) != OK) {
			PX4_ERR("Could not access param %s", str);
		}

		sprintf(str, "CAL_GYRO%u_YSCALE", i);

		if (param_get(param_find(str), &scale[1]) != OK) {
			PX4_ERR("Could not access param %s", str);
		}

		sprintf(str, "CAL_GYRO%u_ZSCALE", i);

		if (param_get(param_find(str), &scale[2]) != OK) {
			PX4_ERR("Could not access param %s", str);
		}

		// offsets factors (x, y, z)
		float offset[3] {};
		sprintf(str, "CAL_GYRO%u_XOFF", i);

		if (param_get(param_find(str), &offset[0]) != OK) {
			PX4_ERR("Could not access param %s", str);
		}

		sprintf(str, "CAL_GYRO%u_YOFF", i);

		if (param_get(param_find(str), &offset[1]) != OK) {
			PX4_ERR("Could not access param %s", str);
		}

		sprintf(str, "CAL_GYRO%u_ZOFF", i);

		if (param_get(param_find(str), &offset[2]) != OK) {
			PX4_ERR("Could not access param %s", str);
		}

		_calibration_offset = matrix::Vector3f{offset};
		_calibration_scale = matrix::Vector3f{scale};

		return;
	}
}

void
PX4Gyroscope::update(hrt_abstime timestamp, float x, float y, float z)
{
	// check for parameter updates
	if (_parameter_update_sub.updated()) {
		// clear update
		parameter_update_s pupdate;
		_parameter_update_sub.copy(&pupdate);

		// update parameters from storage
		ModuleParams::updateParams();
		UpdateCalibration();
	}

	sensor_gyro_s &report = _sensor_gyro_pub.get();
	report.timestamp = timestamp;

	// Apply rotation (before scaling)
	rotate_3f(_rotation, x, y, z);

	const matrix::Vector3f raw{x, y, z};

	// Apply range scale and the calibrating offset/scale
	const matrix::Vector3f val_calibrated{(((raw * report.scaling) - _calibration_offset).emult(_calibration_scale))};

	// Filtered values
	const matrix::Vector3f val_filtered{_filter.apply(val_calibrated)};


	// publish control data (filtered gyro) immediately
	bool publish_control = true;
	sensor_gyro_control_s &control = _sensor_gyro_control_pub.get();

	if (_param_imu_gyro_rate_max.get() > 0) {
		const uint64_t interval = 1e6f / _param_imu_gyro_rate_max.get();

		if (hrt_elapsed_time(&control.timestamp_sample) < interval) {
			publish_control = false;
		}
	}

	if (publish_control) {
		control.timestamp_sample = timestamp;
		val_filtered.copyTo(control.xyz);
		control.timestamp = hrt_absolute_time();
		_sensor_gyro_control_pub.update();	// publish
	}


	// Integrated values
	matrix::Vector3f integrated_value;
	uint32_t integral_dt = 0;

	if (_integrator.put(timestamp, val_calibrated, integrated_value, integral_dt)) {

		// Raw values (ADC units 0 - 65535)
		report.x_raw = x;
		report.y_raw = y;
		report.z_raw = z;

		report.x = val_filtered(0);
		report.y = val_filtered(1);
		report.z = val_filtered(2);

		report.integral_dt = integral_dt;
		report.x_integral = integrated_value(0);
		report.y_integral = integrated_value(1);
		report.z_integral = integrated_value(2);

		_sensor_gyro_pub.update();	// publish
	}
}

void
PX4Gyroscope::print_status()
{
	PX4_INFO("sample rate: %d Hz", _sample_rate);
	PX4_INFO("filter cutoff: %.3f Hz", (double)_filter.get_cutoff_freq());

	PX4_INFO("calibration scale: %.5f %.5f %.5f", (double)_calibration_scale(0), (double)_calibration_scale(1),
		 (double)_calibration_scale(2));
	PX4_INFO("calibration offset: %.5f %.5f %.5f", (double)_calibration_offset(0), (double)_calibration_offset(1),
		 (double)_calibration_offset(2));

	print_message(_sensor_gyro_pub.get());
	print_message(_sensor_gyro_control_pub.get());
}
