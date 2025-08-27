/****************************************************************************
 *
 *   Copyright (c) 2025 PX4 Development Team. All rights reserved.
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

#include "hiwonder_emm.hpp"

HiwonderEMM::HiwonderEMM(const I2CSPIDriverConfig &config) :
	I2C(config),
	I2CSPIDriver(config)
{
}

HiwonderEMM::~HiwonderEMM()
{
	perf_free(_reset_perf);
	perf_free(_bad_register_perf);
	perf_free(_bad_transfer_perf);
}

int HiwonderEMM::init()
{
	printf("I2C::init started\n");
	// const uint8_t cmd[2] = {MOTOR_TYPE_ADDR, MOTOR_TYPE_JGB37_520_12V_110RPM};
	// int motor_type = transfer(cmd, 2, nullptr, 0);
	// const uint8_t cmd2[2] = {MOTOR_ENCODER_POLARITY_ADDR, 0};
	// int encoder_polarity = transfer(cmd2, 2, nullptr, 0);

	int ret = I2C::init();

	if (ret != PX4_OK) {
		DEVICE_DEBUG("I2C::init failed (%i)", ret);
		return ret;
	}

	return Reset() ? 0 : -1;
}

bool HiwonderEMM::Reset()
{
	ScheduleClear();
	ScheduleNow();
	return true;
}

void HiwonderEMM::print_status()
{
	I2CSPIDriverBase::print_status();

	perf_print_counter(_reset_perf);
	perf_print_counter(_bad_register_perf);
	perf_print_counter(_bad_transfer_perf);
}

int HiwonderEMM::probe()
{
	// const uint8_t cmd = ADC_BAT_ADDR;
	// uint8_t buf[2];
	// const int ret = transfer(&cmd, 1, &buf[0], 2);
	const uint8_t cmd = MOTOR_TYPE_ADDR;
	int ret = transfer(&cmd, 1, nullptr, 0);
	return ret;
}

void HiwonderEMM::RunImpl()
{
	printf("Running HiwonderEMM\n");
}

// int read_encoder_values()
// {
// 	return 0;
// }

// int write_motor_speeds(const uint8_t *motor_speeds, const unsigned num_motors)
// {
// 	return 0;
// }
