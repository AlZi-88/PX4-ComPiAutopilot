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

/**
 * @file hiwonder_emm.hpp
 *
 * I2C interface for Hiwonder encoder motor module
 * https://www.hiwonder.com/products/4-channel-encoder-motor-driver
 */

#include <drivers/drv_hrt.h>
#include <lib/drivers/device/i2c.h>
#include <lib/perf/perf_counter.h>
#include <px4_platform_common/i2c_spi_buses.h>

static constexpr uint8_t I2C_ADDR = 0x34; // I2C address
static constexpr uint8_t ADC_BAT_ADDR = 0; // Voltage address
static constexpr uint8_t MOTOR_TYPE_ADDR = 0x14; // Set the motor type
static constexpr uint8_t MOTOR_ENCODER_POLARITY_ADDR = 21; // Set the encoder direction polarity
static constexpr uint8_t MOTOR_FIXED_PWM_ADDR = 31; // Fixed PWM control, open loop
static constexpr uint8_t MOTOR_FIXED_SPEED_ADDR = 51; // Fixed speed control, closed loop
static constexpr uint8_t MOTOR_ENCODER_TOTAL_ADDR = 60; // Total pulse value of 4 encoder motors
static constexpr uint8_t MOTOR_TYPE_WITHOUT_ENCODER = 0; // Motor without encoder
static constexpr uint8_t MOTOR_TYPE_TT = 1; // TT encoder motor
static constexpr uint8_t MOTOR_TYPE_N20 = 2; // N20 encoder motor
static constexpr uint8_t MOTOR_TYPE_JGB37_520_12V_110RPM = 3; // JGB37 encoder motor
static constexpr uint32_t BUS_FREQUENCY = 100000; // I2C bus frequency

class HiwonderEMM : public device::I2C, public I2CSPIDriver<HiwonderEMM>
{
public:
	HiwonderEMM(const I2CSPIDriverConfig &config);
	~HiwonderEMM() override;

	static void print_usage();

	void RunImpl();

	int init() override;
	void print_status() override;

private:
	int probe() override;

	bool Reset();

	// bool Configure();

	perf_counter_t _bad_register_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad register")};
	perf_counter_t _bad_transfer_perf{perf_alloc(PC_COUNT, MODULE_NAME": bad transfer")};
	perf_counter_t _reset_perf{perf_alloc(PC_COUNT, MODULE_NAME": reset")};

	// hrt_abstime _reset_timestamp{0};
	// hrt_abstime _last_config_check_timestamp{0};
	// int _failure_count{0};
};
