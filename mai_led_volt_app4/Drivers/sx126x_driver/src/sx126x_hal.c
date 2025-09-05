/**
 * @file      sx126x_hal.h
 *
 * @brief     Hardware Abstraction Layer for SX126x
 *
 * The Clear BSD License
 * Copyright Semtech Corporation 2021. All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted (subject to the limitations in the disclaimer
 * below) provided that the following conditions are met:
 *     * Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *     * Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the distribution.
 *     * Neither the name of the Semtech corporation nor the
 *       names of its contributors may be used to endorse or promote products
 *       derived from this software without specific prior written permission.
 *
 * NO EXPRESS OR IMPLIED LICENSES TO ANY PARTY'S PATENT RIGHTS ARE GRANTED BY
 * THIS LICENSE. THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND
 * CONTRIBUTORS "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT
 * NOT LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A
 * PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL SEMTECH CORPORATION BE
 * LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
 * CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
 * SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
 * INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
 * CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 * POSSIBILITY OF SUCH DAMAGE.
 */

/*
 * -----------------------------------------------------------------------------
 * --- DEPENDENCIES ------------------------------------------------------------
 */

#include <stdint.h>
#include <stdbool.h>
#include "sx126x_hal.h"

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC TYPES ------------------------------------------------------------
 */

/*
 * -----------------------------------------------------------------------------
 * --- PUBLIC FUNCTIONS PROTOTYPES ---------------------------------------------
 */

/**
 * Radio data transfer - write
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context          Radio implementation parameters
 * @param [in] command          Pointer to the buffer to be transmitted
 * @param [in] command_length   Buffer size to be transmitted
 * @param [in] data             Pointer to the buffer to be transmitted
 * @param [in] data_length      Buffer size to be transmitted
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_write(const void *context,
		const uint8_t *command, const uint16_t command_length,
		const uint8_t *data, const uint16_t data_length) {
	// Cast context
	sx126x_context *ctx = (sx126x_context*) context;

	//Poll the tranmsitters busy pin (active high)
	while (HAL_GPIO_ReadPin(ctx->busy_port, ctx->busy_pin) == GPIO_PIN_SET) {
	}

	// Activate chip select (active low)
	HAL_GPIO_WritePin(ctx->cs_port, ctx->cs_pin, GPIO_PIN_RESET);

	if (HAL_SPI_Transmit(ctx->hspi, (uint8_t*) command, command_length,
	HAL_MAX_DELAY) != HAL_OK) {
		HAL_GPIO_WritePin(ctx->cs_port, ctx->cs_pin, GPIO_PIN_SET);
		return SX126X_HAL_STATUS_ERROR;
	}

	// Transmit data
	if (data != NULL && data_length > 0) {
		if (HAL_SPI_Transmit(ctx->hspi, (uint8_t*) data, data_length,
		HAL_MAX_DELAY) != HAL_OK) {
			HAL_GPIO_WritePin(ctx->cs_port, ctx->cs_pin, GPIO_PIN_SET);
			return SX126X_HAL_STATUS_ERROR;
		}
	}

	// Deactivate chip select
	HAL_GPIO_WritePin(ctx->cs_port, ctx->cs_pin, GPIO_PIN_SET);

	return SX126X_HAL_STATUS_OK;

}

/**
 * Radio data transfer - read
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context          Radio implementation parameters
 * @param [in] command          Pointer to the buffer to be transmitted
 * @param [in] command_length   Buffer size to be transmitted
 * @param [in] data             Pointer to the buffer to be received
 * @param [in] data_length      Buffer size to be received
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_read(const void *context, const uint8_t *command,
		const uint16_t command_length, uint8_t *data,
		const uint16_t data_length) {

	// Cast context to the struct type
	sx126x_context *ctx = (sx126x_context*) context;

	// Activate chip select (active low)
	while (HAL_GPIO_ReadPin(ctx->busy_port, ctx->busy_pin) == GPIO_PIN_SET) {
	}

	HAL_GPIO_WritePin(ctx->cs_port, ctx->cs_pin, GPIO_PIN_RESET);

	if (HAL_SPI_Transmit(ctx->hspi, (uint8_t*) command, command_length,
	HAL_MAX_DELAY) != HAL_OK) {
		HAL_GPIO_WritePin(ctx->cs_port, ctx->cs_pin, GPIO_PIN_SET); // CS high when fail
		return SX126X_HAL_STATUS_ERROR;
	}

	// Receive data
	if (data != NULL && data_length > 0) {
		uint8_t dummy_byte = SX126X_NOP;
		if (HAL_SPI_TransmitReceive(ctx->hspi, &dummy_byte, data, data_length,
		HAL_MAX_DELAY) != HAL_OK) {
			HAL_GPIO_WritePin(ctx->cs_port, ctx->cs_pin, GPIO_PIN_SET); // CS high when fail
			return SX126X_HAL_STATUS_ERROR;
		}
	}

	// Deactivate chip select (active high)
	HAL_GPIO_WritePin(ctx->cs_port, ctx->cs_pin, GPIO_PIN_SET);

	return SX126X_HAL_STATUS_OK;
}

/**
 * Reset the radio
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_reset(const void *context) {
	sx126x_context *ctx = (sx126x_context*) context;

	HAL_Delay(10U);
	HAL_GPIO_WritePin(ctx->reset_port, ctx->reset_pin, GPIO_PIN_RESET);
	HAL_Delay(20U);
	HAL_GPIO_WritePin(ctx->reset_port, ctx->reset_pin, GPIO_PIN_SET);
	HAL_Delay(10U);

	return SX126X_HAL_STATUS_OK;
}

/**
 * Wake the radio up.
 *
 * @remark Shall be implemented by the user
 *
 * @param [in] context Radio implementation parameters
 *
 * @returns Operation status
 */
sx126x_hal_status_t sx126x_hal_wakeup(const void *context) {
	sx126x_context *ctx = (sx126x_context*) context;

	HAL_GPIO_WritePin(ctx->cs_port, ctx->cs_pin, GPIO_PIN_RESET);

	uint8_t nop_cmd = 0x00;

	HAL_SPI_Transmit(ctx->hspi, &nop_cmd, 1, HAL_MAX_DELAY);

	HAL_Delay(3500);

	HAL_GPIO_WritePin(ctx->cs_port, ctx->cs_pin, GPIO_PIN_SET);

	return SX126X_HAL_STATUS_OK;
}

