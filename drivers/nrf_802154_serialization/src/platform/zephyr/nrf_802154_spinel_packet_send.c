/*
 * Copyright (c) 2020, Nordic Semiconductor ASA
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#include <zephyr.h>
#include <openamp/open_amp.h>
#include <logging/log.h>

#include <nrf_802154_serialization_error.h>
#include "../../spinel_base/spinel.h"

#define LOG_LEVEL LOG_LEVEL_INFO
LOG_MODULE_REGISTER(spinel_packet_send);

// Send packet thread details
#define RING_BUFFER_LEN 4
#define SEND_THREAD_STACK_SIZE 1024

static K_SEM_DEFINE(send_sem, 0, RING_BUFFER_LEN);

typedef struct {
	uint32_t len;
	uint8_t data[SPINEL_FRAME_BUFFER_SIZE];
} buffer_t;

static buffer_t ring_buffer[RING_BUFFER_LEN];
static uint8_t rd_idx;
static uint8_t wr_idx;

static struct rpmsg_endpoint *local_ep;

static uint8_t get_rb_idx_plus_1(uint8_t i)
{
	return (i + 1) % RING_BUFFER_LEN;
}

static nrf_802154_ser_err_t spinel_packet_from_thread_send(const uint8_t *data,
							   uint32_t len)
{
	if (get_rb_idx_plus_1(wr_idx) == rd_idx) {
		LOG_ERR("No spinel buffer available to send a new packet");
		return NRF_802154_SERIALIZATION_ERROR_BACKEND_FAILURE;
	}

	LOG_DBG("Scheduling %u bytes for send thread", len);

	buffer_t *buf = &ring_buffer[wr_idx];
	wr_idx = get_rb_idx_plus_1(wr_idx);

	buf->len = len;
	memcpy(buf->data, data, len);

	k_sem_give(&send_sem);
	return (nrf_802154_ser_err_t)len;
}

static void spinel_packet_send_thread_fn(void *arg1, void *arg2, void *arg3)
{
	LOG_DBG("Spinel backend send thread started");
	while (true) {
		k_sem_take(&send_sem, K_FOREVER);
		buffer_t *buf = &ring_buffer[rd_idx];
		uint32_t expected_ret = buf->len;
		LOG_DBG("Sending %u bytes from send thread", buf->len);

		__ASSERT_NO_MSG(local_ep != NULL);
		int ret = rpmsg_send(local_ep, buf->data, buf->len);

		rd_idx = get_rb_idx_plus_1(rd_idx);

		if (ret != expected_ret) {
			nrf_802154_ser_err_data_t err = {
				.reason =
					NRF_802154_SERIALIZATION_ERROR_BACKEND_FAILURE,
			};
			nrf_802154_serialization_error(&err);
		}
	}
}

K_THREAD_DEFINE(spinel_packet_send_thread, SEND_THREAD_STACK_SIZE,
		spinel_packet_send_thread_fn, NULL, NULL, NULL, K_PRIO_COOP(0),
		0, 0);

nrf_802154_ser_err_t spinel_packet_send(struct rpmsg_endpoint *ep,
					const void *p_data, size_t data_len)
{
	if (k_is_in_isr()) {
		if (local_ep == NULL) {
			local_ep = ep;
		}
		return spinel_packet_from_thread_send(p_data, data_len);
	} else {
		LOG_DBG("Sending %u bytes directly", data_len);
		int ret = rpmsg_send(ep, p_data, data_len);
		return ((ret < 0) ?
				NRF_802154_SERIALIZATION_ERROR_BACKEND_FAILURE :
				(nrf_802154_ser_err_t)ret);
	}
}