/* main.c - Application main entry point */

/*
 * Copyright (c) 2024 Nordic Semiconductor ASA
 * Copyright (c) 2015-2016 Intel Corporation
 *
 * SPDX-License-Identifier: Apache-2.0
 */
#include <zephyr/kernel.h>
#include <zephyr/device.h>
#include <zephyr/devicetree.h>

#include <zephyr/bluetooth/bluetooth.h>
#include <zephyr/bluetooth/hci.h>
#include <zephyr/bluetooth/conn.h>
#include <zephyr/bluetooth/uuid.h>
#include <zephyr/bluetooth/gatt.h>

#include <zephyr/drivers/i2c.h>

#include "max30102.h"
#include "mpu6050.h"
#include "data_svc.h"

max_ctx_t * max_c;
mpu_ctx_t * mpu_c;

struct bt_conn *my_connection;

#define DEVICE_NAME 		CONFIG_BT_DEVICE_NAME // Set in proj.conf
#define DEVICE_NAME_LEN        	(sizeof(DEVICE_NAME) - 1)
static K_SEM_DEFINE(ble_init_ok, 0, 1);

void init_max30102(struct i2c_dt_spec * dev)
{
	max_c = max_create(dev);
	uint8_t redLedBrightness = 0x30; // Options: 0=Off to 255=50mA
	uint8_t irLedBrightness = 0x30;	 // Options: 0=Off to 255=50mA
	uint8_t sampleAverage = 4;		 // Options: 1, 2, 4, 8, 16, 32
	uint8_t ledMode = 2;			 // Options: 1 = Red only, 2 = Red + IR
	int sampleRate = 3200;			 // Options: 50, 100, 200, 400, 800, 1000, 1600, 3200
	int pulseWidth = 69;			 // Options: 69, 118, 215, 411
	int adcRange = 4096;			 // Options: 2048, 4096, 8192, 16384
	max_setup(max_c, redLedBrightness, irLedBrightness, sampleAverage, ledMode, sampleRate, pulseWidth, adcRange);
	return;
}

void init_mpu6050(struct i2c_dt_spec * dev)
{
	mpu_c = MPU6050_create(dev);
    MPU6050_reset(mpu_c);
    MPU6050_initialize(mpu_c);
	return;
}

static const struct bt_data ad[] = 
{
	BT_DATA_BYTES(BT_DATA_FLAGS, (BT_LE_AD_GENERAL | BT_LE_AD_NO_BREDR)),
	BT_DATA(BT_DATA_NAME_COMPLETE, DEVICE_NAME, DEVICE_NAME_LEN),
};

static const struct bt_data sd[] = 
{
	BT_DATA_BYTES(BT_DATA_UUID128_ALL, MY_SERVICE_UUID),
};

struct bt_conn *my_connection;

static void connected(struct bt_conn *conn, uint8_t err)
{
	struct bt_conn_info info; 
	char addr[BT_ADDR_LE_STR_LEN];

	my_connection = conn;

	if (err) 
	{
		printk("Connection failed (err %u)\n", err);
		return;
	}
	else if(bt_conn_get_info(conn, &info))
	{
		printk("Could not parse connection info\n");
	}
	else
	{
		bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
		
		printk("Connection established!		\n\
		Connected to: %s					\n\
		Role: %u							\n\
		Connection interval: %u				\n\
		Slave latency: %u					\n\
		Connection supervisory timeout: %u	\n"
		, addr, info.role, info.le.interval, info.le.latency, info.le.timeout);
	}
}

static void disconnected(struct bt_conn *conn, uint8_t reason)
{
	printk("Disconnected (reason %u)\n", reason);
}

static bool le_param_req(struct bt_conn *conn, struct bt_le_conn_param *param)
{
	//If acceptable params, return true, otherwise return false.
	return true; 
}

static void le_param_updated(struct bt_conn *conn, uint16_t interval, uint16_t latency, uint16_t timeout)
{
	struct bt_conn_info info; 
	char addr[BT_ADDR_LE_STR_LEN];
	
	if(bt_conn_get_info(conn, &info))
	{
		printk("Could not parse connection info\n");
	}
	else
	{
		bt_addr_le_to_str(bt_conn_get_dst(conn), addr, sizeof(addr));
		
		printk("Connection parameters updated!	\n\
		Connected to: %s						\n\
		New Connection Interval: %u				\n\
		New Slave Latency: %u					\n\
		New Connection Supervisory Timeout: %u	\n"
		, addr, info.le.interval, info.le.latency, info.le.timeout);
	}
}

static struct bt_conn_cb conn_callbacks = 
{
	.connected				= connected,
	.disconnected   		= disconnected,
	.le_param_req			= le_param_req,
	.le_param_updated		= le_param_updated
};

static void bt_ready(int err)
{
	if (err) 
	{
		printk("BLE init failed with error code %d\n", err);
		return;
	}

	//Configure connection callbacks
	bt_conn_cb_register(&conn_callbacks);

	//Initalize services
	err = my_service_init();

	if (err) 
	{
		printk("Failed to init LBS (err:%d)\n", err);
		return;
	}

	//Start advertising
	err = bt_le_adv_start(BT_LE_ADV_CONN, ad, ARRAY_SIZE(ad),
			      sd, ARRAY_SIZE(sd));
	if (err) 
	{
		printk("Advertising failed to start (err %d)\n", err);
		return;
	}

	printk("Advertising successfully started\n");

	k_sem_give(&ble_init_ok);
}

static void error(void)
{
	while (true) {
		printk("Error!\n");
		/* Spin for ever */
		k_sleep(K_MSEC(1000)); //1000ms
	}
}

int main(void)
{
	static struct i2c_dt_spec mpu6050_dev = I2C_DT_SPEC_GET(DT_NODELABEL(i2c_device0));
	static struct i2c_dt_spec max30102_dev = I2C_DT_SPEC_GET(DT_NODELABEL(i2c_device1));
	if (!i2c_is_ready_dt(&mpu6050_dev))
	{
		printk("MPU6050 is not ready.\n");
		return 0;
	}
	printk("MPU6050 is ready.\n");
	init_mpu6050(&mpu6050_dev);
	printk("MPU6050 is initialized.\n");
	if (!i2c_is_ready_dt(&max30102_dev))
	{
		printk("MAX30102 is not ready.\n");
		return 0;
	}
	printk("MAX30102 is ready.\n");
	init_max30102(&max30102_dev);
	printk("MAX30102 is initialized.\n");
	
	int err = 0;
	uint32_t number = 0;

	printk("Starting BLE peripheral.\n");

	
	err = bt_enable(bt_ready);

	if (err) 
	{
		printk("BLE initialization failed\n");
		error(); //Catch error
	}
	
	/* 	Bluetooth stack should be ready in less than 100 msec. 								\
																							\
		We use this semaphore to wait for bt_enable to call bt_ready before we proceed 		\
		to the main loop. By using the semaphore to block execution we allow the RTOS to 	\
		execute other tasks while we wait. */	
	err = k_sem_take(&ble_init_ok, K_MSEC(500));

	if (!err) 
	{
		printk("Bluetooth initialized\n");
	} else 
	{
		printk("BLE initialization did not complete in time\n");
		error(); //Catch error
	}

	//Initalize services
	err = my_service_init();

	for (;;) 
	{
		// Main loop
		my_service_send(my_connection, (uint8_t *)&number, sizeof(number));
		number++;
		k_sleep(K_MSEC(1000)); // 1000ms
	}

	return 0;
}

