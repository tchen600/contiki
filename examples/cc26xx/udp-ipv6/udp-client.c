/*
 * Copyright (c) 2014, Texas Instruments Incorporated - http://www.ti.com/
 * All rights reserved.
 *
 * Redistribution and use in source and binary forms, with or without
 * modification, are permitted provided that the following conditions
 * are met:
 * 1. Redistributions of source code must retain the above copyright
 *    notice, this list of conditions and the following disclaimer.
 * 2. Redistributions in binary form must reproduce the above copyright
 *    notice, this list of conditions and the following disclaimer in the
 *    documentation and/or other materials provided with the distribution.
 * 3. Neither the name of the copyright holder nor the names of its
 *    contributors may be used to endorse or promote products derived
 *    from this software without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 * ``AS IS'' AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 * LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 * FOR A PARTICULAR PURPOSE ARE DISCLAIMED.  IN NO EVENT SHALL THE
 * COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 * INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES
 * (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 * SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION)
 * HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT,
 * STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
 * ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED
 * OF THE POSSIBILITY OF SUCH DAMAGE.
 */
/*---------------------------------------------------------------------------*/
/**
 * \addtogroup cc26xx-platforms
 * @{
 *
 * \defgroup cc26xx-examples CC26xx Example Projects
 *
 * Example projects for CC26xx-based platforms.
 * @{
 *
 * \defgroup cc26xx-demo CC26xx Demo Project
 *
 *   Example project demonstrating the CC26xx platforms
 *
 *   This example will work for the following boards:
 *   - srf06-cc26xx: SmartRF06EB + CC26XX EM
 *   - sensortag-cc26xx: CC26XX sensortag
 *
 *   By default, the example will build for the srf06-cc26xx board. To switch
 *   between platforms:
 *   - make clean
 *   - make BOARD=sensortag-cc26xx savetarget
 *
 *     or
 *
 *     make BOARD=srf06-cc26xx savetarget
 *
 *   This is an IPv6/RPL-enabled example. Thus, if you have a border router in
 *   your installation (same RDC layer, same PAN ID and RF channel), you should
 *   be able to ping6 this demo node.
 *
 *   This example also demonstrates CC26xx BLE operation. The process starts
 *   the BLE beacon daemon (implemented in the RF driver). The daemon will
 *   send out a BLE beacon periodically. Use any BLE-enabled application (e.g.
 *   LightBlue on OS X or the TI BLE Multitool smartphone app) and after a few
 *   seconds the cc26xx device will be discovered.
 *
 * - etimer/clock : Every CC26XX_DEMO_LOOP_INTERVAL clock ticks the LED defined
 *                  as CC26XX_DEMO_LEDS_PERIODIC will toggle and the device
 *                  will print out readings from some supported sensors
 * - sensors      : Some sensortag sensors are read asynchronously (see sensor
 *                  documentation). For those, this example will print out
 *                  readings in a staggered fashion at a random interval
 * - Buttons      : CC26XX_DEMO_SENSOR_1 button will toggle CC26XX_DEMO_LEDS_BUTTON
 *                - CC26XX_DEMO_SENSOR_2 turns on LEDS_REBOOT and causes a
 *                  watchdog reboot
 *                - The remaining buttons will just print something
 *                - The example also shows how to retrieve the duration of a
 *                  button press (in ticks). The driver will generate a
 *                  sensors_changed event upon button release
 * - Reed Relay   : Will toggle the sensortag buzzer on/off
 *
 * @{
 *
 * \file
 *     Example demonstrating the cc26xx platforms
 */
#include "contiki.h"
#include "sys/etimer.h"
#include "sys/ctimer.h"
#include "dev/leds.h"
#include "board-peripherals.h"
#include "rf-core/rf-ble.h"
#include "ti-lib.h"
#include <stdio.h>
#include <stdint.h>
#include "contiki-lib.h"
#include "contiki-net.h"
#include "net/ip/resolv.h"
#include <stdbool.h>
//#define DEBUG DEBUG_PRINT
#define DEBUG DEBUG_NONE
#include "net/ip/uip-debug.h"
#include <math.h>
#include <sensors.h>

/*---------------------------------------------------------------------------*/
#define CC26XX_DEMO_LOOP_INTERVAL       (CLOCK_SECOND * 4)
#define CC26XX_DEMO_LEDS_PERIODIC       LEDS_YELLOW
#define CC26XX_DEMO_LEDS_BUTTON         LEDS_RED
#define CC26XX_DEMO_LEDS_REBOOT         LEDS_ALL
/*---------------------------------------------------------------------------*/
#define CC26XX_DEMO_SENSOR_NONE         (void *)0xFFFFFFFF

//#define CC26XX_DEMO_SENSOR_1     &button_left_sensor
//#define CC26XX_DEMO_SENSOR_2     &button_right_sensor
#define BOARD_SENSORTAG		1

#define CC26XX_DEMO_SENSOR_3     CC26XX_DEMO_SENSOR_NONE
#define CC26XX_DEMO_SENSOR_4     CC26XX_DEMO_SENSOR_NONE
#define CC26XX_DEMO_SENSOR_5     &reed_relay_sensor
#define RAD_TO_DEGREE           (float)(180.0 / 3.14159)

/*---------------------------------------------------------------------------*/
static struct etimer et;
#define MAX_TX_LEN 128
#define MAX_PAYLOAD_LEN MAX_TX_LEN
char tx_str[MAX_TX_LEN] = {0};
static char node_idx = 0;
static struct uip_udp_conn *client_conn;
static double compAngleX, compAngleY;
static clock_time_t time_stamp = 0;

/*---------------------------------------------------------------------------*/
PROCESS(cc26xx_demo_process, "cc26xx demo process");
AUTOSTART_PROCESSES(&cc26xx_demo_process);
/*---------------------------------------------------------------------------*/
/*
 * Update sensor readings in a staggered fashion every SENSOR_READING_PERIOD
 * ticks + a random interval between 0 and SENSOR_READING_RANDOM ticks
 */
#define SENSOR_READING_PERIOD (CLOCK_SECOND / 10)
#define SENSOR_REPORT_TIMES_PER_SECOND 2
#define SENSOR_READING_PER_REPORT   (10 / 2)
//#define SENSOR_READING_RANDOM (CLOCK_SECOND << 4)
static struct ctimer mpu_timer;
/*---------------------------------------------------------------------------*/
static void init_mpu_reading(void *not_used);
/*---------------------------------------------------------------------------*/
static void
print_mpu_reading(int reading, char *val_str)
{
  if(reading < 0) {
    reading = -reading;
    sprintf(val_str, "-%d.%02d,", reading / 100, reading % 100);
  }
  else {
    sprintf(val_str, "%d.%02d,", reading / 100, reading % 100);
  }
  printf("%s", val_str);
}

/*---------------------------------------------------------------------------*/
static void
get_mpu_reading()
{
  int value;
  char val_str[32] = {0};
  double accX, accY, accZ;
  double gyroXRate, gyroYRate,
         pitch, roll;
  double dt;

  clock_time_t next = SENSOR_READING_PERIOD;

  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_X);
  gyroXRate = (double)value / 100.0;
#if DEBUG
  printf("MPU Gyro: X=");
  print_mpu_reading(value, val_str);
  printf(" deg/sec\n");
  strcat(tx_str, val_str);  
#endif

  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Y);
  gyroYRate = (double)value / 100.0;
#if DEBUG
  printf("MPU Gyro: Y=");
  print_mpu_reading(value, val_str);
  printf(" deg/sec\n");
  strcat(tx_str, val_str);  
#endif

  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_GYRO_Z);
#if DEBUG
  printf("MPU Gyro: Z=");
  print_mpu_reading(value, val_str);
  printf(" deg/sec\n");
  strcat(tx_str, val_str);  
#endif

  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_X);
  accX = value; //actually, should be value / 100.0
#if DEBUG
  printf("MPU Acc: X=");
  print_mpu_reading(value, val_str);
  printf(" G\n");
  strcat(tx_str, val_str);
#endif

  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Y);
  accY = value;
#if DEBUG
  printf("MPU Acc: Y=");
  print_mpu_reading(value, val_str);
  printf(" G\n");
  strcat(tx_str, val_str);  
#endif

  value = mpu_9250_sensor.value(MPU_9250_SENSOR_TYPE_ACC_Z);
  accZ = value;
#if DEBUG
  printf("MPU Acc: Z=");
  print_mpu_reading(value, val_str);
  printf(" G\n");
  strcat(tx_str, val_str);
#endif

  pitch = atan(accX / sqrt(pow(accY,2) + pow(accZ, 2))) * RAD_TO_DEGREE;
  printf("pitch=");
  print_mpu_reading(pitch * 100.0, val_str);
  printf(" degree\n");
  strcat(tx_str, val_str);

  roll  = atan(accY / sqrt(pow(accX,2) + pow(accZ, 2))) * RAD_TO_DEGREE;
  printf("roll=");
  print_mpu_reading(roll * 100.0, val_str);
  printf(" degree\n");
  strcat(tx_str, val_str);

  if (!time_stamp) {
	  time_stamp = clock_time();
	  compAngleX = roll;
	  compAngleY = pitch;
  } else {
     dt = (double)((unsigned long)clock_time() - time_stamp) / ((double)CLOCK_SECOND);

     time_stamp = clock_time();
#if 0
     if (roll < -90 || roll > 90) {
        compAngleX = roll;
     }

     if (pitch < -90 || roll > 90) {
        compAngleY = pitch;
     }
#endif
     compAngleX = 0.93 * (compAngleX + gyroXRate * dt) + 0.07 * roll;
     compAngleY = 0.93 * (compAngleY + gyroYRate * dt) + 0.07 * pitch;

     printf("Pitch=");
     print_mpu_reading(compAngleY * 100.0, val_str);
     printf(" degree\n");
     strcat(tx_str, val_str);

     printf("Roll=");
     print_mpu_reading(compAngleX * 100.0, val_str);
     printf(" degree\n");
     strcat(tx_str, val_str);
  }

//  SENSORS_DEACTIVATE(mpu_9250_sensor);

  ctimer_set(&mpu_timer, next, init_mpu_reading, NULL);
}

/*---------------------------------------------------------------------------*/
static void
init_mpu_reading(void *not_used)
{
  mpu_9250_sensor.configure(SENSORS_ACTIVE, MPU_9250_SENSOR_TYPE_ALL);
}

/*---------------------------------------------------------------------------*/
static void
send_message(void)
{
  static int seq_id = 0;
  int remainder = seq_id % SENSOR_READING_PER_REPORT;

  if (remainder == 0) {
     printf("Client sending to: ");
     PRINT6ADDR(&client_conn->ripaddr);
   //  sprintf(buf, "Hello %d from the client", ++seq_id);
     printf(" (msg(%d): %s)\n", seq_id, tx_str);
   #if SEND_TOO_LARGE_PACKET_TO_TEST_FRAGMENTATION
     uip_udp_packet_send(client_conn, tx_str, UIP_APPDATA_SIZE);
   #else /* SEND_TOO_LARGE_PACKET_TO_TEST_FRAGMENTATION */
     uip_udp_packet_send(client_conn, tx_str, strlen(tx_str));
   #endif /* SEND_TOO_LARGE_PACKET_TO_TEST_FRAGMENTATION */
  }
  ++seq_id;
}
/*---------------------------------------------------------------------------*/
static void
print_local_addresses(void)
{
  int i;
  uint8_t state;

  printf("Client IPv6 addresses: ");
  for(i = 0; i < UIP_DS6_ADDR_NB; i++) {
    state = uip_ds6_if.addr_list[i].state;
    if(uip_ds6_if.addr_list[i].isused &&
       (state == ADDR_TENTATIVE || state == ADDR_PREFERRED)) {
      PRINT6ADDR(&uip_ds6_if.addr_list[i].ipaddr);
      printf("\n");
    }
  }
}

/*---------------------------------------------------------------------------*/
PROCESS_THREAD(cc26xx_demo_process, ev, data)
{
  uip_ipaddr_t ipaddr;
char node_idx_0, node_idx_1;
  PROCESS_BEGIN();

  printf("CC26XX udp-client, UIP_APPDATA_SIZE %d\n", UIP_APPDATA_SIZE);
/*
 * Tao: DP0 & DP1 pins are used to number the slave node,
 * so server can know which node sends the message
 */
  ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_DP0);
  ti_lib_ioc_io_port_pull_set(BOARD_IOID_DP0, IOC_NO_IOPULL);
  ti_lib_ioc_pin_type_gpio_input(BOARD_IOID_DP1);
  ti_lib_ioc_io_port_pull_set(BOARD_IOID_DP1, IOC_NO_IOPULL);

node_idx_0 = ti_lib_gpio_pin_read(BOARD_IOID_DP0);
node_idx_1 = ti_lib_gpio_pin_read(BOARD_IOID_DP1);

node_idx = node_idx_0 | (node_idx_1 << 1); 
node_idx = 0; // hard code to 0 for now
  /* Init the BLE advertisement daemon */
  rf_ble_beacond_config(0, BOARD_STRING);
  rf_ble_beacond_start();

  etimer_set(&et, CC26XX_DEMO_LOOP_INTERVAL);
  init_mpu_reading(NULL);

  print_local_addresses();
  uip_ip6addr(&ipaddr, 0xfe80, 0, 0, 0, 0x0212, 0x4b00, 0x079b, 0x3986);
  /* new connection with remote host */
  client_conn = udp_new(&ipaddr, UIP_HTONS(3000), NULL);
  udp_bind(client_conn, UIP_HTONS(3001));

  printf("Created a connection with the server ");
  PRINT6ADDR(&client_conn->ripaddr);
  printf(" local/remote port %u/%u\n",
	UIP_HTONS(client_conn->lport), UIP_HTONS(client_conn->rport));

  while(1) {

    PROCESS_YIELD();

    if(ev == PROCESS_EVENT_TIMER) {
      if(data == &et) {
        leds_toggle(CC26XX_DEMO_LEDS_PERIODIC);
        etimer_set(&et, CC26XX_DEMO_LOOP_INTERVAL);
      }
    } else if(ev == sensors_event) {
	if(ev == sensors_event && data == &mpu_9250_sensor) {
		//which node reports
		sprintf(tx_str, "%d,", node_idx);
		get_mpu_reading();
		send_message();
      }
    }
  }

  PROCESS_END();
}

