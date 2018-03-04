#if 0

#include "mbed.h"
#include "rtos.h"
#include "XNucleoIKS01A2.h"

#define SENS	70

/**
 * 	Global Variables
 */

static XNucleoIKS01A2 *mems_exp_brd = XNucleoIKS01A2::instance(D14, D15, D4, D5);
static LSM6DSLSensor * acc_gyro = mems_exp_brd->acc_gyro;
int32_t axes[3];
int32_t offs[3];
float partial[3];
float final[3];

/*
 *  Function Prototypes
 */
void init_vector(int32_t *, uint8_t);
void init_vector(float *, uint8_t);

int main(void)
{
	uint8_t Id;
	uint32_t k = 0;

	init_vector(partial, 3);
	init_vector(final, 3);

	acc_gyro->enable_x();
	acc_gyro->enable_g();
	printf("\nInitialization done!\n");
	acc_gyro->read_id(&Id);
	printf("MEMS Id: 0x%x\n", Id);
	wait (1.5);

	acc_gyro->get_g_axes(axes);
	printf("LSM6DSL [gyro/mdps]: %6ld,\t%6ld,\t%6ld\n", axes[0], axes[1], axes[2]);

	// Set the offsets
	for (int i=0; i<3; i++) offs[i] -= axes[i];
	printf("Offsets: %6ld,\t%6ld,\t%6ld\n", offs[0], offs[1], offs[2]);

	while(true)
	{
		acc_gyro->get_x_axes(axes);
		printf("Ax: %6ld,\tAy: %6ld,\tAz: %6ld\n", axes[0], axes[1], axes[2]);
		acc_gyro->get_g_axes(axes);
		for(int i=0; i<3; i++) axes[i] -= offs[i];
		printf("Corrected Gx: %6ld,\tGy: %6ld,\tGz: %6ld\n", axes[0], axes[1], axes[2]);
		k++;
		wait_ms(250);
		for(int i=0; i<3; i++)
		{
			partial[i] = axes[i] * SENS / 1000;
			partial[i] /= 1000;
			if((axes[i] > 150) || (axes[i] <-150)) final[i] += partial[i];
		}
		printf("Final AngX: %f,\tAngY: %f,\tAngZ: %f\n", final[0], final[1], final[2]);
	}
}

void init_vector(uint32_t *vect, uint8_t dim)
{
	for(int i=0; i<dim; i++) *(vect + i*4) = 0;
}

void init_vector(float * vect, uint8_t dim)
{
	for(int i=0; i<dim; i++) *(vect + i*sizeof(*vect)) = 0.0;
}
#endif

/* Includes */
#include "mbed.h"
#include "rtos.h"
#include "XNucleoIKS01A2.h"
#include <ExtendedClock.h>

#define TIMER_TICK_ms		100

/* Instantiate the expansion board */
static XNucleoIKS01A2 *mems_expansion_board = XNucleoIKS01A2::instance(D14, D15, D4, D5);

/* Retrieve the composing elements of the expansion board */
static LSM303AGRMagSensor *magnetometer = mems_expansion_board->magnetometer;
static HTS221Sensor *hum_temp = mems_expansion_board->ht_sensor;
static LPS22HBSensor *press_temp = mems_expansion_board->pt_sensor;
static LSM6DSLSensor *acc_gyro = mems_expansion_board->acc_gyro;
static LSM303AGRAccSensor *accelerometer = mems_expansion_board->accelerometer;

void baud(int baudrate)
{
	Serial pc(USBTX, USBRX);
	pc.baud(baudrate);
}


/* Helper function for printing floats & doubles */
static char *print_double(char* str, double v, int decimalDigits=2)
{
  int i = 1;
  int intPart, fractPart;
  int len;
  char *ptr;

  /* prepare decimal digits multiplicator */
  for (;decimalDigits!=0; i*=10, decimalDigits--);

  /* calculate integer & fractinal parts */
  intPart = (int)v;
  fractPart = (int)((v-(double)(int)v)*i);

  /* fill in integer part */
  sprintf(str, "%i.", intPart);

  /* prepare fill in of fractional part */
  len = strlen(str);
  ptr = &str[len];

  /* fill in leading fractional zeros */
  for (i/=10;i>1; i/=10, ptr++) {
    if (fractPart >= i) {
      break;
    }
    *ptr = '0';
  }

  /* fill in (rest of) fractional part */
  sprintf(ptr, "%i", fractPart);

  return str;
}

/**
 * Synchronization
 */
Semaphore timer(1);

/**
 * Time synchronization thread
 */
void synch_thread(void const *name)
{
	while(true)
	{
		timer.release();
		Thread::wait(TIMER_TICK_ms);
	}
}

/* Simple main function */
int main() {
  uint8_t id;
  float value1, value2;
  char buffer1[32], buffer2[32];
  int32_t axes[3];
  int32_t gyro[3];
  uint32_t count = 0;
  uint32_t old_timing = 0;
  Thread synThd;

  /* Enable all sensors */
  hum_temp->enable();
  press_temp->enable();
  magnetometer->enable();
  accelerometer->enable();
  acc_gyro->enable_x();
  acc_gyro->enable_g();

  baud(115200);

  // Start threads
  synThd.start(callback(synch_thread, (void *)"Synch Thread"));

  printf("\r\n--- Starting new run ---\r\n");

  hum_temp->read_id(&id);
  printf("HTS221  humidity & temperature    = 0x%X\r\n", id);
  press_temp->read_id(&id);
  printf("LPS22HB  pressure & temperature   = 0x%X\r\n", id);
  magnetometer->read_id(&id);
  printf("LSM303AGR magnetometer            = 0x%X\r\n", id);
  accelerometer->read_id(&id);
  printf("LSM303AGR accelerometer           = 0x%X\r\n", id);
  acc_gyro->read_id(&id);
  printf("LSM6DSL accelerometer & gyroscope = 0x%X\r\n", id);

  printf("count, AccX, AccY, AccZ, GyroX, GyroY, GyroZ\n");

  // Start the clock

  while(1) {
#if 0
    printf("\r\n");

    hum_temp->get_temperature(&value1);
    hum_temp->get_humidity(&value2);
    printf("HTS221: [temp] %7s C,   [hum] %s%%\r\n", print_double(buffer1, value1), print_double(buffer2, value2));

    press_temp->get_temperature(&value1);
    press_temp->get_pressure(&value2);
    printf("LPS22HB: [temp] %7s C, [press] %s mbar\r\n", print_double(buffer1, value1), print_double(buffer2, value2));

    printf("---\r\n");

    magnetometer->get_m_axes(axes);
    printf("LSM303AGR [mag/mgauss]:  %6ld, %6ld, %6ld\r\n", axes[0], axes[1], axes[2]);

    accelerometer->get_x_axes(axes);
    printf("LSM303AGR [acc/mg]:  %6ld, %6ld, %6ld\r\n", axes[0], axes[1], axes[2]);

    acc_gyro->get_x_axes(axes);
    printf("LSM6DSL [acc/mg]:      %6ld, %6ld, %6ld\r\n", axes[0], axes[1], axes[2]);

    acc_gyro->get_g_axes(axes);
    printf("LSM6DSL [gyro/mdps]:   %6ld, %6ld, %6ld\r\n", axes[0], axes[1], axes[2]);

    wait(1.5);
#endif

    timer.wait();
    acc_gyro->get_x_axes(axes);
    acc_gyro->get_g_axes(gyro);
    uint32_t foo = clock_ms();
    printf("start,%d,%d,%d,%d,%d,%d,%d,#\r\n", foo - old_timing, axes[0], axes[1], axes[2], gyro[0], gyro[1], gyro[2]);
    old_timing = foo;
  }
}

#if 0
/**
 ******************************************************************************
 * @file    main.cpp
 * @author  CLab
 * @version V1.0.0
 * @date    2-December-2016
 * @brief   Simple Example application for using the X_NUCLEO_IKS01A2 
 *          MEMS Inertial & Environmental Sensor Nucleo expansion board.
 ******************************************************************************
 * @attention
 *
 * <h2><center>&copy; COPYRIGHT(c) 2016 STMicroelectronics</center></h2>
 *
 * Redistribution and use in source and binary forms, with or without modification,
 * are permitted provided that the following conditions are met:
 *   1. Redistributions of source code must retain the above copyright notice,
 *      this list of conditions and the following disclaimer.
 *   2. Redistributions in binary form must reproduce the above copyright notice,
 *      this list of conditions and the following disclaimer in the documentation
 *      and/or other materials provided with the distribution.
 *   3. Neither the name of STMicroelectronics nor the names of its contributors
 *      may be used to endorse or promote products derived from this software
 *      without specific prior written permission.
 *
 * THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
 * AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
 * IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
 * DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
 * FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
 * DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
 *  SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 * CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
 * OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 * OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 *
 ******************************************************************************
*/ 

/* Includes */
#include "mbed.h"
#include "XNucleoIKS01A2.h"

/* Instantiate the expansion board */
static XNucleoIKS01A2 *mems_expansion_board = XNucleoIKS01A2::instance(D14, D15, D4, D5);

/* Retrieve the composing elements of the expansion board */
static LSM6DSLSensor *acc_gyro = mems_expansion_board->acc_gyro;

InterruptIn mybutton(USER_BUTTON);

volatile int mems_event = 0;
volatile int toggle_hw_event_enable = 0;
static int hw_event_is_enabled = 1;
uint16_t step_count = 0;

/* User button callback. */
void pressed_cb() {
  toggle_hw_event_enable = 1;
}

/* Interrupt 1 callback. */
void int1_cb() {
  mems_event = 1;
}

/* Interrupt 2 callback. */
void int2_cb() {
  mems_event = 1;
}

/* Print the orientation. */
void send_orientation() {
  uint8_t xl = 0;
  uint8_t xh = 0;
  uint8_t yl = 0;
  uint8_t yh = 0;
  uint8_t zl = 0;
  uint8_t zh = 0;
  
  acc_gyro->get_6d_orientation_xl(&xl);
  acc_gyro->get_6d_orientation_xh(&xh);
  acc_gyro->get_6d_orientation_yl(&yl);
  acc_gyro->get_6d_orientation_yh(&yh);
  acc_gyro->get_6d_orientation_zl(&zl);
  acc_gyro->get_6d_orientation_zh(&zh);
  
  if ( xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 1 && zh == 0 ) {
    printf( "\r\n  ________________  " \
            "\r\n |                | " \
            "\r\n |  *             | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |________________| \r\n" );
  }
  
  else if ( xl == 1 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 0 ) {
    printf( "\r\n  ________________  " \
            "\r\n |                | " \
            "\r\n |             *  | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |________________| \r\n" );
  }
  
  else if ( xl == 0 && yl == 0 && zl == 0 && xh == 1 && yh == 0 && zh == 0 ) {
    printf( "\r\n  ________________  " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |  *             | " \
            "\r\n |________________| \r\n" );
  }
  
  else if ( xl == 0 && yl == 1 && zl == 0 && xh == 0 && yh == 0 && zh == 0 ) {
    printf( "\r\n  ________________  " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |                | " \
            "\r\n |             *  | " \
            "\r\n |________________| \r\n" );
  }
  
  else if ( xl == 0 && yl == 0 && zl == 0 && xh == 0 && yh == 0 && zh == 1 ) {
    printf( "\r\n  __*_____________  " \
            "\r\n |________________| \r\n" );
  }
  
  else if ( xl == 0 && yl == 0 && zl == 1 && xh == 0 && yh == 0 && zh == 0 ) {
    printf( "\r\n  ________________  " \
            "\r\n |________________| " \
            "\r\n    *               \r\n" );
  }
  
  else {
    printf( "None of the 6D orientation axes is set in LSM6DSL - accelerometer.\r\n" );
  }
}

/* Simple main function */
int main() {
  /* Attach callback to User button press */
  mybutton.fall(&pressed_cb);
  /* Attach callback to LSM6DSL INT1 */
  acc_gyro->attach_int1_irq(&int1_cb);
  /* Attach callback to LSM6DSL INT2 */
  acc_gyro->attach_int2_irq(&int2_cb);
  
  /* Enable LSM6DSL accelerometer */
  acc_gyro->enable_x();
  /* Enable HW events. */
  acc_gyro->enable_pedometer();
  acc_gyro->enable_tilt_detection();
  acc_gyro->enable_free_fall_detection();
  acc_gyro->enable_single_tap_detection();
  acc_gyro->enable_double_tap_detection();
  acc_gyro->enable_6d_orientation();
  acc_gyro->enable_wake_up_detection();
  
  printf("\r\n--- Starting new run ---\r\n");
 
  while(1) {
    if (mems_event) {
      mems_event = 0;
      LSM6DSL_Event_Status_t status;
      acc_gyro->get_event_status(&status);
      if (status.StepStatus) {
        /* New step detected, so print the step counter */
        acc_gyro->get_step_counter(&step_count);
        printf("Step counter: %d\r\n", step_count);
      }

      if (status.FreeFallStatus) {
        /* Output data. */
        printf("Free Fall Detected!\r\n");
      }

      if (status.TapStatus) {
        /* Output data. */
        printf("Single Tap Detected!\r\n");
      }

      if (status.DoubleTapStatus) {
        /* Output data. */
        printf("Double Tap Detected!\r\n");
      }

      if (status.D6DOrientationStatus) {
        /* Send 6D Orientation */
        send_orientation();
      }

      if (status.TiltStatus) {
        /* Output data. */
        printf("Tilt Detected!\r\n");
      }

      if (status.WakeUpStatus) {
        /* Output data. */
        printf("Wake Up Detected!\r\n");
      }
    }

    if (toggle_hw_event_enable) {
      toggle_hw_event_enable = 0;
      if (hw_event_is_enabled == 0) {
        /* Enable HW events. */
        acc_gyro->enable_pedometer();
        acc_gyro->enable_tilt_detection();
        acc_gyro->enable_free_fall_detection();
        acc_gyro->enable_single_tap_detection();
        acc_gyro->enable_double_tap_detection();
        acc_gyro->enable_6d_orientation();
        acc_gyro->enable_wake_up_detection();
        hw_event_is_enabled = 1;
      } else {
        acc_gyro->disable_pedometer();
        acc_gyro->disable_tilt_detection();
        acc_gyro->disable_free_fall_detection();
        acc_gyro->disable_single_tap_detection();
        acc_gyro->disable_double_tap_detection();
        acc_gyro->disable_6d_orientation();
        acc_gyro->disable_wake_up_detection();
        hw_event_is_enabled = 0;
      }
    }
  }
}
#endif
