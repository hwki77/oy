/*
  This example code is in the public domain.

  This example code is distributed in the hope that it will be useful,
  but WITHOUT ANY WARRANTY; without even the implied warranty of
  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
*/

/*
This example is a demo of the analog-to-digital converter. 
It will show the digital value of pin ANALOG_PIN in the Monitor logs.
*/
#include<stdio.h>
#include<math.h>

#include "vmtype.h"
#include "vmboard.h"
#include "vmsystem.h"
#include "vmlog.h"
#include "vmdcl.h"
#include "vmdcl_gpio.h"
#include "vmdcl_adc.h"
#include "vmtimer.h"
#include "ResID.h"
#include "vmdatetime.h"
#include "vmthread.h"

unsigned long mean1 = 1024*1000;
unsigned long rmss1 = 1;

vm_mutex_t mutex_RMS;

#if defined(__HDK_LINKIT_ONE_V1__)
    #define ANALOG_PIN  VM_PIN_D14
#elif defined(__HDK_LINKIT_ASSIST_2502__)
    #define ANALOG_PIN  VM_PIN_P10
#else
    #error ¡° Board not support¡±
#endif

static VMUINT32 g_adc_result = 0;
static VM_DCL_HANDLE g_gpio_handle_A0 = VM_DCL_HANDLE_INVALID;
static VM_DCL_HANDLE g_gpio_handle_A1 = VM_DCL_HANDLE_INVALID;
static void adc_demo_handle_sysevt(VMINT message, VMINT param);

vm_dcl_adc_control_create_object_t obj_data;
VMINT status = 0 , i;
vm_dcl_adc_control_send_start_t start_data;

#define SPI_SCLK			27
#define SPI_MOSI			28
#define SPI_MISO			29
#define SPI_SS				10

VM_DCL_HANDLE gpio_handle_SCLK;
VM_DCL_HANDLE gpio_handle_MOSI;
VM_DCL_HANDLE gpio_handle_MISO;
VM_DCL_HANDLE gpio_handle_SS;
int i;
int counter=0;
long acs_value;
int reading;
double rms_value_not_streaming = 0.0;
double rms_value = 0.0;
double nb_of_voltages = 0;
long sum_of_voltages_squared = 0;
vm_dcl_gpio_control_level_status_t data;
VM_TIME_UST_COUNT start_count;
VM_TIME_UST_COUNT end_count;
VM_TIME_UST_COUNT duration;

long readACS() {
  return acs_value;
}

void writeRMS(double* localRMS, double sovs, double nov){
	vm_mutex_lock(&mutex_RMS);
	*localRMS = sqrt(sovs/nov);
	rms_value_not_streaming = sqrt(sovs/nov);
	vm_log_info("%f",*localRMS);
	vm_mutex_unlock(&mutex_RMS);
}

void readRMS(){
	vm_mutex_lock(&mutex_RMS);
	rms_value = rms_value_not_streaming;
	vm_log_info("%f",rms_value);
	vm_mutex_unlock(&mutex_RMS);
}

void updateACS() {
	reading = readACS()*2;
	long current = mean1/500 -  (reading * 2) ;
	mean1 =  mean1  - (mean1 /4096)  + reading;
	rmss1 = rmss1  - rmss1 / 1000 +  (current * current);
}

void rms_not_streaming(VM_THREAD_HANDLE handle, void* user_data){
	double local_RMS = 0.0;

	while(1){
		sum_of_voltages_squared += ((acs_value*5.0)/1023.0)*((acs_value*5.0)/1023.0);
		nb_of_voltages++;
		if(nb_of_voltages>100.0){
			writeRMS(&local_RMS,sum_of_voltages_squared,nb_of_voltages);
			//vm_mutex_lock(mutex_RMS);
			//local_RMS = sqrt(sum_of_voltages_squared/nb_of_voltages);
			//rms_value_not_streaming = sqrt(sum_of_voltages_squared/nb_of_voltages);
			//vm_log_info("%f",rms_value_not_streaming);
			//vm_mutex_unlock(mutex_RMS);
			/*vm_log_info("rms");
			vm_log_info("%f",rms_value_not_streaming);*/
			nb_of_voltages = 0.0;
			sum_of_voltages_squared = 0.0;
		}
		vm_thread_sleep(10);
	}
}


void printACS_reading() {
	vm_log_info("reading");
	vm_log_info("%d",reading);
	vm_log_info("ACS mean");
	vm_log_info("%lu",mean1);
	vm_log_info("rmss");
	vm_log_info("%lu",rmss1);

}

void rms() {
	for(i = 0; i < 1000; i++) {
		updateACS();
		vm_thread_sleep(1);
	}
	printACS_reading();
}

void vm_main(void) 
{
    /* register system events handler */
    vm_pmng_register_system_event_callback(adc_demo_handle_sysevt);
}

void adc_demo_callback(void* parameter, VM_DCL_EVENT event, VM_DCL_HANDLE device_handle)
{
    vm_dcl_callback_data_t *data;
    vm_dcl_adc_measure_done_confirm_t * result;
    vm_dcl_adc_control_send_stop_t stop_data;
    VMINT status = 0;


    //vm_log_info("adc_demo_callback - START");
    if(parameter!=NULL)
      {
        data = ( vm_dcl_callback_data_t*)parameter;
        result = (vm_dcl_adc_measure_done_confirm_t *)(data->local_parameters);

        if( result != NULL )
        {
            double *p;

            p =(double*)&(result->value);
            g_adc_result = (unsigned int)*p;
            vm_log_info("adc_demo_callback,result = %d;",g_adc_result);
        }
     }

    /* Stop ADC */
    //stop_data.owner_id = vm_dcl_get_owner_id();
    //status = vm_dcl_control(g_gpio_handle_A0,VM_DCL_ADC_COMMAND_SEND_STOP,(void *)&stop_data);

    //vm_log_info("adc_demo_callback,result = %d;",g_adc_result);
   // vm_thread_sleep(1000);
    //adc_demo_callback(parameter,event,device_handle);

    //vm_dcl_close(g_gpio_handle_A0);

    //vm_log_info("adc_demo_callback - END");
}


static void adc_demo(void)
{
    vm_dcl_adc_control_create_object_t obj_data;
    VMINT status = 0 , i;
    vm_dcl_adc_control_send_start_t start_data;

    //vm_log_info("adc_demo - START");

    /* Set ANALOG_PIN to mode 2 */
    g_gpio_handle_A0 = vm_dcl_open(VM_DCL_GPIO, 1);
    vm_dcl_control(g_gpio_handle_A0,VM_DCL_GPIO_COMMAND_SET_MODE_2,NULL);
    vm_dcl_close(g_gpio_handle_A0);

    vm_log_info("adc_demo - set ANALOG_PIN to mode_2, handle = %d", g_gpio_handle_A0);

    /* Open ANALOG_PIN as ADC device */
    g_gpio_handle_A0 = vm_dcl_open(VM_DCL_ADC,1);
    /* register ADC result callback */
    status = vm_dcl_register_callback(g_gpio_handle_A0, VM_DCL_ADC_GET_RESULT ,(vm_dcl_callback)adc_demo_callback, (void *)NULL);

    /* Indicate to the ADC module driver to notify the result. */
    obj_data.owner_id = vm_dcl_get_owner_id();
    /* Set physical ADC channel which should be measured. */
    obj_data.channel = VM_DCL_ADC_YM_CHANNEL;
    /* Set measurement period, the unit is in ticks. */
    obj_data.period = 1;
    /* Measurement count. */
    obj_data.evaluate_count = 1000;
    /* Whether to send message to owner module or not. */
    obj_data.send_message_primitive = 1;

    /* setup ADC object */
    status = vm_dcl_control(g_gpio_handle_A0,VM_DCL_ADC_COMMAND_CREATE_OBJECT,(void *)&obj_data);

    /* start ADC */
    start_data.owner_id = vm_dcl_get_owner_id();
    status = vm_dcl_control(g_gpio_handle_A0,VM_DCL_ADC_COMMAND_SEND_START,(void *)&start_data);

    //vm_log_info("adc_demo - END");

    return;
}

/* Bitbanging to simulate SPI */
void bitbang(){
	//start_count = vm_time_ust_get_count();
	counter=0;
	while(counter<100){ // 1000 is 100ksps

		vm_dcl_control(gpio_handle_SS, VM_DCL_GPIO_COMMAND_SET_MODE_0, NULL);
		vm_dcl_control(gpio_handle_SS, VM_DCL_GPIO_COMMAND_SET_DIRECTION_OUT, NULL);
		vm_dcl_control(gpio_handle_SS, VM_DCL_GPIO_COMMAND_WRITE_HIGH, NULL);
		vm_dcl_control(gpio_handle_SCLK, VM_DCL_GPIO_COMMAND_SET_MODE_0, NULL);
		vm_dcl_control(gpio_handle_SCLK, VM_DCL_GPIO_COMMAND_SET_DIRECTION_OUT, NULL);
		vm_dcl_control(gpio_handle_SCLK, VM_DCL_GPIO_COMMAND_WRITE_LOW, NULL);
		vm_dcl_control(gpio_handle_SS, VM_DCL_GPIO_COMMAND_SET_MODE_0, NULL);
		vm_dcl_control(gpio_handle_SS, VM_DCL_GPIO_COMMAND_SET_DIRECTION_OUT, NULL);
		vm_dcl_control(gpio_handle_SS, VM_DCL_GPIO_COMMAND_WRITE_LOW, NULL);



		int commandout = 0;
		commandout |= 0x18; //  # start bit + single-ended bit
		commandout <<= 3; //    # we only need to send 5 bits here

		for (i=0; i<5; i++) {
			if (commandout & 0x80){
				vm_dcl_control(gpio_handle_MOSI, VM_DCL_GPIO_COMMAND_SET_MODE_0, NULL);
				vm_dcl_control(gpio_handle_MOSI, VM_DCL_GPIO_COMMAND_SET_DIRECTION_OUT, NULL);
				vm_dcl_control(gpio_handle_MOSI, VM_DCL_GPIO_COMMAND_WRITE_HIGH, NULL);
			}
			else{
				vm_dcl_control(gpio_handle_MOSI, VM_DCL_GPIO_COMMAND_SET_MODE_0, NULL);
				vm_dcl_control(gpio_handle_MOSI, VM_DCL_GPIO_COMMAND_SET_DIRECTION_OUT, NULL);
				vm_dcl_control(gpio_handle_MOSI, VM_DCL_GPIO_COMMAND_WRITE_LOW, NULL);
			}

			commandout <<= 1;
			vm_dcl_control(gpio_handle_SCLK, VM_DCL_GPIO_COMMAND_SET_MODE_0, NULL);
			vm_dcl_control(gpio_handle_SCLK, VM_DCL_GPIO_COMMAND_SET_DIRECTION_OUT, NULL);
			vm_dcl_control(gpio_handle_SCLK, VM_DCL_GPIO_COMMAND_WRITE_HIGH, NULL);
			vm_dcl_control(gpio_handle_SCLK, VM_DCL_GPIO_COMMAND_SET_MODE_0, NULL);
			vm_dcl_control(gpio_handle_SCLK, VM_DCL_GPIO_COMMAND_SET_DIRECTION_OUT, NULL);
			vm_dcl_control(gpio_handle_SCLK, VM_DCL_GPIO_COMMAND_WRITE_LOW, NULL);
		}


		int adcout = 0;
		// read in one empty bit, one null bit and 10 ADC bits
		for (i=0; i<12; i++) {
			vm_dcl_control(gpio_handle_SCLK, VM_DCL_GPIO_COMMAND_SET_MODE_0, NULL);
			vm_dcl_control(gpio_handle_SCLK, VM_DCL_GPIO_COMMAND_SET_DIRECTION_OUT, NULL);
			vm_dcl_control(gpio_handle_SCLK, VM_DCL_GPIO_COMMAND_WRITE_HIGH, NULL);
			vm_dcl_control(gpio_handle_SCLK, VM_DCL_GPIO_COMMAND_SET_MODE_0, NULL);
			vm_dcl_control(gpio_handle_SCLK, VM_DCL_GPIO_COMMAND_SET_DIRECTION_OUT, NULL);
			vm_dcl_control(gpio_handle_SCLK, VM_DCL_GPIO_COMMAND_WRITE_LOW, NULL);
			adcout <<= 1;
			vm_dcl_control(gpio_handle_MISO, VM_DCL_GPIO_COMMAND_SET_MODE_0, NULL);
			vm_dcl_control(gpio_handle_MISO, VM_DCL_GPIO_COMMAND_SET_DIRECTION_IN, NULL);
			vm_dcl_control(gpio_handle_MISO, VM_DCL_GPIO_COMMAND_READ,(void*)&data);
			if (data.level_status!=0){
				adcout |= 0x1;
			}
		}

		vm_dcl_control(gpio_handle_SS, VM_DCL_GPIO_COMMAND_SET_MODE_0, NULL);
		vm_dcl_control(gpio_handle_SS, VM_DCL_GPIO_COMMAND_SET_DIRECTION_OUT, NULL);
		vm_dcl_control(gpio_handle_SS, VM_DCL_GPIO_COMMAND_WRITE_HIGH, NULL);

		adcout >>= 1; //      # first bit is 'null' so drop it
		acs_value = adcout;
		//return acs_value;
		//vm_log_info("%d",acs_value);
		counter++;
	}
	 //end_count = vm_time_ust_get_count();
	// duration = vm_time_ust_get_duration(start_count, end_count);
	 //vm_log_info("%d",duration);
}

void adc_demo_handle_sysevt(VMINT message, VMINT param){
    switch (message){
    case VM_EVENT_CREATE:
        /* delay for logs */
        vm_thread_sleep(5000);

        gpio_handle_SS = vm_dcl_open(VM_DCL_GPIO, SPI_SS);

        gpio_handle_SCLK = vm_dcl_open(VM_DCL_GPIO, SPI_SCLK);

        gpio_handle_MOSI = vm_dcl_open(VM_DCL_GPIO, SPI_MOSI);

        gpio_handle_MISO = vm_dcl_open(VM_DCL_GPIO, SPI_MISO);

        vm_timer_create_non_precise(100, bitbang, NULL);
        //vm_timer_create_non_precise(1000,rms,NULL);
        //vm_timer_create_non_precise(100,rms_not_streaming,NULL);
        //
        vm_mutex_init(&mutex_RMS);
        vm_thread_create(rms_not_streaming,NULL,129);
        //readRMS();
        //vm_thread_create(test_message,NULL,130);
        //vm_thread_create(test_message2,NULL,131);
        //vm_thread_create(test_message,NULL,130);
        //rms();

        //vm_log_info("Sample of ADC - Start.");
        //adc_demo();
        //gpio_handle = vm_dcl_open(VM_DCL_GPIO,1);

        //vm_dcl_config_pin_mode(1,VM_DCL_PIN_MODE_ADC);
        /* Setup pin A1 for analog read */
        /*g_gpio_handle_A1 = vm_dcl_open(VM_DCL_GPIO, 1);
        vm_dcl_control(g_gpio_handle_A1,VM_DCL_GPIO_COMMAND_SET_MODE_2,NULL);
        vm_dcl_close(g_gpio_handle_A1);

        g_gpio_handle_A1 = vm_dcl_open(VM_DCL_ADC,1);*/

        /* register ADC result callback */
        //status = vm_dcl_register_callback(g_gpio_handle_A1, VM_DCL_ADC_GET_RESULT ,(vm_dcl_callback)adc_demo_callback, (void *)NULL);

        /* Indicate to the ADC module driver to notify the result. */
        //obj_data.owner_id = vm_dcl_get_owner_id();
        /* Set physical ADC channel which should be measured. */
        //obj_data.channel = VM_DCL_ADC_YM_CHANNEL;
        //obj_data.channel = PIN2CHANNEL(1);
        /* Set measurement period, the unit is in ticks. */
        //obj_data.period = 1;
        /* Measurement count. */
        //obj_data.evaluate_count = 1;
        /* Whether to send message to owner module or not. */
        //obj_data.send_message_primitive = 1;

        //vm_dcl_control(g_gpio_handle_A1,VM_DCL_ADC_COMMAND_CREATE_OBJECT,(void *)&obj_data);

        /* start ADC */
        //start_data.owner_id = vm_dcl_get_owner_id();
        //status = vm_dcl_control(g_gpio_handle_A1,VM_DCL_ADC_COMMAND_SEND_START,(void *)&start_data);


        //vm_timer_create_precise(1000, adc_demo, NULL);
        break;

    case VM_EVENT_QUIT:
        vm_log_info("Sample of ADC - End.");
        break;
    }
}
