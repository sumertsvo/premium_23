#include "at32f421_clock.h"


//#define DEBUG
//#define DEBUG_PCB
#define DEBUG_AUTOROTATION

/*
#define PIN_STATE_ALARM 			GPIOA,GPIO_PINS_2
#define PIN_STATE_MOTOR				GPIOA,GPIO_PINS_3
#define PIN_CONTROL_RELAY 			GPIOB,GPIO_PINS_5
#define PIN_POWER_RELAY				GPIOB,GPIO_PINS_4
#define PIN_LED_RED 				GPIOA,GPIO_PINS_12
#ifdef DEBUG_PCB
#define PIN_ZUMMER 					GPIOA,GPIO_PINS_15//14
#else
#define PIN_ZUMMER 					GPIOA,GPIO_PINS_14//14
#endif
#define PIN_POWER_SENSOR 			GPIOA,GPIO_PINS_11
#define PIN_SENSOR_1 				GPIOA,GPIO_PINS_1
#define PIN_SENSOR_2 				GPIOA,GPIO_PINS_0
#define PIN_FUN 					GPIOB,GPIO_PINS_3
*/

#define PIN_SENSOR_1 			GPIOA,GPIO_PINS_3
#define PIN_SENSOR_2 			GPIOA,GPIO_PINS_4
#define PIN_SENSOR_3 			GPIOA,GPIO_PINS_5
#define PIN_SENSOR_4 			GPIOA,GPIO_PINS_6
#define PIN_METER_1 			GPIOA,GPIO_PINS_1
#define PIN_METER_2 			GPIOA,GPIO_PINS_2
#define PIN_BAT_VOLT 			GPIOA,GPIO_PINS_7
#define PIN_FUN 					GPIOA,GPIO_PINS_0
#define PIN_SOURSE_OFF 		GPIOB,GPIO_PINS_11
#define PIN_BAT_TEST 			GPIOB,GPIO_PINS_10
#define PIN_LED_1 				GPIOF,GPIO_PINS_0
#define PIN_LED_2 				GPIOC,GPIO_PINS_15
#define PIN_LED_3 				GPIOC,GPIO_PINS_14
#define PIN_LED_4 				GPIOC,GPIO_PINS_13
#define PIN_LED_RED 			GPIOF,GPIO_PINS_7
#define PIN_LED_GREEN 		GPIOF,GPIO_PINS_6
#define PIN_LED_BLUE 			GPIOA,GPIO_PINS_12
#define PIN_RF_POWER 			GPIOB,GPIO_PINS_8
#define PIN_MODULE_POWER 	GPIOB,GPIO_PINS_9
#define PIN_POWER_RELAY 	GPIOB,GPIO_PINS_13
#define PIN_CONTROL_RELAY GPIOB,GPIO_PINS_12
#define PIN_ZUMMER 				GPIOA,GPIO_PINS_11
#define PIN_RS485_DIRECT 	GPIOB,GPIO_PINS_2
#define PIN_POWER_SENSOR 	GPIOB,GPIO_PINS_2
#define PIN_TKEY 					GPIOB,GPIO_PINS_14
#define PIN_RADIO_LED 		GPIOB,GPIO_PINS_15
#define PIN_BUTTON 				GPIOF,GPIO_PINS_1
#define PIN_SPI_OUT 			GPIOB,GPIO_PINS_5
#define PIN_SPI_INP 			GPIOB,GPIO_PINS_4
#define PIN_SPI_CLK 			GPIOB,GPIO_PINS_3
#define PIN_SPI_SEL 			GPIOA,GPIO_PINS_15
#define PIN_MCU_RX_T 			GPIOB,GPIO_PINS_7
#define PIN_MCU_TX_T 			GPIOB,GPIO_PINS_6
#define PIN_MCU_RX			 	GPIOA,GPIO_PINS_10
#define PIN_MCU_TX			 	GPIOA,GPIO_PINS_9
#define PIN_STATE_MOTOR	 	GPIOB,GPIO_PINS_0
#define PIN_STATE_ALARM	 	GPIOA,GPIO_PINS_8
#define SWDCLK					 	GPIOA,GPIO_PINS_14
#define SWDIO						 	GPIOA,GPIO_PINS_13

/*_____________________________________________________________________*/


/*DEFAULT_SETTINGS*/
#define VERSION 1
const char SHORT_ZUMMER_DELAY = 30;
const char LONG_ZUMMER_DELAY = 120;
const char FRIMWARE_VERSION_EEPROM_ADR = 0x01;
const unsigned AUTOROTATION_DAYS = 14;
const char MOVING_WAIT_DELAY = 1;
const unsigned LOW_WATER_RESISTANSE = 20000; 
const unsigned HIGH_WATER_RESISTANSE = 25000; 
const unsigned UP_RESISTANSE = 20000; 

const char WSP_MEAS_COUNT = 4;
const char FUN_MEAS_COUNT = 3; 
const char BUTTON_MEAS_COUNT =3;

const char RELAY_POWER_WORK_DELAY = 30; // sec
const char RELAY_GAP = 1; //sec
const char MELODY_REPEAT_DELAY = 30; //min
#ifdef DEBUG_AUTOROTATION
const uint32_t AUTOROTATION_DELAY = 300;
#else
const uint32_t AUTOROTATION_DELAY = (AUTOROTATION_DAYS * 24 * 60 * 60); //D*H*M*S
#endif
/*voltages*/
uint16_t BAD_WSP_VOLTAGE = 0;
uint16_t GOOD_WSP_VOLTAGE = 0;
uint16_t adc_result_0;
uint16_t adc_result_1;
char fun_result;
char sensor_index;
/*_____________________________________________________________________*/

/**FLAGS*/
static union {
    uint32_t value;
    struct {

        unsigned ALARM_ON : 1;
        unsigned ALARM_OFF : 1;
        unsigned FUN_HIGH : 1;
        unsigned FUN_LOW : 1;
        unsigned ALLOW_MEASURE : 1;
        unsigned ALLOW_FUN : 1;
        unsigned MEASURING: 1;
        unsigned TARGET_POS_CLOSED: 1;

        unsigned TARGET_POS_OPENED: 1;
        unsigned OPENING : 1;
        unsigned OPENED : 1;
        unsigned CLOSING : 1;
        unsigned CLOSED : 1;
        unsigned RELAY_POWER_ON : 1;
        unsigned RELAY_CONTROL_ON : 1;
        unsigned TONE_ON : 1;

        unsigned TONE_OFF : 1;
        unsigned SIREN : 1;
        unsigned ZUM_BUSY : 1;
        unsigned MOVING_ALLOWED : 1;
        unsigned : 1;
        unsigned : 1;
        unsigned LED_ON : 1;
        unsigned SEC_LOCK : 1;

        unsigned AUTOROTATION_WORK : 1;
        unsigned MELODY_ON : 1;
        unsigned  LAST_BEEP_LONG: 1;
        unsigned  MSTICK_ALLOW: 1;
        unsigned  : 1;
        unsigned  : 1;
        unsigned  : 1;
        unsigned  : 1;
    } bits;
} ff;


struct {
        unsigned LED_1 : 1;
        unsigned LED_2 : 1;
        unsigned LED_3 : 1;
        unsigned LED_4 : 1;
        unsigned RED : 1;
        unsigned GREEN : 1;
        unsigned BLUE: 1;
        unsigned LED_1_ON: 1;

        unsigned LED_2_ON: 1;
        unsigned LED_3_ON : 1;
        unsigned LED_4_ON : 1;
        char SET_RED ;
        char SET_GREEN;
        char SET_BLUE ;
        unsigned  : 1;
        unsigned  : 1;

        unsigned  : 1;
        unsigned  : 1;
        unsigned  : 1;
        unsigned  : 1;
        unsigned : 1;
        unsigned : 1;
        unsigned  : 1;
        unsigned  : 1;

        unsigned  : 1;
        unsigned  : 1;
        unsigned  : 1;
        unsigned  : 1;
        unsigned  : 1;
        unsigned  : 1;
        unsigned  : 1;
        unsigned  : 1;
} led_flags;

/*_____________________________________________________________________*/




/*TIMES*/

/*sec_div*/
uint32_t time_rotation;
unsigned time_relay_power; 
unsigned time_relay_control;
unsigned time_relay_gap;

uint64_t tone_gap_millis;
char sec_count = 0;
char time_melody; //minute
char time_moving_wait;


/*ms_div*/

uint64_t millis = 0 ;
unsigned ms_tone_delay = 0;
/*_____________________________________________________________________*/



/*counters*/
char beep_short_count;
char beep_long_count;
char beep_double_count;

/*���������������������������������������������������������������������*/
/*���������������������������������������������������������������������*/


/**
  * @brief  init adc clock function
  * @param  none
  * @retval none
  */
void mx_adc_clock_init(void){
  crm_adc_clock_div_set(CRM_ADC_DIV_2);
}



/*SERVICE*/

/*led*/



void switch_color(){
	
	static char color_ms_counter;
	
	++color_ms_counter;
	
	
	if (led_flags.SET_RED>color_ms_counter)
		led_flags.RED=1;
	else 
		led_flags.RED=0;
	
	if (led_flags.SET_GREEN>color_ms_counter)
		led_flags.GREEN=1;
	else 
		led_flags.GREEN=0;
	
	if (led_flags.SET_BLUE>color_ms_counter)
		led_flags.BLUE=1;
	else 
		led_flags.BLUE=0;
	
if (color_ms_counter>=5){

	led_flags.RED=0;
	led_flags.GREEN=0;
	led_flags.BLUE=0;
  
		color_ms_counter=0;
}
}

void framing(){
	static char frame_ms_counter;
	static char next;
	
	++frame_ms_counter;
	if (frame_ms_counter==19){
		
		led_flags.LED_1=0;
		led_flags.LED_2=0;
		led_flags.LED_3=0;
		led_flags.LED_4=0;
		
		++next;
		if (next==4){
			next=0;
		}
		
	}
	
	if (frame_ms_counter>19){
		
		switch (next){
			case (0):
				led_flags.LED_1=led_flags.LED_1_ON;
			break;
			case (1):
				led_flags.LED_2=led_flags.LED_2_ON;
			break;
			case (2):
				led_flags.LED_3=led_flags.LED_3_ON;
			break;
			case (3):
				led_flags.LED_4=led_flags.LED_4_ON;
			break;
		
		}
	frame_ms_counter=0;
	}
}

void switch_led(){
	framing();
	switch_color();
}
	
/*sound*/
void start_tone() {
    ff.bits.ZUM_BUSY = 1;
    ff.bits.TONE_ON = 1;
    ff.bits.TONE_OFF = 0;
}

void stop_tone() {

  ff.bits.ZUM_BUSY = 0;
    ff.bits.TONE_ON = 0;
    ff.bits.TONE_OFF = 1;

}

void beep_short() {

    if (beep_short_count > 0)	beep_short_count--;
    ms_tone_delay = SHORT_ZUMMER_DELAY;
    ff.bits.LAST_BEEP_LONG = 0;
    start_tone();
}


void beep_long() {

    if (beep_long_count > 0) 	beep_long_count--;
    ms_tone_delay = LONG_ZUMMER_DELAY;
    ff.bits.LAST_BEEP_LONG = 1;
    start_tone();
}


void beep_double() {
    if (ff.bits.LAST_BEEP_LONG) {
        beep_short();
    } else {
        beep_long();
    }
}

/*moving*/
void go_close() {

    if (!ff.bits.CLOSING && !ff.bits.CLOSED && ff.bits.MOVING_ALLOWED) {
        ff.bits.CLOSING = 1;
        ff.bits.OPENED = 0;
        ff.bits.OPENING = 0;

        ff.bits.RELAY_POWER_ON = 0;
        ff.bits.RELAY_CONTROL_ON = 1;

        time_relay_control = RELAY_GAP + RELAY_POWER_WORK_DELAY + RELAY_GAP;
        time_relay_power = RELAY_POWER_WORK_DELAY;
        time_relay_gap = RELAY_GAP;

        time_rotation = 0;

    }
}

void go_open() {

    if (!ff.bits.OPENED && !ff.bits.OPENING && ff.bits.MOVING_ALLOWED) {
        ff.bits.OPENING = 1;
        ff.bits.CLOSED = 0;
        ff.bits.CLOSING = 0;


        ff.bits.RELAY_CONTROL_ON = 0;
        ff.bits.RELAY_POWER_ON = 1;

        time_relay_power = RELAY_POWER_WORK_DELAY;
        return;
    }
}



void close() {
    if (!ff.bits.CLOSED && !ff.bits.OPENED)
    {
        ff.bits.OPENED=1;
    }
    if (ff.bits.OPENED && ff.bits.TARGET_POS_CLOSED && !ff.bits.OPENING && !ff.bits.CLOSING) {
        go_close();
    }
}



void open() {
    if (!ff.bits.CLOSED && !ff.bits.OPENED)
    {
        ff.bits.CLOSED=1;
    }
    if (ff.bits.CLOSED && ff.bits.TARGET_POS_OPENED) {
        go_open();
    }
}


void relay_tick() {

    if (ff.bits.OPENING && ff.bits.CLOSING) {
        return;
    }


    if (ff.bits.OPENING) {
        if (time_relay_power > 0) {
            time_relay_power--;
            if (time_relay_power == 0) {
                ff.bits.RELAY_POWER_ON = 0;
                ff.bits.OPENED = 1;
                ff.bits.OPENING = 0;
                ff.bits.AUTOROTATION_WORK = 0;
               // beep_short_count = 1;
            }
        }
    }


    if (ff.bits.CLOSING) {

        if (time_relay_gap == 0) {
            if (time_relay_power > 0) {
                ff.bits.RELAY_POWER_ON = 1;
                time_relay_power--;
            } else {
                ff.bits.RELAY_POWER_ON = 0;
            }
        } else {
            time_relay_gap--;
        }

        if (time_relay_control > 0) {
            time_relay_control--;
            if (time_relay_control == 0) {
                ff.bits.RELAY_CONTROL_ON = 0;
                ff.bits.CLOSED = 1;
                ff.bits.CLOSING = 0;
             //   beep_short_count=2;
            }
        }
    }

}


/*logic*/
void start_alarm() {
    ff.bits.TARGET_POS_CLOSED=1;
    ff.bits.TARGET_POS_OPENED=0;
    ff.bits.ALARM_ON = 1;
    ff.bits.ALARM_OFF = 0;
    ff.bits.MELODY_ON = 1;
    ff.bits.SIREN = 1;
	
	led_flags.SET_RED=3;
	led_flags.SET_GREEN=0;
	led_flags.SET_BLUE=0;
	
	led_flags.LED_1_ON=1;
	led_flags.LED_2_ON=1;
	led_flags.LED_3_ON=1;
	led_flags.LED_4_ON=1;
    sec_count=0;
}

void clear_alarm() {
    ff.bits.ALARM_ON = 0;
    ff.bits.ALARM_OFF = 1;
}

void fun_work() {
    {
        if (
            ff.bits.TARGET_POS_OPENED &&
            ff.bits.FUN_LOW &&
            !ff.bits.FUN_HIGH &&
            ff.bits.ALARM_OFF &&
            ff.bits.MOVING_ALLOWED &&
            !ff.bits.OPENING &&
            !ff.bits.CLOSING &&
            (ff.bits.CLOSED || !(ff.bits.OPENED || ff.bits.CLOSED) ) ) {
            beep_short_count = 1;
            open();
        };

        if (
            ff.bits.TARGET_POS_CLOSED &&
            ff.bits.FUN_HIGH &&
            ff.bits.MOVING_ALLOWED &&
            !ff.bits.FUN_LOW &&
            (ff.bits.OPENED || !(ff.bits.OPENED || ff.bits.CLOSED) ) &&
            !ff.bits.OPENING &&
            !ff.bits.CLOSING)
            //&& !ff.bits.AUTOROTATION_WORK)
        {
            beep_short_count = 2;
            close();
        }

    }
}


void autorotation_work() {

    if ((time_rotation > AUTOROTATION_DELAY) &&
            ff.bits.CLOSED &&
            ff.bits.ALARM_OFF &&
			ff.bits.TARGET_POS_OPENED
       ) {
        open();
        time_rotation = 0;
		ff.bits.TARGET_POS_CLOSED=0;
    }
    if ((time_rotation > AUTOROTATION_DELAY) &&
            ff.bits.OPENED
       ) {
		  ff.bits.TARGET_POS_CLOSED =1;
        close();
        beep_long_count=1;
    }

}


/*���������������������������������������������������������������������*/

/*TIMES*/

void minute_tick() {

    if (time_melody > 0) {
        time_melody--;
    } else {
        if (time_melody == 0) {
            ff.bits.SIREN = 1;
            time_melody = MELODY_REPEAT_DELAY;
        }
    };

}


void sec_30_work() {
    if (ff.bits.MELODY_ON) {
        if (ff.bits.SIREN) {
            ff.bits.SIREN = 0;
        } else {
            beep_short_count = 3;  //connect to ms200
        }
    }
}



void sec_work() {

    ff.bits.SEC_LOCK = 1;
    sec_count++;

	if (sec_count==1){
		led_flags.LED_1_ON=1;
	}
	
		if (sec_count==2){
		led_flags.LED_2_ON=1;
	}
		
		if (sec_count==3){
		led_flags.LED_3_ON=1;
	}
		
		if (sec_count==4){
		led_flags.LED_4_ON=1;
	}
		
		if (sec_count==5){
		led_flags.LED_1_ON=0;
			led_flags.LED_2_ON=0;
			led_flags.LED_3_ON=0;
			led_flags.LED_4_ON=0;
	}

	
    //back-forward gap
    if (!ff.bits.MOVING_ALLOWED) {
        if (time_moving_wait > 0) {
            time_moving_wait--;
        } else {
            ff.bits.MOVING_ALLOWED = 1;
        }
    }

    //autorotation tick
    if (!ff.bits.CLOSED) {
        time_rotation++;
    }
    relay_tick();


    //led tick
    if (ff.bits.ALARM_ON || ff.bits.CLOSING || ff.bits.OPENING) {
        ff.bits.LED_ON = !ff.bits.LED_ON;
    }
    else {
        static char iled;
        iled++;
        if (iled > 2) {
            ff.bits.LED_ON = !ff.bits.LED_ON;
            iled = 0;
        }

    }

    //melody tick
    if (ff.bits.ALARM_ON) {

        if (sec_count == 30|| sec_count==60) {
            sec_30_work();
        }

        if (sec_count == 60) {
            minute_tick();
            sec_count = 0;
        }

    }
}

void ms_200_work() {

    ff.bits.SEC_LOCK = 0;

    if (ff.bits.ALARM_ON) {
        if (ff.bits.SIREN) {
            beep_double();
        } else {
            if (beep_short_count > 0) {     //connect to sec30
                beep_short();
            }

        }
    } else if (ff.bits.ALARM_OFF) {


        if ((beep_short_count > 0) && (beep_long_count > 0)) {
            beep_double();
        } else {
            if (beep_short_count > 0) {
                beep_short();
            }
            if (beep_long_count > 0) {
                beep_long();
            }
        }

    }
}



void ms_100_work() {

    static char switch_sens_delay;

    static char f;

    if ( !ff.bits.ALARM_ON) {

        ++switch_sens_delay;
        if(switch_sens_delay>4) {
            switch_sens_delay=0;
            if (sensor_index == 1)
            {
                adc_ordinary_channel_set(ADC1,ADC_CHANNEL_3,1,ADC_SAMPLETIME_239_5);
                sensor_index = 0;

            }
            else
            {
                adc_ordinary_channel_set(ADC1,ADC_CHANNEL_4,1,ADC_SAMPLETIME_239_5);
                sensor_index = 1;
            }

        }


        if (!f) {
            gpio_bits_set(PIN_POWER_SENSOR);
            ff.bits.MEASURING=1;
            f=1;
        } else {
            adc_ordinary_conversion_trigger_set(ADC1,ADC12_ORDINARY_TRIG_SOFTWARE,TRUE);
            f=0;
        }
    }
}

void ms_tick() {


	
    static uint64_t ms200_count = 0;
    static uint64_t ms100_count = 0;
    static uint64_t ms1000_count = 0;

			ff.bits.MSTICK_ALLOW =0;
    if (ms_tone_delay > 0) {
        ms_tone_delay--;
    }   else {
        stop_tone();
    }

    if (ms100_count <=millis) {
        ms100_count = millis+100;
        ms_100_work();
    }

    if (ms200_count <= millis) {
        ms200_count = millis + 200;
        ms_200_work();
    }


    if (ms1000_count <= millis) {
        ms1000_count = millis+1000;
        //  if (!ff.bits.SEC_LOCK)
        sec_work();
    }
	
    ++millis;
   
}

/*���������������������������������������������������������������������*/

/*HARDWARE*/
char PIN_FUN_STATE_GetValue() {
    return(fun_result);
}
uint16_t   ADC_GetConversion() {
    if (adc_result_0<adc_result_1)
        return (adc_result_0);
    else return(adc_result_1);
};


void gpio_set(gpio_type *PORT, uint32_t PIN, gpio_drive_type DRIVE, gpio_mode_type MODE, gpio_output_type OUT_TYPE, gpio_pull_type PULL ) {

    gpio_init_type pinx;

    gpio_init_type *pina = &pinx;

    pinx.gpio_drive_strength= DRIVE;
    pinx.gpio_mode =MODE;
    pinx.gpio_out_type=OUT_TYPE;
    pinx.gpio_pins = PIN;
    pinx.gpio_pull = PULL;

    gpio_init( PORT,pina);

}

void timer_init() {


    nvic_irq_enable(TMR6_GLOBAL_IRQn,35,36);

    TMR6->iden_bit.ovfien =1;

    TMR6 ->ctrl1_bit.ocmen = 0;

    TMR6 ->ctrl1_bit.ovfen = 0;

    tmr_channel_buffer_enable(TMR6,TRUE);

    tmr_base_init(TMR6,1,370);

    tmr_counter_enable(TMR6,TRUE);



}

void my_gpio_init(){
	
	/*
	gpio_set(PIN_ZUMMER,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);

    gpio_set(PIN_LED_RED,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);

    gpio_set(PIN_POWER_RELAY,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);

    gpio_set(PIN_CONTROL_RELAY,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);

    gpio_set(PIN_FUN,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_INPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);

    gpio_set(PIN_POWER_SENSOR,
             GPIO_DRIVE_STRENGTH_STRONGER,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_UP);

    gpio_set(PIN_SENSOR_1,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_ANALOG,
             GPIO_OUTPUT_OPEN_DRAIN,
             GPIO_PULL_NONE);

    gpio_set(PIN_SENSOR_2,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_ANALOG,
             GPIO_OUTPUT_OPEN_DRAIN,
             GPIO_PULL_NONE);

    gpio_set(PIN_STATE_ALARM,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);
			 
	gpio_set(PIN_STATE_MOTOR,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);

	*/

gpio_set(PIN_SENSOR_1,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_ANALOG,
             GPIO_OUTPUT_OPEN_DRAIN,
             GPIO_PULL_NONE);
gpio_set(PIN_SENSOR_2,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_ANALOG,
             GPIO_OUTPUT_OPEN_DRAIN,
             GPIO_PULL_NONE);
gpio_set(PIN_SENSOR_3,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_ANALOG,
             GPIO_OUTPUT_OPEN_DRAIN,
             GPIO_PULL_NONE);
gpio_set(PIN_SENSOR_4,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_ANALOG,
             GPIO_OUTPUT_OPEN_DRAIN,
             GPIO_PULL_NONE);
gpio_set(PIN_METER_1,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_ANALOG,
             GPIO_OUTPUT_OPEN_DRAIN,
             GPIO_PULL_NONE);
gpio_set(PIN_METER_2,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_ANALOG,
             GPIO_OUTPUT_OPEN_DRAIN,
             GPIO_PULL_NONE);
gpio_set(PIN_BAT_VOLT,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_ANALOG,
             GPIO_OUTPUT_OPEN_DRAIN,
             GPIO_PULL_NONE);
gpio_set(PIN_FUN,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_ANALOG,
             GPIO_OUTPUT_OPEN_DRAIN,
             GPIO_PULL_NONE);
gpio_set(PIN_SOURSE_OFF,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);
gpio_set(PIN_BAT_TEST,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);
gpio_set(PIN_LED_1,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);
gpio_set(PIN_LED_2,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);
gpio_set(PIN_LED_3,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);
gpio_set(PIN_LED_4,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);
gpio_set(PIN_LED_RED,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);
gpio_set(PIN_LED_GREEN,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);
gpio_set(PIN_LED_BLUE,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);
gpio_set(PIN_RF_POWER,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);
gpio_set(PIN_MODULE_POWER,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);
gpio_set(PIN_POWER_RELAY,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);
gpio_set(PIN_CONTROL_RELAY,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);
gpio_set(PIN_ZUMMER,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);
gpio_set(PIN_RS485_DIRECT,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);
gpio_set(PIN_POWER_SENSOR,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);
gpio_set(PIN_TKEY,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);
gpio_set(PIN_RADIO_LED,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_OUTPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);
gpio_set(PIN_BUTTON,
             GPIO_DRIVE_STRENGTH_MODERATE,
             GPIO_MODE_INPUT,
             GPIO_OUTPUT_PUSH_PULL,
             GPIO_PULL_NONE);
	

}



void hardware_init() {
#ifndef DEBUG
	
	/*
    wdt_register_write_enable(TRUE);
    wdt_divider_set(WDT_CLK_DIV_8);
    wdt_register_write_enable(FALSE);
    wdt_enable();

    wdt_counter_reload();
	
	*/
#endif

 //   crm_hick_sclk_frequency_select(CRM_HICK_SCLK_8MHZ);
 //   crm_clock_source_enable (CRM_CLOCK_SOURCE_HICK,TRUE);

 //   crm_hick_divider_select(CRM_HICK48_NODIV);



 //   crm_ahb_div_set(CRM_AHB_DIV_1);


    crm_periph_clock_enable(CRM_GPIOA_PERIPH_CLOCK,TRUE);
    crm_periph_clock_enable(CRM_GPIOB_PERIPH_CLOCK,TRUE);
		crm_periph_clock_enable(CRM_GPIOC_PERIPH_CLOCK,TRUE);
		crm_periph_clock_enable(CRM_GPIOF_PERIPH_CLOCK,TRUE);
		
    crm_periph_clock_enable(CRM_ADC1_PERIPH_CLOCK,TRUE);
    crm_periph_clock_enable(CRM_TMR6_PERIPH_CLOCK,TRUE);


    
		my_gpio_init();

    timer_init();

    nvic_irq_enable(ADC1_CMP_IRQn,37,38);

    adc_base_config_type *adc1;
    adc_base_default_para_init(adc1);
//   adc1 ->repeat_mode = FALSE;
    adc_base_config(ADC1,adc1);

    adc_enable(ADC1,TRUE);
    adc_interrupt_enable(ADC1,ADC_CCE_INT,TRUE);
    sensor_index = 1;
    adc_ordinary_channel_set(ADC1,ADC_CHANNEL_1,1,ADC_SAMPLETIME_239_5);

}


void hardware_work() {
    gpio_bits_write(PIN_STATE_ALARM,(confirm_state) (ff.bits.ALARM_ON));
		gpio_bits_write(PIN_STATE_MOTOR,(confirm_state) (ff.bits.OPENING || ff.bits.OPENED));
    gpio_bits_write(PIN_CONTROL_RELAY,(confirm_state) (ff.bits.RELAY_CONTROL_ON));
    gpio_bits_write(PIN_POWER_RELAY,(confirm_state) (ff.bits.RELAY_POWER_ON));
   
//	gpio_bits_write(PIN_LED_RED,(confirm_state) (ff.bits.LED_ON));
	
		gpio_bits_write(PIN_LED_RED,(confirm_state) (led_flags.RED));
		gpio_bits_write(PIN_LED_GREEN,(confirm_state) (led_flags.GREEN));
		gpio_bits_write(PIN_LED_BLUE,(confirm_state) (led_flags.BLUE));
	
		gpio_bits_write(PIN_LED_1,(confirm_state) (led_flags.LED_1));
		gpio_bits_write(PIN_LED_2,(confirm_state) (led_flags.LED_2));
		gpio_bits_write(PIN_LED_3,(confirm_state) (led_flags.LED_3));
		gpio_bits_write(PIN_LED_4,(confirm_state) (led_flags.LED_4));
	
	 
    if (ff.bits.TONE_OFF) {
        gpio_bits_reset(PIN_ZUMMER);
    };
}


void zummer_switch() {

    if(ff.bits.TONE_ON) gpio_bits_write(GPIOA,GPIO_PINS_11,(confirm_state) (!GPIOA ->odt_bit.odt11));   //todo


#ifdef DEBUG
    if(ff.bits.TONE_ON) gpio_bits_write(GPIOA,GPIO_PINS_11,(confirm_state) (!GPIOA ->odt_bit.odt11));   //todo
#endif

}
void get_wsp() {

    if (ff.bits.ALLOW_MEASURE) {

        BAD_WSP_VOLTAGE = (LOW_WATER_RESISTANSE / ((UP_RESISTANSE + LOW_WATER_RESISTANSE) / 4096));
        GOOD_WSP_VOLTAGE =(HIGH_WATER_RESISTANSE / ((UP_RESISTANSE + HIGH_WATER_RESISTANSE) / 4096));

        static signed char bad_measures_counter = 0;
        uint16_t res = ADC_GetConversion();
        gpio_bits_reset(PIN_POWER_SENSOR);
        if (res < BAD_WSP_VOLTAGE) {
            bad_measures_counter++;
        } else {
            if (res > GOOD_WSP_VOLTAGE) {
                bad_measures_counter--;
            }
        }
        if (bad_measures_counter > WSP_MEAS_COUNT) {
            start_alarm();
            bad_measures_counter = WSP_MEAS_COUNT;
        }
        if (bad_measures_counter < -WSP_MEAS_COUNT) {
            clear_alarm();
            bad_measures_counter = -WSP_MEAS_COUNT;
        }
        ff.bits.ALLOW_MEASURE = 0;
    }
}
void get_fun() {
    if (ff.bits.ALLOW_FUN) {
        static signed char fun_counter;

        if (PIN_FUN_STATE_GetValue()) fun_counter--;
        else fun_counter++;

        if (fun_counter > FUN_MEAS_COUNT) {
            fun_counter = FUN_MEAS_COUNT;
            ff.bits.FUN_LOW = 0;
            ff.bits.FUN_HIGH = 1;
            ff.bits.TARGET_POS_CLOSED=1;
            ff.bits.TARGET_POS_OPENED=0;
        } else if (fun_counter<-FUN_MEAS_COUNT) {
            fun_counter = -FUN_MEAS_COUNT;
            ff.bits.FUN_LOW = 1;
            ff.bits.FUN_HIGH = 0;
            ff.bits.TARGET_POS_CLOSED=0;
            ff.bits.TARGET_POS_OPENED=1;
        }
        ff.bits.ALLOW_FUN = 0;
    }
}

void get_button() {
    
        static signed char button_counter;

        if (gpio_input_data_bit_read(PIN_BUTTON)==0) button_counter--;
        else button_counter++;

        if (button_counter > BUTTON_MEAS_COUNT) {
            button_counter = BUTTON_MEAS_COUNT;
            ++led_flags.SET_RED;
					if (led_flags.SET_RED==5)
						led_flags.SET_RED=0;
					
					
        } else if (button_counter<-BUTTON_MEAS_COUNT) {
            button_counter = -BUTTON_MEAS_COUNT;
           
        }
     
    
}







void TMR6_GLOBAL_IRQHandler(void) {


    static char i =0;
    ++i;
    if (i>=8) {
			
			ff.bits.MSTICK_ALLOW =1;
       
        i=0;
    }
		
		
	/*	 static char f =0;
		if(f>0){
			
	*/
		switch_led();
			
		/*	
			f=0;
		}
			else ++f;
		*/
    
		
		zummer_switch();
    
		
		tmr_period_value_set(TMR6,1);
    TMR6 ->ists_bit.ovfif =0;
}

void ADC1_CMP_IRQHandler(void) {
    wdt_counter_reload();
    if(sensor_index==0)
    {
        adc_result_0	= adc_ordinary_conversion_data_get(ADC1);
    }
    else
    {
        adc_result_1	= adc_ordinary_conversion_data_get(ADC1);
    }
    fun_result = gpio_input_data_bit_read(PIN_FUN);

    ff.bits.ALLOW_MEASURE = 1;
    ff.bits.ALLOW_FUN =1;
}


void start_setup() {
    hardware_init(); // initialize the device

    ff.value = 0;

    gpio_bits_reset(PIN_POWER_RELAY);
    gpio_bits_reset(PIN_CONTROL_RELAY);
    gpio_bits_reset(PIN_STATE_ALARM);
		gpio_bits_reset(PIN_STATE_MOTOR);
    gpio_bits_reset(PIN_POWER_SENSOR);
    gpio_bits_reset(PIN_ZUMMER);
    gpio_bits_reset(PIN_LED_RED);

    time_rotation = 0;
    time_relay_power = 0;
    time_relay_control = 0;
    time_relay_gap = 0;
    ms_tone_delay = 0;


    time_melody = 0;



}
/*���������������������������������������������������������������������*/


int main(void)
{
  system_clock_config();

  mx_adc_clock_init();
	
	 start_setup();

	
	led_flags.SET_RED=0;
	led_flags.SET_BLUE=0;
	led_flags.SET_GREEN=3;
	
  while(1)
  {
	
		 wdt_counter_reload();
    hardware_work();

		if(ff.bits.MSTICK_ALLOW){
			ms_tick();			
		}

        if (!ff.bits.ALARM_ON) {

            get_fun();
            fun_work();

            get_wsp();

            autorotation_work();

        } else {
     close();
        };
				
  }
	
	
}





