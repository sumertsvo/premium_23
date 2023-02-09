#include "at32f421_clock.h"


//COMPILATION SETTINGS
#define DEBUG
//#define DEBUG_PCB
//#define DEBUG_AUTOROTATION



#define PIN_SENSOR_2 				GPIOA,GPIO_PINS_3
#define PIN_SENSOR_3 				GPIOA,GPIO_PINS_4
#define PIN_SENSOR_4 				GPIOA,GPIO_PINS_5
#define PIN_SENSOR_1 				GPIOA,GPIO_PINS_6
#define PIN_METER_1 				GPIOA,GPIO_PINS_1
#define PIN_METER_2 				GPIOA,GPIO_PINS_2
#define PIN_BAT_VOLT 				GPIOA,GPIO_PINS_7
#define PIN_FUN 						GPIOA,GPIO_PINS_0
#define PIN_SOURSE_OFF 			GPIOB,GPIO_PINS_11
#define PIN_BAT_TEST 				GPIOB,GPIO_PINS_10
#define PIN_LED_2 					GPIOF,GPIO_PINS_0
#define PIN_LED_3 					GPIOC,GPIO_PINS_15
#define PIN_LED_4 					GPIOC,GPIO_PINS_14
#define PIN_LED_1 					GPIOC,GPIO_PINS_13
#define PIN_LED_RED 				GPIOF,GPIO_PINS_7
#define PIN_LED_GREEN 			GPIOF,GPIO_PINS_6
#define PIN_LED_BLUE 				GPIOA,GPIO_PINS_12
#define PIN_RF_POWER 				GPIOB,GPIO_PINS_8
#define PIN_MODULE_POWER 		GPIOB,GPIO_PINS_9
#define PIN_POWER_RELAY 		GPIOB,GPIO_PINS_13
#define PIN_CONTROL_RELAY 	GPIOB,GPIO_PINS_12
#define PIN_ZUMMER 					GPIOA,GPIO_PINS_11
#define PIN_RS485_DIRECT 		GPIOB,GPIO_PINS_2
#define PIN_POWER_SENSOR 		GPIOB,GPIO_PINS_2
#define PIN_TKEY 						GPIOB,GPIO_PINS_14
#define PIN_RADIO_LED 			GPIOB,GPIO_PINS_15
#define PIN_BUTTON 					GPIOF,GPIO_PINS_1
#define PIN_SPI_OUT 				GPIOB,GPIO_PINS_5
#define PIN_SPI_INP 				GPIOB,GPIO_PINS_4
#define PIN_SPI_CLK 				GPIOB,GPIO_PINS_3
#define PIN_SPI_SEL 				GPIOA,GPIO_PINS_15
#define PIN_MCU_RX_T 				GPIOB,GPIO_PINS_7
#define PIN_MCU_TX_T 				GPIOB,GPIO_PINS_6
#define PIN_MCU_RX			 		GPIOA,GPIO_PINS_10
#define PIN_MCU_TX			 		GPIOA,GPIO_PINS_9
#define PIN_STATE_MOTOR	 		GPIOB,GPIO_PINS_0
#define PIN_STATE_ALARM	 		GPIOA,GPIO_PINS_8
#define SWDCLK							GPIOA,GPIO_PINS_14
#define SWDIO								GPIOA,GPIO_PINS_13

/*_____________________________________________________________________*/


/*DEFAULT_SETTINGS*/
#define VERSION 1
const char 				SHORT_ZUMMER_DELAY = 30;
const char 				LONG_ZUMMER_DELAY = 120;
const char 				FRIMWARE_VERSION_EEPROM_ADR = 0x01;
const unsigned 		AUTOROTATION_DAYS = 14;
const char 				MOVING_WAIT_DELAY = 1;
const unsigned 		LOW_WATER_RESISTANSE = 20000; 
const unsigned 		HIGH_WATER_RESISTANSE = 25000; 
const unsigned 		UP_RESISTANSE = 20000; 

const char 				WSP_MEAS_COUNT = 4;
const char 				FUN_MEAS_COUNT = 3; 
const char 				BUTTON_MEAS_COUNT =3;
const char 				BUTTON_LONG_MEAS_COUNT =3;

const char 				RELAY_POWER_WORK_DELAY = 15; // sec
const char 				RELAY_GAP = 1; //sec
const char 				MELODY_REPEAT_DELAY = 30; //min

#ifdef DEBUG_AUTOROTATION
const uint32_t AUTOROTATION_DELAY = 300;
#else
const uint32_t AUTOROTATION_DELAY = (AUTOROTATION_DAYS * 24 * 60 * 60); //D*H*M*S
#endif

/*voltages*/
uint16_t BAD_WSP_VOLTAGE = 0;
uint16_t GOOD_WSP_VOLTAGE = 0;
uint16_t adc_result_0 = 0;
uint16_t adc_result_1 = 0;
uint16_t adc_result_2 = 0;
uint16_t adc_result_3 = 0;

char fun_result;
char sensor_index;

struct color {
		char R;
    char G;
    char B;
};
/*_____________________________________________________________________*/

/**FLAGS*/
static union {
		uint64_t  value;
    struct {

        unsigned 	ALARM_ON : 1;
        unsigned 	ALARM_OFF : 1;
        unsigned 	FUN_HIGH : 1;
        unsigned 	FUN_LOW : 1;
        unsigned 	ALLOW_MEASURE : 1;
        unsigned 	ALLOW_FUN : 1;
        unsigned 	MEASURING: 1;
        unsigned 	TARGET_POS_CLOSED: 1;

        unsigned 	TARGET_POS_OPENED: 1;
        unsigned 	OPENING : 1;
        unsigned 	OPENED : 1;
        unsigned 	CLOSING : 1;
        unsigned 	CLOSED : 1;
        unsigned 	RELAY_POWER_ON : 1;
        unsigned 	RELAY_CONTROL_ON : 1;
        unsigned 	TONE_ON : 1;

        unsigned 	TONE_OFF : 1;
        unsigned 	SIREN : 1;
        unsigned 	ZUM_BUSY : 1;
        unsigned 	MOVING_ALLOWED : 1;
        unsigned 	AUTOROTATION_WORK: 1;
        unsigned 	MELODY_ON: 1;
        unsigned 	MSTICK_ALLOW: 1;
        unsigned 	SEC_LOCK : 1;

        unsigned  LAST_BEEP_LONG: 1;
        unsigned  LOADED: 1;
        unsigned  STAND_WAIT: 1;
        unsigned  BUTTON_CLOSE: 1;
        unsigned  ALLOW_WSP_0: 1;
        unsigned  ALLOW_WSP_1: 1;
        unsigned  ALLOW_WSP_2: 1;
        unsigned  ALLOW_WSP_3: 1;
		
				unsigned S1_ALARM: 1;
        unsigned S2_ALARM: 1;
        unsigned S3_ALARM: 1;
        unsigned S4_ALARM: 1;
        unsigned : 1;
        unsigned : 1;
        unsigned : 1;
        unsigned : 1;
		
				unsigned : 1;
        unsigned : 1;
        unsigned : 1;
        unsigned : 1;
        unsigned : 1;
        unsigned : 1;
        unsigned : 1;
        unsigned : 1;
		
				unsigned : 1;
        unsigned : 1;
        unsigned : 1;
        unsigned : 1;
        unsigned : 1;
        unsigned : 1;
        unsigned : 1;
        unsigned : 1;
		
				unsigned : 1;
        unsigned : 1;
        unsigned : 1;
        unsigned : 1;
        unsigned : 1;
        unsigned : 1;
        unsigned : 1;
        unsigned : 1;

    } bits;
} flags;


struct {
        unsigned 	LED_1 : 1;
        unsigned 	LED_2 : 1;
        unsigned 	LED_3 : 1;
        unsigned 	LED_4 : 1;
        unsigned 	RED : 1;
        unsigned 	GREEN : 1;
        unsigned 	BLUE: 1;
        unsigned 	LED_1_ON: 1;

        unsigned 	LED_2_ON: 1;
        unsigned 	LED_3_ON : 1;
        unsigned 	LED_4_ON : 1;
				struct 		color active_color;
				struct 		color collors[64];
        char 			SET_GREEN;
        char 			SET_BLUE;
        char 			SET_RED;
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
uint32_t 	time_rotation;
unsigned 	time_relay_power; 
unsigned 	time_relay_control;
unsigned 	time_relay_gap;

uint64_t 	tone_gap_millis;
char 			sec_count = 0;
char 			time_melody; //minute
char 			time_moving_wait;


/*ms_div*/

uint64_t 	millis = 0 ;
unsigned 	ms_tone_delay = 0;
/*_____________________________________________________________________*/

char led_matrix [8][4] =
{
{1,0,0,0},
{1,1,0,0},
{1,1,1,0},
{1,1,1,1},
{0,1,1,1},
{0,0,1,1},
{0,0,0,1},
{1,0,0,0}
};

char stand_matrix [24][3] =	
{
{4,0,0},
{0,0,0},
{2,2,0},
{0,0,0},
{0,4,0},
{0,0,0},
{0,2,2},
{0,0,0},
{0,0,4},
{0,0,0},
{2,0,2},
{0,0,0},
{4,0,0},
{0,0,0},
{2,2,0},
{0,0,0},
{0,4,0},
{0,0,0},
{0,2,2},
{0,0,0},
{0,0,4},
{0,0,0},
{2,0,2},
{0,0,0}
};



/*counters*/
char beep_short_count;
char beep_long_count;
char beep_double_count;

/*¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦*/
/*¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦*/


//STANDART_CLOCK_CONFIG
void mx_adc_clock_init(void){
  crm_adc_clock_div_set(CRM_ADC_DIV_2);
}






/*SERVICE*/

/*led*/






char led_rotate_right_tick()
{
	static char i=0;
	++i;
	if (i==8) i=0;
	
	led_flags.LED_1_ON= led_matrix[i][0];
	led_flags.LED_2_ON= led_matrix[i][1];
	led_flags.LED_3_ON= led_matrix[i][2];
	led_flags.LED_4_ON= led_matrix[i][3];
	
	return i;
}

char led_rotate_left_tick()
{
	
	static char i=0;
	
	if (i==0) i=8;
	--i;
	led_flags.LED_1_ON= led_matrix[i][0];
	led_flags.LED_2_ON= led_matrix[i][1];
	led_flags.LED_3_ON= led_matrix[i][2];
	led_flags.LED_4_ON= led_matrix[i][3];

	return i;
}


void led_standby()
{    
	static char i;
	
	
	/*
	led_flags.LED_1_ON = 1;
	led_flags.LED_2_ON = 1;
	led_flags.LED_3_ON = 1;
	led_flags.LED_4_ON = 1;
	*/
	
	

	if (i==24) i=0;
	
	led_flags.SET_RED= 		stand_matrix[i][0];
	led_flags.SET_GREEN= 	stand_matrix[i][1];
	led_flags.SET_BLUE= 	stand_matrix[i][2];
	
	if (led_rotate_right_tick()==0) {
	static	char k=0;
		++k;
		if (k==3) {
			k=0;
		++i;
		}
	}
	 

}


void switch_color()
{
	
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

void framing()
{
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

void switch_led()
{
	framing();
	switch_color();
}
	
/*SOUND*/
void start_tone()
{
    flags.bits.ZUM_BUSY = 1;
    flags.bits.TONE_ON = 1;
    flags.bits.TONE_OFF = 0;
}

void stop_tone()
{

  flags.bits.ZUM_BUSY = 0;
    flags.bits.TONE_ON = 0;
    flags.bits.TONE_OFF = 1;

}

void beep_short()
{

    if (beep_short_count > 0)	beep_short_count--;
    ms_tone_delay = SHORT_ZUMMER_DELAY;
    flags.bits.LAST_BEEP_LONG = 0;
    start_tone();
}


void beep_long()
{

    if (beep_long_count > 0) 	beep_long_count--;
    ms_tone_delay = LONG_ZUMMER_DELAY;
    flags.bits.LAST_BEEP_LONG = 1;
    start_tone();
}


void beep_double()
{
    if (flags.bits.LAST_BEEP_LONG) {
        beep_short();
    } else {
        beep_long();
    }
}

/*moving*/
void go_close() 
{

   // if (!flags.bits.CLOSING && !flags.bits.CLOSED && flags.bits.MOVING_ALLOWED) {
        flags.bits.CLOSING = 1;
        flags.bits.OPENED = 0;
        flags.bits.OPENING = 0;

        flags.bits.RELAY_POWER_ON = 0;
        flags.bits.RELAY_CONTROL_ON = 1;

        time_relay_control = RELAY_GAP + RELAY_POWER_WORK_DELAY + RELAY_GAP;
        time_relay_power = RELAY_POWER_WORK_DELAY;
        time_relay_gap = RELAY_GAP;

        time_rotation = 0;

  //  }
}

void go_open()
{

 //   if (!flags.bits.OPENED && !flags.bits.OPENING && flags.bits.MOVING_ALLOWED) {
        flags.bits.OPENING = 1;
        flags.bits.CLOSED = 0;
        flags.bits.CLOSING = 0;


        flags.bits.RELAY_CONTROL_ON = 0;
        flags.bits.RELAY_POWER_ON = 1;

        time_relay_power = RELAY_POWER_WORK_DELAY;
        return;
   // }
}



void close()
{
    if (!flags.bits.CLOSED && !flags.bits.OPENED)
    {
        flags.bits.OPENED=1;
    }
    if (flags.bits.OPENED && flags.bits.TARGET_POS_CLOSED  && !flags.bits.CLOSING) //&& !flags.bits.OPENING)
		{
        go_close();
    }
}



void open()
{
    if (!flags.bits.CLOSED && !flags.bits.OPENED)
    {
        flags.bits.CLOSED=1;
    }
    if (flags.bits.CLOSED && flags.bits.TARGET_POS_OPENED && !flags.bits.OPENING) {
        go_open();
    }
}


void relay_tick()
{

    if (flags.bits.OPENING && flags.bits.CLOSING) {
        return;
    }


    if (flags.bits.OPENING) {
        if (time_relay_power > 0) {
            time_relay_power--;
            if (time_relay_power == 0) {
                flags.bits.RELAY_POWER_ON = 0;
                flags.bits.OPENED = 1;
                flags.bits.OPENING = 0;
                flags.bits.AUTOROTATION_WORK = 0;
				flags.bits.STAND_WAIT=1;
               // beep_short_count = 1;
            }
        }
    }


    if (flags.bits.CLOSING) {

        if (time_relay_gap == 0) {
            if (time_relay_power > 0) {
                flags.bits.RELAY_POWER_ON = 1;
                time_relay_power--;
            } else {
                flags.bits.RELAY_POWER_ON = 0;
            }
        } else {
            time_relay_gap--;
        }

        if (time_relay_control > 0) {
            time_relay_control--;
            if (time_relay_control == 0) {
                flags.bits.RELAY_CONTROL_ON = 0;
                flags.bits.CLOSED = 1;
                flags.bits.CLOSING = 0;
				flags.bits.STAND_WAIT=1;
             //   beep_short_count=2;
            }
        }
    }

}


/*logic*/
void start_alarm()
{
	
	
    flags.bits.TARGET_POS_CLOSED=1;
    flags.bits.TARGET_POS_OPENED=0;
    flags.bits.ALARM_ON = 1;
    flags.bits.ALARM_OFF = 0;
    flags.bits.MELODY_ON = 1;
    flags.bits.SIREN = 1;
	
	
	
	
    sec_count=0; 
		
	led_flags.SET_RED=3;
	led_flags.SET_GREEN=0;
	led_flags.SET_BLUE=0;
	
}

void clear_alarm()
{
    flags.bits.ALARM_ON = 0;
    flags.bits.ALARM_OFF = 1;
	led_flags.SET_RED=0;
	led_flags.SET_GREEN=0;
	led_flags.SET_BLUE=0;
	flags.bits.SIREN=0;
	flags.bits.MELODY_ON=0;
	
	led_flags.LED_1_ON=0;
	led_flags.LED_2_ON=0;
	led_flags.LED_3_ON=0;
	led_flags.LED_4_ON=0;
}

void fun_work()
{
    {
        if (
            flags.bits.TARGET_POS_OPENED &&
            flags.bits.FUN_LOW &&
            !flags.bits.FUN_HIGH &&
            flags.bits.ALARM_OFF &&
            flags.bits.MOVING_ALLOWED &&
            !flags.bits.OPENING &&
            //!flags.bits.CLOSING &&
            !flags.bits.OPENED)
		//(flags.bits.CLOSED || !(flags.bits.OPENED || flags.bits.CLOSED) ) ) 
		{
            beep_short_count = 1;
            open();
        };

        if (
            flags.bits.TARGET_POS_CLOSED &&
            flags.bits.FUN_HIGH &&
            !flags.bits.FUN_LOW && 
			flags.bits.MOVING_ALLOWED &&
            //(flags.bits.OPENED || !(flags.bits.OPENED || flags.bits.CLOSED) ) &&
           // !flags.bits.OPENING &&
            !flags.bits.CLOSING&&
		    !flags.bits.CLOSED)
            //&& !flags.bits.AUTOROTATION_WORK)
        {
            beep_short_count = 2;
            close();
        }

    }
}


void autorotation_work()
{

    if ((time_rotation > AUTOROTATION_DELAY) &&
            flags.bits.CLOSED &&
            flags.bits.ALARM_OFF &&
			flags.bits.TARGET_POS_OPENED
       ) {
        open();
        time_rotation = 0;
		flags.bits.TARGET_POS_CLOSED=0;
    }
    if ((time_rotation > AUTOROTATION_DELAY) &&
            flags.bits.OPENED
       ) {
		  flags.bits.TARGET_POS_CLOSED =1;
        close();
        beep_long_count=1;
    }

}


/*¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦*/

/*TIMES*/

void minute_tick()
{

	flags.bits.STAND_WAIT=0;
	flags.bits.LOADED =1;
    if (time_melody > 0) {
        time_melody--;
    } else {
        if (time_melody == 0) {
            flags.bits.SIREN = 1;
            time_melody = MELODY_REPEAT_DELAY;
        }
    };

}


void sec_30_work()
{
    if (flags.bits.MELODY_ON) {
        if (flags.bits.SIREN) {
            flags.bits.SIREN = 0;
        } else {
            beep_short_count = 3;  //connect to ms200
        }
    }
}



void sec_work()
{

    flags.bits.SEC_LOCK = 1;
    sec_count++;

	

	
    //back-forward gap
    if (!flags.bits.MOVING_ALLOWED) {
        if (time_moving_wait > 0) {
            time_moving_wait--;
        } else {
            flags.bits.MOVING_ALLOWED = 1;
        }
    }

    //autorotation tick
    if (!flags.bits.CLOSED) {
        time_rotation++;
    }
    relay_tick();


	
    /*old led tick
    if (flags.bits.ALARM_ON || flags.bits.CLOSING || flags.bits.OPENING) {
        flags.bits.LED_ON = !flags.bits.LED_ON;
    }
    else {
        static char iled;
        iled++;
        if (iled > 2) {
            flags.bits.LED_ON = !flags.bits.LED_ON;
            iled = 0;
        } 

    }      */

    //melody tick
    if (flags.bits.ALARM_ON) {

        if (sec_count == 30|| sec_count==60) {
            sec_30_work();
        }

        if (sec_count == 60) {
            minute_tick();
            sec_count = 0;
        }

    }
}

void ms_200_work()
{

	
	static char load_count=0;
	static char l_count=0;
	
	
	if (flags.bits.ALARM_ON)
	{
		led_flags.SET_BLUE = 0;
		led_flags.SET_GREEN =0;
		led_flags.SET_RED = 4;
		
		led_flags.LED_1_ON =flags.bits.S1_ALARM;
		led_flags.LED_2_ON =flags.bits.S2_ALARM;
		led_flags.LED_3_ON =flags.bits.S3_ALARM;
		led_flags.LED_4_ON =flags.bits.S4_ALARM;
		
	} else 
	{
		
		
		if (flags.bits.OPENING)
		{
		led_flags.SET_BLUE = 4;
		led_flags.SET_RED =0;
		led_flags.SET_GREEN =4;
		led_rotate_right_tick();
		}
	
		if (flags.bits.CLOSING)
		{
		led_flags.SET_BLUE = 4;
		led_flags.SET_RED =4;
		led_flags.SET_GREEN =0;
		led_rotate_left_tick();
		}
	    if (!flags.bits.CLOSING && !flags.bits.OPENING)
		{
			if (load_count>=3  )
			{
			led_standby();
			}
				
		}
	
	}
	
	
	if (load_count<3){
	++l_count;	
	if (l_count==1){
		led_flags.LED_1_ON=1;
	}
	
		if (l_count==2){
		led_flags.LED_2_ON=1;
	}
		
		if (l_count==3){
		led_flags.LED_3_ON=1;
	}
		
		if (l_count==4){
		led_flags.LED_4_ON=1;
	}
		
		if (l_count==5){
			led_flags.LED_1_ON=0;
			led_flags.LED_2_ON=0;
			led_flags.LED_3_ON=0;
			led_flags.LED_4_ON=0;
			l_count=0;
			++load_count;
	} 
	}      
	 //*/ 
	
	
    flags.bits.SEC_LOCK = 0;

    if (flags.bits.ALARM_ON) {
        if (flags.bits.SIREN) {
            beep_double();
        } else {
            if (beep_short_count > 0) {     //connect to sec30
                beep_short();
            }

        }
    } else if (flags.bits.ALARM_OFF) {


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




void get_button();

void ms_100_work()
{

    static char switch_sens_delay;

    static char f;
	
	 get_button();
      

  //  if ( !flags.bits.ALARM_ON) {

        ++switch_sens_delay;
        if(switch_sens_delay>4) {
            switch_sens_delay=0;
        }


        if (!f) {
            gpio_bits_set(PIN_POWER_SENSOR);
            flags.bits.MEASURING=1;
            f=1;
        } else {
			  switch (sensor_index){
				
				case 0:
                adc_ordinary_channel_set(ADC1,ADC_CHANNEL_3,1,ADC_SAMPLETIME_239_5);
				//adc_ordinary_conversion_trigger_set(ADC1,ADC12_ORDINARY_TRIG_SOFTWARE,TRUE);
                sensor_index = 1;
				break;
				
                case 1:
                adc_ordinary_channel_set(ADC1,ADC_CHANNEL_4,1,ADC_SAMPLETIME_239_5);
				//adc_ordinary_conversion_trigger_set(ADC1,ADC12_ORDINARY_TRIG_SOFTWARE,TRUE);
                sensor_index = 2;
				break;
				
				case 2:
                adc_ordinary_channel_set(ADC1,ADC_CHANNEL_5,1,ADC_SAMPLETIME_239_5);
				//adc_ordinary_conversion_trigger_set(ADC1,ADC12_ORDINARY_TRIG_SOFTWARE,TRUE);
                sensor_index = 3;
				break;
				
				case 3:
                adc_ordinary_channel_set(ADC1,ADC_CHANNEL_6,1,ADC_SAMPLETIME_239_5);
				//adc_ordinary_conversion_trigger_set(ADC1,ADC12_ORDINARY_TRIG_SOFTWARE,TRUE);
                sensor_index = 0;
                break;
			}
			    adc_interrupt_enable(ADC1,ADC_CCE_INT,TRUE);
            adc_ordinary_conversion_trigger_set(ADC1,ADC12_ORDINARY_TRIG_SOFTWARE,TRUE);
            f=0;
        }
   // }
}

void ms_tick()
{


	
    static uint64_t ms200_count = 0;
    static uint64_t ms100_count = 0;
    static uint64_t ms1000_count = 0;

			flags.bits.MSTICK_ALLOW =0;
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
        //  if (!flags.bits.SEC_LOCK)
        sec_work();
    }
	
    ++millis;
   
}

/*¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦*/

/*HARDWARE*/
char PIN_FUN_STATE_GetValue()
{
    return(fun_result);
}



void gpio_set(gpio_type *PORT, uint32_t PIN, gpio_drive_type DRIVE, gpio_mode_type MODE, gpio_output_type OUT_TYPE, gpio_pull_type PULL )
{

    gpio_init_type pinx;

    gpio_init_type *pina = &pinx;

    pinx.gpio_drive_strength= DRIVE;
    pinx.gpio_mode =MODE;
    pinx.gpio_out_type=OUT_TYPE;
    pinx.gpio_pins = PIN;
    pinx.gpio_pull = PULL;

    gpio_init( PORT,pina);

}

void timer_init()
{


    nvic_irq_enable(TMR6_GLOBAL_IRQn,35,36);

    TMR6->iden_bit.ovfien =1;

    TMR6 ->ctrl1_bit.ocmen = 0;

    TMR6 ->ctrl1_bit.ovfen = 0;

    tmr_channel_buffer_enable(TMR6,TRUE);

    tmr_base_init(TMR6,1,370);

    tmr_counter_enable(TMR6,TRUE);



}

void my_gpio_init()
{
	
	

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
             GPIO_MODE_INPUT,
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



void hardware_init()
{
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

    nvic_irq_enable(ADC1_CMP_IRQn,15,16);

    adc_base_config_type *adc1;
    adc_base_default_para_init(adc1);
   //adc1 ->repeat_mode = FALSE;
    adc_base_config(ADC1,adc1);

    adc_enable(ADC1,TRUE);
   // adc_interrupt_enable(ADC1,ADC_CCE_INT,TRUE);
    sensor_index = 0;
    adc_ordinary_channel_set(ADC1,ADC_CHANNEL_4,1,ADC_SAMPLETIME_239_5);

}


void hardware_work()
{
    gpio_bits_write(PIN_STATE_ALARM,(confirm_state) (flags.bits.ALARM_ON));
		gpio_bits_write(PIN_STATE_MOTOR,(confirm_state) (flags.bits.OPENING || flags.bits.OPENED));
    gpio_bits_write(PIN_CONTROL_RELAY,(confirm_state) (flags.bits.RELAY_CONTROL_ON));
    gpio_bits_write(PIN_POWER_RELAY,(confirm_state) (flags.bits.RELAY_POWER_ON));
   
//	gpio_bits_write(PIN_LED_RED,(confirm_state) (flags.bits.LED_ON));
	
		gpio_bits_write(PIN_LED_RED,(confirm_state) (led_flags.RED));
		gpio_bits_write(PIN_LED_GREEN,(confirm_state) (led_flags.GREEN));
		gpio_bits_write(PIN_LED_BLUE,(confirm_state) (led_flags.BLUE));
	
		gpio_bits_write(PIN_LED_1,(confirm_state) (led_flags.LED_1));
		gpio_bits_write(PIN_LED_2,(confirm_state) (led_flags.LED_2));
		gpio_bits_write(PIN_LED_3,(confirm_state) (led_flags.LED_3));
		gpio_bits_write(PIN_LED_4,(confirm_state) (led_flags.LED_4));
	
	 
    if (flags.bits.TONE_OFF) {
        gpio_bits_reset(PIN_ZUMMER);
    };
}


void zummer_switch()
{

    if(flags.bits.TONE_ON) gpio_bits_write(GPIOA,GPIO_PINS_11,(confirm_state) (!GPIOA ->odt_bit.odt11));   //todo


#ifdef DEBUG
    if(flags.bits.TONE_ON) gpio_bits_write(GPIOA,GPIO_PINS_11,(confirm_state) (!GPIOA ->odt_bit.odt11));   //todo
#endif

}
void get_wsp()
{


	
	
    if (flags.bits.ALLOW_MEASURE) {

        BAD_WSP_VOLTAGE = (LOW_WATER_RESISTANSE / ((UP_RESISTANSE + LOW_WATER_RESISTANSE) / 4096));
        GOOD_WSP_VOLTAGE =(HIGH_WATER_RESISTANSE / ((UP_RESISTANSE + HIGH_WATER_RESISTANSE) / 4096));

        static signed char bad_measures_counter_0 = 0;
		static signed char bad_measures_counter_1 = 0;
		static signed char bad_measures_counter_2 = 0;
		static signed char bad_measures_counter_3 = 0;
		uint16_t res;
		
		
		
        
			if (flags.bits.ALLOW_WSP_0){
				
			res = adc_result_0;
			
			if (res < BAD_WSP_VOLTAGE) {
				bad_measures_counter_0++;
			} else 
				if (res > GOOD_WSP_VOLTAGE) {
                bad_measures_counter_0--;
            }
			
			if (bad_measures_counter_0 > WSP_MEAS_COUNT) {
				led_flags.LED_1_ON=1;
				flags.bits.S1_ALARM=1;
				start_alarm();
				bad_measures_counter_0 = WSP_MEAS_COUNT; 
			}
			 
			 if (bad_measures_counter_0 < -WSP_MEAS_COUNT) {
				bad_measures_counter_0 = -WSP_MEAS_COUNT;
			}
			 
			flags.bits.ALLOW_WSP_0=0;
			gpio_bits_reset(PIN_POWER_SENSOR);
		}
			
		
		
		
			if (flags.bits.ALLOW_WSP_1){
			 res = adc_result_1;
			
			if (res < BAD_WSP_VOLTAGE) {
				bad_measures_counter_1++;
			} else 
				if (res > GOOD_WSP_VOLTAGE) {
                bad_measures_counter_1--;
            }
			
			
			 if (bad_measures_counter_1 > WSP_MEAS_COUNT) {
				led_flags.LED_2_ON=1;
				 flags.bits.S2_ALARM=1;
				start_alarm();
				bad_measures_counter_1 = WSP_MEAS_COUNT; 
			 }
			 
			 if (bad_measures_counter_1 < -WSP_MEAS_COUNT) {
				bad_measures_counter_1 = -WSP_MEAS_COUNT;
			}
			 flags.bits.ALLOW_WSP_1=0;
           gpio_bits_reset(PIN_POWER_SENSOR);
		}
		
			
		
		
		
			if (flags.bits.ALLOW_WSP_2){
			 res = adc_result_2;
			
			if (res < BAD_WSP_VOLTAGE) {
				bad_measures_counter_2++;
			} else 
				if (res > GOOD_WSP_VOLTAGE) {
                bad_measures_counter_2--;
            }
			
			if (bad_measures_counter_2 > WSP_MEAS_COUNT) {
				led_flags.LED_3_ON=1;
				flags.bits.S3_ALARM=1;
				start_alarm();
				bad_measures_counter_2 = WSP_MEAS_COUNT; 
			}
			 
			 if (bad_measures_counter_2 < -WSP_MEAS_COUNT) {
				bad_measures_counter_2 = -WSP_MEAS_COUNT;
			}
			 flags.bits.ALLOW_WSP_2=0;
		gpio_bits_reset(PIN_POWER_SENSOR);	 
		}
		
			
		
		
		
		
			if (flags.bits.ALLOW_WSP_3){
			 res = adc_result_3;
			
			if (res < BAD_WSP_VOLTAGE) {
				bad_measures_counter_3++;
			} else 
				if (res > GOOD_WSP_VOLTAGE) {
                bad_measures_counter_3--;
            }
			
			if (bad_measures_counter_3 > WSP_MEAS_COUNT) {
				led_flags.LED_4_ON=1;
				flags.bits.S4_ALARM=1;
				start_alarm();
				bad_measures_counter_3 = WSP_MEAS_COUNT; 
			}
			 
			 if (bad_measures_counter_3 < -WSP_MEAS_COUNT) {
				bad_measures_counter_3 = -WSP_MEAS_COUNT;
			}
			 flags.bits.ALLOW_WSP_3=0;
		gpio_bits_reset(PIN_POWER_SENSOR);	 
		}
		
		
	
		
		
		
        //gpio_bits_reset(PIN_POWER_SENSOR);
		
	    if(
			
		bad_measures_counter_0==-WSP_MEAS_COUNT &&
		bad_measures_counter_1==-WSP_MEAS_COUNT &&
		bad_measures_counter_2==-WSP_MEAS_COUNT &&
		bad_measures_counter_3==-WSP_MEAS_COUNT &&
		!flags.bits.ALARM_ON
		
		)
		
		flags.bits.ALARM_OFF=1;
		
        flags.bits.ALLOW_MEASURE = 0;
    }
}


void get_fun()
{
    if (flags.bits.ALLOW_FUN) {
        static signed char fun_counter;

        if (PIN_FUN_STATE_GetValue()) fun_counter--;
        else fun_counter++;

        if (fun_counter > FUN_MEAS_COUNT) {
            fun_counter = FUN_MEAS_COUNT;
            flags.bits.FUN_LOW = 0;
            flags.bits.FUN_HIGH = 1;
            flags.bits.TARGET_POS_CLOSED=1;
            flags.bits.TARGET_POS_OPENED=0;
			flags.bits.STAND_WAIT=1;
        } else if (fun_counter<-FUN_MEAS_COUNT) {
			
			
			
            fun_counter = -FUN_MEAS_COUNT;
            flags.bits.FUN_LOW = 1;
            flags.bits.FUN_HIGH = 0;
            flags.bits.TARGET_POS_CLOSED=0;
            flags.bits.TARGET_POS_OPENED=1;
			flags.bits.STAND_WAIT=1;
			
		
		
			
        }
        flags.bits.ALLOW_FUN = 0;
    }
}


void get_button()
{
    
        static signed char button_counter;

	 //   static signed char button_long_counter;
	
        if (gpio_input_data_bit_read(PIN_BUTTON)==0) button_counter--;
        else button_counter++;

        if (button_counter > BUTTON_MEAS_COUNT) {
            button_counter = BUTTON_MEAS_COUNT;
					
					
        } else if (button_counter<-BUTTON_MEAS_COUNT) {
            button_counter = -BUTTON_MEAS_COUNT;
			
          if (flags.bits.ALARM_OFF)
		  {
			beep_long();
			flags.bits.BUTTON_CLOSE = !flags.bits.BUTTON_CLOSE;			  
		  }
		 if (flags.bits.ALARM_ON)
		{	
		clear_alarm(); 
		}
		flags.bits.STAND_WAIT=1;
        }
     
    
}




void TMR6_GLOBAL_IRQHandler(void)
{


    static char i =0;
    ++i;
    if (i>=8) {
			flags.bits.MSTICK_ALLOW =1;
        i=0;
    }
		
		switch_led(); 
		
		zummer_switch();
    
		tmr_period_value_set(TMR6,1);
    TMR6 ->ists_bit.ovfif =0;
}

void ADC1_CMP_IRQHandler(void)
{
    
	//wdt_counter_reload();
	
	if (flags.bits.MEASURING){
		
		
	if (sensor_index==0) {
        adc_result_0	= adc_ordinary_conversion_data_get(ADC1);
		flags.bits.ALLOW_WSP_0=1;
	}
		
	if (sensor_index==1) {
        adc_result_1	= adc_ordinary_conversion_data_get(ADC1);
		flags.bits.ALLOW_WSP_1=1;
	}
		
	if (sensor_index==2) {
        adc_result_2	= adc_ordinary_conversion_data_get(ADC1);
		flags.bits.ALLOW_WSP_2=1;
	}
		
	if (sensor_index==3) {
        adc_result_3	= adc_ordinary_conversion_data_get(ADC1);
		flags.bits.ALLOW_WSP_3=1;
	}
		
		
	fun_result = gpio_input_data_bit_read(PIN_FUN);	
	
	

    

	flags.bits.MEASURING = 0;
    flags.bits.ALLOW_MEASURE = 1;
    flags.bits.ALLOW_FUN =1; 

	}
	
	  adc_interrupt_enable(ADC1,ADC_CCE_INT,FALSE);
}



void start_setup()
{
    hardware_init(); // initialize the device

    flags.value = 0;

    gpio_bits_reset(PIN_POWER_RELAY);
    gpio_bits_reset(PIN_CONTROL_RELAY);
    gpio_bits_reset(PIN_STATE_ALARM);
	gpio_bits_reset(PIN_STATE_MOTOR);
    gpio_bits_reset(PIN_POWER_SENSOR);
    gpio_bits_reset(PIN_ZUMMER);
    gpio_bits_reset(PIN_LED_RED);
	gpio_bits_reset(PIN_BAT_TEST);
	gpio_bits_reset(PIN_SOURSE_OFF);
	gpio_bits_reset(PIN_RADIO_LED);
	gpio_bits_reset(PIN_RF_POWER);
	gpio_bits_reset(PIN_MODULE_POWER);

    time_rotation = 0;
    time_relay_power = 0;
    time_relay_control = 0;
    time_relay_gap = 0;
    ms_tone_delay = 0;


    time_melody = 0;



}
/*¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦¦*/


int main(void)
{
	
	//STARTUP_INIT
	system_clock_config();
	mx_adc_clock_init();
	start_setup();

	//ONE_TIME WORK
	led_flags.SET_RED=0;
	led_flags.SET_BLUE=0;
	led_flags.SET_GREEN=3;
	
  while(1)
  {
	
	//	 wdt_counter_reload();
     hardware_work();

		if(flags.bits.MSTICK_ALLOW){
			ms_tick();			
		}
         get_wsp();
        if (!flags.bits.ALARM_ON) {

            get_fun();
            fun_work();

           

            autorotation_work();

        } else {
     close();
		
        };
				
  }
	
	
}

/*

*/



