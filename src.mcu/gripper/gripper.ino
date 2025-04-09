/* Robotic Gripper
 * 
 * 
 * Author: Viki (a) Vignesh Natarajan [ www.vikiworks.io ]
 */


/* A DC Motor's number of rotation and driection of rotation 
 * are computed using reading from rotary encoder signal pin. 
 * The encoder I am using has 2 signal pins. Such encodres with 2 signal pins are called as 
 * Quadrature Encoder. The signal pins are marked as  "Signal Pin or CH.A/CH.B or C1/C2" 
 * 
 */
  
  
#include "slog.hpp"

/* Vary this value to change speed */
#define SPEED 255       

#define ARDUINO
//#define TEENSY

#define TIMER_START 0
#define TIMER_END	1

#define ENC_START 0
#define ENC_END	1

boolean debug = true;

#ifdef ARDUINO 
	#define pinChannel1           2      /* Signal Pin 1 of Encoder */  
	/* On Arduino interrups are supported on pin 1 and 2 */
	#define pinChannel2           3      /* Signal Pin 2 of Encoder */
	#define pinDirection1         8      /* Direction Pin Pin */
	#define pinDirection2         9      /* Direction Pin Pin */
	#define pinSpeed              10     /* Speed control Pin */
	#warning "BOARD => ARDUINO"
#endif

#ifdef TEENSY
	#define pinChannel1           11      /* Signal Pin 1 of Encoder */ 
										/* On Teensy interrups are supported on pin 5 and 6*/
	#define pinChannel2           12      /* Signal Pin 2 of Encoder */
	#define pinDirection          5		/* Direction Pin Pin */
	#define pinSpeed              6		/* Speed control Pin */
	#warning "BOARD => TEENSY"
#endif
  
  
#define CLK      1         /* Clockwise Rotation */
#define ACLK    -1         /* Counter Clockwise Rotation [Anti Clockwise]*/
  
#define GRIPPER_OPEN  CLK
#define GRIPPER_CLOSE ACLK

#define SUCCESS		 true
#define UNSUCCESSFUL false

/* Motor is allowed to stall for 2000 milli seconds    
 * [motor shaft is not rotation even after supplying power - due to load] 
 */
#define MOTOR_STALL_LIMIT 1000   
#define CALIB_INTERVAL 2000

/* Check and fill this value from your motor datasheet
 * Number of encoder counts per revolution
 */
#define STEPS_PER_REVOLUTION 86

/*	angCtr: 
 *
 *  This variable counts the number of steps moved by motor shaft 
 */

int angCtr = 0;          

/* angDirection:
 *		Tracks DC motor shaft's direction of rotation 
 */
int angDirection = CLK;         

/* readingChannel1:
 *
 *		Encoder 1 reading
 */
int readingChannel1 = 0;         

/* readingChannel1_P:
 *
 *		Encoder 1 reading X milli seconds before
 */

int readingChannel1_P = 0;       

/*	curr_time:
 *		Milliseconds passed since arduino started running the program.
 */
unsigned long curr_time = 0;     

/*	prev_time:
 *		Milliseconds passed since arduino started running the program.
 */

unsigned long prev_time = 0;     

/*	initialization_status:
 *		stores the initialization status of the system
 */
boolean initialization_status = UNSUCCESSFUL;

boolean initialization_status_print = UNSUCCESSFUL;

/* mean_rate_open:
 *		Tracks average rotation per second for a given pulse-width	
 *		when gripper opens.
 */
double mean_rate_open = 0;

/* mean_rate_close:
 *		Tracks average rotation per second for a given pulse-width	
 *		when gripper closes.
 */
double mean_rate_close = 0;

/* From motor start to stop it doesn't rotate at the same speed,
 * speed varies due to initial acceleration(kick-start). 
 * Keeping an accelaration offset to handle certain scenarios 
 * 
 * By just changing the gripper_factor we can adjust the holding torque applied to the 
 * object gripper holds
 * 
 * Can be any number, 0 to 10000
 * 
 */
int gripper_factor_open = 100;
int gripper_factor_close = 100;
/* Gripper open and closing limit
 */
int gripper_open_limit = 100;
int gripper_close_limit = 100;

double gop = 0.0;		 /* Current gripper open percentage 0 to 1*/

int goEncTicks = 0;	 /* Number of encoder ticks to completely open gripper */
boolean goSet = false;	/* status Number of encoder ticks to completely open gripper */

int init_motor_driver()
{
  	pinMode (pinDirection1, OUTPUT);
  	pinMode (pinDirection2, OUTPUT);
  	pinMode (pinSpeed, OUTPUT);
  
  	digitalWrite(pinDirection1, LOW);
  	digitalWrite(pinDirection2, LOW);
  	analogWrite(pinSpeed, LOW);
  	return 0;
}


void init_encoder()
{
  	pinMode (pinChannel1, INPUT);
  	pinMode (pinChannel2, INPUT);
  	readingChannel1 = digitalRead(pinChannel1);
  
  	/*Keep in mind, interrupt callback should not have any argument and return value*/
  	attachInterrupt(digitalPinToInterrupt(pinChannel1), count_steps, CHANGE);
}

void count_steps()
{
  	readingChannel1 = digitalRead(pinChannel1);
   
  	/* If previous reading and current reading is not same. 
  	 * This implies pulse has occured ( motor spin has occured )
  	 */
  	if(readingChannel1 != readingChannel1_P){
  		/* Increment rotation counter */
  		angCtr += 1;
  	}
   
  	/* Store pevious channel 1 reading */
    readingChannel1_P = readingChannel1;
}

void count_steps_n_direction()
{
    readingChannel1 = digitalRead(pinChannel1);
    
    /* If previous reading and current reading is not same. 
    * This implies pulse has occured ( motor spin has occured )
    */
    if(readingChannel1 != readingChannel1_P){
    /*Channel B reading*/
        int readingChannel2 = digitalRead(pinChannel2);
    
        /* Increment rotation counter */
        angCtr += 1;
    
        /* According to the datasheet, 
         * If current value of channel a and channel b are same, 
         * motor is spinning anticlockwise. otherwise motor is spinning
         * clockwise.
         */
    
        if(readingChannel1 == readingChannel2){   
        	  angDirection = ACLK;
        }else{                                    
            angDirection = CLK;
        }
    
    }
    
    /* Store pevious channel 1 reading */
    readingChannel1_P = readingChannel1;

}

/* The firmness of grab is determined by this value, it is a percentage
 * Give value from 0 to 1 
 */
int set_holding_torque(int percentage)
{
    gripper_factor_close = mean_rate_close * percentage;
}


int test_encoder_n_motor()
{
    	int steps_rotated;
    	slog(msg_status, "Encoder/Motor Test\n");
    
    	delay(CALIB_INTERVAL);
    	encoder_counter(ENC_START);
    	drive_motor_clockwise(SPEED);
      delay(CALIB_INTERVAL);
      
    	stop_motor();
    	steps_rotated = encoder_counter(ENC_END);
    
    	if(steps_rotated > 0){
    		  slog(msg_success, "Motor Clockwise Spin\n");
    	}else{
    		  slog(msg_nsuccess, "Motor/Encoder Clockwise Spin\n");
    		  return UNSUCCESSFUL;
    	}
    
    	delay(CALIB_INTERVAL);
    	encoder_counter(ENC_START);
    	drive_motor_cclockwise(SPEED);
      delay(CALIB_INTERVAL);
    	stop_motor();
    	steps_rotated = encoder_counter(ENC_END);
    	
    	if(steps_rotated > 0){
    		  slog(msg_success, "Motor C-Clockwise Spin\n");
    	}else{
    		  slog(msg_nsuccess, "Motor/Encoder C-Clockwise Spin\n");
    		  return UNSUCCESSFUL;
    	}
    
    	return SUCCESS;
}

/* Keeping seperate function for clockwise and counter-clockwise, 
 * because I don't want any if check inside the control instructions.
 * This is to get more precise encoder reading, 
 * since encoder is attached to an interrupt. 
 */
 
void drive_motor_clockwise(int _speed)
{
	  digitalWrite(pinDirection1, LOW);
	  digitalWrite(pinDirection2, HIGH);
	  analogWrite(pinSpeed, _speed); 
}

/* Keeping seperate function for clockwise and counter-clockwise, 
 * because I don't want any if check inside the control instructions.
 * This is to get more precise encoder reading, 
 * since encoder is attached to an interrupt. 
 */
 
void drive_motor_cclockwise(int _speed)
{
      digitalWrite(pinDirection1, HIGH);
      digitalWrite(pinDirection2, LOW);  
      analogWrite(pinSpeed, _speed); 
}


void stop_motor()
{
      analogWrite(pinSpeed, 0); 
}

double millis2seconds(double time_millis)
{
	    return (time_millis / 1000);
}

/* return the number of steps to rotate for a given percentage */
double compute_steps(double percentage)
{
	/* Number of encoder ticks to completely open gripper */
	return ((double)goEncTicks * percentage);
}

void gripper_act(int action, int steps)
{	
	String message = "";
	slog(msg_status, "gripper_act()\n");

  if(steps <= 0){
      slog(msg_status, "Zero steps to move\n");
  }

	int gripper_factor = 0;

	int steps_rotated = 0;
	int total_steps_rotated = 0;
	double time_elapsed_ms = 0;
	void (*fp_motor_driver)(int) = NULL;

	/* Current rate of rotation */
	double curr_rate = 0;
	/*Number of iterations*/
	double mean_rate = 0.0;

	if(action == GRIPPER_OPEN){
		mean_rate = mean_rate_open;
		fp_motor_driver = drive_motor_clockwise;
		gripper_factor = gripper_factor_open;
	}else if(action == GRIPPER_CLOSE){
		mean_rate = mean_rate_close;
		fp_motor_driver = drive_motor_cclockwise;
		gripper_factor = gripper_factor_close;
	}

	encoder_counter(ENC_START);
	while(1){
		fp_motor_driver(SPEED); delay(100);		
		steps_rotated = encoder_counter(ENC_END);
		total_steps_rotated += steps_rotated;
			
		/*BUG: Need to fix drift error - adding a buffer value*/
		if((total_steps_rotated + 10) > steps){
				stop_motor();
				break;
		}
		encoder_counter(ENC_START);
	}
}

void gripper_actuate_fully(int action)
{
	String message = "";
	slog(msg_status, "gripper_actuate_fully()\n");

	int gripper_factor = 0;

	int steps_rotated = 0;
	int total_steps_rotated = 0;
	double time_elapsed_ms = 0;
	void (*fp_motor_driver)(int) = NULL;

	/* Current rate of rotation */
	double curr_rate = 0;
	/*Number of iterations*/
	double mean_rate = 0.0;

	if(action == GRIPPER_OPEN){
		mean_rate = mean_rate_open;
		fp_motor_driver = drive_motor_clockwise;
		gripper_factor = gripper_factor_open;
	}else if(action == GRIPPER_CLOSE){
		mean_rate = mean_rate_close;
		fp_motor_driver = drive_motor_cclockwise;
		gripper_factor = gripper_factor_close;
	}

	encoder_counter(ENC_START);
	timer(TIMER_START);
	while(1){
		fp_motor_driver(SPEED); delay(100);
		time_elapsed_ms += timer(TIMER_END);    
		timer(TIMER_START);   

		if(time_elapsed_ms > MOTOR_STALL_LIMIT){
			steps_rotated = encoder_counter(ENC_END);
			total_steps_rotated += steps_rotated;
			encoder_counter(ENC_START);
			/* Current rate of rotation in rotation/seconds */
			curr_rate = steps_rotated / millis2seconds(time_elapsed_ms);

			/*reset elapsed time*/
			time_elapsed_ms = 0;

      if(debug == true){
          message  = String("current rotation rate : ");
          message += String(curr_rate);
          message += String(" | mean rotation rate : ");
          message += String(mean_rate) + String("\n");
          slog(msg_status, message);
      }

			/* If current rotation rate is less than mean, It implies
			 * there is an obstacle/object in the motor path
			 */

			if((curr_rate + gripper_factor) < mean_rate){
				stop_motor();
        message = "Gripper identified an object or reached its limit \n";
        slog(msg_status, message);
        message  = String("current rotation rate : ");
        message += String(curr_rate);
        message += String(" | mean rotation rate : ");
        message += String(mean_rate) + String("\n");
        slog(msg_status, message);
				break;
			}

		}
	}

	if(goSet == false){
		goEncTicks = total_steps_rotated;
	}

}

int timer(int action)
{
	  static int start_time;	
	  static int end_time;	
	  int time_diff = 0;

	  if(action == TIMER_START){
		    start_time = millis();
		    end_time = start_time;
	  }else if(action == TIMER_END){
		    end_time = millis();
		    time_diff = end_time - start_time;
		    return time_diff;
	  }

	  return 0;
}

int encoder_counter(int action)
{
	  static int start_enc;	
	  static int end_enc;	
	  int steps_rotated = 0;

	  if(action == ENC_START){
		    start_enc = angCtr;
		    end_enc = angCtr;
	  }else if(action == ENC_END){
		    end_enc = angCtr;
		    steps_rotated = end_enc - start_enc;
		    return steps_rotated;
	  }

	  return 0;
}

double degrees_rotated()
{
	int t_angCtr = angCtr;
	return 	( STEPS_PER_REVOLUTION / t_angCtr );
}

/* identify mean_rate of rotation/sec at the set pwm */
void compute_mean_rate()
{
	String message = "";
    /*time elapsed in seconds*/
    double time_elapsed_ms = 0.0;
    double time_elapsed_sec = 0.0;
    int steps_rotated = 0;

    slog(msg_status, "Encoder mean callibration\n");
    
    delay(CALIB_INTERVAL);
    
    timer(TIMER_START);
    encoder_counter(ENC_START);
    drive_motor_clockwise(SPEED); delay(CALIB_INTERVAL); stop_motor();

    steps_rotated = encoder_counter(ENC_END);
    time_elapsed_ms = timer(TIMER_END);
    time_elapsed_sec = millis2seconds(time_elapsed_ms);
 
    mean_rate_open = steps_rotated / time_elapsed_sec;

    slog(msg_status, "Intentional rest for 10 seconds\n");
    /* Delay betweeen open and close gripper */
    delay(10000);
    
    timer(TIMER_START);
    encoder_counter(ENC_START);
    
    drive_motor_cclockwise(SPEED); delay(CALIB_INTERVAL);  stop_motor();
    
    steps_rotated = encoder_counter(ENC_END);
    time_elapsed_ms = timer(TIMER_END);
    time_elapsed_sec = millis2seconds(time_elapsed_ms);
   
    mean_rate_close = steps_rotated / time_elapsed_sec;
    
    message = String("Mean rate of rotation/sec [ mean_open : ") + \
				String(mean_rate_open) + String(" ] [ pwm : ") + \
				String(SPEED) + String(" ]\n");

    slog(msg_status, message);
    
    message = String("Mean rate of rotation/sec [ mean_close : ") + \
				String(mean_rate_close) + String(" ] [ pwm : ") + \
				String(SPEED) + String(" ]\n");

    slog(msg_status, message);
    
    slog(msg_status, "Encoder mean callibration\n");
}

/*Number of steps to fully open*/
int compute_gripper_open_steps()
{
  String message = "";
	delay(2000);
	
	goSet = false;
	gripper_actuate_fully(GRIPPER_OPEN);
  message = String("Steps required to fully open gripper : ") + String(goEncTicks) + String("\n");
  slog(msg_essential, message);
	goSet = true;
	
	delay(2000);
	
	/*Reset gripper to closed state*/
  message = String("Resetting gripper\n");
  slog(msg_essential, message);
	gripper_actuate_fully(GRIPPER_CLOSE);

 
  message = String("Steps required to fully open gripper : ") + String(goEncTicks) + String("\n");
  slog(msg_essential, message);
}

void setup() 
{
  	Serial.begin(9600);

    /*Connect to serial and send some data to initialize*/
    while(1){
        if(Serial.available() > 0){
            char command = Serial.read();
            if(command == 'i'){
                slog(msg_essential, "Initializing system\n");
                break;
            }
        }
        delay(100);
    }
  
    init_motor_driver();  
    slog(msg_success, "Motor driver initialization\n");
  
    
    init_encoder();
    slog(msg_success, "Quadrature Encoder initialization\n");
    
    int ret = test_encoder_n_motor();
  
    if(ret == SUCCESS){
        initialization_status = SUCCESS;
        slog(msg_success, "System Initialization\n");
    }else{
        initialization_status = UNSUCCESSFUL;
        slog(msg_nsuccess, "System Initialization\n");
    }
  
    if(initialization_status == SUCCESS){
       compute_mean_rate();
		   compute_gripper_open_steps();
    }

    slog(msg_success, "Listening for command\n");
}
/* Value is between 0 and 100
 */
void actuate_gripper(double percentage)
{
  String message = "";
  int operation = 0;
  message = String("Given actuator percantage : ") + String(percentage) + String("\n");
  slog(msg_status, message);

  message = String("Current actuator percantage : ") + String(gop) + String("\n");
  slog(msg_status, message);

	/* Percentage to actuate is the diffrence of 
	 * current actuation(in percentage)  and 
	 * percentage we got from serial monitor
	 */
	double actuation_percentage = percentage - gop;

  if(actuation_percentage == 0.0){
      slog(msg_nsuccess, "Already set to same percentage. No operation\n");
      return;
  }


  if(actuation_percentage < 0.0){ /*close gripper*/
      operation = GRIPPER_CLOSE;

      /*make the percentage positive*/
      actuation_percentage  = actuation_percentage * (-1);
  }else{
      operation = GRIPPER_OPEN;
  }

  
  double steps  = 0.0;

	/*Percentage Gripper Open/Close*/
	if(goSet != true){
		message = String("Gripper initial open percentage is not set\n");
		slog(msg_nsuccess, message);
		return;
	}
	
	steps = compute_steps(actuation_percentage);
  message = String("Steps to rotate : ") + String(steps) + String("\n");
  slog(msg_essential, message);

  gripper_act(operation, (int)steps);

  gop = percentage;
}

void check_commands()
{
  int cmd_value = 0;
  String message = "";
	if(Serial.available() > 0){
		String serial_cmd = Serial.readString();
    message = String("Command Entered : ") + serial_cmd + String("\n");
    slog(msg_essential, message);

    /* COMMANDS:
     * 
     * o        = fully open gripper
     * c        = fully close gripper
     * ap<xxx>  = actuate gripper by a percentage(xxx)
     * OL<xxx>  = Set open Limit for gripper by a value (xxx)
     * CL<xxx>  = Set close limit for gripper by a value (xxx)
     * GO<xxx>  = Set Gripper Open Factor by a value (xxx)
     * GC<xxx>  = Set Gripper Close Factor by a value (xxx)
     */


		if(serial_cmd[0] == 'o'){
			slog(msg_status, "Opening Gripper Fully\n");
			gripper_actuate_fully(GRIPPER_OPEN);
		}else if(serial_cmd[0] == 'c'){
      slog(msg_status, "Closing Gripper Fully\n");
			gripper_actuate_fully(GRIPPER_CLOSE);
		}else if(serial_cmd[0] == 'a' && serial_cmd[1] == 'p'){ /*Gripper actuation percentage*/
			cmd_value = get_cmd_value(serial_cmd);
			message = "Gripper open/close with specified percentage:" + String(cmd_value) + String("\n");
			slog(msg_essential, message);
			double percentage = (double) cmd_value / 100.00;
			actuate_gripper(percentage);
		}else if(serial_cmd[0] == 'O' && serial_cmd[1] == 'L'){ /*Open Limit*/
			cmd_value = get_cmd_value(serial_cmd);
			message = "Configuring gripper open limit : " + String(cmd_value) + String("\n");
			slog(msg_essential, message);
			gripper_open_limit = cmd_value; 
		}else if(serial_cmd[0] == 'C' && serial_cmd[1] == 'L'){ /*Close Limit*/
			cmd_value = get_cmd_value(serial_cmd);
			message = "Configuring gripper close limit" + String(cmd_value) + String("\n");
			slog(msg_essential, message);
			gripper_close_limit = cmd_value; 
		}else if(serial_cmd[0] == 'G' && serial_cmd[1] == 'O'){ /*Gripper Factor Open*/
			cmd_value = get_cmd_value(serial_cmd);
			message = "Configuring gripper factor open" + String(cmd_value) + String("\n");
			slog(msg_essential, message);
			gripper_factor_open = cmd_value; 
		}else if(serial_cmd[0] == 'G' && serial_cmd[1] == 'C'){ /*Gripper Factor Close*/
			cmd_value = get_cmd_value(serial_cmd);
			message = "Configuring gripper factor close" + String(cmd_value) + String("\n");
			slog(msg_essential, message);
			gripper_factor_close = cmd_value; 
		}else{
			return;
		}
  }
}

int get_cmd_value(String cmd){
	String value = "";
	/* Configuration values always starts with index 2 */
	for(int i=2; i<cmd.length(); i++){
		value += (char) cmd[i];
	}

	return value.toInt();
}

void loop() 
{
	int cmd_value;
	if(initialization_status != SUCCESS){
		if(initialization_status_print != SUCCESS){
			slog(msg_nsuccess, "System Initialization, Reset and try again\n");
			initialization_status_print = SUCCESS;
		}
		return;
	}

	check_commands();
}
