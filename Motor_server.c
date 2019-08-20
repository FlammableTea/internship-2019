/**
 * @file Motor_server.c
 *
 * This program creates a server which waits for positional data from a client program.
 * It then moves the connected motor(s) accordingly through a PID algorithm.
 * This program needs a LOT of work
 * TODO: Tune PID control (atm the motor basically breaks when it hits the deadband)
 * TODO: Control multiple motors
 * TODO: Replace positional data from encoders to positional data from position sensors to track needle position instead of motor encoder position
 * TODO: Find ratio of encoder ticks to movement of needle
 * TODO: create a function to increment the motor small amounts
 * TODO: Have this program boot on startup
 */

#include <stdio.h>
#include <robotcontrol.h> // includes ALL Robot Control subsystems

#include <unistd.h>
#include <stdio.h>
#include <sys/socket.h>
#include <stdlib.h>
#include <netinet/in.h>
#include <string.h>
#include <arpa/inet.h>
#define PORT  6666 //port for listening to position commands


// function declarations
void on_pause_press();
void on_pause_release();
int rc_motor_moveto(int motor, int setpoint, double Kp, double Ki, double Kd);


/*
 * This program creates a server that accepts position/speed inputs
 * from a client program and moves a DC motor accordingly.
 *
 * RC template contains these critical components
 * - ensure no existing instances are running and make new PID file
 * - start the signal handler
 * - initialize subsystems you wish to use
 * - while loop that checks for EXITING condition
 * - cleanup subsystems at the end
 * @return     0 during normal operation, -1 on error
 */
int main()
{
    int freq_hz = RC_MOTOR_DEFAULT_PWM_FREQ;
	// make sure another instance isn't running
	// if return value is -3 then a background process is running with
	// higher privaledges and we couldn't kill it, in which case we should
	// not continue or there may be hardware conflicts. If it returned -4
	// then there was an invalid argument that needs to be fixed.
	if(rc_kill_existing_process(2.0)<-2) return -1;

	// start signal handler so we can exit cleanly
	if(rc_enable_signal_handler()==-1){
		fprintf(stderr,"ERROR: failed to start signal handler\n");
		return -1;
	}

	// initialize pause button
	if(rc_button_init(RC_BTN_PIN_PAUSE, RC_BTN_POLARITY_NORM_HIGH, RC_BTN_DEBOUNCE_DEFAULT_US)){
		fprintf(stderr,"ERROR: failed to initialize pause button\n");
		return -1;
	}
    if(rc_motor_init_freq(freq_hz)) return -1;
    if(rc_encoder_eqep_init()) return -1;

	// Assign functions to be called when button events occur
	rc_button_set_callbacks(RC_BTN_PIN_PAUSE,on_pause_press,on_pause_release);

	// make PID file to indicate your project is running
	// due to the check made on the call to rc_kill_existing_process() above
	// we can be fairly confident there is no PID file already and we can
	// make our own safely.
	rc_make_pid_file();


	// Keep looping until state changes to EXITING
	rc_set_state(RUNNING);

    printf("Initializing motor server...\n");

    //////////////////////////////////////////////////////
    // Start up TCP/IP Sever and listen for motor run command
    //////////////////////////////////////////////////////

    int server_fd, new_socket, target_pos, target_spd;
    struct sockaddr_in address;
    int opt = 1;
    int addrlen = sizeof(address);
    int client_is_connected = 0; 
    char recv_buff[512] = {0};

    //char *error_msg = "Invalid input. Please enter an integer or enter 'q' to shut down.";
    //char *shutdown_msg = "Shutdown command recieved. Exiting...";

    // Creating socket file descriptor
    if ((server_fd = socket(AF_INET, SOCK_STREAM, 0)) == 0)
    {
        perror("socket failed");
        exit(EXIT_FAILURE);
    }
    // Forcefully attaching socket to the port 6666
    if (setsockopt(server_fd, SOL_SOCKET, SO_REUSEADDR | SO_REUSEPORT, &opt, sizeof(opt))) {
        perror("setsockopt");
        exit(EXIT_FAILURE);
    }
    address.sin_family = AF_INET;
    address.sin_addr.s_addr = inet_addr("192.168.7.2");
    address.sin_port = htons(PORT);

    if (bind(server_fd, (struct sockaddr *)&address, sizeof(address))<0)
    {
        perror("bind failed");
        exit(EXIT_FAILURE);
    }
    if (listen(server_fd, 3) < 0)
    {
        perror("listen");
        exit(EXIT_FAILURE);
    }
    if ((new_socket = accept(server_fd, (struct sockaddr *)&address, (socklen_t*)&addrlen))<0)
    {
        perror("accept");
        exit(EXIT_FAILURE);
    }
    printf("Server initialized. Press Ctrl+C or hold pause button down for 2 seconds to exit.\n");
  

    //int bytesRead = 0;
    
    

    while(rc_get_state()!=EXITING){
		//do things based on the state
		if(rc_get_state()==RUNNING){
			rc_led_set(RC_LED_GREEN, 1);
			rc_led_set(RC_LED_RED, 0);

            
            //memset(&send_buff, 0, sizeof(send_buff));//clear the buffers
           
            memset(&recv_buff, 0, sizeof(recv_buff));
            //printf("Awaiting client response...\n");
            read(new_socket, recv_buff, 512); //the read function will pause program execution until data is recieved
            printf("Apparently not.\n");
            //There are three types of messages we can recieve from our client program
            //1) Position control message: format is "pos_{integer}\0"
            //2) Speed control message: format is "spd_{double}\0"
            //3) Quit message" "q\0" or "Q\0"

            char* msgType;
            char* msgNum;
            printf("1\n");
            int msgSize = (int)sizeof(recv_buff);
            printf("2\n");

			if(msgSize >= 6){  //pos_{int}\0 is 6 chars at minimum
                msgType = strtok(recv_buff, "_");
                msgNum = strtok(NULL, "_");
                //printf(msgType); printf("\n");
                //printf(msgNum); printf("\n");
                if(strcmp(msgType, "pos") == 0)
                {
					rc_motor_moveto(1, atoi(msgNum), 5, 0, 0);
				}
                else if(strcmp(msgType, "spd") == 0)
                {
                    rc_motor_set(1, atof(msgNum));
                }
                else printf("Invalid input. 1 \n");
                printf("3\n");
            }		
            else{
                if (strcmp(recv_buff, "Q") == 0) {
                    printf("Shutdown command recieved. Exiting...\n");
                    shutdown(new_socket, 0);
                    break;
                }
                else
                {
                    printf("Invalid input. 2\n");
                }
                printf("4\n");
            }
        }
		else{
			rc_led_set(RC_LED_GREEN, 0);
			rc_led_set(RC_LED_RED, 1);
            printf("5\n");
		}
		//always sleep at some point
		rc_usleep(100000);
	}

	// turn off LEDs and close file descriptors
    rc_set_state(EXITING);
	rc_led_set(RC_LED_GREEN, 0);
	rc_led_set(RC_LED_RED, 0);
	rc_led_cleanup();
	rc_button_cleanup();	// stop button handlers
	rc_remove_pid_file();	// remove pid file LAST
	return 0;
}

/**
 * Rotates motor to desired setpoint using a PID algorithm.
 */
int rc_motor_moveto(int motor, int setpoint, double Kp, double Ki, double Kd){

    //setup variables
    const double MIN_DUTY = -0.8;
    const double MAX_DUTY = 0.8;
    const double V_MIN = 0.05;
    const double neg_V_MIN = -0.05;
    const int DEADBAND = 1;

    int current_pos = rc_encoder_eqep_read(motor);
    int target_pos = setpoint;

    int err;
    int last_err; 
    double duty;
    double integral;
    double derivative;
    double dt = 0.008; //ideally this value would be found by testing how long each loop iteration takes to run but for now this is a decent default value(?)
    bool first_iter = true; //derivative PID value cant be found until the second iteration, so we use this flag to decide whether or not to calculate the derivative.


    //This PID algorthim is adapted from https://www.phidgets.com/docs21/DC_Motor_-_PID_Control
    while(rc_get_state()==RUNNING){ //Catches SIGINT to cleanly exit
        last_err = err;
        current_pos = rc_encoder_eqep_read(motor);
        err = target_pos - current_pos;

          // exit condition
        if(abs(err) <= DEADBAND){
            rc_motor_set(motor, 0.0);
            err = 0;
            break;
        }
        else{ duty = (Kp*err) + (Ki*integral) + (Kd*derivative);         } // PID duty calcution

        // upper/lower limit for speed
        if (duty > MAX_DUTY) {                  duty = MAX_DUTY;        }
        else if (duty < MIN_DUTY) {             duty = MIN_DUTY;        }
        else if(duty < V_MIN && duty > 0){      duty = V_MIN;        }
        else if(duty > neg_V_MIN && duty < 0){  duty = neg_V_MIN;     }
        else {integral += err*dt;       }   // no I term when plateau
        // D term
        if(!first_iter) derivative = (err - last_err)/dt;
        else first_iter = false;
        // Set motor duty cycle
        rc_motor_set(motor, duty);

        printf("Current position: %i\n", current_pos);
        printf("Current duty cycle: %f\n", duty);
    }
    rc_motor_set(motor, 0.0); //Turns off the motor if we exited with a SIGINT

    current_pos = rc_encoder_eqep_read(motor);
    printf("The setpoint was %i. The program reached %i.\n", target_pos, current_pos);
    return 0;
}


/**
 * Make the Pause button toggle between paused and running states.
 */
void on_pause_release()
{
	if(rc_get_state()==RUNNING)	rc_set_state(PAUSED);
	else if(rc_get_state()==PAUSED)	rc_set_state(RUNNING);
	return;
}

/**
* If the user holds the pause button for 2 seconds, set state to EXITING which
* triggers the rest of the program to exit cleanly.
**/
void on_pause_press()
{
	int i;
	const int samples = 100; // check for release 100 times in this period
	const int us_wait = 2000000; // 2 seconds

	// now keep checking to see if the button is still held down
	for(i=0;i<samples;i++){
		rc_usleep(us_wait/samples);
		if(rc_button_get_state(RC_BTN_PIN_PAUSE)==RC_BTN_STATE_RELEASED) return;
	}
	printf("long press detected, shutting down\n");
	rc_set_state(EXITING);
	return;
}
