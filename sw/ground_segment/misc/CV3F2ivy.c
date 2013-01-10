/*
 * Paparazzi $Id$
 *
 * Copyright (C) 2012 Arnaud van Helden
 *
 * This file is part of paparazzi.
 *
 * paparazzi is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2, or (at your option)
 * any later version.
 *
 * paparazzi is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with paparazzi; see the file COPYING.  If not, write to
 * the Free Software Foundation, 59 Temple Place - Suite 330,
 * Boston, MA 02111-1307, USA.
 *
 */



 #include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <ctype.h>
#include <unistd.h>
#include <fcntl.h>
#include <errno.h>
#include <termios.h>

#include <signal.h>

#include <Ivy/ivy.h>
#include <Ivy/ivyloop.h>
#include <Ivy/timer.h>
#include <Ivy/version.h>


typedef enum { FALSE = 0, TRUE } BOOL;

#define PACKET_LENGTH 150
#define MIN_PACKET_LENGTH 103

// global variables
int fd, ac_id = 45;
const char *device;
unsigned char packet[PACKET_LENGTH];
TimerId tid;
BOOL want_alive_msg = FALSE;


/// Handler for Ctrl-C, exits the main loop
void sigint_handler(int sig) {
  IvyStop();
  TimerRemove(tid);
  close(fd);
}

/// open the serial port with the appropiate settings
void open_port(const char* device) {
  fd = open(device, O_RDWR | O_NOCTTY );
  if (fd == -1) {
    fprintf(stderr, "open_port: unable to open device %s - ", device);
    perror(NULL);
    exit(EXIT_FAILURE);
  }
  // setup connection options
  struct termios options;

  // get the current options
  tcgetattr(fd, &options);

  // set local mode, enable receiver, set comm. options:
  // 8 data bits, 1 stop bit, no parity, 4800 Baud
  options.c_cflag = CLOCAL | CREAD | CS8 | B4800 ;

  // write options back to port
  tcsetattr(fd, TCSANOW, &options);

}

/// disable transactions and empty queue
void reset_station() {
  char newline = '\n', bytes = 0;
  fprintf(stderr, "Resetting communication\n");
  // send a \n (wakeup and cancel all running transmits)
  bytes = write(fd, &newline, 1);
  // read and discard everything that might be left in the queue
  close(fd);
  sleep(1);
  open_port(device);
}
BOOL send_loop(){
    int bytes, d_counter =0;
    unsigned char Packat[5];
    bzero (packet, PACKET_LENGTH);
    //retrieve the values one at the time
    while (d_counter < 120){
  bytes = read(fd, Packat, 1);
   
  strncat((char *)packet,(char *)Packat,1);
//    fprintf(stderr, "Received packet %s \n",Packat); 

   d_counter++;
    }
//    bytes = strlen((char *)Packat);
//fprintf(stderr, "Packat length %i \n",bytes);      
bytes = strlen((char *)packet);
//fprintf(stderr, "Packet length %i \n",bytes);      

fprintf(stderr, "Received packet:\n %s \n",packet);    
  if (bytes < MIN_PACKET_LENGTH) {
    
    fprintf(stderr, "Received packet is incomplete, only %i of %i bytes\n",
	    bytes, PACKET_LENGTH);
    return FALSE;
  
  } else {
    return TRUE;
  }
}
/// send a LOOP command (read sensor data) to the station and get the packet back
BOOL get_values(){
    int bytes;
  bytes = read(fd, packet, PACKET_LENGTH);
   fprintf(stderr, "Received packet %s \n",packet);
   
     
  if (bytes < MIN_PACKET_LENGTH) {
    fprintf(stderr, "Received packet is incomplete, only %i of %i bytes\n",
	    bytes, PACKET_LENGTH);
    return FALSE;
  
  } else {
    return TRUE;
  }
}



/// get the relevant data from the packet and sent it as Ivy message
void decode_and_send_to_ivy() {
  /* This is a typical line from the wind sensor
   * $IIMWV,179.0,R,000.30,N,A\r\n 	 first line 
   * $WIXDR,C,020.0,C,,\r\n      	 second line 
   * $PLCJ,5B,5B,5F,5F,31,\r\n     	 third line 
   * 
   * First the $ with identifier has to be found then values 
   * need to be taken from it. 
   * Then the message is broken down for the values
   */
//fprintf(stderr, "begin IVY \n");
char Wind[] = "$IIMWV";
char Temperature[] = "$WIXDR";
float windspeed_knt=0,winddir_deg=0,temp_degC=0,pstatic_Pa = 0;
int i=0;
unsigned char compare[7], temp1[6], winddir[6], windspeed[7];
int ret;

for (i=0;i<PACKET_LENGTH-10;i++){
 
  ret = sprintf((char *)compare, "%c%c%c%c%c%c" ,packet[i] ,packet[i+1] ,packet[i+2] ,packet[i+3] ,packet[i+4] ,packet[i+5]);

  
  if (strncmp((char *)compare,Wind,6)==0 && i<PACKET_LENGTH-23){
    //fprintf(stderr, "wind found \n");
    ret = sprintf((char *)winddir, "%c%c%c%c%c" ,packet[i+7],packet[i+8],packet[i+9],packet[i+10],packet[i+11]);
    ret = sprintf((char *)windspeed, "%c%c%c%c%c%c" ,packet[i+15],packet[i+16],packet[i+17],packet[i+18],packet[i+19],packet[i+20]);
    
    //	 printf("The windspeed is %s \n", windspeed);
    //	 printf("The wind direction is %s \n", winddir);
//    printf("beginning found at %i \n",i);
    winddir_deg = strtof((char *)winddir,NULL);//deg
    windspeed_knt = strtof((char *)windspeed,NULL)*0.51444;//m/s
//     printf("The wind direction is %f \n", winddir_deg);
//     printf("The wind speed is %f \n",windspeed_knt);
  }
  else if (strncmp((char *)compare,Temperature,6)==0 && i<PACKET_LENGTH-16){
    //fprintf(stderr, "temp found \n");
    ret = sprintf((char *)temp1, "%c%c%c%c%c" ,packet[i+9],packet[i+10],packet[i+11],packet[i+12],packet[i+13]);
    temp_degC = strtof((char *)temp1,NULL);
//     printf("the Temperature is %s \n", temp1);
//     printf("the Temperature is %f \n", temp_degC);
//     printf("beginning found at %i \n",i); 
    }
//     else{
//       pstatic_Pa = 99.0;
//       temp_degC = 0.0;
//       winddir_deg = 0.0;
//       windspeed_knt = 0.0;
//     
//     }
  }
  
 

  // TODO get the real MD5 for the aircraft id
  if (want_alive_msg)
    IvySendMsg("%d ALIVE 0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0\n", ac_id);

  // format has to match declaration in conf/messages.xml
  IvySendMsg("%d WEATHER %f %f %f %f\n",
    ac_id,pstatic_Pa, temp_degC, windspeed_knt, winddir_deg);
//  fprintf(stderr, "IVY verzonden \n");
}

/// Get data from the station and send it via Ivy
/** This function is executed by the timer
 */
void handle_timer (TimerId id, void *data, unsigned long delta) {
 if (send_loop()) decode_and_send_to_ivy();
}

void print_usage(int argc, char ** argv) {
  fprintf(stderr, "Usage: %s [-a] [-b <bus>] [-d <device>] [-i <aircraft_id>] [-s <delay time in seconds>]\n",
          argv[0]);
};

/// Main function
int main(int argc, char **argv) {
  // default values for options
  const char
    *defaultbus = "127.255.255.255:2010",
    *bus = defaultbus,
    *defaultdevice = "/dev/ttyUSB0";
  device = defaultdevice;
  long delay = 2000;

  // parse options
  char c;
  while ((c = getopt (argc, argv, "hab:d:i:s:")) != EOF) {
    switch (c) {
    case 'h':
      print_usage(argc, argv);
      exit(EXIT_SUCCESS);
      break;
    case 'a':
      want_alive_msg = TRUE;
      break;
    case 'b':
      bus = optarg;
      break;
    case 'd':
      device = optarg;
      break;
    case 'i':
      ac_id = atoi(optarg);
      break;
    case 's':
      delay = atoi(optarg)*1000;
      break;
    case '?':
      if (optopt == 'a' || optopt == 'b' || optopt == 'd' || optopt == 's')
        fprintf (stderr, "Option -%c requires an argument.\n", optopt);
      else if (isprint (optopt))
        fprintf (stderr, "Unknown option `-%c'.\n", optopt);
      else
        fprintf (stderr, "Unknown option character `\\x%x'.\n", optopt);
      print_usage(argc, argv);
      exit(EXIT_FAILURE);
    default:
      abort ();
    }
  }


  // make Ctrl-C stop the main loop and clean up properly
  signal(SIGINT, sigint_handler);

  bzero (packet, PACKET_LENGTH);
  open_port(device);

  // setup Ivy communication
  IvyInit("CV3F2ivy", "READY", 0, 0, 0, 0);
  IvyStart(bus);

  // create timer
  tid = TimerRepeatAfter (0, delay, handle_timer, 0);

#if IVYMINOR_VERSION == 8
  IvyMainLoop (NULL,NULL);
#else
  IvyMainLoop ();
#endif

  return 0;
}
