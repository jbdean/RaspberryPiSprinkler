/*
 * SundayWater.c
 * This is called by cron every Sunday.
 * The crontab is:
 * 0 6,18 * 5,10 0 /home/pi/Sprinkler/SundayWater
 *
 ***********************************************************************
 *
 *    wiringPi is free software: you can redistribute it and/or modify
 *    it under the terms of the GNU Lesser General Public License as published by
 *    the Free Software Foundation, either version 3 of the License, or
 *    (at your option) any later version.
 *
 *    wiringPi is distributed in the hope that it will be useful,
 *    but WITHOUT ANY WARRANTY; without even the implied warranty of
 *    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *    GNU Lesser General Public License for more details.
 *
 *    You should have received a copy of the GNU Lesser General Public License
 *    along with wiringPi.  If not, see <http://www.gnu.org/licenses/>.
 ***********************************************************************
 */

#include <stdio.h>
#include <wiringPi.h>

#define RANGE		100
#define	NUM_ZONES	  8
#define ON			  0
#define OFF			  1
#define ZONE1         0
#define ZONE2         2
#define ZONE3         3
#define ZONE4         4
#define ZONE5         5
#define ZONE6         6
#define ZONE7        25
#define ZONE8        27

// zone served, gpio number, wire color
//lawn, left hedges, front hedges, right hedges, drip,
//   0,           2,            3,            4,    5,
// grn,         ylw,          blu,          orn,  gry,
//
// 6(6) red, 7(25) prp


int zoneMap [NUM_ZONES] = { 0, 2, 3, 4, 5, 6, 25, 27 } ;//needs mapping

int times [NUM_ZONES] = { 180000, 180000, 180000, 180000, 360000, 500, 500, 500 } ;
// 180000 ms is three minutes

int main (void) {
  int i, j ;
  printf ("Raspberry Pi Sunday Water\n") ;

  //wiringPiSetup () ;
  if(wiringPiSetup() == -1) { //wiringPiSetupPhys
	  printf("Wiring Pi setup failed\n");
	  return 1;
  }
  for(i = 0; i < NUM_ZONES; i++) {
    pinMode (zoneMap[i], OUTPUT) ;
    digitalWrite (zoneMap[i], OFF) ;	// Off
  }
    
  for (i = 0;i < NUM_ZONES; i++){
      
    digitalWrite (zoneMap[i], ON) ;	// On
    printf ("Zone %d on\n", i) ;
    delay (times[i]) ;		    // mS
    digitalWrite (zoneMap[i], OFF) ;	// Off
    printf ("Zone %d off\n", i) ;
    delay (500) ;
  }

  return 0 ;
}

