/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>
#include <avr/io.h>
/*End of auto generated code by Atmel studio */


//Beginning of Auto generated function prototypes by Atmel Studio
void detect();
//End of Auto generated function prototypes by Atmel Studio

void wait(volatile int multiple, volatile char time_choice);
void delay_T_msec_timer1(volatile char choice);

// magnetic security lock Rev 0
//this uses two arduinos/ ATMEGAs to act as a lock and key with a magnasphere and electromagnet
// this is the lock side of the system
// the voltage across the magnet sphere is read at the same freqency the signal is sent but it is time delayed by one half period so that the signal is read in the middle 
// of the electromagnet on/off period
// the sequence begins by sending one bit for synchronization then a 19 bit signal is sent for a total of 20 bits sent
//on light is on pin 8 and is actitvated when accuracy is 100% 
// written by Aaron McVaugh 9/24/2022
// Edited by Anna Strauss 11/30/2022

//pins
int trip = 2; // magnasphere pin
long const sample_freq = 50; // period of the electromagnet on/off state. Determines frequency of signal
int const code_length =floor(((1000/sample_freq))-1); // length of signal in bits

int start; // var for detecting synchronization bit

int value[200]; // read signal var
volatile bool cycle= false; // bool for start of reading sequence
float accuracy;
int acc_count=0;
int inc=0;
int const light=8;
int const lightRed=9;
//timer vars
unsigned long timer=0;
unsigned long total_time;

unsigned long sample_timer;
unsigned long sample_total_time;
int const sample_number=30;
int sample_i=0;
int sample_sum=0;

bool first= true;
//int code[200] = {1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0,1,0}; //signal to be sent
int code[200] = {1,1,1,0,0,0,0,0,1,1,0,1,0,0,1,1,1,1,0,0,1,1,1,1,0,1,1,0,0,1,1,0,0,1,0,1,1,1,1,1,1,0,1,0,1,1,0,1,0,1,1,1,0,0,0,0,0,0,0,1,0,0,1,0,1,1,1,0,0,1,0,1,1,1,1,0,1,0,1,1,1,0,1,1,1,1,1,0,1,0,0,1,1,0,0,0,0,1,0,0,1,1,1,0,1,0,1,1,0,0,1,1,1,1,1,0,0,1,0,0,0,1,0,0,1,0,1,0,0,0,0,1,0,0,1,1,1,0,1,1,0,1,0,1,0,0,0,1,1,0,0,0,1,1,0,1,1,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,1,1,1,1,0,1,1,1,0,1,1,0,0,0,1,1,1,0,0,1,1,0};
//int code[49] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0};
  
void setup() {
 
//pin mode, serial begin
pinMode(trip,INPUT);
pinMode(light,OUTPUT);
pinMode(lightRed,OUTPUT);
Serial.begin(9600);
digitalWrite(light,LOW);
digitalWrite(lightRed,LOW);
Serial.println("begin");
//attachInterrupt(digitalPinToInterrupt(trip), detect, FALLING);

//setup for motor
DDRD = 1<<PD6 | 1<<PD5;// Make OC0A (PD6) and OC0B (PD5) output bits -- these are the PWM pins;
DDRC = 0b00000011; // Make PC0 and PC1 output pins

OCR0A = 0x00;       // Load $00 into OCR0 to set initial duty cycle to 0 (motor off)
TCCR0A = 0b10000011; //1<<COM0A1 | 0<<COM0A0 | 1<<WGM01 | 1<<WGM00;      // Set non-inverting mode on OC0A pin (COMA1:0 = 10; Fast PWM (WGM1:0 bits = bits 1:0 = 11) (Note that we are not affecting OC0B because COMB0:1 bits stay at default = 00)
TCCR0B = 0b00000011; //0<<CS02 | 1<<CS01 | 1<<CS00; // Set base PWM frequency (CS02:0 - bits 2-0 = 011 for prescaler of 64, for approximately 1kHz base frequency)
// PWM is now running on selected pin at selected base frequency.  Duty cycle is set by loading/changing value in OCR0A register.
//PORTC = 0b00000001; // Set forward direction

}

void loop() {
acc_count = 0;
inc = 0;

start = digitalRead(trip);

// IMPORTANT: for a normally closed switch start==LOW to begin reading signal. for a normally open switch start==HIGH to begin reading signal
if (start == LOW){ //start == HIGH for normally open
  // if the magnasphere is tripped wait half a period, and start reading the signal
  digitalWrite(light,LOW);
  digitalWrite(lightRed,LOW);
  timer = millis(); // timer start
  delay(sample_freq-8);
  cycle = true;
}


if (cycle == true){
  // will loop for number of bits in signal waiting one period then reading the voltage across the magnasphere
  for (int i = 0; i < code_length; i++) {
    if(first){
      delay(sample_freq);
    }
    else{
    delayMicroseconds(sample_freq*1000-(micros()-sample_timer));
    }
    sample_timer = micros();
    sample_i=0;
    sample_sum=0;
    while(sample_i<sample_number){
      
      sample_sum = sample_sum+digitalRead(trip);
      sample_i++;
      
    }

    if (sample_sum<(sample_number/2)){
      value[i] = 0;
    }
    else{
      value[i] = 1;
    }
    //value[i] = sample_sum;
    sample_total_time = micros()-sample_timer;
    
    
  }
  
  // report the total time it took to recieve signal should be (signal_length+1) * sample_freq -sample_freq/2
Serial.println(" ");
  total_time = millis()-timer;
  Serial.print("Signal recieved in: ");
  Serial.print(total_time);
  Serial.println(" milliseconds");
  
  //print in seperate for loop so there are no delays in reading
  for (int i = 0; i < code_length; i++) {
    Serial.println(value[i]);
    if (value[i] == code[i]){
      acc_count++;
      
    }
   inc++;
  }
  accuracy = acc_count/inc;
  Serial.println("Accuracy");
Serial.println(acc_count);
Serial.println(inc);
Serial.println(accuracy);
if (acc_count==0){
  digitalWrite(lightRed,HIGH);

  
  if (PORTC == 0b00000001) { //if forward, set backward
	  PORTC = 0b00000010; }
  else { //set forward
	  PORTC = 0b00000001;}

	OCR0A = 255;	//0xfb; // 191 is an arbitrary value showing that we load a number between 0-255 into OCR0A
	wait(8000,2);
	OCR0A = 0;
	#PORTC = 0b00000000; //added to reduce power, but change did not work not sure why
}
else
{
  digitalWrite(light,HIGH);
}

  //wait 1 sec and end loop
  delay(1000);
  cycle = false;
  digitalWrite(light,LOW);
  digitalWrite(lightRed,LOW);
}

}
void detect() {
  if (cycle == false) {cycle = true;}
  
}


void wait(volatile int multiple, volatile char time_choice) {
  /* This subroutine calls others to create a delay.
     Total delay = multiple*T, where T is in msec and is the delay created by the called function.
  
    Inputs: multiple = number of multiples to delay, where multiple is the number of times an actual delay loop is called.
    Outputs: None
  */
  
  while (multiple > 0) {
    delay_T_msec_timer1(time_choice); 
    multiple--;
  }
} // end wait()

void delay_T_msec_timer1(volatile char choice) {
  //
  // ***Note that since the Timer1 register is 16 bits, the delays can be much higher than shown here.
  // This subroutine creates a delay of T msec using TIMER1 with prescaler on clock, where, for a 16MHz clock:
  //T = 0.125 msec for prescaler set to 8 and count of 250 (preload counter with 65,535-5)
  //T = 1 msec for prescaler set to 64 and count of 250 (preload counter with 65,535-5)
  //T = 4 msec for prescaler set to 256 and count of 250 (preload counter with 65,535-5)
  //T = 16 msec for prescaler set to 1,024 and count of 250 (preload counter with 65,535-5)
  //Default: T = .0156 msec for no prescaler and count of 250 (preload counter with 65,535-5)

  //Inputs: None
  //Outputs: None

  TCCR1A = 0x00; // clears WGM00 and WGM01 (bits 0 and 1) to ensure Timer/Counter is in normal mode.
  TCNT1 = 0;  // preload load TIMER1 with 5 if counting to 255 (count must reach 65,535-5 = 250)
  // or preload with 0 and count to 250

  switch ( choice ) { // choose prescaler
    case 1:
    TCCR1B = 1<<CS11;//TCCR1B = 0x02; // Start TIMER1, Normal mode, crystal clock, prescaler = 8
    break;
    case 2:
    TCCR1B =  1<<CS11 | 1<<CS10;//TCCR1B = 0x03;  // Start TIMER1, Normal mode, crystal clock, prescaler = 64
    break;
    case 3:
    TCCR1B = 1<<CS12;//TCCR1B = 0x04; // Start TIMER1, Normal mode, crystal clock, prescaler = 256
    break;
    case 4:
    TCCR1B = 1<<CS12 | 1<<CS10;//TCCR1B = 0x05; // Start TIMER1, Normal mode, crystal clock, prescaler = 1024
    break;
    default:
    TCCR1A = 1<<CS10;//TCCR1B = 0x01; Start TIMER1, Normal mode, crystal clock, no prescaler
    break;
  }

  //while ((TIFR1 & (0x1<<TOV1)) == 0); // wait for TOV1 to roll over at 255 (requires preload of 65,535-5 to make count = 250)
  // How does this while loop work?? See notes
  while (TCNT1 < 0xfa); // exits when count = 250 (requires preload of 0 to make count = 250)

  TCCR1B = 0x00; // Stop TIMER1
  //TIFR1 = 0x1<<TOV1;  // Clear TOV1 (note that this is an odd bit in that it
  //is cleared by writing a 1 to it)

} // end delay_T_msec_timer1()
