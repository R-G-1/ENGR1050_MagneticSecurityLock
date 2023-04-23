/*Begining of Auto generated code by Atmel studio */
#include <Arduino.h>

/*End of auto generated code by Atmel studio */


//Beginning of Auto generated function prototypes by Atmel Studio
//End of Auto generated function prototypes by Atmel Studio

// magnetic security lock Rev 0
//this uses two arduinos/ ATMEGAs to act as a lock and key with a magnasphere and electromagnet
// this is the key side of the system
// electromagnet is controled with a power mosfet and using digitalWrite to the mosfet pin. Start sending the signal by pushing the button
// the sequence begins by sending one bit for synchronization then a 19 bit signal is sent for a total of 20 bits sent
// written by Aaron McVaugh 9/24/2022
// Edited by Anna Strauss 11/30/2022
//pins
int magnet = 10; //pin for electromagnet control
int button = 5; // pin for reading push button
int button_wrong = 6;



int const sample_freq = 50; // period of the electromagnet on/off state. Determines frequency of signal
int const code_length =floor(((1000/sample_freq))-1); // length of signal in bits

int start; // var for reading button
int start_wrong; // var for reading button
int code[200];
int wrong[200] = {1,1,1,0,1,0,0,0,1,0,0,1,0,0,1,1,1,1,0,0,1,1,1,1,0,1,1,0,0,1,1,0,0,1,0,1,1,1,1,1,1,0,1,0,1,1,0,1,0,1,1,1,0,0,0,0,0,0,0,1,0,0,1,0,1,1,1,0,0,1,0,1,1,1,1,0,1,0,1,1,1,0,1,1,1,1,1,0,1,0,0,1,1,0,0,0,0,1,0,0,1,1,1,0,1,0,1,1,0,0,1,1,1,1,1,0,0,1,0,0,0,1,0,0,1,0,1,0,0,0,0,1,0,0,1,1,1,0,1,1,0,1,0,1,0,0,0,1,1,0,0,0,1,1,0,1,1,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,1,1,1,1,0,1,1,1,0,1,1,0,0,0,1,1,1,0,0,1,1,0}; //signal to be sent
int right[200] = {1,1,1,0,0,0,0,0,1,1,0,1,0,0,1,1,1,1,0,0,1,1,1,1,0,1,1,0,0,1,1,0,0,1,0,1,1,1,1,1,1,0,1,0,1,1,0,1,0,1,1,1,0,0,0,0,0,0,0,1,0,0,1,0,1,1,1,0,0,1,0,1,1,1,1,0,1,0,1,1,1,0,1,1,1,1,1,0,1,0,0,1,1,0,0,0,0,1,0,0,1,1,1,0,1,0,1,1,0,0,1,1,1,1,1,0,0,1,0,0,0,1,0,0,1,0,1,0,0,0,0,1,0,0,1,1,1,0,1,1,0,1,0,1,0,0,0,1,1,0,0,0,1,1,0,1,1,0,1,0,1,0,1,0,0,0,0,0,0,0,0,0,1,0,0,0,0,1,1,1,1,1,0,1,1,1,0,1,1,0,0,0,1,1,1,0,0,1,1,0};
//int code[49] = {1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,1,0};
//int code[49] = {,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,1};
bool cycle= false; // bool to start sending signal

//timer variables
unsigned long timer=0;
unsigned long total_time;

void setup() {
//pin modes and serial begin
pinMode(magnet,OUTPUT);
pinMode(button,INPUT);
pinMode(button_wrong,INPUT);


Serial.begin(9600);
digitalWrite(magnet,LOW);

}

void loop() {

start = digitalRead(button); //read push button 
start_wrong = digitalRead(button_wrong); //reads from the wrong push button
if (1){ //always send right signal for testing
  //if button was pushed switch cycle to true and start sending the signal
  timer = millis(); //start timer
  cycle = true;
  for (int i = 0; i<code_length; i++)
  {
    code[i] = right[i];  
  }
}
else if (start_wrong == HIGH)
{
  timer = millis();
  cycle = true;
  for (int i = 0; i<code_length; i++)
  {
    code[i] = wrong[i];  
  }}

if (cycle == true){
  //turn electromagnet on for one bit so that the lock can synchronize with the key
  digitalWrite(magnet,HIGH);
  delay(sample_freq);
  for (int i = 0; i < code_length; i++) {
    // loops through the signal. if the next bit is 0 turn the electromagnet OFF if the next bit is 1 turn the electromagnet ON
    if (code[i] == 0){
      digitalWrite(magnet,LOW);
    }
    else if (code[i] == 1){
      digitalWrite(magnet,HIGH);
    }
    // wait one period for bit to be sent
    delay(sample_freq); 
  }
  //turn off magnet at end of signal
  digitalWrite(magnet,LOW);

  //report the total time it took to send the signal. Should be (signal_length+1) * sample_freq
  total_time = millis() - timer;
  Serial.print("Signal sent in: ");
  Serial.print(total_time);
  Serial.println(" milliseconds");
  
  // wait for a second and end loop
  delay(1000);
  cycle = false;
}
delay(3000); //delay for 3 seconds to avoid error when signal sent too quickly
}
