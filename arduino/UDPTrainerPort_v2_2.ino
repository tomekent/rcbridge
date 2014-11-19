#include <SPI.h>
#include <Ethernet.h>
#include <EthernetUDP.h>

#define channel_count        8     // Number of channels to parse
#define BYTES                16    // Is fixed to 2 bytes per channel 
#define default_servo_value  1500  // set the default servo value
#define min_servo_value      1000  // Minimum servo pwm value allowable
#define max_servo_value      2000  // Maximum servo pwm value allowable
#define PPM_FrLen            22000 // set the PPM frame length in microseconds (1ms = 1000µs)
#define PPM_PulseLen         300   // set the pulse length
#define onState              1     // set polarity of the pulses: 1 is positive, 0 is negative
#define sigPin               9     // set PPM signal output pin on the arduino
#define UDP_timeout          500   // set timeout on receiving a UDP message before setting default servo positions
#define FSpin                7     // digital pin to check for red stop button, ok when high, STOP when low

// UDP Setup
byte mac[] = {0xDE, 0xAD, 0xBE, 0xEF, 0xFE, 0xED};
IPAddress ip(192, 168, 10, 178);
unsigned int localPort = 27200;

// An EthernetUDP instance to let us send and receive packets over UDP
EthernetUDP UDP;

// buffers for receiving data
char buf[UDP_TX_PACKET_MAX_SIZE]; //buffer to hold incoming packet

// Servo values
int ppm[channel_count];           // Expect values between 1000 and 2000
int ppm_defaults[channel_count];  // Expect values between 1000 and 2000
uint16_t posn_in[8];

// Failsafe checks
bool fs_set;

// Checksum
uint16_t chksum;
uint16_t chksum_in;

// Timers
long last_update, now;

void setup(){ 
  // Start the Ethernet and UDP:
  Ethernet.begin(mac,ip);
  UDP.begin(localPort);
  
  // Set default ppm values
  for(int i=0; i<channel_count; i++) {
    ppm[i] = default_servo_value;
    ppm_defaults[i] = default_servo_value;
  }
  
  // Set failsafe check flag
  fs_set = 0;

  // Setup the ppm pin
  pinMode(sigPin, OUTPUT);
  digitalWrite(sigPin, !onState);
  
  // Setup the big red stop button pin
  pinMode(FSpin, INPUT);
  
  // Initialize the timers
  cli();
  TCCR1A = 0;              // set entire TCCR1 register to 0
  TCCR1B = 0;
  OCR1A = 100;             // compare match register, change this
  TCCR1B |= (1 << WGM12);  // turn on CTC mode
  TCCR1B |= (1 << CS11);   // 8 prescaler: 0,5 microseconds at 16mhz
  TIMSK1 |= (1 << OCIE1A); // enable timer compare interrupt
  sei();
  
  last_update = millis();
}

void loop(){
  // if there's data available, read a packet
  int packetSize = UDP.parsePacket();
 
  if(packetSize)
  {
    // read the packet into packetBufffer
    UDP.read(buf,UDP_TX_PACKET_MAX_SIZE);
    
    // Transmit packet
    if (buf[0] == 'T') {
      if (buf[1] == 1) { 
          	
        // Read servo positions
        for (int i=0;i<8;i++) {
          posn_in[i] = (uint8_t)buf[i*2+2] + (((uint8_t)buf[i*2+3]) << 8);
        }
        
        // Read given checksum
        chksum_in = (uint8_t)buf[18] + (((uint8_t)buf[19]) << 8);
        
        // Compute checksum
        chksum = 0;
        for (int i=0;i<18;i++) {
          chksum += (uint8_t)buf[i];
        }
        
        // Compare checksums
        if (chksum_in == chksum) {
          // Only copy if no one pressed the stop button
          if (digitalRead(FSpin) == HIGH) {
            for (int i=0;i<channel_count;i++) {
              // Turn the position into a ppm value
              ppm[i] = posn_in[i] + 1000;
              
              // Check the position is within limits
              if (ppm[i] > max_servo_value)
                ppm[i] = max_servo_value;
              if (ppm[i] < min_servo_value)
                ppm[i] = min_servo_value;
            }
          }
          
          // Update timer - valid packet
          last_update = millis();
        }
      }
    }
    
    // Failsafe Positions packet
    if (buf[0] == 'F') {
      if (buf[1] == 'P') { 
          	
        // Read servo positions
        for (int i=0;i<8;i++) {
          posn_in[i] = (uint8_t)buf[i*2+2] + (((uint8_t)buf[i*2+3]) << 8);
        }
        
        // Read given checksum
        chksum_in = (uint8_t)buf[18] + (((uint8_t)buf[19]) << 8);
        
        // Compute checksum
        chksum = 0;
        for (int i=0;i<18;i++) {
          chksum += (uint8_t)buf[i];
        }
        
        // Compare and copy
        if (chksum_in == chksum) {
          for (int i=0;i<channel_count;i++) {
            // Turn the position into a ppm value
            ppm_defaults[i] = posn_in[i] + 1000;
            
            // Check the position is within limits
            if (ppm_defaults[i] > max_servo_value)
              ppm_defaults[i] = max_servo_value;
            if (ppm_defaults[i] < min_servo_value)
              ppm_defaults[i] = min_servo_value;
          }
          
          // We have updated the failsafe position, update flag
          fs_set = 1;
          
          // Update timer - valid packet
          last_update = millis();
        }
      }
    }
  }
  else {
    // No packet, check timeout
    now = millis();
    if (now - last_update > UDP_timeout) {
      // Set default ppm values
      for (int i=0; i<channel_count; i++) {
        ppm[i] = ppm_defaults[i];
      }
      last_update = now;
    }
  }
  
  // Update PPM values if the stop button was pressed
  if (digitalRead(FSpin) == LOW) {
      // Set default ppm values
      for (int i=0; i<channel_count; i++) {
        ppm[i] = ppm_defaults[i];
      }
  }    
}

ISR(TIMER1_COMPA_vect){
  static boolean state = true;
  
  TCNT1 = 0;
  
  // blank pulse between channels
  if (state) {
    if (fs_set)  // Only send PPM if we have a failsafe position
      digitalWrite(sigPin, onState);
    OCR1A = PPM_PulseLen * 2 - 1;
    state = false;
  }
  
  //
  else {
    static byte cur_chan_numb;
    static unsigned int calc_rest;
  
    if (fs_set)  // Only send PPM if we have a failsafe position
      digitalWrite(sigPin, !onState);
    state = true;

    // End of ppm train
    if (cur_chan_numb >= channel_count) { 
      cur_chan_numb = 0;
      calc_rest += PPM_PulseLen;
      OCR1A = (PPM_FrLen - calc_rest) * 2 - 1;
      calc_rest = 0;
    }
    
    // Compute next channel
    else {
      OCR1A = (ppm[cur_chan_numb] - PPM_PulseLen) * 2 - 1;
      calc_rest += ppm[cur_chan_numb];
      cur_chan_numb++;
    }     
  }
}
