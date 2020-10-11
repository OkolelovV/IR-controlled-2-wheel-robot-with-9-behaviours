#define M21 5
#define M22 6
#define M11 9
#define M12 10

#define leftsensor 7
#define rightsensor 8

int left, right;

#include <IRremote.h>
#define res_pin 11

IRrecv receiver(res_pin);
decode_results results;
int res;
float t, timer, delta, wait_time;

#include "Ultrasonic.h"
Ultrasonic ultra(12, 13);
float dist;

boolean reached, started;

#define MAXSPEED 255
#define SAFEDIST 10

#define buzzer 3
#define red A1

int counter, state = 0;

void setup() {
  pinMode(M11, OUTPUT);
  pinMode(M12, OUTPUT);
  pinMode(M21, OUTPUT);
  pinMode(M22, OUTPUT);
  pinMode(buzzer, OUTPUT);
  pinMode(red, OUTPUT);
  receiver.enableIRIn();
  randomSeed(analogRead(A5));
}

void loop() {
  res = 0;
  if (receiver.decode(&results)){
    digitalWrite(red, HIGH);
    delay(50);
    digitalWrite(red, LOW);
    res = results.value;
    switch (res){
      case 0x6897:
        state = 1;
        beep(1100);
        break; 
      case 0xFFFF9867:
        state = 2;
        beep(1200);
        break;
      case 0xFFFFB04F:
        state = 3;
        beep(1300);
        break;
      case 0x30CF:
        state = 4;
        beep(1400);
        break;
      case 0x18E7:
        state = 5;
        beep(1500);
        break; 
      case 0x7A85:
        state = 6;
        beep(1600);
        break;
      case 0x10EF:
        state = 7;
        beep(1700);
        break;
      case 0x38C7:
        state = 8;
        counter = 0;
        started = false;
        t = millis();
        beep(1800);
        break;
      case 0x5AA5:
        state = 9;
        counter = 0;
        wait_time = 2000;
        beep(1900);
        break;
      case 0x4AB5:
        state = 0;
        beep(1000);
        break;      
    }
    receiver.resume();
  }

  switch (state){
    case 1:
      function_1(res);
      break;
    case 2:
      function_2(res);
      break;
    case 3:
      function_3();
      break;
    case 4:
      function_4();
      break;
    case 5:
      function_5();
      break;
    case 6:
      function_6();
      break;
    case 7:
      function_7();
      break;
    case 8:
      function_8(res);
      break;
    case 9:
      function_9();
      break;
    case 0:
      go_stop();
      break;
  }
}


//IR remote control
void function_1(int res){
  if (res){
    t = millis();
    switch (res){
      case 0x629D:
        go_forward(MAXSPEED);
        break; 
      case 0xFFFFA857:
        go_back(MAXSPEED);
        break;
      case 0x22DD:
        go_left(MAXSPEED);
        break;
      case 0xFFFFC23D:
        go_right(MAXSPEED);
        break;
    }
  }
  else if (millis() - t > 200){ go_stop(); }
}

//IR reserse remote control
void function_2(int res){
  if (res){
    t = millis();
    switch (res){
      case 0x629D:
        go_back(MAXSPEED);
        break; 
      case 0xFFFFA857:
        go_forward(MAXSPEED);
        break;
      case 0x22DD:
        go_left(MAXSPEED);
        break;
      case 0xFFFFC23D:
        go_right(MAXSPEED);
        break;
    }
  }
  else if (millis() - t > 200){ go_stop(); }  
}

//tracking a line with obstacle avoidance (reverse)
void function_3(){
  dist = ultra.read(CM); 
  right = digitalRead(rightsensor);
  left = digitalRead(leftsensor);
  if (dist < SAFEDIST) {go_reverse();}
  if (!right) {go_right(MAXSPEED);}
  if (!left) {go_left(MAXSPEED);}
  if (right && left) {go_forward(MAXSPEED);}
  if (!right && !left) {go_stop();} //not on the ground
}

//random exploring with obstacle avoidance
void function_4(){
  dist = ultra.read(CM);
  if (dist < SAFEDIST) {
    beep(500);
    go_stop();
    delay(500);
    t = random(1000, 3000);
    go_back(255);
    delay(t);
    t = random(0, 1);
    if (t){
      go_right(MAXSPEED);
    }else{
      go_left(MAXSPEED);
    }
    t = random(500, 1000);
    delay(t);
    go_stop();
  }
  else{
    go_forward(MAXSPEED);
  }
}

//searching for the closest object
void function_5(){
  float min_dist = 400;
  go_right(MAXSPEED);
  for (int i = 0; i < 50; i+=1){
    dist = ultra.read();
    if (dist < min_dist){
      min_dist = dist;
    }
    delay(50);
  }
  beep(500);
  delay(1000);
  t = millis();
  while (1){
    dist = ultra.read();
    
    if (dist <= min_dist*1.05){   //found again + uncertainty
      go_stop();
      beep(4000);
      beep(4000);
      beep(4000);
      delay(1000);
      go_forward(MAXSPEED);
      while (dist > 5){
        dist = ultra.read();
      }
      delay(500);
      beep(2000);
      beep(2000);
      break;
    }
    
    if (millis() - t > 4000){   //failed to find again
      go_stop();
      beep(500);
      break;
    }
  }
  
  go_stop();
  state = 0;  
}

//automatic lawn mower (staying in the circle)
void function_6(){
  right = digitalRead(rightsensor);
  left = digitalRead(leftsensor);
  if (!right){
    go_back(MAXSPEED);
    beep(500);
    t = random(100, 600);
    delay(t);
    t = random(0, 4);             //make higher possibility to explore in left direction
    if (t){ go_left(MAXSPEED); }
    else{   go_right(MAXSPEED);  }
    t = random(500, 700);
    delay(t);
    go_stop();    
  }
  else if (!left){
    go_back(MAXSPEED);
    beep(2000);
    t = random(100, 600);
    delay(t);
    t = random(0, 4);             //make higher possibility to explore in left direction
    if (t){ go_right(MAXSPEED); }
    else{ go_left(MAXSPEED);  }
    t = random(500, 700);
    delay(t);
    go_stop();    
  }
  else{
    go_forward(MAXSPEED);
  }
}

//searching for a treasure inside a circle
void function_7(){
  go_forward(MAXSPEED);
  right = digitalRead(rightsensor);
  left = digitalRead(leftsensor);
  if (!right || !left){
    t = millis();
    while(!right || !left){
      right = digitalRead(rightsensor);
      left = digitalRead(leftsensor);  
    }
    delta = millis() - t;
    if (delta > 200){
      go_stop();
      delay(500);
      beep(4000);
      go_right(MAXSPEED);
      delay(500);
      beep(4000);
      delay(500);
      go_stop();
      state = 0;
    }
    else{
      go_back(MAXSPEED);
      delay(delta + 500);
      go_stop();
      t = random(0, 1);
      if (t){ go_right(MAXSPEED); }
      else{   go_left(MAXSPEED);  }
      t = random(500, 1000);
      delay(t);
      go_stop();
    }
  }
}

//speedy parking game
void function_8(int res){
  function_1(res);      //control movement
  
  dist = ultra.read();

  if (millis() - t > 60000){      //time limit
    beep(500);
    beep(500);
    beep(500);
    state = 0;
  }
  else{
    if (!started  && dist > 15){
      started = true;
    }
    
    if (started){
      if (dist < 8 || dist > 12){
        reached = false;
      }
      else{
        if (!reached){
          timer = millis();
          reached = true;
        }
        else if (reached && millis() - timer > 3000){
          if (counter < 2){
            digitalWrite(red, HIGH);
            beep(1500);
            digitalWrite(red, LOW);
            started = false;
            counter++;
          }
          else{
            for (int i = 0; i < 2; i++){
              digitalWrite(red, HIGH);
              beep(1500);
              digitalWrite(red, LOW);
              beep(1500);
            }
            state = 0;
          }
        }
      }
    }
  }
}

//following hand commands
void function_9(){
  reached = false;
  dist = ultra.read();
  if (dist >= 1 && dist <= 15){
    reached = true;
    t = millis();
    while (dist >= 1 && dist <= 15 && millis() - t < 2000){
      dist = ultra.read();
      }
      if (millis() - t > 2000){
        while(1){
          dist = ultra.read();
          if (dist >= 11 && dist <= 20) {
            go_forward(MAXSPEED/2);
            }
          else if (dist >= 5 && dist <= 9) {
            go_back(MAXSPEED/2);
            }
          else if (dist == 10){
            go_stop();
          }
          else if (dist < 5 || dist > 20) {
            beep(1000);
            go_stop();
            counter = 0;
            wait_time = 2000;
            break;
          }
        }
      }
      else{
        counter++;
        wait_time += 1000;
      }
  }

  if (reached && counter == 1){
    timer = millis();
  }
  if (counter > 0 && millis() - timer > wait_time){
    if (counter == 2){
      go_right(MAXSPEED);
      delay(1500);
    }
    if (counter == 3){
      go_left(MAXSPEED);
      delay(1500);
    }
    beep(1500);
    go_stop();
    counter = 0;
    wait_time = 2000;
  }
}


void go_reverse(){
  int i = 0;
  while (1){
    go_left(MAXSPEED);
    left = digitalRead(leftsensor);
    right = digitalRead(rightsensor);
    if (!right && !left) {
      go_stop();
      break;
    }
    if (!left) {i = 1;}
    if (left && i) {break;}
  }
}

void beep(int frequency){
  tone(buzzer, frequency, 200);
  delay(400);
}

void go_forward(int val){
  analogWrite(M11, val);
  analogWrite(M12, LOW);
  analogWrite(M21, val);
  analogWrite(M22, LOW);
}
  
void go_back(int val){
  analogWrite(M11, LOW);
  analogWrite(M12, val);
  analogWrite(M21, LOW);
  analogWrite(M22, val);
}

void go_right(int val){
  analogWrite(M11, val);
  analogWrite(M12, LOW);
  analogWrite(M21, LOW);
  analogWrite(M22, val);
}

void go_left(int val){
  analogWrite(M11, LOW);
  analogWrite(M12, val);
  analogWrite(M21, val);
  analogWrite(M22, LOW);
}

void go_stop(){
  analogWrite(M11, LOW);
  analogWrite(M12, LOW);
  analogWrite(M21, LOW);
  analogWrite(M22, LOW);  
}
