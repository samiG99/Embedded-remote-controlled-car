#include <Arduino_FreeRTOS.h>
#include <boarddefs.h>
#include <IRremote.h>
#include <semphr.h>
#include <IRremoteInt.h>
#include <ir_Lego_PF_BitStreamEncoder.h>

#define POWER 0x00FF629D
#define A 0x00FF22DD
#define B 0x00FF02FD
#define C 0x00FFC23D
#define UP 0x00FF9867
#define DOWN 0x00FF38C7
#define LEFT 0x00FF30CF
#define RIGHT 0x00FF7A85
#define SELECT 0x00FF18E7

SemaphoreHandle_t mutex = xSemaphoreCreateMutex();

int distance = 100;
int critical_distance = 20;

int Trig = A0; //Vcc, Trig, Echo, and Gnd)
int Echo = A1;

int left_forward = 7;
int left_backward = 8;

int right_forward = 9;
int right_backward = 10;

int left_speed_pin = 5;
int right_speed_pin = 6;

#define RECV_PIN 2

IRrecv receiver(RECV_PIN);
decode_results results;

void distance_sensor() {
  pinMode(Trig, OUTPUT);
  pinMode(Echo, INPUT);
  for (;;) {
    vTaskDelay(50);
    Serial.println("distance_sensor:");

    digitalWrite(Trig, HIGH);
    delayMicroseconds(10);
    digitalWrite(Trig, LOW);
    
    xSemaphoreTake(mutex, portMAX_DELAY);
    distance = ((340 / 2) * pow(10, -4)) * pulseIn(Echo, HIGH); // * (340/2(tur och retur) m/s) / 10^-6 = m * 100 =  cm
    xSemaphoreGive(mutex);
    Serial.println(distance);
  }
}


void IR_sensor() {
  receiver.enableIRIn();
  for (;;) {
    vTaskDelay(50);

    if (receiver.decode(&results)) {
      Serial.println("IR_sensor:");
      Serial.println(results.value, HEX);
      int value = results.value;
      receiver.resume();
      set_speed (value);

      set_direction(value);
    }

  }

}

void Stop() {
  for (;;) {
    vTaskDelay(50);
    Serial.println("Stop:");
    Serial.println(results.value, HEX);

    xSemaphoreTake(mutex, portMAX_DELAY);
    if (distance <= critical_distance && results.value == UP) {
      set_direction(SELECT);
    }
    xSemaphoreGive(mutex);

  }
}


void set_speed(int value) {
  switch (value) {
    case A:
      analogWrite(left_speed_pin, 255.f / 2.f);
      analogWrite(right_speed_pin, 255.f / 2.f);
      break;
    case B:
      analogWrite(left_speed_pin, 255.f * 0.7f);
      analogWrite(right_speed_pin, 255.f * 0.7f);
      break;
    case C:
      analogWrite(left_speed_pin, 255.f);
      analogWrite(right_speed_pin, 255.f);
      break;
  }
}


void set_direction(int value) {
  switch (value) {
    case UP:

      xSemaphoreTake(mutex, portMAX_DELAY);
      if (distance <= critical_distance) break;
      xSemaphoreGive(mutex);

      digitalWrite(left_forward, HIGH);
      digitalWrite(left_backward, LOW);
      digitalWrite(right_forward, HIGH);
      digitalWrite(right_backward, LOW);
      break;
    case DOWN:
      digitalWrite(left_forward, LOW);
      digitalWrite(left_backward, HIGH);
      digitalWrite(right_forward, LOW);
      digitalWrite(right_backward, HIGH);
      break;
    case LEFT:
      digitalWrite(left_forward, HIGH);
      digitalWrite(left_backward, LOW);
      digitalWrite(right_forward, LOW);
      digitalWrite(right_backward, LOW);
      break;
    case RIGHT:
      digitalWrite(left_forward, LOW);
      digitalWrite(left_backward, LOW);
      digitalWrite(right_forward, HIGH);
      digitalWrite(right_backward, LOW);
      break;
    case SELECT:
      digitalWrite(left_forward, LOW);
      digitalWrite(left_backward, LOW);
      digitalWrite(right_forward, LOW);
      digitalWrite(right_backward, LOW);
      break;
  }
}


void setup() {
  Serial.begin(9600);
  delay(500);

  xTaskCreate(distance_sensor, "distance_sensor",  100, NULL, 1, NULL);
  xTaskCreate(Stop, "Stop",  100, NULL, 1, NULL);
  xTaskCreate(IR_sensor, "IR_sensor",  100, NULL, 0, NULL);

}

void loop() {

}
