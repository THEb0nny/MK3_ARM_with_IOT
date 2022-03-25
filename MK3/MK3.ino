// https://russianblogs.com/article/9027795529/
// https://microkontroller.ru/arduino-projects/podklyuchenie-datchika-holla-k-arduino/
// https://forum.arduino.cc/t/50hz-pwm-on-both-pin-9-and-10/930720/8
// https://wokwi.com/projects/316702102141272640
// http://examen-technolab.ru/instuctions/tv-0441-m-2.pdf

#include "AccelStepper.h"
#include <Servo.h>
 
// Определение метода шагового двигателя
#define FULLSTEP 4 // Параметры полного шага
#define HALFSTEP 8 // Параметры полушага
 
// Определение контактов шагового двигателя 
#define MOTOR1_PIN1 38 // Контакт IN1 платы драйвера двигателя ULN2003 подключен к 28BYJ48
#define MOTOR1_PIN2 44 // Вывод IN2 платы драйвера двигателя ULN2003 подключен к 28BYJ48
#define MOTOR1_PIN3 45 // Вывод платы драйвера двигателя ULN2003 IN3 подключен к 28BYJ48 на втором
#define MOTOR1_PIN4 46 // Вывод IN4 платы драйвера двигателя ULN2003 подключен к 28BYJ48

#define MOTOR2_PIN1 12 // Контакт IN1 платы драйвера двигателя ULN2003 подключен к 28BYJ48
#define MOTOR2_PIN2 11 // Вывод IN2 платы драйвера двигателя ULN2003 подключен к 28BYJ48
#define MOTOR2_PIN3 10 // Контакт IN3 платы драйвера двигателя ULN2003 подключен к 28BYJ48
#define MOTOR2_PIN4 9 // Вывод IN4 платы драйвера двигателя ULN2003 подключен к 28BYJ48
                                   
#define MOTOR3_PIN1 8 // Контакт IN1 платы драйвера двигателя ULN2003 подключен к 28BYJ48
#define MOTOR3_PIN2 7 // Вывод IN2 платы драйвера двигателя ULN2003 подключен к 28BYJ48
#define MOTOR3_PIN3 6 // Вывод платы драйвера двигателя ULN2003 IN# подключен к 28BYJ48 на втором
#define MOTOR3_PIN4 3 // Вывод IN4 платы драйвера двигателя ULN2003 подключен к 28BYJ48

#define SERVO_PIN 2

Servo servo;

// Определяем объекты шагового двигателя
AccelStepper stepper1(HALFSTEP, MOTOR1_PIN1, MOTOR1_PIN3, MOTOR1_PIN2, MOTOR1_PIN4);
AccelStepper stepper2(HALFSTEP, MOTOR2_PIN1, MOTOR2_PIN3, MOTOR2_PIN2, MOTOR2_PIN4);
AccelStepper stepper3(HALFSTEP, MOTOR3_PIN1, MOTOR3_PIN3, MOTOR3_PIN2, MOTOR3_PIN4);

void setup() {
  stepper1.setMaxSpeed(500.0); // Максимальная скорость двигателя 1 500
  stepper1.setAcceleration(50.0); // Ускорение двигателя 50.0
  stepper2.setMaxSpeed(500.0);
  stepper2.setAcceleration(50.0);
  stepper3.setMaxSpeed(500.0);
  stepper3.setAcceleration(50.0);
  servo.attach(SERVO_PIN);
}
 
void loop() {
  if (stepper1.currentPosition() == 0 && stepper2.currentPosition() == 0){
    stepper1.moveTo(2048); // Двигатель № 1 вращается на полукруга
    stepper2.moveTo(2048); // Двигатель № 2 вращается один раз
    stepper3.moveTo(2048); // Двигатель № 2 вращается один раз
  } else if (stepper1.currentPosition() == 2048 && stepper2.currentPosition() == 2048){
    stepper1.moveTo(0); // Двигатель №1 вращается на полукруга
    stepper2.moveTo(0); // Двигатель № 2 вращается один раз
    stepper3.moveTo(0); // Двигатель № 2 вращается один раз
  }
  stepper1.run(); // Двигатель 1 работает
  stepper2.run(); // Двигатель 2 работает
  stepper3.run(); // Двигатель 3 работает
  
  servo.write(180);
  delay(1000);
  servo.write(0);
  delay(1000);
}
