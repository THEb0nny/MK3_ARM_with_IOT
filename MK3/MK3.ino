// https://russianblogs.com/article/9027795529/
// https://microkontroller.ru/arduino-projects/podklyuchenie-datchika-holla-k-arduino/
// https://forum.arduino.cc/t/50hz-pwm-on-both-pin-9-and-10/930720/8
// https://wokwi.com/projects/316702102141272640
// http://examen-technolab.ru/instuctions/tv-0441-m-2.pdf
// https://vk.com/wall-174310634_40
// http://forum.amperka.ru/threads/%D0%91%D0%B8%D0%B1%D0%BB%D0%B8%D0%BE%D1%82%D0%B5%D0%BA%D0%B0-accelstepper.11388/#post-132258
// http://www.airspayce.com/mikem/arduino/AccelStepper/index.html
// https://github.com/NicoHood/PinChangeInterrupt

#include "AccelStepper.h"
#include "PinChangeInterrupt.h"
#include <Servo.h>
 
// Определение метода шагового двигателя
#define FULLSTEP 4 // Параметры полного шага
#define HALFSTEP 8 // Параметры полушага
 
// Определение контактов шагового двигателя
#define MOTOR1_PIN1 8 // Контакт IN1 платы драйвера двигателя ULN2003 подключен к 28BYJ48
#define MOTOR1_PIN2 7 // Вывод IN2 платы драйвера двигателя ULN2003 подключен к 28BYJ48
#define MOTOR1_PIN3 6 // Вывод платы драйвера двигателя ULN2003 IN# подключен к 28BYJ48 на втором
#define MOTOR1_PIN4 3 // Вывод IN4 платы драйвера двигателя ULN2003 подключен к 28BYJ48

#define MOTOR2_PIN1 38 // Контакт IN1 платы драйвера двигателя ULN2003 подключен к 28BYJ48
#define MOTOR2_PIN2 44 // Вывод IN2 платы драйвера двигателя ULN2003 подключен к 28BYJ48
#define MOTOR2_PIN3 45 // Вывод платы драйвера двигателя ULN2003 IN3 подключен к 28BYJ48 на втором
#define MOTOR2_PIN4 46 // Вывод IN4 платы драйвера двигателя ULN2003 подключен к 28BYJ48

#define MOTOR3_PIN1 12 // Контакт IN1 платы драйвера двигателя ULN2003 подключен к 28BYJ48
#define MOTOR3_PIN2 11 // Вывод IN2 платы драйвера двигателя ULN2003 подключен к 28BYJ48
#define MOTOR3_PIN3 10 // Контакт IN3 платы драйвера двигателя ULN2003 подключен к 28BYJ48
#define MOTOR3_PIN4 9 // Вывод IN4 платы драйвера двигателя ULN2003 подключен к 28BYJ48

#define HALL_SEN1_PIN 69 // Пин датчика холла 1 - A15
#define HALL_SEN2_PIN 68 // Пин датчика холла 2 - A14
#define HALL_SEN3_PIN 67 // Пин датчика холла 3 - A13

#define CLAW_SERVO_PIN 5 // Пин сервопривода

Servo servo; // Инициализируем объект серво

// Определяем объекты шагового двигателя
AccelStepper stepper1(HALFSTEP, MOTOR1_PIN1, MOTOR1_PIN3, MOTOR1_PIN2, MOTOR1_PIN4);
AccelStepper stepper2(HALFSTEP, MOTOR2_PIN1, MOTOR2_PIN3, MOTOR2_PIN2, MOTOR2_PIN4);
AccelStepper stepper3(HALFSTEP, MOTOR3_PIN1, MOTOR3_PIN3, MOTOR3_PIN2, MOTOR3_PIN4);

volatile byte hall1State = LOW; // Переменная для записи значения состояни датчика холла 1
volatile byte hall2State = LOW; // Переменная для записи значения состояни датчика холла 2
volatile byte hall3State = LOW; // Переменная для записи значения состояни датчика холла 3

byte robotState = 0; // Переменая конечного автомата нахождения в состоянии робота

// Функция-обработчик прерывания датчика холла 1
void HallSensor1Handler(void) {
  hall1State = !hall1State;
}

// Функция-обработчик прерывания датчика холла 2
void HallSensor2Handler(void) {
  hall2State = !hall2State;
}

// Функция-обработчик прерывания датчика холла 3
void HallSensor3Handler(void) {
  hall3State = !hall3State;
}

void setup() {
  Serial.begin(9600); // Инициализируем скорость передачи данных с компом
  pinMode(HALL_SEN1_PIN, INPUT_PULLUP); // Настраиваем пин с датчиком холла 1
  pinMode(HALL_SEN2_PIN, INPUT_PULLUP); // Настраиваем пин с датчиком холла 2
  pinMode(HALL_SEN3_PIN, INPUT_PULLUP); // Настраиваем пин с датчиком холла 3
  attachPCINT(digitalPinToPCINT(HALL_SEN1_PIN), HallSensor1Handler, CHANGE); // Настраиваем прерывание датчика холла 1
  attachPCINT(digitalPinToPCINT(HALL_SEN2_PIN), HallSensor2Handler, CHANGE); // Настраиваем прерывание датчика холла 2
  attachPCINT(digitalPinToPCINT(HALL_SEN3_PIN), HallSensor3Handler, CHANGE); // Настраиваем прерывание датчика холла 3
  stepper1.setMaxSpeed(1300); // Максимальная скорость двигателя 1500
  stepper1.setAcceleration(200); // Ускорение двигателя 50
  stepper2.setMaxSpeed(1300);
  stepper2.setAcceleration(200);
  stepper3.setMaxSpeed(1300);
  stepper3.setAcceleration(200);
  servo.attach(CLAW_SERVO_PIN); // Подключение серво
}
 
void loop() {
  Serial.println(String(hall1State) + ", " + String(hall2State) + ", " + String(hall3State));
  if (robotState == 0) { // Состояние 0 - роботу переместиться в нелевые позиции при старте
    // Если значение датчика холла 1
    if (hall1State == LOW) {
      // Вращение манипулятора по часовой стрелке
      stepper1.setSpeed(1000); // Установить скорость (в шагах за секунду)
      stepper1.runSpeed(); // Начать движение с текущей заданной скоростью (без плавного ускорения)
    } else { // Иначе выходит датчик холла 1 сработал
      stepper1.stop(); // Максимально быстрая остановка (без замедления), используя текущие параметры скорости и ускорения
      stepper1.setCurrentPosition(0); // Установить счетчик как текущую позицию. Полезно как задание нулевой координаты. Обнуляет текущую скорость до нуля
    }
    // Если значение датчика холла 1
    if (hall2State == LOW) {
      // Вращение манипулятора по часовой стрелке
      stepper2.setSpeed(1000); // Установить скорость (в шагах за секунду)
      stepper2.runSpeed(); // Начать движение с текущей заданной скоростью (без плавного ускорения)
    } else { // Иначе выходит датчик холла 1 сработал
      stepper2.stop(); // Максимально быстрая остановка (без замедления), используя текущие параметры скорости и ускорения
      stepper2.setCurrentPosition(0); // Установить счетчик как текущую позицию. Полезно как задание нулевой координаты. Обнуляет текущую скорость до нуля
    }
    // Если значение датчика холла 1
    if (hall3State == LOW) {
      // Вращение манипулятора по часовой стрелке
      stepper3.setSpeed(1000); // Установить скорость (в шагах за секунду)
      stepper3.runSpeed(); // Начать движение с текущей заданной скоростью (без плавного ускорения)
    } else { // Иначе выходит датчик холла 1 сработал
      stepper3.stop(); // Максимально быстрая остановка (без замедления), используя текущие параметры скорости и ускорения
      stepper3.setCurrentPosition(0); // Установить счетчик как текущую позицию. Полезно как задание нулевой координаты. Обнуляет текущую скорость до нуля
    }
    // Если все 3 датчика холла сработали
    if (hall1State == LOW && hall2State == LOW && hall3State == LOW) { 
      robotState = 1; // Записываем другое состояние конечного автомата
    }
  } else if (robotState == 1) { // Состояние 1
    // Пример кода в robotState = 1 
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
  }
  // Нельзя тут использовать delay, т.к. будет блокироваться запуск шаговика run или runSpeed
  /*
  servo.write(180);
  delay(1000);
  servo.write(0);
  delay(1000);
  */
}
