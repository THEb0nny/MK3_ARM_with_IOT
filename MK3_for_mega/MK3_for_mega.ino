// https://russianblogs.com/article/9027795529/
// https://microkontroller.ru/arduino-projects/podklyuchenie-datchika-holla-k-arduino/
// https://forum.arduino.cc/t/50hz-pwm-on-both-pin-9-and-10/930720/8
// https://wokwi.com/projects/316702102141272640
// http://examen-technolab.ru/instuctions/tv-0441-m-2.pdf
// https://vk.com/wall-174310634_40
// http://forum.amperka.ru/threads/%D0%91%D0%B8%D0%B1%D0%BB%D0%B8%D0%BE%D1%82%D0%B5%D0%BA%D0%B0-accelstepper.11388/#post-132258
// http://www.airspayce.com/mikem/arduino/AccelStepper/index.html
// https://github.com/NicoHood/PinChangeInterrupt
// https://github.com/GyverLibs/TimerMs
// https://lastminuteengineers.com/28byj48-stepper-motor-arduino-tutorial/

#include "AccelStepper.h"
#include "PinChangeInterrupt.h"
#include <Servo.h>

#define WIFI_SERIAL Serial3 // Serial1: пины 19(RX1) и 18(TX1); Serial2: пины 17(RX2) и 16(TX2); Serial3: пины 15(RX3) и 14(TX3)

#define MAX_TAKE_VAL_AT_SERIAL 7 // Максимальное количество значений в строку монитора порта при ручном управлении

// Определение метода шагового двигателя
#define FULLSTEP 4 // Параметры полного шага
#define HALFSTEP 8 // Параметры полушага
#define STEPMODE HALFSTEP // Выбранный режим шага

#define STEPS_PER_REVOLUTION (STEPMODE == 4 ? 2048 : 4096) // Шагов за один оборот, 4096 если HALFSTEP, а если FULLSTEP - 2048
#define ANGLE_PER_STEP 360 / STEPS_PER_REVOLUTION // Угол за один шаг
 
// Определение контактов шагового двигателя
#define STEPPER1_PIN1 33 // Контакт IN1 платы драйвера двигателя ULN2003 подключен к 28BYJ48
#define STEPPER1_PIN2 32 // Вывод IN2 платы драйвера двигателя ULN2003 подключен к 28BYJ48
#define STEPPER1_PIN3 31 // Вывод платы драйвера двигателя ULN2003 IN3 подключен к 28BYJ48
#define STEPPER1_PIN4 30 // Вывод IN4 платы драйвера двигателя ULN2003 подключен к 28BYJ48

#define STEPPER2_PIN1 37 // Контакт IN1 платы драйвера двигателя ULN2003 подключен к 28BYJ48
#define STEPPER2_PIN2 36 // Вывод IN2 платы драйвера двигателя ULN2003 подключен к 28BYJ48
#define STEPPER2_PIN3 35 // Вывод платы драйвера двигателя ULN2003 IN3 подключен к 28BYJ48
#define STEPPER2_PIN4 34 // Вывод IN4 платы драйвера двигателя ULN2003 подключен к 28BYJ48

#define STEPPER3_PIN1 12 // Контакт IN1 платы драйвера двигателя ULN2003 подключен к 28BYJ48
#define STEPPER3_PIN2 11 // Вывод IN2 платы драйвера двигателя ULN2003 подключен к 28BYJ48
#define STEPPER3_PIN3 10 // Контакт IN3 платы драйвера двигателя ULN2003 подключен к 28BYJ48
#define STEPPER3_PIN4 9 // Вывод IN4 платы драйвера двигателя ULN2003 подключен к 28BYJ48

#define STEPPER_MIN_SPEED 100 // Минимальная скорость шагового двигателя, которую можно будет установить
#define STEPPER_MAX_SPEED 1300 // Максимальная скорость шагового двигателя, которкую можно установить
#define STEPPER_DEFAULT_SPEED 1000 // Максимальная скорость шагового двигателя (в шагах за секунду)

#define STEPPER_MIN_ACCEL 10 // Минимальное ускорение шагового двигателя, которую можно будет установить
#define STEPPER_MAX_ACCEL 200 // Максимальное ускорение шагового двигателя, которую можно установить
#define STEPPER_DEFAULT_ACCEL 200 // Начальное ускорение шагового двигателя

#define HALL_SEN1_PIN 69 // Пин датчика холла 1 - A15
#define HALL_SEN2_PIN 68 // Пин датчика холла 2 - A14
#define HALL_SEN3_PIN 67 // Пин датчика холла 3 - A13

#define CLAW_SERVO_PIN 5 // Пин сервопривода

Servo servo; // Инициализируем объект серво

// Определяем объекты шагового двигателя
AccelStepper stepper1(STEPMODE, STEPPER1_PIN1, STEPPER1_PIN3, STEPPER1_PIN2, STEPPER1_PIN4);
AccelStepper stepper2(STEPMODE, STEPPER2_PIN1, STEPPER2_PIN3, STEPPER2_PIN2, STEPPER2_PIN4);
AccelStepper stepper3(STEPMODE, STEPPER3_PIN1, STEPPER3_PIN3, STEPPER3_PIN2, STEPPER3_PIN4);

volatile byte hall1State = LOW; // Переменная для записи значения состояни датчика холла 1
volatile byte hall2State = LOW; // Переменная для записи значения состояни датчика холла 2
volatile byte hall3State = LOW; // Переменная для записи значения состояни датчика холла 3

byte robotState = 0; // Переменая конечного автомата нахождения в состоянии робота

int j1_speed = STEPPER_DEFAULT_SPEED, j2_speed = STEPPER_DEFAULT_SPEED, j3_speed = STEPPER_DEFAULT_SPEED;
int j1_step_pos, j2_step_pos, j3_step_pos;
int j1_deg_pos, j2_deg_pos, j3_deg_pos, claw_pos;
int j1_deg_prev, j2_deg_prev, j3_deg_prev, claw_pos_prev;

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
  Serial.begin(115200); // Инициализируем скорость передачи данных с компом
  WIFI_SERIAL.begin(115200); // Инициализируем скорость передачи данных с ESP-01
  pinMode(HALL_SEN1_PIN, INPUT_PULLUP); // Настраиваем пин с датчиком холла 1
  pinMode(HALL_SEN2_PIN, INPUT_PULLUP); // Настраиваем пин с датчиком холла 2
  pinMode(HALL_SEN3_PIN, INPUT_PULLUP); // Настраиваем пин с датчиком холла 3
  attachPCINT(digitalPinToPCINT(HALL_SEN1_PIN), HallSensor1Handler, CHANGE); // Настраиваем прерывание датчика холла 1
  attachPCINT(digitalPinToPCINT(HALL_SEN2_PIN), HallSensor2Handler, CHANGE); // Настраиваем прерывание датчика холла 2
  attachPCINT(digitalPinToPCINT(HALL_SEN3_PIN), HallSensor3Handler, CHANGE); // Настраиваем прерывание датчика холла 3
  stepper1.setSpeed(STEPPER_DEFAULT_ACCEL); // Записываем скорость j1
  stepper1.setMaxSpeed(STEPPER_MAX_SPEED); // Максимальная скорость двигателя
  stepper1.setAcceleration(STEPPER_DEFAULT_ACCEL); // Ускорение двигателя
  stepper2.setSpeed(STEPPER_DEFAULT_ACCEL);
  stepper2.setMaxSpeed(STEPPER_MAX_SPEED);
  stepper2.setAcceleration(STEPPER_DEFAULT_ACCEL);
  stepper3.setSpeed(STEPPER_DEFAULT_ACCEL);
  stepper3.setMaxSpeed(STEPPER_MAX_SPEED);
  stepper3.setAcceleration(STEPPER_DEFAULT_ACCEL);
  servo.attach(CLAW_SERVO_PIN); // Подключение серво
}
 
void loop() {
  if (robotState == 1) ReadFromWiFiSerial(); // Считываем заначения из Serial только при состоянии ожидания считывания новых значений
  if (robotState == 0) { // Состояние 0 - роботу переместиться в нелевые позиции при старте
    // Установить скорость (в шагах за секунду)
    // Если значение датчика холла 1
    if (hall1State == LOW) {
      // Вращение манипулятора по часовой стрелке
      stepper1.setSpeed(j1_speed); // Установить скорость (в шагах за секунду)
      stepper1.runSpeed(); // Начать движение с текущей заданной скоростью (без плавного ускорения)
    } else if (stepper1.isRunning()) { // Иначе выходит датчик холла 1 сработал и если двигатель вращался, тогда однократно остановить его
      stepper1.stop(); // Максимально быстрая остановка (без замедления), используя текущие параметры скорости и ускорения
      stepper1.setCurrentPosition(0); // Установить счетчик как текущую позицию. Полезно как задание нулевой координаты. Обнуляет текущую скорость до нуля        
    }
    // Если значение датчика холла 2
    if (hall2State == LOW) {
      stepper2.setSpeed(-j2_speed); // Установить скорость (в шагах за секунду)
      stepper2.runSpeed(); // Начать движение с текущей заданной скоростью (без плавного ускорения)
    } else if (stepper2.isRunning()) { // Иначе выходит датчик холла 2 сработал и если двигатель вращался, тогда однократно остановить его
      stepper2.stop(); // Максимально быстрая остановка (без замедления), используя текущие параметры скорости и ускорения
      stepper2.setCurrentPosition(0); // Установить счетчик как текущую позицию. Полезно как задание нулевой координаты. Обнуляет текущую скорость до нуля
    }
    // Если значение датчика холла 3
    if (hall3State == LOW) {
      stepper3.setSpeed(j3_speed); // Установить скорость (в шагах за секунду)
      stepper3.runSpeed(); // Начать движение с текущей заданной скоростью (без плавного ускорения)
    } else if (stepper3.isRunning()) { // Иначе выходит датчик холла 3 сработал и если двигатель вращался, тогда однократно остановить его
      stepper3.stop(); // Максимально быстрая остановка (без замедления), используя текущие параметры скорости и ускорения
      stepper3.setCurrentPosition(0); // Установить счетчик как текущую позицию. Полезно как задание нулевой координаты. Обнуляет текущую скорость до нуля
    }
    // Если все 3 датчика холла сработали
    if (hall1State == HIGH && hall2State == HIGH && hall3State == HIGH) {
      robotState = 1; // Записываем другое состояние конечного автомата
      Serial.println("robotState: 1");
    }
  } else if (robotState == 1) { // Состояние 1 - ожидание новых значений для перемещения
    // Проверяем изменились ли состояния переменных на новые
    if (j1_deg_pos != j1_deg_prev || j2_deg_pos != j2_deg_prev || j3_deg_pos != j3_deg_prev || claw_pos != claw_pos_prev) {
      // Переводим градусы в шаги для шаговиков
      j1_step_pos = DegToStep(j1_deg_pos);
      j2_step_pos = DegToStep(j2_deg_pos);
      j3_step_pos = DegToStep(j3_deg_pos);
      robotState = 2; // Переводим в состояние перемещения
      Serial.println("robotState: 2");
    }
  } else if (robotState == 2) { // Состояние 2 - перемещения моторов в новую позицию
    if (stepper1.currentPosition() != j1_step_pos) { // Шаговик 1 работает пока не достиг позицию
      stepper1.moveTo(j1_step_pos); // Установить двигателю позицию для вращения
      stepper1.run(); // Работать двигателю
    } else { // Достиг позиции
      stepper1.stop(); // Останавливаем шаговик
      j1_deg_prev = j1_deg_pos; // Перезаписывем переменные о значениях, которые были выполнены в последний раз
    }
    if (stepper2.currentPosition() != j2_step_pos) { // Шаговик 2 работает пока не достиг позицию
      stepper2.moveTo(j2_step_pos); // Установить двигателю позицию для вращения
      stepper2.run(); // Работать двигателю
    } else { // Достиг позиции
      stepper2.stop(); // Останавливаем шаговик
      j2_deg_prev = j2_deg_pos; // Перезаписывем переменные о значениях, которые были выполнены в последний раз
    }
    if (stepper3.currentPosition() != j3_step_pos) { // Шаговик 3 работает пока не достиг позицию
      stepper3.moveTo(j3_step_pos); // Установить двигателю позицию для вращения
      stepper3.run(); // Работать двигателю
    } else { // Достиг позиции
      stepper3.stop(); // Останавливаем шаговик
      j3_deg_prev = j3_deg_pos; // Перезаписывем переменные о значениях, которые были выполнены в последний раз
    }
    if (stepper1.currentPosition() == j1_step_pos && stepper2.currentPosition() == j2_step_pos && stepper3.currentPosition() == j3_step_pos) {
      if (claw_pos != claw_pos_prev) { // Если новое значение не равно старому
        servo.write(claw_pos); // Повернуться серво
        delay(100); // Задержка
        claw_pos_prev = claw_pos; // Записать новое значение положения, которое было в последний раз
      }
      robotState = 1; // Переключаем в состояние ожидания новых значений
      Serial.println("robotState: 1");
    }
  }
  // Нельзя тут использовать delay, т.к. будет блокироваться запуск шаговика run или runSpeed
}

// Перевод градусов в шаги
float DegToStep(float deg) {
  float steps = deg / ((float) ANGLE_PER_STEP);
  return steps;
}

// Функция считывания значений по Serial от WI-FI модуля
void ReadFromWiFiSerial() {
  // Если приходят данные из Wi-Fi модуля - отправим их в порт компьютера
  if (WIFI_SERIAL.available()) {
    String inputValues[MAX_TAKE_VAL_AT_SERIAL]; // Массив входящей строки
    String key[MAX_TAKE_VAL_AT_SERIAL]; // Массив ключей
    int values[MAX_TAKE_VAL_AT_SERIAL]; // Массив значений
    // Встроенная функция readStringUntil будет читать все данные, пришедшие в UART до специального символа — '\n' (перенос строки).
    // Он появляется в паре с '\r' (возврат каретки) при передаче данных функцией Serial.println().
    // Эти символы удобно передавать для разделения команд, но не очень удобно обрабатывать. Удаляем их функцией trim().
    String inputStr = WIFI_SERIAL.readStringUntil('\n');
    inputStr.trim(); // Чистим символы
    char strBuffer[99]; // Создаём пустой массив символов
    inputStr.toCharArray(strBuffer, 99); // Перевести строку в массив символов последующего разделения по пробелам
    // Считываем x и y разделённых пробелом, а также z и инструмент
    for (byte i = 0; i < MAX_TAKE_VAL_AT_SERIAL; i++) {
      inputValues[i] = (i == 0 ? String(strtok(strBuffer, " ")) : String(strtok(NULL, " ")));
      inputValues[i].replace(" ", ""); // Убрать возможные пробелы между символами
    }
    for (byte i = 0; i < MAX_TAKE_VAL_AT_SERIAL; i++) {
      if (inputValues[i] == "") continue; // Если значение пустое, то перейти на следующий шаг цикла
      String inputValue = inputValues[i]; // Записываем в строку обрезанную часть пробелами
      byte separatorIndexTmp = inputValue.indexOf("="); // Узнаём позицию знака равно
      byte separatorIndex = (separatorIndexTmp != 255 ? separatorIndexTmp : inputValue.length());
      key[i] = inputValue.substring(0, separatorIndex); // Записываем ключ с начала строки до знака равно
      values[i] = (inputValue.substring(separatorIndex + 1, inputValue.length())).toInt(); // Записываем значение с начала цифры до конца строки
      if (key[i] == "claw_pos") {
        claw_pos = values[i]; // Записываем позицию servo клешни
      } else if (key[i] == "j1_deg_pos") {
        j1_deg_pos = values[i]; // Записываем позицию в граусах j1
      } else if (key[i] == "j1_deg_pos") {
        j2_deg_pos = values[i]; // Записываем позицию в граусах j2
      } else if (key[i] == "j3_deg_pos") {
        j3_deg_pos = values[i]; // Записываем позицию в граусах j3
      } else if (key[i] == "j1_speed") {
        j1_speed = constrain(values[i], STEPPER_MIN_SPEED, STEPPER_MAX_SPEED);
        stepper1.setSpeed(j1_speed); // Записываем скорость j1
      } else if (key[i] == "j2_speed") {
        j2_speed = constrain(values[i], STEPPER_MIN_SPEED, STEPPER_MAX_SPEED);
        stepper2.setSpeed(j2_speed); // Записываем скорость j2
      } else if (key[i] == "j3_speed") {
        j3_speed = constrain(values[i], STEPPER_MIN_SPEED, STEPPER_MAX_SPEED);
        stepper3.setSpeed(j3_speed); // Записываем скорость j3
      } else if (key[i] == "j1_accel") {
        stepper1.setAcceleration(constrain(values[i], STEPPER_MIN_ACCEL, STEPPER_MAX_ACCEL)); // Записываем ускорение j1
      } else if (key[i] == "j2_accel") {
        stepper2.setAcceleration(constrain(values[i], STEPPER_MIN_ACCEL, STEPPER_MAX_ACCEL)); // Записываем ускорение j2
      } else if (key[i] == "j3_accel") {
        stepper3.setAcceleration(constrain(values[i], STEPPER_MIN_ACCEL, STEPPER_MAX_ACCEL)); // Записываем ускорение j3
      }
      if (key[i].length() > 0) { // Печать ключ и значение, если ключ существует
        Serial.println(String(key[i]) + " = " + String(values[i]));
      }
    }
  }
}
