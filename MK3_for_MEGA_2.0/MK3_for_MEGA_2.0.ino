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
// https://github.com/GyverLibs/GParser

#include "AccelStepper.h"
#include "PinChangeInterrupt.h"
#include <Servo.h>
#include <GParser.h>

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

Servo claw_servo; // Инициализируем объект серво

// Определяем объекты шаговых двигателей
AccelStepper stepper1(STEPMODE, STEPPER1_PIN1, STEPPER1_PIN3, STEPPER1_PIN2, STEPPER1_PIN4);
AccelStepper stepper2(STEPMODE, STEPPER2_PIN1, STEPPER2_PIN3, STEPPER2_PIN2, STEPPER2_PIN4);
AccelStepper stepper3(STEPMODE, STEPPER3_PIN1, STEPPER3_PIN3, STEPPER3_PIN2, STEPPER3_PIN4);

volatile byte hall1State = LOW; // Переменная для записи значения состояни датчика холла 1
volatile byte hall2State = LOW; // Переменная для записи значения состояни датчика холла 2
volatile byte hall3State = LOW; // Переменная для записи значения состояни датчика холла 3

byte robotState = 0; // Переменая конечного автомата нахождения в состоянии робота

int j1_speed = STEPPER_DEFAULT_SPEED, j2_speed = STEPPER_DEFAULT_SPEED, j3_speed = STEPPER_DEFAULT_SPEED; // Переменные для хранения скорости двигателей
int j1_step_pos = 0, j2_step_pos = 0, j3_step_pos = 0; // Переменные для хранения значений сколько шагов нужно выполнить
int j1_deg_pos = 0, j2_deg_pos = 0, j3_deg_pos = 0, claw_pos = 90; // Переменные для хранения значений, которые были считаны по Serial
int j1_deg_prev = 0, j2_deg_prev = 0, j3_deg_prev = 0, claw_pos_prev = claw_pos; // Переменные для хранения значений, которые были получены по Serial и выполнены в прошлый раз

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
  Serial.setTimeout(5); // Позволяет задать время ожидания данных
  WIFI_SERIAL.setTimeout(5); // Позволяет задать время ожидания данных
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
  claw_servo.attach(CLAW_SERVO_PIN); // Подключение серво
  claw_servo.write(claw_pos); // Установить позицию серво клешни при старте
}
 
void loop() {
  if (robotState == 0) { // Состояние 0 - роботу переместиться в нелевые позиции при старте
    MoveToZeroPos(); // Вызвать функцию перемещения манипулятора моторами на нулевую позицию
  } else if (robotState == 1) { // Состояние 1 - ожидание новых значений для перемещения
    ParseFromSerialInputValues(true); // Считать с Serial значения полученные от контроллера ESP
    // Проверяем изменились ли состояния переменных входных значений на новые
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
      stepper1.moveTo(j1_step_pos); // Установить двигателю 1 позицию для вращения
      stepper1.run(); // Работать двигателю 1
    } else { // Достиг позиции
      stepper1.stop(); // Останавливаем шаговик 2
      j1_deg_prev = j1_deg_pos; // Перезаписывем переменные о значениях, которые были выполнены в последний раз
    }
    if (stepper2.currentPosition() != j2_step_pos) { // Шаговик 2 работает пока не достиг позицию
      stepper2.moveTo(j2_step_pos); // Установить двигателю 2 позицию для вращения
      stepper2.run(); // Работать двигателю 2
    } else { // Достиг позиции
      stepper2.stop(); // Останавливаем шаговик 2
      j2_deg_prev = j2_deg_pos; // Перезаписывем переменные о значениях, которые были выполнены в последний раз
    }
    if (stepper3.currentPosition() != j3_step_pos) { // Шаговик 3 работает пока не достиг позицию
      stepper3.moveTo(j3_step_pos); // Установить двигателю 3 позицию для вращения
      stepper3.run(); // Работать двигателю 3
    } else { // Достиг позиции
      stepper3.stop(); // Останавливаем шаговик 3
      j3_deg_prev = j3_deg_pos; // Перезаписывем переменные о значениях, которые были выполнены в последний раз
    }
    if (stepper1.currentPosition() == j1_step_pos && stepper2.currentPosition() == j2_step_pos && stepper3.currentPosition() == j3_step_pos) {
      if (claw_pos != claw_pos_prev) { // Если новое значение не равно старому
        claw_servo.write(claw_pos); // Повернуться серво
        delay(100); // Задержка
        claw_pos_prev = claw_pos; // Записать новое значение положения, которое было в последний раз
      }
      robotState = 1; // Переключаем в состояние ожидания новых значений
      Serial.println("robotState: 1");
    }
  }
  // Нельзя тут использовать delay, т.к. будет блокироваться запуск шаговика run или runSpeed
}

// Функция поворота на нулевую позицию всеми шаговиками
void MoveToZeroPos() {
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
}

// Перевод градусов в шаги
float DegToStep(float deg) {
  float steps = deg / ((float) ANGLE_PER_STEP);
  return steps;
}

// Парсинг значений из Serial от WI-FI модуля
void ParseFromSerialInputValues(bool debug) {
  if (Serial.available() > 2) { // Если что-то прислали
    char inputStr[64]; // Массив символов для записи из Serial
    int amount = Serial.readBytesUntil(';', inputStr, 64); // Считать посимвольно до символа конца пакета точки с запятой и записать количество полученных байт в переменную
    inputStr[amount] = NULL; // Если отправляющее устройство не отправит нулевой символ, то он не запишется в буффер и вывод строк будет некорректным, решение дописать вручную и т.о. закрываем строку
    GParser data(inputStr, ','); // Парсим массив символов по символу запятой
    int am = data.split(); // Получаем количество данных, внимание, ломает строку!
    for (int i = 0; i < am; i++) {
      String tmpStr = data[i];
      tmpStr.replace(" ", ""); // Удалить пробел, если он был введёт по ошибке
      tmpStr.trim(); // Удаление ведущими и конечные пробелы
      char tmpCharArr[tmpStr.length()];
      tmpStr.toCharArray(tmpCharArr, tmpStr.length() + 1);
      if (debug) Serial.println(String(i) + ") " + tmpStr); // Вывести начальную строку
      GParser data2(tmpCharArr, ':'); // Парсим массив символов по символу запятой
      int am2 = data2.split(); // Получаем количество данных, внимание, ломает строку!
      if (am2 > 1) { // Если существует не только ключ, а ещё и значение
        String key = data2[0]; // Ключ - первое значение
        int value = data2.getInt(1); // Значение - второе
        if (debug) Serial.println("key: " + key + ", value: " + String(value)); // Вывод
        // Присваивание значений
        if (key == "claw_pos") {
          claw_pos = value; // Записываем позицию servo клешни
        } else if (key == "j1_pos") {
          j1_deg_pos = value; // Записываем позицию в градусах j1
        } else if (key == "j2_pos") {
          j2_deg_pos = value; // Записываем позицию в градусах j2
        } else if (key == "j3_pos") {
          j3_deg_pos = value; // Записываем позицию в градусах j3
        } else if (key == "j1_speed") {
          j1_speed = constrain(value, STEPPER_MIN_SPEED, STEPPER_MAX_SPEED); // Огранчиваем входные значения скоростей для j1
          stepper1.setSpeed(j1_speed); // Записываем скорость j1
        } else if (key == "j2_speed") {
          j2_speed = constrain(value, STEPPER_MIN_SPEED, STEPPER_MAX_SPEED); // Огранчиваем входные значения скоростей для j2
          stepper2.setSpeed(j2_speed); // Записываем скорость j2
        } else if (key == "j3_speed") {
          j3_speed = constrain(value, STEPPER_MIN_SPEED, STEPPER_MAX_SPEED); // Огранчиваем входные значения скоростей для j3
          stepper3.setSpeed(j3_speed); // Записываем скорость j3
        } else if (key == "j1_accel") {
          stepper1.setAcceleration(constrain(value, STEPPER_MIN_ACCEL, STEPPER_MAX_ACCEL)); // Записываем ускорение j1
        } else if (key == "j2_accel") {
          stepper2.setAcceleration(constrain(value, STEPPER_MIN_ACCEL, STEPPER_MAX_ACCEL)); // Записываем ускорение j2
        } else if (key == "j3_accel") {
          stepper3.setAcceleration(constrain(value, STEPPER_MIN_ACCEL, STEPPER_MAX_ACCEL)); // Записываем ускорение j3
        }
      }
      if (debug) Serial.println(); // Перевод на новую строку для разделения значений, которые были введены
    }
  }
}