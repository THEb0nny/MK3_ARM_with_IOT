// https://github.com/GyverLibs/GyverHub

#include <Arduino.h>
#include <GyverHub.h>

#define AP_SSID "Connectify_Mi_Router3" // Имя точки доступа
#define AP_PASS "2016APP1336" // Пароль от точки доступа

#define MQTT_PORT 13994 // Порт MQTT
#define MQTT_USR_NAME "u_MK3" // Имя пользователя MQTT брокера
#define MQTT_USR_PASS "NJZ3Xw77" // Пароль пользователя MQTT брокера

#define STEPPER_MIN_SPEED 100 // Минимальная скорость шагового двигателя, которую можно будет установить
#define STEPPER_MAX_SPEED 1300 // Максимальная скорость шагового двигателя, которкую можно установить
#define STEPPER_DEFAULT_SPEED 1000 // Максимальная скорость шагового двигателя (в шагах за секунду)

#define STEPPER_MIN_ACCEL 10 // Минимальное ускорение шагового двигателя, которую можно будет установить
#define STEPPER_MAX_ACCEL 200 // Максимальное ускорение шагового двигателя, которую можно установить
#define STEPPER_DEFAULT_ACCEL 200 // Начальное ускорение шагового двигателя

#define STEPPERS_SPEED_STEP_SLIDERS 10 // Шаг изменения скорости в слайдере
#define STEPPERS_ACCEL_STEP_SLIDERS 10 // Шаг изменения ускорений в слайдере

#define DEBUG false // Вывод значений для дебага в Serial

GyverHub hub("Manipulator", "MK3", ""); // Инициализация объекта hub с префиксом, имя, иконка

int j1_pos = 0, j2_pos = 0, j3_pos = 0, claw_pos = 90;
int j1_speed = STEPPER_DEFAULT_SPEED, j2_speed = STEPPER_DEFAULT_SPEED, j3_speed = STEPPER_DEFAULT_SPEED;
int j1_accel = STEPPER_DEFAULT_ACCEL, j2_accel = STEPPER_DEFAULT_ACCEL, j3_accel = STEPPER_DEFAULT_ACCEL;
int x_val = 0, y_val = 0, z_val = 0;

// Переменные для интерфейса
GHbutton send_btn; // Виджет кнопки

uint8_t tab;

void setup() {
  Serial.begin(115200); // Инициализация Serial для общения по UART
  WiFi.mode(WIFI_STA);
  WiFi.begin(AP_SSID, AP_PASS);
  if (DEBUG) Serial.println();
  while (WiFi.status() != WL_CONNECTED) { // Ждём пока подключимся к WiFi
    delay(500);
    if (DEBUG) Serial.print(".");
  }
  Serial.println();
  if (DEBUG) Serial.println(WiFi.localIP()); // Печать локального IP адреса
  hub.setupMQTT("m8.wqtt.ru", MQTT_PORT, MQTT_USR_NAME, MQTT_USR_PASS); // Настройка MQTT
  hub.onBuild(build); // Подключаем билдер
  hub.begin(); // Запускаем систему
}

void loop() {
  hub.tick(); // Обязательно тикаем тут для обновления
}

// Это билдер, который вызывается библиотекой для сборки интерфейса, чтения значений и прочего
void build() {
  hub.Tabs(&tab, "Joints control,IK control,Steppers settings");
  if (tab == 0) {
    hub.BeginWidgets(); // Начинает новую горизонтальную строку виджетов
    hub.WidgetSize(100); // Указываем, что последующие виджеты будут шириной 100%
    if (hub.Slider(&j1_pos, GH_UINT8, F("J1 degress"), 0, 360, 1) && DEBUG) { // К слайдеру подключена переменная j1_pos_slider
      Serial.println("Slider J1 deg: " + String(j1_pos)); // Переменная уже обновилась и новое значение доступно во всей области определения
    }
    if (hub.Slider(&j2_pos, GH_UINT8, F("J2 degress"), 0, 360, 1) && DEBUG) { // К слайдеру подключена переменная j2_pos_slider
      Serial.println("Slider J2 deg: " + String(j2_pos)); // Переменная уже обновилась и новое значение доступно во всей области определения
    }
    if (hub.Slider(&j3_pos, GH_UINT8, F("J3 degress"), 0, 360, 1) && DEBUG) { // К слайдеру подключена переменная j3_pos_slider
      Serial.println("Slider J3 deg: " + String(j3_pos)); // Переменная уже обновилась и новое значение доступно во всей области определения
    }
    hub.EndWidgets(); // Закончим отрисовку виджетов
  } else if (tab == 1) {
    hub.BeginWidgets(); // Начинает новую горизонтальную строку виджетов
    hub.WidgetSize(100); // Указываем, что последующие виджеты будут шириной 100%
    if (hub.Slider(&x_val, GH_UINT8, F("X"), 0, 360, 1) && DEBUG) { // К слайдеру подключена переменная j1_pos_slider
      Serial.println("Slider X: " + String(x_val)); // Переменная уже обновилась и новое значение доступно во всей области определения
    }
    if (hub.Slider(&y_val, GH_UINT8, F("Y"), 0, 360, 1) && DEBUG) { // К слайдеру подключена переменная j2_pos_slider
      Serial.println("Slider Y: " + String(y_val)); // Переменная уже обновилась и новое значение доступно во всей области определения
    }
    if (hub.Slider(&z_val, GH_UINT8, F("Z"), 0, 360, 1) && DEBUG) { // К слайдеру подключена переменная j3_pos_slider
      Serial.println("Slider Z: " + String(z_val)); // Переменная уже обновилась и новое значение доступно во всей области определения
    }
    hub.EndWidgets(); // Закончим отрисовку виджетов
  }
  if (tab == 0 || tab == 1) {
    hub.BeginWidgets(); // Начинает новую горизонтальную строку виджетов
    if (hub.Slider(&claw_pos, GH_UINT8, F("Servo Claw position"), 0, 180, 1) && DEBUG) { // К слайдеру подключена переменная servo_claw_pos_slider
      Serial.println("Slider Сlaw pos: " + String(claw_pos)); // Переменная уже обновилась и новое значение доступно во всей области определения
    }
    hub.EndWidgets(); // Закончим отрисовку виджетов
  }
  if (tab == 2) {
    hub.BeginWidgets(); // Начинает новую горизонтальную строку виджетов
    hub.WidgetSize(100); // Указываем, что последующие виджеты будут шириной 100%
    if (hub.Slider(&j1_speed, GH_UINT8, F("J1 speed"), STEPPER_MIN_SPEED, STEPPER_MAX_SPEED, STEPPERS_SPEED_STEP_SLIDERS) && DEBUG) { // К слайдеру подключена переменная j1_speed
      Serial.println("Slider J1 speed: " + String(j1_speed)); // Переменная уже обновилась и новое значение доступно во всей области определения
    }
    if (hub.Slider(&j1_accel, GH_UINT8, F("J1 acceleration"), STEPPER_MIN_ACCEL, STEPPER_MAX_ACCEL, STEPPERS_ACCEL_STEP_SLIDERS) && DEBUG) { // К слайдеру подключена переменная j1_accel
      Serial.println("Slider J1 accel: " + String(j1_accel)); // Переменная уже обновилась и новое значение доступно во всей области определения
    }
    if (hub.Slider(&j2_speed, GH_UINT8, F("J2 speed"), STEPPER_MIN_SPEED, STEPPER_MAX_SPEED, STEPPERS_SPEED_STEP_SLIDERS) && DEBUG) {
      Serial.println("Slider J2 speed: " + String(j2_speed));
    }
    if (hub.Slider(&j2_accel, GH_UINT8, F("J2 acceleration"), STEPPER_MIN_ACCEL, STEPPER_MAX_ACCEL, STEPPERS_ACCEL_STEP_SLIDERS) && DEBUG) {
      Serial.println("Slider J1 accel: " + String(j2_accel));
    }
    if (hub.Slider(&j3_speed, GH_UINT8, F("J3 speed"), STEPPER_MIN_SPEED, STEPPER_MAX_SPEED, STEPPERS_SPEED_STEP_SLIDERS) && DEBUG) {
      Serial.println("Slider J3 speed: " + String(j3_speed));
    }
    if (hub.Slider(&j3_accel, GH_UINT8, F("J3 acceleration"), STEPPER_MIN_ACCEL, STEPPER_MAX_ACCEL, STEPPERS_ACCEL_STEP_SLIDERS) && DEBUG) {
      Serial.println("Slider J1 accel: " + String(j3_accel));
    }
    hub.EndWidgets(); // Закончим отрисовку виджетов
  }
  hub.Space(25); // Отступ в пикселах
  hub.BeginWidgets(); // Начинает новую горизонтальную строку виджетов
  hub.WidgetSize(100); // Указываем, что последующие виджеты будут шириной
  if (hub.Button(&send_btn, F("Send"), GH_YELLOW)) { // К кнопке подключена переменная send_btn
    if (DEBUG) Serial.println("Button send pressed");
    if (tab == 0) { // Проверяем в какой вкладке находимся, такие значения и будем отправлять
      Serial.println("j1_pos:" + String(j1_pos) + "," + "j2_pos:" + String(j2_pos) + "," + "j3_pos:" + String(j3_pos) + "," + "claw_pos:" + String(claw_pos) + ";");
    } else if (tab == 1) {
      Serial.println("x_val:" + String(x_val) + "," + "y_val:" + String(y_val) + "," + "z_val:" + String(z_val) + "," + "claw_pos:" + String(claw_pos) + ";");
    } else if (tab == 2) {
      Serial.println("j1_speed:" + String(j1_speed) + "," + "j1_accel:" + String(j1_accel) + "," + "j2_speed:" + String(j2_speed) + "," + "j2_accel:" + String(j2_accel) + "j3_speed:" + String(j3_speed) + "," + "j3_accel:" + String(j3_accel) + ";");
    }
  }
  hub.EndWidgets(); // Закончим отрисовку виджетов
}