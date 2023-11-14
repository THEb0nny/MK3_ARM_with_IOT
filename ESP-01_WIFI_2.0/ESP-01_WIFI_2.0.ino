// https://github.com/GyverLibs/GyverHub

#include <Arduino.h>
#include <GyverHub.h>

#define AP_SSID "Connectify_Mi_Router3" // Имя точки доступа
#define AP_PASS "2016APP1336" // Пароль от точки доступа

#define MQTT_PORT 13994 // Порт MQTT
#define MQTT_USR_NAME "u_MK3" // Имя пользователя MQTT брокера
#define MQTT_USR_PASS "NJZ3Xw77" // Пароль пользователя MQTT брокера

GyverHub hub("Manipulator", "MK3", ""); // Инициализация объекта hub с префиксом, имя, иконка

int j1_pos_val = 0, j2_pos_val = 0, j3_pos_val = 0, claw_pos_val = 90;
int j1_speed_val = 0, j2_speed_val = 0, j3_speed_val = 0;
int j1_accel_val = 0, j2_accel_val = 0, j3_accel_val = 0;
int x_val = 0, y_val = 0, z_val = 0;

// Переменные для интерфейса
GHbutton send_btn; // Виджет кнопки

uint8_t tab;

void setup() {
  Serial.begin(115200); // Инициализация Serial для общения по UART
  WiFi.mode(WIFI_STA);
  WiFi.begin(AP_SSID, AP_PASS);
  Serial.println();
  while (WiFi.status() != WL_CONNECTED) { // Ждём пока подключимся к WiFi
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println(WiFi.localIP()); // Печать локального IP адреса
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
    if (hub.Slider(&j1_pos_val, GH_UINT8, F("J1 degress"), 0, 360, 1)) { // К слайдеру подключена переменная j1_pos_slider
      Serial.println("Slider J1 deg: " + String(j1_pos_val)); // Переменная уже обновилась и новое значение доступно во всей области определения
    }
    if (hub.Slider(&j2_pos_val, GH_UINT8, F("J2 degress"), 0, 360, 1)) { // К слайдеру подключена переменная j2_pos_slider
      Serial.println("Slider J2 deg: " + String(j2_pos_val)); // Переменная уже обновилась и новое значение доступно во всей области определения
    }
    if (hub.Slider(&j3_pos_val, GH_UINT8, F("J3 degress"), 0, 360, 1)) { // К слайдеру подключена переменная j3_pos_slider
      Serial.println("Slider J3 deg: " + String(j3_pos_val)); // Переменная уже обновилась и новое значение доступно во всей области определения
    }
    if (hub.Slider(&claw_pos_val, GH_UINT8, F("Servo Claw position"), 0, 180, 1)) { // К слайдеру подключена переменная servo_claw_pos_slider
      Serial.println("Slider Сlaw pos: " + String(claw_pos_val)); // Переменная уже обновилась и новое значение доступно во всей области определения
    }
    hub.EndWidgets(); // Закончим отрисовку виджетов
  }
  if (tab == 1) {
    hub.BeginWidgets(); // Начинает новую горизонтальную строку виджетов
    hub.WidgetSize(100); // Указываем, что последующие виджеты будут шириной 100%
    if (hub.Slider(&x_val, GH_UINT8, F("X"), 0, 360, 1)) { // К слайдеру подключена переменная j1_pos_slider
      Serial.println("Slider X: " + String(x_val)); // Переменная уже обновилась и новое значение доступно во всей области определения
    }
    if (hub.Slider(&y_val, GH_UINT8, F("Y"), 0, 360, 1)) { // К слайдеру подключена переменная j2_pos_slider
      Serial.println("Slider Y: " + String(y_val)); // Переменная уже обновилась и новое значение доступно во всей области определения
    }
    if (hub.Slider(&z_val, GH_UINT8, F("Z"), 0, 360, 1)) { // К слайдеру подключена переменная j3_pos_slider
      Serial.println("Slider Z: " + String(z_val)); // Переменная уже обновилась и новое значение доступно во всей области определения
    }
    hub.EndWidgets(); // Закончим отрисовку виджетов
  }
  if (tab == 2) {
    hub.BeginWidgets(); // Начинает новую горизонтальную строку виджетов
    hub.WidgetSize(100); // Указываем, что последующие виджеты будут шириной 100%
    if (hub.Slider(&j1_speed_val, GH_UINT8, F("J1 speed"), 0, 360, 1)) { // К слайдеру подключена переменная j1_pos_slider
      Serial.println("Slider J1 speed: " + String(j1_speed_val)); // Переменная уже обновилась и новое значение доступно во всей области определения
    }
    if (hub.Slider(&j1_accel_val, GH_UINT8, F("J1 acceleration"), 0, 360, 1)) { // К слайдеру подключена переменная j1_pos_slider
      Serial.println("Slider J1 accel: " + String(j1_accel_val)); // Переменная уже обновилась и новое значение доступно во всей области определения
    }
    if (hub.Slider(&j2_speed_val, GH_UINT8, F("J2 speed"), 0, 360, 1)) { // К слайдеру подключена переменная j2_pos_slider
      Serial.println("Slider J2 speed: " + String(j2_speed_val)); // Переменная уже обновилась и новое значение доступно во всей области определения
    }
    if (hub.Slider(&j2_accel_val, GH_UINT8, F("J2 acceleration"), 0, 360, 1)) { // К слайдеру подключена переменная j1_pos_slider
      Serial.println("Slider J1 accel: " + String(j2_accel_val)); // Переменная уже обновилась и новое значение доступно во всей области определения
    }
    if (hub.Slider(&j3_speed_val, GH_UINT8, F("J3 speed"), 0, 360, 1)) { // К слайдеру подключена переменная j3_pos_slider
      Serial.println("Slider J3 speed: " + String(j3_speed_val)); // Переменная уже обновилась и новое значение доступно во всей области определения
    }
    if (hub.Slider(&j3_accel_val, GH_UINT8, F("J3 acceleration"), 0, 360, 1)) { // К слайдеру подключена переменная j1_pos_slider
      Serial.println("Slider J1 accel: " + String(j3_accel_val)); // Переменная уже обновилась и новое значение доступно во всей области определения
    }
    hub.EndWidgets(); // Закончим отрисовку виджетов
  }
  hub.Space(25); // Отступ в пикселах
  hub.BeginWidgets(); // Начинает новую горизонтальную строку виджетов
  hub.WidgetSize(100); // Указываем, что последующие виджеты будут шириной
  if (hub.Button(&send_btn, F("Send"), GH_YELLOW)) { // К кнопке подключена переменная send_btn
    Serial.println(send_btn ? "Button send pressed" : "Button send release");
  }
  hub.EndWidgets(); // Закончим отрисовку виджетов
}