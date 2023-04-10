// https://github.com/GyverLibs/GyverPortal
// https://github.com/GyverLibs/GyverPortal/blob/main/examples/demos/actionClick/actionClick.ino
// https://github.com/GyverLibs/GyverPortal/tree/main/examples/demos/demoAllComponents

#include <GyverPortal.h>

#define AP_SSID "dimdimdim" // Имя точки доступа
#define AP_PASS "08001800" // Пароль от точки доступа

GyverPortal ui; // Создаём объект интерфейса

int j1_val = 0, j2_val = 0, j3_val = 0, jServo_val = 90;
int x_val = 0, y_val = 0, z_val = 0;
bool j1_val_isUpd, j2_val_isUpd, j3_val_isUpd, jServo_val_isUpd;
bool x_val_isUpd, y_val_isUpd, z_val_isUpd;

void build() {
  GP.BUILD_BEGIN();
  //GP.THEME(GP_DARK);
  GP.THEME(GP_LIGHT);

  GP.ONLINE_CHECK(); // проверять статус платы, зажми reset и смотри на название вкладки браузера
  GP.setTimeout(3000); // Установить время проверки подключения
  GP.TITLE("MK3 Control Dashboard");
  GP.HR();

  M_BLOCK_TAB(
    "Joints control",
    M_BOX(GP.LABEL("Slider J1"); GP.SLIDER("j1", j1_val, 0, 500); );
    M_BOX(GP.LABEL("Slider J2"); GP.SLIDER("j2", j2_val, 0, 500); );
    M_BOX(GP.LABEL("Slider J3"); GP.SLIDER("j3", j3_val, 0, 500); );
    M_BOX(GP.LABEL("Slider J_SERVO"); GP.SLIDER("servo", jServo_val, 0, 180); );
  );

  M_BLOCK_TAB(
    "IK control",
    GP.LABEL("Inputs");
    M_BOX(GP.LABEL("x"); GP.NUMBER("ik_x", "number", x_val); );
    M_BOX(GP.LABEL("y"); GP.NUMBER("ik_y", "number", y_val); );
    M_BOX(GP.LABEL("z"); GP.NUMBER("ik_z", "number", z_val); );
  );

  GP.BUTTON("send_btn", "Send");
  GP.BUILD_END();
}

void setup() {
  Serial.begin(115200);
  WiFi.mode(WIFI_STA);
  WiFi.begin(AP_SSID, AP_PASS);
  while (WiFi.status() != WL_CONNECTED) {
    delay(500);
    Serial.print(".");
  }
  Serial.println();
  Serial.println(WiFi.localIP());

  // Подключаем конструктор и запускаем
  ui.attachBuild(build);
  ui.attach(action);
  ui.start();
}

void action() {
  if (ui.click()) { // Если был клиек по любому компоненту
    if (ui.clickInt("j1", j1_val)) j1_val_isUpd = true;
    if (ui.clickInt("j2", j2_val)) j2_val_isUpd = true;
    if (ui.clickInt("j3", j3_val)) j3_val_isUpd = true;
    if (ui.clickInt("jServo", jServo_val)) jServo_val_isUpd = true;
    if (ui.clickInt("ik_x", x_val)) x_val_isUpd = true;
    if (ui.clickInt("ik_y", y_val)) y_val_isUpd = true;
    if (ui.clickInt("ik_z", z_val)) z_val_isUpd = true;

    if (ui.click("send_btn")) { // Отправить по нажатию кнопки
      if (j1_val_isUpd) {
        Serial.println("j1=" + String(j1_val));
        j1_val_isUpd = false;
      }
      if (j2_val_isUpd) {
        Serial.println("j2=" + String(j2_val));
        j2_val_isUpd = false;
      }
      if (j3_val_isUpd) {
        Serial.println("j3=" + String(j3_val));
        j3_val_isUpd = false;
      }
      if (jServo_val_isUpd) {
        Serial.println("jServo=" + String(jServo_val));
        jServo_val_isUpd = false;
      }
      if (x_val_isUpd) {
        Serial.println("x=" + String(x_val));
        x_val_isUpd = false;
      }
      if (y_val_isUpd) {
        Serial.println("y=" + String(y_val));
        y_val_isUpd = false;
      }
      if (z_val_isUpd) {
        Serial.println("z=" + String(z_val));
        z_val_isUpd = false;
      }
    }
  }
}

void loop() {
  ui.tick();
}
