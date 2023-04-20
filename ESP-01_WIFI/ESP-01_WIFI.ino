// https://github.com/GyverLibs/GyverPortal
// https://github.com/GyverLibs/GyverPortal/blob/main/examples/demos/actionClick/actionClick.ino
// https://github.com/GyverLibs/GyverPortal/tree/main/examples/demos/demoAllComponents

#include <GyverPortal.h>

#define AP_SSID "dimdimdim" // Имя точки доступа
#define AP_PASS "08001800" // Пароль от точки доступа

GyverPortal ui; // Создаём объект интерфейса

int j1_pos_val = 0, j2_pos_val = 0, j3_pos_val = 0, claw_pos_val = 90;
int j1_speed_val = 0, j2_speed_val = 0, j3_speed_val = 0;
int j1_accel_val = 0, j2_accel_val = 0, j3_accel_val = 0;
int x_val = 0, y_val = 0, z_val = 0;
bool j1_val_upd, j2_val_upd, j3_val_upd, claw_pos_val_upd;
bool j1_speed_val_upd, j2_speed_val_upd, j3_speed_val_upd;
bool j1_accel_val_upd, j2_accel_val_upd, j3_accel_val_upd;
bool x_val_upd, y_val_upd, z_val_upd;

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
    M_BOX(GP.LABEL("Slider J1 degress"); GP.SLIDER("j1_pos_val", j1_pos_val, 0, 360); );
    M_BOX(GP.LABEL("Slider J2 degress"); GP.SLIDER("j2_pos_val", j2_pos_val, 0, 360); );
    M_BOX(GP.LABEL("Slider J3 degress"); GP.SLIDER("j3_pos_val", j3_pos_val, 0, 360); );
    M_BOX(GP.LABEL("Slider CLAW POSITION"); GP.SLIDER("claw_pos_val", claw_pos_val, 0, 180); );
  );

  M_BLOCK_TAB(
    "IK control",
    GP.LABEL("Inputs");
    M_BOX(GP.LABEL("x"); GP.NUMBER("ik_x", "number", x_val); );
    M_BOX(GP.LABEL("y"); GP.NUMBER("ik_y", "number", y_val); );
    M_BOX(GP.LABEL("z"); GP.NUMBER("ik_z", "number", z_val); );
  );

  M_BLOCK_TAB(
    "Steppers settings",
    GP.LABEL("Inputs");
    M_BOX(GP.LABEL("j1 speed"); GP.NUMBER("j1_speed_val", "number", j1_speed_val); );
    M_BOX(GP.LABEL("j1 accel"); GP.NUMBER("j1_accel_val", "number", j1_accel_val); );
    M_BOX(GP.LABEL("j2 speed"); GP.NUMBER("j2_speed_val", "number", j2_speed_val); );
    M_BOX(GP.LABEL("j2 accel"); GP.NUMBER("j2_accel_val", "number", j2_accel_val); );
    M_BOX(GP.LABEL("j3 speed"); GP.NUMBER("j3_speed_val", "number", j3_speed_val); );
    M_BOX(GP.LABEL("j3 accel"); GP.NUMBER("j3_accel_val", "number", j3_accel_val); );
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
    if (ui.clickInt("j1_pos_val", j1_pos_val)) j1_val_upd = true;
    if (ui.clickInt("j2_pos_val", j2_pos_val)) j2_val_upd = true;
    if (ui.clickInt("j3_pos_val", j3_pos_val)) j3_val_upd = true;
    if (ui.clickInt("claw_pos_val", claw_pos_val)) claw_pos_val_upd = true;
    if (ui.clickInt("ik_x", x_val)) x_val_upd = true;
    if (ui.clickInt("ik_y", y_val)) y_val_upd = true;
    if (ui.clickInt("ik_z", z_val)) z_val_upd = true;
    if (ui.clickInt("j1_speed_val", j1_speed_val)) j1_speed_val_upd = true;
    if (ui.clickInt("j1_accel_val", j1_accel_val)) j1_accel_val_upd = true;
    if (ui.clickInt("j2_speed_val", j2_speed_val)) j2_speed_val_upd = true;
    if (ui.clickInt("j2_accel_val", j2_accel_val)) j2_accel_val_upd = true;
    if (ui.clickInt("j3_speed_val", j3_speed_val)) j3_speed_val_upd = true;
    if (ui.clickInt("j3_accel_val", j3_accel_val)) j3_accel_val_upd = true;

    if (ui.click("send_btn")) { // Отправить по нажатию кнопки
      String out_str = ""; // Создаём стоку для передачи по Serial
      if (j1_val_upd) {
        out_str += "j1_deg_pos=" + String(j1_pos_val) + " ";
        j1_val_upd = false;
      }
      if (j2_val_upd) {
        out_str += "j2_deg_pos=" + String(j2_pos_val) + " ";
        j2_val_upd = false;
      }
      if (j3_val_upd) {
        out_str += "j3_deg_pos=" + String(j3_pos_val) + " ";
        j3_val_upd = false;
      }
      if (j1_speed_val_upd) {
        out_str += "j1_speed=" + String(j1_speed_val) + " ";
        j1_speed_val_upd = false;
      }
      if (j2_speed_val_upd) {
        out_str += "j2_speed=" + String(j2_speed_val) + " ";
        j2_speed_val_upd = false;
      }
      if (j3_speed_val_upd) {
        out_str += "j3_speed=" + String(j3_speed_val) + " ";
        j3_speed_val_upd = false;
      }
      if (j1_accel_val_upd) {
        out_str += "j1_accel=" + String(j1_accel_val) + " ";
        j1_accel_val_upd = false;
      }
      if (j2_accel_val_upd) {
        out_str += "j2_accel=" + String(j2_accel_val) + " ";
        j2_accel_val_upd = false;
      }
      if (j3_accel_val_upd) {
        out_str += "j3_accel=" + String(j3_accel_val) + " ";
        j3_accel_val_upd = false;
      }
      if (claw_pos_val_upd) {
        out_str += "claw_pos=" + String(claw_pos_val) + " ";
        claw_pos_val_upd = false;
      }
      if (x_val_upd) {
        out_str += "x=" + String(x_val) + " ";
        x_val_upd = false;
      }
      if (y_val_upd) {
        out_str += "y=" + String(y_val) + " ";
        y_val_upd = false;
      }
      if (z_val_upd) {
        out_str += "z=" + String(z_val);
        z_val_upd = false;
      }
      Serial.println(out_str); // Отправляем собранную строку по Serial
    }
  }
}

void loop() {
  ui.tick();
}
