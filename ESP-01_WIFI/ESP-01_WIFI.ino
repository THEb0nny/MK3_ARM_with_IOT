// https://github.com/GyverLibs/GyverPortal
// https://github.com/GyverLibs/GyverPortal/blob/main/examples/demos/actionClick/actionClick.ino
// https://github.com/GyverLibs/GyverPortal/tree/main/examples/demos/demoAllComponents

#define AP_SSID "dimdimdim"
#define AP_PASS "08001800"

#include <GyverPortal.h>

GyverPortal ui;

int j1_val = 0, j2_val = 0, j3_val = 0;
int x_val = 0, y_val = 0, z_val = 0;

void build() {
  GP.BUILD_BEGIN();
  //GP.THEME(GP_DARK);
  GP.THEME(GP_LIGHT);

  GP.TITLE("MK3 Control Dashboard");
  GP.HR();

  M_BLOCK_TAB(
    "Joints control",
    M_BOX(GP.LABEL("Slider J1"); GP.SLIDER("j1", j1_val, 0, 1000); );
    M_BOX(GP.LABEL("Slider J2"); GP.SLIDER("j2", j2_val, 0, 1000); );
    M_BOX(GP.LABEL("Slider J3"); GP.SLIDER("j3", j3_val, 0, 1000); );
  );

  M_BLOCK_TAB(
    "IK control",
    GP.LABEL("Inputs");
    M_BOX(GP.LABEL("x"); GP.NUMBER("ik_x", "number", x_val); );
    M_BOX(GP.LABEL("y"); GP.NUMBER("ik_y", "number", y_val); );
    M_BOX(GP.LABEL("z"); GP.NUMBER("ik_z", "number", z_val); );
  );

  GP.BUTTON("send_btn", "Send");
  //GP.BUTTON_MINI("", "Btn Mini");

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
  if (ui.click()) {
    // Проверяем компоненты и обновляем переменные
    if (ui.clickInt("j1", j1_val)) {
      Serial.print("J1: ");
      Serial.println(j1_val);
    }
    if (ui.clickInt("j2", j2_val)) {
      Serial.print("J2: ");
      Serial.println(j2_val);
    }
    if (ui.clickInt("j3", j3_val)) {
      Serial.print("J3: ");
      Serial.println(j3_val);
    }
    if (ui.clickInt("ik_x", x_val)) {
      Serial.print("X: ");
      Serial.println(x_val);
    }
    if (ui.clickInt("ik_y", y_val)) {
      Serial.print("Y: ");
      Serial.println(y_val);
    }
    if (ui.clickInt("ik_z", z_val)) {
      Serial.print("Z: ");
      Serial.println(z_val);
    }
    if (ui.click("send_btn")) {
      Serial.println("Button click");
    }
  }
}

void loop() {
  ui.tick();
}
