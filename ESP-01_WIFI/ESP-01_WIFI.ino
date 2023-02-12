// https://github.com/GyverLibs/GyverPortal
// https://github.com/GyverLibs/GyverPortal/blob/main/examples/demos/actionClick/actionClick.ino
// https://github.com/GyverLibs/GyverPortal/tree/main/examples/demos/demoAllComponents

#define AP_SSID "freetime"
#define AP_PASS ""

#include <GyverPortal.h>

GyverPortal ui;

int m1_val = 0, m2_val = 0, m3_val = 0;

void build() {
  GP.BUILD_BEGIN();
  //GP.THEME(GP_DARK);
  GP.THEME(GP_LIGHT);

  GP.TITLE("MK3 Conrol Dashboard");
  GP.HR();

  M_BLOCK_TAB(
    "Control",
    GP.LABEL("Inputs");
    M_BOX(GP.LABEL("Pos stepper 1"); GP.NUMBER("m1_input_num", "number", m1_val); );
    M_BOX(GP.LABEL("Pos stepper 2"); GP.NUMBER("m2_input_num", "number", m2_val); );
    M_BOX(GP.LABEL("Pos stepper 3"); GP.NUMBER("m3_input_num", "number", m3_val); );
    M_BOX(GP.LABEL("Slider Stepper 1"); GP.SLIDER("m1_input_slider_num", m1_val, 0, 1000); );
    M_BOX(GP.LABEL("Slider Stepper 2"); GP.SLIDER("m2_input_slider_num", m2_val, 0, 1000); );
    M_BOX(GP.LABEL("Slider Stepper 3"); GP.SLIDER("m3_input_slider_num", m3_val, 0, 1000); );
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
    if (ui.clickInt("m1_input_num", m1_val)) {
      Serial.print("Number: ");
      Serial.println(m1_val);
    }

    if (ui.clickInt("m1_input_slider_num", m1_val)) {
      Serial.print("Slider: ");
      Serial.println(m1_val);
    }

    if (ui.click("send_btn")) {
      Serial.println("Button click");
    }
  }
}

void loop() {
  ui.tick();
}
