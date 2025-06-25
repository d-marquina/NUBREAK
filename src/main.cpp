#include <Arduino.h>
#include <DabbleESP32.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>

// --- OLED ---
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
Adafruit_SSD1306 pantalla(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, -1);

// --- Pines servos ---
#define PIN_SERVO_MALETERA 9
#define PIN_SERVO_TIMON 15
#define PIN_SERVO_EXTRA 18

// --- Pines botones ---
#define BTN_MALETERA 5
#define BTN_TIMON 20
#define BTN_EXTRA 21

// --- Pines micromotores con L298N ---
#define M1_IN1 16
#define M1_IN2 17
#define M2_IN1 19
#define M2_IN2 22

// --- Potenciómetro ---
#define PIN_POT A0

// --- PWM servo (usando ledc para ESP32) ---
#define FREQ_SERVO 50
#define CHANNEL_MALETERA 0
#define CHANNEL_TIMON 1
#define CHANNEL_EXTRA 2
#define RESOLUTION 16

// --- Umbrales del potenciómetro ---
int valor_min = 0;
int valor_max = 4095;
int UMBRAL_BAJO;
int UMBRAL_ALTO;

// --- Variables ---
bool maletera_abierta = false;
bool timon_derecha = false;
bool servo_extra_abierto = false;
bool esperando_boton_maletera = true;

const int POS_CERRADO = 1800;
const int POS_ABIERTO = 5764;
const int DUTY_0 = 1800;
const int DUTY_180 = 7064;
const int EXTRA_0 = 2800;
const int EXTRA_180 = 5064;
const int PASOS = 65;

// --- Funciones auxiliares ---
void moverServo(uint8_t canal, uint16_t start, uint16_t end) {
  int paso = (end - start) / PASOS;
  int duty = start;
  for (int i = 0; i < PASOS; i++) {
    ledcWrite(canal, duty);
    duty += paso;
    delay(10);
  }
}

void mover_maletera_con_animacion(bool abrir) {
  int start = abrir ? POS_CERRADO : POS_ABIERTO;
  int end = abrir ? POS_ABIERTO : POS_CERRADO;
  moverServo(CHANNEL_MALETERA, start, end);
}

void mover_timon() {
  int start = timon_derecha ? DUTY_180 : DUTY_0;
  int end = timon_derecha ? DUTY_0 : DUTY_180;
  moverServo(CHANNEL_TIMON, start, end);
  timon_derecha = !timon_derecha;
}

void mover_servo_extra() {
  int start = servo_extra_abierto ? EXTRA_180 : EXTRA_0;
  int end = servo_extra_abierto ? EXTRA_0 : EXTRA_180;
  moverServo(CHANNEL_EXTRA, start, end);
  servo_extra_abierto = !servo_extra_abierto;
}

void setup() {
  Serial.begin(115200);
  Dabble.begin("MyCar");

  // OLED
  if (!pantalla.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println("Fallo OLED");
  }
  pantalla.clearDisplay();
  pantalla.display();

  // Botones
  pinMode(BTN_MALETERA, INPUT_PULLUP);
  pinMode(BTN_TIMON, INPUT_PULLUP);
  pinMode(BTN_EXTRA, INPUT_PULLUP);

  // Motores
  pinMode(M1_IN1, OUTPUT);
  pinMode(M1_IN2, OUTPUT);
  pinMode(M2_IN1, OUTPUT);
  pinMode(M2_IN2, OUTPUT);

  // PWM para servos
  ledcSetup(CHANNEL_MALETERA, FREQ_SERVO, RESOLUTION);
  ledcAttachPin(PIN_SERVO_MALETERA, CHANNEL_MALETERA);
  ledcSetup(CHANNEL_TIMON, FREQ_SERVO, RESOLUTION);
  ledcAttachPin(PIN_SERVO_TIMON, CHANNEL_TIMON);
  ledcSetup(CHANNEL_EXTRA, FREQ_SERVO, RESOLUTION);
  ledcAttachPin(PIN_SERVO_EXTRA, CHANNEL_EXTRA);

  // Umbrales de potenciómetro
  UMBRAL_BAJO = valor_min + (valor_max - valor_min) / 3;
  UMBRAL_ALTO = valor_min + 2 * (valor_max - valor_min) / 3;
}

void loop() {
  Dabble.processInput();

  int valor = analogRead(PIN_POT);

  // Control de motores
  if (valor < UMBRAL_BAJO) {
    digitalWrite(M1_IN1, LOW); digitalWrite(M1_IN2, HIGH);
    digitalWrite(M2_IN1, LOW); digitalWrite(M2_IN2, HIGH);
  } else if (valor > UMBRAL_ALTO) {
    digitalWrite(M1_IN1, HIGH); digitalWrite(M1_IN2, LOW);
    digitalWrite(M2_IN1, HIGH); digitalWrite(M2_IN2, LOW);
  } else {
    digitalWrite(M1_IN1, LOW); digitalWrite(M1_IN2, LOW);
    digitalWrite(M2_IN1, LOW); digitalWrite(M2_IN2, LOW);
  }

  // Control por botones
  if (digitalRead(BTN_MALETERA) == LOW && esperando_boton_maletera) {
    maletera_abierta = !maletera_abierta;
    mover_maletera_con_animacion(maletera_abierta);
    esperando_boton_maletera = false;
    delay(200);
  }
  if (digitalRead(BTN_MALETERA) == HIGH) {
    esperando_boton_maletera = true;
  }

  if (digitalRead(BTN_TIMON) == LOW) {
    mover_timon();
    while (digitalRead(BTN_TIMON) == LOW) delay(50);
  }

  if (digitalRead(BTN_EXTRA) == LOW) {
    mover_servo_extra();
    while (digitalRead(BTN_EXTRA) == LOW) delay(50);
  }

  // Aquí puedes incluir control remoto con Dabble (Gamepad, etc.)
  if (GamePad.isUpPressed()) {
    digitalWrite(M1_IN1, HIGH); digitalWrite(M1_IN2, LOW);
    digitalWrite(M2_IN1, HIGH); digitalWrite(M2_IN2, LOW);
  }
  if (GamePad.isDownPressed()) {
    digitalWrite(M1_IN1, LOW); digitalWrite(M1_IN2, HIGH);
    digitalWrite(M2_IN1, LOW); digitalWrite(M2_IN2, HIGH);
  }
  if (GamePad.isLeftPressed() || GamePad.isRightPressed()) {
    // Dirección asistida con servo_timon
    mover_timon();
  }
}
