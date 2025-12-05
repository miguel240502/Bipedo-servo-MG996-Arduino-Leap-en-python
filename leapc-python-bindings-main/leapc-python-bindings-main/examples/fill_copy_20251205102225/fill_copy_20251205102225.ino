#include <Servo.h>

// ====== Objetos Servo ======
Servo tobilloDer;
Servo rodillaDer;
Servo caderaDer;

Servo tobilloIzq;
Servo rodillaIzq;
Servo caderaIzq;

// ====== Pines en fila PWM (shield Mega v2.4) ======
const int PIN_TOBILLO_DER = 2;  // PWM D2
const int PIN_RODILLA_DER = 3;  // PWM D3
const int PIN_CADERA_DER  = 4;  // PWM D4

const int PIN_TOBILLO_IZQ = 5;  // PWM D5
const int PIN_RODILLA_IZQ = 6;  // PWM D6
const int PIN_CADERA_IZQ  = 7;  // PWM D7

// ====== Sensor ultrasonico (HC-SR04) ======
const int TRIG_PIN = 8;   // D8
const int ECHO_PIN = 9;   // D9
const int DIST_UMBRAL_CM = 20;  // distancia de seguridad

// ====== Postura base (PARADO) con tus valores ======
const int OFFSET_TD = 98;   // tobillo der
const int OFFSET_RD = 170;  // rodilla der
const int OFFSET_CD = 110;  // cadera der

const int OFFSET_TI = 64;   // tobillo izq
const int OFFSET_RI = 100;  // rodilla izq
const int OFFSET_CI = 186;  // cadera izq (el servo la limitará a 180)

// ====== Parámetros del paso (todo suave y corto) ======
const int LIFT_RODILLA = 15;   // flexión de rodilla
const int LIFT_TOBILLO = 10;  // punta/talón
const int SWING_CADERA = 15;  // avance de la pierna

const int STEP_FRAMES = 20;   // frames por transición
const int FRAME_DELAY = 15;   // ms entre frames

// ---------- Estructura de pose ----------
struct Pose {
  int cd, rd, td;  // pierna derecha
  int ci, ri, ti;  // pierna izquierda
};

// Pose de pie
Pose PARADO = {
  OFFSET_CD, OFFSET_RD, OFFSET_TD,
  OFFSET_CI, OFFSET_RI, OFFSET_TI
};

// ---------- Funciones de movimiento ----------
void aplicarPose(const Pose &p) {
  caderaDer.write(p.cd);
  rodillaDer.write(p.rd);
  tobilloDer.write(p.td);

  caderaIzq.write(p.ci);
  rodillaIzq.write(p.ri);
  tobilloIzq.write(p.ti);
}

void irAPose(const Pose &objetivo, int frames = STEP_FRAMES, int frameDelay = FRAME_DELAY) {
  Pose actual = {
    caderaDer.read(), rodillaDer.read(), tobilloDer.read(),
    caderaIzq.read(), rodillaIzq.read(), tobilloIzq.read()
  };

  for (int f = 1; f <= frames; f++) {
    Pose inter;
    inter.cd = actual.cd + (objetivo.cd - actual.cd) * f / frames;
    inter.rd = actual.rd + (objetivo.rd - actual.rd) * f / frames;
    inter.td = actual.td + (objetivo.td - actual.td) * f / frames;

    inter.ci = actual.ci + (objetivo.ci - actual.ci) * f / frames;
    inter.ri = actual.ri + (objetivo.ri - actual.ri) * f / frames;
    inter.ti = actual.ti + (objetivo.ti - actual.ti) * f / frames;

    aplicarPose(inter);
    delay(frameDelay);
  }
}

// ---------- Poses del paso ----------
// Paso con pierna derecha
Pose SHIFT_IZQ = {
  OFFSET_CD,                     // cadera der
  OFFSET_RD,
  OFFSET_TD,
  OFFSET_CI - SWING_CADERA/2,    // cadera izq se inclina para tomar peso
  OFFSET_RI,
  OFFSET_TI
};

Pose LIFT_DER = {
  OFFSET_CD,
  OFFSET_RD + LIFT_RODILLA,      // rodilla der flexiona
  OFFSET_TD - LIFT_TOBILLO,      // pie der a "punta"
  OFFSET_CI - SWING_CADERA/2,
  OFFSET_RI,
  OFFSET_TI
};

Pose SWING_DER = {
  OFFSET_CD + SWING_CADERA,      // cadera der hacia adelante
  OFFSET_RD + LIFT_RODILLA,
  OFFSET_TD - LIFT_TOBILLO,
  OFFSET_CI - SWING_CADERA/2,
  OFFSET_RI,
  OFFSET_TI
};

Pose SUPPORT_DER = {
  OFFSET_CD + SWING_CADERA,
  OFFSET_RD,                     // rodilla der vuelve a recto
  OFFSET_TD,
  OFFSET_CI - SWING_CADERA/2,
  OFFSET_RI,
  OFFSET_TI
};

// Paso con pierna izquierda (simétrico)
Pose SHIFT_DER = {
  OFFSET_CD - SWING_CADERA/2,    // peso a derecha
  OFFSET_RD,
  OFFSET_TD,
  OFFSET_CI,
  OFFSET_RI,
  OFFSET_TI
};

Pose LIFT_IZQ = {
  OFFSET_CD - SWING_CADERA/2,
  OFFSET_RD,
  OFFSET_TD,
  OFFSET_CI - SWING_CADERA/2,
  OFFSET_RI - LIFT_RODILLA,      // rodilla izq flexiona
  OFFSET_TI + LIFT_TOBILLO       // pie izq a "punta"
};

Pose SWING_IZQ = {
  OFFSET_CD - SWING_CADERA/2,
  OFFSET_RD,
  OFFSET_TD,
  OFFSET_CI - SWING_CADERA,      // cadera izq hacia adelante
  OFFSET_RI - LIFT_RODILLA,
  OFFSET_TI + LIFT_TOBILLO
};

Pose SUPPORT_IZQ = {
  OFFSET_CD - SWING_CADERA/2,
  OFFSET_RD,
  OFFSET_TD,
  OFFSET_CI - SWING_CADERA,
  OFFSET_RI,
  OFFSET_TI
};

// ---------- Secuencia de caminar ----------
void darPaso() {
  // Paso derecho
  irAPose(SHIFT_IZQ);     // pasar peso a izquierda
  irAPose(LIFT_DER);      // levantar pierna derecha
  irAPose(SWING_DER);     // llevar pierna derecha adelante
  irAPose(SUPPORT_DER);   // apoyar derecha
  irAPose(PARADO);        // volver al centro

  // Paso izquierdo
  irAPose(SHIFT_DER);     // pasar peso a derecha
  irAPose(LIFT_IZQ);      // levantar pierna izquierda
  irAPose(SWING_IZQ);     // llevar pierna izquierda adelante
  irAPose(SUPPORT_IZQ);   // apoyar izquierda
  irAPose(PARADO);        // volver al centro
}

// ---------- Ultrasónico ----------

long medirDistanciaCm() {
  digitalWrite(TRIG_PIN, LOW);
  delayMicroseconds(2);
  digitalWrite(TRIG_PIN, HIGH);
  delayMicroseconds(10);
  digitalWrite(TRIG_PIN, LOW);

  long duracion = pulseIn(ECHO_PIN, HIGH, 30000UL); // timeout 30 ms

  if (duracion == 0) {
    return -1;  // sin eco
  }

  long distancia = duracion * 0.034 / 2; // cm
  return distancia;
}

// Maniobra sencilla de esquive (ajústala viendo el robot)
void esquivarObstaculo() {
  Serial.println("ESQUIVANDO OBSTACULO...");

  // Por ejemplo, usar secuencia tipo paso izquierdo
  irAPose(SHIFT_DER);
  irAPose(LIFT_IZQ);
  irAPose(SWING_IZQ);
  irAPose(SUPPORT_IZQ);
  irAPose(PARADO);
}

// ================== SETUP Y LOOP ==================
void setup() {
  // Mismo baud que usas en Python (115200)
  Serial.begin(115200);

  tobilloDer.attach(PIN_TOBILLO_DER);
  rodillaDer.attach(PIN_RODILLA_DER);
  caderaDer.attach(PIN_CADERA_DER);

  tobilloIzq.attach(PIN_TOBILLO_IZQ); 
  rodillaIzq.attach(PIN_RODILLA_IZQ);
  caderaIzq.attach(PIN_CADERA_IZQ);

  // Pines del ultrasónico
  pinMode(TRIG_PIN, OUTPUT);
  pinMode(ECHO_PIN, INPUT);

  delay(500);
  aplicarPose(PARADO);
  delay(500);
  Serial.println("Bipedo listo. Esperando comandos 1–5 por Serial y monitoreando obstaculos...");
}

void loop() {

  // 1) Revisar obstaculos primero
  long d = medirDistanciaCm();

  if (d > 0 && d < DIST_UMBRAL_CM) {
    Serial.print("OBSTACULO DETECTADO a ");
    Serial.print(d);
    Serial.println(" cm.");

    // Pararse completamente
    irAPose(PARADO);
    Serial.println("DETENIDO 5 segundos.");
    delay(5000);

    // Maniobra de esquive
    esquivarObstaculo();

    // Después del esquive salimos del loop; en el siguiente ciclo
    // volverá a revisar distancia y luego comandos
    return;
  }

  // 2) Si no hay obstáculo cercano, atender comandos del Leap (1–5)
  if (Serial.available() > 0) {
    char c = Serial.read();

    if (c == '\n' || c == '\r') {
      return;
    }

    if (c >= '1' && c <= '5') {
      int n = c - '0';
      Serial.print("Dedo(s) detectados: ");
      Serial.println(n);
    }

    switch (c) {
      case '1':
        Serial.println("Comando 1: PARADO");
        irAPose(PARADO);
        break;

      case '2':
        Serial.println("Comando 2: UN PASO (der+izq)");
        darPaso();
        break;

      case '3':
        Serial.println("Comando 3: DOS PASOS");
        darPaso();
        darPaso();
        break;

      case '4':
        Serial.println("Comando 4: INCLINAR A IZQUIERDA (SHIFT_IZQ)");
        irAPose(SHIFT_IZQ);
        break;

      case '5':
        Serial.println("Comando 5: INCLINAR A DERECHA (SHIFT_DER)");
        irAPose(SHIFT_DER);
        break;

      default:
        Serial.print("Comando desconocido: ");
        Serial.println(c);
        break;
    }
  }

  // Si no hay obstaculos ni comandos, se queda en la última pose
}
