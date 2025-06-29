/**
 * @file main.cpp
 * @brief Firmware para um dispositivo multifuncional baseado em ESP32.
 * @author Grupo Alfa (Igor Gustavo, Nycollas Luan, Mateus Paiva, Rolly Santos)
 * @date 2023-10-27
 *
 * Este projeto implementa um dispositivo portátil com as seguintes funções:
 * - Estroboscópio digital com ajuste de FPM e fase.
 * - Tacômetro digital (RPM) usando um sensor IR.
 * - Vibrômetro usando um acelerômetro ADXL345.
 * - Lanterna.
 * - Modo de teste de hardware.
 * A interface é controlada por um display OLED, um encoder rotativo e quatro botões.
 */
 
#include <Preferences.h>         // Biblioteca para armazenar dados na memória flash NVS (Non-Volatile Storage)
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <Adafruit_Sensor.h>
#include <Adafruit_ADXL345_U.h>
#define ENCODER_DO_NOT_USE_INTERRUPTS
#include <Encoder.h>
#include "bitmap_logo.h"
#include <arduinoFFT.h>

// ==== Configurações de Hardware ====
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET    -1
#define ENCODER_PIN_A 16
#define ENCODER_PIN_B 17
#define BUTTON_MENU   5
#define BUTTON_SET    15
#define BUTTON_DOUBLE 27
#define BUTTON_HALF   26
#define BUTTON_ENC    25
#define LED_PIN       2       // Pino do LED de alta potência para o estroboscópio/lanterna
#define SENSOR_IR_PIN 4       // Pino do sensor infravermelho para medição de RPM


Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Preferences prefs;  
Encoder encoder(ENCODER_PIN_A, ENCODER_PIN_B);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(123);
bool adxlAvailable = false;

//==== Inicialização do medidor de FFT ====
#define SAMPLE_RATE 500        // Hz
#define FFT_SIZE 512 //Resolução da frequência 256, 512 ou 1024
double accBuffer[FFT_SIZE];
double timeBuffer[FFT_SIZE];
ArduinoFFT<double> FFT = ArduinoFFT<double>(accBuffer, timeBuffer, FFT_SIZE, SAMPLE_RATE);

// --- VARIÁVEIS GLOBAIS DE ESTADO ---

// ==== Estados e enum de Modos ====
enum class Mode { HOME, FREQUENCY, RPM, LANTERN, VIBROMETER, TEST, ABOUT, NUM_MODES};
Mode currentMode = Mode::HOME;
Mode selectedMode = Mode::HOME;
bool inMenu = true;
bool inSubmenu = false;
bool inEncoder = false;

// ==== Variáveis do Modo Estroboscópio ====

float fpm = 300; // Valor inicial de FPM
long STB_lastEncoderPos = 0;

float STB_phaseDegrees = 0.0;               // Ajuste de fase em graus
unsigned long STB_partTime = 1000000;       // Duração de meio ciclo em microssegundos
unsigned long STB_phaseDelayMicros = 0;     // Atraso de fase em microssegundos
bool STB_calc = true;                       // Indica se os cálculos precisam ser refeitos
bool STB_firstPulse = true;                 // Para aplicar a fase apenas uma vez
bool STB_outputEnabled = false;             // Controla se a saída está ativa

hw_timer_t *timer = NULL;
portMUX_TYPE STB_timerMux = portMUX_INITIALIZER_UNLOCKED;
volatile bool STB_pulseState = false;

const int minFPM = 30;                   // Limite mínimo de FPM
const int maxFPM = 40000;                 // Limite máximo de FPM
//-------------------------------------

// ==== Variáveis do Vibrometro ====
enum class VibroState {VIBRO_HOME, VIBRO_IDLE, VIBRO_CALIB, VIBRO_CONFIG, VIBRO_MEASURE, VIBRO_RESULT};
VibroState vibroState = VibroState::VIBRO_HOME;

// ==== Medição ====
float ax, ay, az;
float aRMS, aPeak, stdDev, vRMS, freqDominant;
float velocitySum = 0;

unsigned long lastSampleTime = 0;
int sampleCount = 0;

bool isMeasuring = false;
bool isCalibrating = false;

unsigned long measureStartTime = 0;
unsigned long measureDuration = 0;
unsigned long calibrationStartTime = 0;
unsigned long calibrationDuration = 0;

int secondsLeft = 0;
int secondsLeftCalib = 0;

float offsetX = 0, offsetY = 0, offsetZ = 0;
//-------------------------------------
// ==== Variáveis de Gravação do tempo de vibração ====
int timeMeasure = 30;
int timeCalib = 15;

// ==== Variáveis do RPM ====
#define TEMPO_LEITURA_RPM 1000  // em milissegundos (1 segundo)
unsigned long ultimaLeitura = 0;
int contagemPulsos = 0;
bool estadoAnterior = LOW;
float rpmValue = 0;  // Valor de RPM calculado

bool Lant_calc = false;
bool TESTE_calc = false;
float TESTE_fpm = 0;
//-------------------------------------

// ====Mensagem sobre a equie ====
// Lista de mensagens
String lines[] = {
  "Agradecimento:",
  "Evandro Padilha",
  "Renato",
  "Vitor Santarosa",
  "Alex Penteado",
  "Bruno",
  "Epaminondas",
  "Fernando",
  "Fabio Camarinha",
  "Gabriela"
};
const int totalLines = sizeof(lines) / sizeof(lines[0]);
int topLineIndex = 0;  // Índice da linha superior visível
int lineShowMsg = 5;

// ==== Temporizador ====
struct TimerMicros {
  unsigned long start;
  unsigned long duration;
  void startTimer(unsigned long d) {
    duration = d*1000000;
    start = micros();
  }
  bool isExpired() {
    return (micros() - start) >= duration;
  }
  bool isRunning() {
    return (micros() - start) < duration;
  }
  // Tempo decorrido em microssegundos
  unsigned long elapsed() {
    return micros() - start;
  }
  // Tempo restante em microssegundos
  unsigned long remaining() {
    if (isExpired()) return 0;
    return duration - (micros() - start);
  }
  // Tempo restante em segundos (float)
  float remainingSeconds() {
    return remaining() / 1000000.0;
  }
};
TimerMicros msgTimer; //Pode criar quantos TimerMicros for necessário
TimerMicros fpmTest;

// ==== Função de interrupção do timer. Alterna o estado do LED e aplica o atraso de fase na primeira chamada. ====
void IRAM_ATTR onTimer() {
  static bool phaseApplied = false;
  portENTER_CRITICAL_ISR(&STB_timerMux);

  if (!STB_outputEnabled) {
    digitalWrite(LED_PIN, LOW);
    portEXIT_CRITICAL_ISR(&STB_timerMux);
    return;
  }

  if (STB_firstPulse && !phaseApplied && STB_phaseDelayMicros > 0) {
    timerAlarm(timer, STB_phaseDelayMicros, true, 0);  // Atraso de fase
    phaseApplied = true;
  } else {
    STB_pulseState = !STB_pulseState;
    digitalWrite(LED_PIN, STB_pulseState);
    timerAlarm(timer, STB_partTime, true, 0);          // Próximo intervalo
    STB_firstPulse = false;
    phaseApplied = false;
  }

  portEXIT_CRITICAL_ISR(&STB_timerMux);
}

// Atualiza os valores com base na entrada de FPM. Recalcula os tempos de ciclo e atraso de fase.
void updateValues() {
  if (STB_calc) {
    unsigned long cycleTimeMicros = 60000000UL / fpm;
    STB_partTime = cycleTimeMicros / 2;
    STB_phaseDelayMicros = (STB_phaseDegrees / 360.0) * cycleTimeMicros;
    STB_firstPulse = true;
    STB_calc = false;
  } else if (Lant_calc) {
    unsigned long cycleTimeMicros = 60000000UL / 7200;
    STB_partTime = cycleTimeMicros / 2;
    STB_phaseDelayMicros = (STB_phaseDegrees / 360.0) * cycleTimeMicros;
    STB_firstPulse = true;
  } else if (TESTE_calc) {
    unsigned long cycleTimeMicros = 60000000UL / TESTE_fpm;
    STB_partTime = cycleTimeMicros / 2;
    STB_phaseDelayMicros = (STB_phaseDegrees / 360.0) * cycleTimeMicros;
    STB_firstPulse = true;
  }
}

// ==== Ajusta o FPM multiplicando-o por um fator ====
void adjustFPM(float factor) {
  fpm = constrain(fpm * factor, minFPM, maxFPM);
}

bool checkButtonDebounce(uint8_t pin, bool &lastState, unsigned long &lastDebounceTime, unsigned long debounceDelayMicros = 200000) {
  bool currentState = digitalRead(pin);
  unsigned long now = micros();
  if (currentState == LOW && lastState == HIGH && (now - lastDebounceTime > debounceDelayMicros)) {
    lastDebounceTime = now;
    lastState = currentState;
    return true; // Botão pressionado com debounce válido
  }
  lastState = currentState;
  return false;
}

// --- FUNÇÕES DE LÓGICA ---
// ==== Grava os valores na memória ====
void updateValuesRec(){
  prefs.begin("config", false);  // Abre o namespace "config" no modo gravação
  if (selectedMode == Mode::VIBROMETER) {
    prefs.putInt("timeMeasure", timeMeasure);      // Salva o valor
    prefs.putInt("timeCalib", timeCalib);      // Salva o valor
  }else{
    // Salva o FPM atual na memória não-volátil
    prefs.putInt("fpm", fpm);      // Salva o valor
  }
  prefs.end();                   // Fecha o namespace
}

// ==== Funçoes do Vibrometro calibrar e medir ====

// Inicia calibração com tempo em segundos
void startCalibration(int durationSeconds) {
  isCalibrating = true;
  calibrationDuration = durationSeconds * 1000UL;
  calibrationStartTime = millis();
  offsetX = 0;
  offsetY = 0;
  offsetZ = 0;
  sampleCount = 0;
  secondsLeftCalib = durationSeconds;
  vibroState = VibroState::VIBRO_CALIB;
  Serial.println("Iniciando calibração...");
}

void updateCalibration() {
  if (!isCalibrating) return;

  unsigned long now = millis();
  unsigned long elapsed = now - calibrationStartTime;
  unsigned long remaining = (calibrationDuration > elapsed) ? (calibrationDuration - elapsed) : 0;
  secondsLeftCalib = remaining / 1000;

  sensors_event_t event;
  accel.getEvent(&event);
  offsetX += event.acceleration.x;
  offsetY += event.acceleration.y;
  offsetZ += event.acceleration.z;
  sampleCount++;

  if (elapsed >= calibrationDuration) {
    offsetX /= sampleCount;
    offsetY /= sampleCount;
    offsetZ /= sampleCount;
    isCalibrating = false;
    vibroState = VibroState::VIBRO_IDLE;
    Serial.println("Calibração concluída!");
    Serial.print("Offset X: "); Serial.println(offsetX);
    Serial.print("Offset Y: "); Serial.println(offsetY);
    Serial.print("Offset Z: "); Serial.println(offsetZ);
  }
}
//vibroState = VibroState::VIBRO_CONFIG;
void startMeasurement(int durationSeconds) {
  isMeasuring = true;
  measureStartTime = millis();
  measureDuration = durationSeconds * 1000UL;
  secondsLeft = durationSeconds;

  sampleCount = 0;
  velocitySum = 0;
  aPeak = 0;
  aRMS = 0;
  stdDev = 0;
  freqDominant = 0;
  lastSampleTime = 0;
  vibroState = VibroState::VIBRO_CONFIG;
  Serial.println("Iniciando medição...");
}
//vibroState = VibroState::VIBRO_MEASURE;
void updateMeasurement() {
  if (!isMeasuring) return;

  unsigned long now = millis();
  unsigned long elapsed = now - measureStartTime;
  unsigned long remaining = (measureDuration > elapsed) ? (measureDuration - elapsed) : 0;
  secondsLeft = remaining / 1000;

  if (micros() - lastSampleTime >= 1000000UL / SAMPLE_RATE && sampleCount < FFT_SIZE) {
    lastSampleTime = micros();

    sensors_event_t event;
    accel.getEvent(&event);

    float x = event.acceleration.x - offsetX;
    float y = event.acceleration.y - offsetY;
    float z = event.acceleration.z - offsetZ;

    float acc = sqrt(x * x + y * y + z * z);

    accBuffer[sampleCount] = acc;
    timeBuffer[sampleCount] = 0.0;
    velocitySum += acc / SAMPLE_RATE;

    if (acc > aPeak) aPeak = acc;
    sampleCount++;
  }

  if (elapsed >= measureDuration && sampleCount == FFT_SIZE) {
    isMeasuring = false;

    float sum = 0, sumSq = 0;
    for (int i = 0; i < FFT_SIZE; i++) {
      sum += accBuffer[i];
      sumSq += accBuffer[i] * accBuffer[i];
    }

    float mean = sum / FFT_SIZE;
    aRMS = sqrt(sumSq / FFT_SIZE);

    float variance = 0;
    for (int i = 0; i < FFT_SIZE; i++) {
      variance += pow(accBuffer[i] - mean, 2);
    }
    stdDev = sqrt(variance / FFT_SIZE);
    vRMS = velocitySum / FFT_SIZE;

    // Remoção de offset DC
    double dcOffset = 0;
    for (int i = 0; i < FFT_SIZE; i++) {
      dcOffset += accBuffer[i];
    }
    dcOffset /= FFT_SIZE;
    for (int i = 0; i < FFT_SIZE; i++) {
      accBuffer[i] -= dcOffset;
    }

    FFT.windowing(FFT_WIN_TYP_HAMMING, FFT_FORWARD);
    FFT.compute(FFT_FORWARD);
    FFT.complexToMagnitude();

    double maxMag = 0;
    int index = 0;
    for (int i = 1; i < FFT_SIZE / 2; i++) {
      if (accBuffer[i] > maxMag && accBuffer[i] > 5.0) { // Filtro de ruído
        maxMag = accBuffer[i];
        index = i;
      }
    }

    freqDominant = (index * SAMPLE_RATE) / (float)FFT_SIZE;

    Serial.print("Magnitude FFT pico: "); Serial.println(maxMag);

    // Filtro adicional para descartar leitura de frequência em estado estático
    if (aRMS < 0.15 && stdDev < 0.05 && vRMS < 0.01 && maxMag < 5.0) {
      freqDominant = 0;
    }
    vibroState = VibroState::VIBRO_MEASURE;
    Serial.println("\n--- RESULTADO ---");
    Serial.print("Pico Acel: "); Serial.print(aPeak, 3); Serial.println(" m/s²");
    Serial.print("RMS Acel: "); Serial.print(aRMS, 3); Serial.println(" m/s²");
    Serial.print("Desvio padrão: "); Serial.print(stdDev, 3); Serial.println(" m/s²");
    Serial.print("Velocidade RMS: "); Serial.print(vRMS, 3); Serial.println(" m/s");
    Serial.print("Freq. dominante: "); Serial.print(freqDominant, 2); Serial.println(" Hz");
    Serial.println("-----------------");
  }
}

// ==== Setup dos botões ====
void setupButtons(){
  pinMode(BUTTON_MENU, INPUT_PULLUP);
  pinMode(BUTTON_SET, INPUT_PULLUP);
  pinMode(BUTTON_DOUBLE, INPUT_PULLUP);
  pinMode(BUTTON_HALF, INPUT_PULLUP);
  pinMode(BUTTON_ENC, INPUT_PULLUP);
}
// ==== Setup ====
void setup() {
  // Inicializa o pino do LED
  pinMode(LED_PIN, OUTPUT);
  digitalWrite(LED_PIN, LOW);
  // Inicializa o pino do sensor IR
  pinMode(SENSOR_IR_PIN, INPUT);
  setupButtons();

  // Inicia a comunicação I2C com os pinos corretos
  Wire.begin(21, 22);  // SDA = 21, SCL = 22
  // Recupera o último FPM salvo na memória
  prefs.begin("config", true);    // Abre o namespace "config" em modo de leitura
  fpm = prefs.getInt("fpm", 30);  // Carrega 'fpm', usa 30 como padrão se não existir
  timeCalib = prefs.getInt("timeCalib", 30);  // Carrega 'timeCalib', usa 30 como padrão se não existir
  timeMeasure = prefs.getInt("timeMeasure", 30);  // Carrega 'timeMeasure', usa 30 como padrão se não existir
  prefs.end();
  STB_calc = true;

  Serial.begin(115200);

  adxlAvailable = accel.begin();
  if (adxlAvailable) accel.setRange(ADXL345_RANGE_4_G);
  
  if (!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) {
    Serial.println(F("Falha ao iniciar OLED"));
    while (true);
  }

  // Inicializa o timer com resolução de 1us (1MHz)
  timer = timerBegin(1000000);
  timerAttachInterrupt(timer, onTimer);
  timerAlarm(timer, STB_partTime, true, 0);

  // Mostra a tela de splashAdd commentMore actions
  display.clearDisplay();
  display.drawBitmap(0, 0, bitmap, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
  display.display();
  delay(2000);
  display.clearDisplay();
  display.drawBitmap(0, 20, senai_logo, 128, 32, SSD1306_WHITE);
  display.display();
  delay(2000);
  display.clearDisplay();
}

// ==== Loop principal ====
void loop() {
  handleInput();
  updateValues(); //Stroboscópio fica com a função rodando, mas não executa os leds. Apenas os contadores do timer.
  updateMeasurement();
  updateCalibration();
  // --- MÁQUINA DE ESTADOS PRINCIPAL (EXECUÇÃO DO MODO ATUAL) ---
  if (inMenu) {
    drawMenu();
    STB_outputEnabled=false;
    TESTE_calc = false;
    Lant_calc = false;
    vibroState = VibroState::VIBRO_HOME;
  } else {
    drawScreen(currentMode);
  }

  if (!inMenu) {
    switch (selectedMode) {
      case Mode::FREQUENCY: {
        STB_outputEnabled=true;
        //Comandos para alterar o valor usando o Encoder
        long newPos = encoder.read() / 4;  // Dividido por 4 para reduzir sensibilidade
        if (inSubmenu) {
          int delta = newPos - STB_lastEncoderPos;
          STB_phaseDegrees = constrain(STB_phaseDegrees + delta, 0, 359);
          STB_lastEncoderPos = newPos;
          updateValuesRec();
          STB_calc = true;
        } else if (!inSubmenu) {
          int delta = newPos - STB_lastEncoderPos;
          if (inEncoder) {
            delta= delta * 10;
          }
          fpm = constrain(fpm + delta, minFPM, maxFPM);
          STB_lastEncoderPos = newPos;
          updateValuesRec();
          STB_calc = true;
        }
        break;
      }
      case Mode::RPM:{
        unsigned long agora = millis();
        bool estadoAtual = digitalRead(SENSOR_IR_PIN);
  
        // Detecta pulso (borda de subida: LOW -> HIGH)
        if (estadoAtual == HIGH && estadoAnterior == LOW) {
          contagemPulsos++;
        }
        estadoAnterior = estadoAtual;
        // A cada TEMPO_LEITURA_RPM milissegundos, calcula e imprime o RPM
        if (agora - ultimaLeitura >= TEMPO_LEITURA_RPM) {
          ultimaLeitura = agora;
          // Se há 1 marca por rotação, então: RPM = pulsos * 60
          int rpm = contagemPulsos * 60;
          rpmValue = rpm;
          // Reinicia a contagem
          contagemPulsos = 0;
        }
        break;
      }
      case Mode::LANTERN:
        STB_outputEnabled=true;
        Lant_calc = true;
        break;
      case Mode::TEST: {
        static unsigned long ultimoTempo = 0;
        static float fpmAtual = 30.0;
        static const float passo = 30.0;
        static const unsigned long intervaloTeste = 100; // 2 segundos entre testes

        unsigned long agora = millis();
        if(fpmTest.isRunning()){
          if (agora - ultimoTempo >= intervaloTeste) {
            TESTE_fpm = fpmAtual;
            updateValues();
            fpmAtual += passo;
            if (fpmAtual >= maxFPM) {
              fpmAtual = 30.0; // reinicia o ciclo
            }
            ultimoTempo = agora;
          }
        } else if(fpmTest.isExpired()){
          fpmAtual=30;
          STB_outputEnabled=false;
          TESTE_calc = false;
        } else{
          STB_outputEnabled=false;
          TESTE_calc = false;
        }
        break;
      }
    }
  }
}

// ==== Tratamento de botões ====
void handleInput() {
  static bool lastMenuState = HIGH;
  static bool lastSetState  = HIGH;
  static unsigned long lastDebounceTimeMenu = 0;
  static unsigned long lastDebounceTimeSet  = 0;

  if (checkButtonDebounce(BUTTON_MENU, lastMenuState, lastDebounceTimeMenu)) {
    if (inSubmenu) {
      // Se estiver no submenu, apenas sai dele
      inSubmenu = false;
    } else if (!inMenu) {
      // Se estiver fora do menu, entrar no menu com o modo atual
      currentMode = selectedMode;
      inMenu = true;
    } else {
      currentMode = static_cast<Mode>((static_cast<int>(currentMode) + 1) % static_cast<int>(Mode::NUM_MODES));
    }
  }

  if (checkButtonDebounce(BUTTON_SET, lastSetState, lastDebounceTimeSet)) {
    if (inMenu) {
      selectedMode = currentMode;
      inMenu = false;
    } else {
      if (selectedMode == Mode::VIBROMETER) {
        switch (vibroState) {
          //Clique SET para iniciar
          case VibroState::VIBRO_HOME:
            vibroState = VibroState::VIBRO_CALIB;
            break;
          case VibroState::VIBRO_CALIB:
            startCalibration(timeCalib); //10 segundos
            break;
          case VibroState::VIBRO_IDLE:
            startMeasurement(timeMeasure); //10 segundos
            updateValuesRec();
            break;
          case VibroState::VIBRO_CONFIG:
            //vibroState = VibroState::VIBRO_MEASURE;
            //measureSeismicActivity();
            break;
          case VibroState::VIBRO_MEASURE:
            vibroState = VibroState::VIBRO_RESULT;
            break;
          case VibroState::VIBRO_RESULT:
            vibroState = VibroState::VIBRO_HOME;
            break;
        }
      } else if (selectedMode == Mode::RPM) {
        inSubmenu = false;
        if(rpmValue>=30){
          fpm = rpmValue;    //FPM recebe o valor de RPM
          msgTimer.startTimer(2); //Inicia a contagem para exibir a mensagem de gravando por 2 segundos
        }
      } else if(selectedMode == Mode::TEST){
        STB_outputEnabled=true;
        TESTE_calc = true;
        fpmTest.startTimer(10);
      }else if(selectedMode == Mode::LANTERN || currentMode == Mode::ABOUT){
        inSubmenu = false;
      } else {
        inSubmenu = !inSubmenu;
        STB_outputEnabled=false;
      }
    }
  }

  if (checkButtonDebounce(BUTTON_DOUBLE, lastSetState, lastDebounceTimeSet)) {
    if (inMenu) {
      currentMode = static_cast<Mode>((static_cast<int>(currentMode) + 1) % static_cast<int>(Mode::NUM_MODES));
    } else if(selectedMode == Mode::HOME){
      if (topLineIndex < totalLines - lineShowMsg) { // lineShowMsg = linhas cabem na tela
        topLineIndex++;
      }
    } else if(currentMode == Mode::VIBROMETER && vibroState == VibroState::VIBRO_CALIB){
      timeCalib = constrain(timeCalib - 5, 5, 30);
    }  else if(currentMode == Mode::VIBROMETER && vibroState == VibroState::VIBRO_IDLE){
      timeMeasure = constrain(timeMeasure - 10, 10, 60);
    } else {
      if (selectedMode == Mode::FREQUENCY && !inSubmenu) {
        adjustFPM(0.5);
        STB_calc = true;
      }
    }
  }

  if (checkButtonDebounce(BUTTON_HALF, lastSetState, lastDebounceTimeSet)) {
    if (inMenu) {
      currentMode = static_cast<Mode>((static_cast<int>(currentMode) - 1) % static_cast<int>(Mode::NUM_MODES));
    } else if(selectedMode == Mode::HOME){
      if (topLineIndex > 0) {
        topLineIndex--;
      }
    } else if(currentMode == Mode::VIBROMETER && vibroState == VibroState::VIBRO_CALIB){
      timeCalib = constrain(timeCalib + 5, 5, 30);
    } else if(currentMode == Mode::VIBROMETER && vibroState == VibroState::VIBRO_IDLE){
      timeMeasure = constrain(timeMeasure + 10, 10, 60);
    } else {
      if (selectedMode == Mode::FREQUENCY && !inSubmenu) {
        adjustFPM(2.0);
        STB_calc = true;
      }
    }
  }

  if (checkButtonDebounce(BUTTON_ENC, lastMenuState, lastDebounceTimeMenu)) {
    inEncoder = !inEncoder;
    if (inEncoder) {
      //FAZ
    } else {
      //NÃO FAZ
    }
  }
}

// ==== Tela do Menu ====
void drawMenu() {
  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println("Selecione o modo:");
  display.setTextSize(2);
  display.setCursor(0, 20);
  display.println(getModeName(currentMode));
  display.setTextSize(1);
  display.setCursor(0, 56);
  display.println("MENU=Prox SET=Entrar");
  display.display();
}

// ==== Tela de cada Modo ====
void drawScreen(Mode mode) {
  // Limita a taxa de atualização para evitar flickering e consumo de CPU
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate < 50) return;  // Atualiza a cada 50ms (20 FPS)
  lastUpdate = millis();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.println(getModeName(mode));
  display.drawLine(0, 10, SCREEN_WIDTH, 10, SSD1306_WHITE);
  display.setCursor(0, 14);

  if (currentMode == Mode::RPM) {
    display.print("RPM: ");
    display.print((int)rpmValue);
    display.setCursor(90, 14);
    display.print("S: ");
    display.print(rpmValue > 0 ? "OK" : "X");
    display.setCursor(0, 26);
    display.print(msgTimer.isRunning() ? "Gravando FPM..." : "Gravar use SET");
    display.setCursor(0, 56);
    display.println("Menu=Voltar SET=Gravar");

  } else if (currentMode == Mode::ABOUT) {
    display.println("Grupo Alfa: ");
    display.println("Igor Gustavo");
    display.println("Nycollas Luan");
    display.println("Mateus Paiva");
    display.println("Rolly Santos");
    display.drawBitmap(90, 15, trofeu, 32, 32, SSD1306_WHITE);
    display.setCursor(0, 56);
    display.println("alfasweb.com.br");
  } else if (currentMode == Mode::TEST) {
    display.print(fpmTest.isRunning() ? "Testando" : "Testar");
    display.setCursor(0, 56);
    display.println(fpmTest.isRunning() ? "Aguarde..." : "SET=Iniciar Teste");
  } else if (currentMode == Mode::LANTERN) {
    display.print("LANTERNA");
    display.setCursor(0, 56);
    display.println("MENU=Voltar");
  } else if (currentMode == Mode::HOME) {
    for (int i = 0; i < lineShowMsg; i++) {
      int lineIndex = topLineIndex + i;
      if (lineIndex < totalLines) {
        display.setCursor(0, (i + 1) * 10);
        display.print(lines[lineIndex]);
      }
    }
  } else if (currentMode == Mode::FREQUENCY) {
    display.setTextSize(2);
    display.setCursor(0, 14);
    display.print((int)fpm);
    display.setTextSize(1);
    display.setCursor(70, 22);
    display.print("FPM");
    display.setCursor(0, 34);
    display.print("Hz:  ");
    display.print(fpm / 60.0, 2);
    display.setCursor(0, 46);
    display.print("Fase: ");
    display.print(STB_phaseDegrees);
    display.print((char)247);
    if(inSubmenu){
      display.setCursor(80, 46);
      display.println("<");
      display.setCursor(0, 56);
      display.println(">>Editar Phase");
    }
  } else if (currentMode == Mode::VIBROMETER){
    if (adxlAvailable){
      switch (vibroState) {
        case VibroState::VIBRO_HOME:
          display.println("Coloque sobre o motor para calibrar");
          break;
        case VibroState::VIBRO_CALIB:
         if(!isCalibrating){
          display.print("Tempo de calibracao: ");
          display.print(timeCalib);
          display.println("s");
          display.println("");
          display.println("SET para calibrar");
         }else{
          display.println("Calibrando...");
          display.println("");
          display.print("Tempo restante: ");
          display.print(secondsLeftCalib);
          display.println("s");
         }
          break;
        case VibroState::VIBRO_IDLE:
          display.print("Tempo de medida: ");
          display.print(timeMeasure);
          display.println("s");
          display.println("");
          display.println("Ligue o motor, pres-");
          display.println("sione SET p/ iniciar");
        break;
        case VibroState::VIBRO_CONFIG:
          display.println("Medindo...");
          display.println("");
          display.print("Tempo restante: ");
          display.print(secondsLeft);
          display.println("s");
          break;
        case VibroState::VIBRO_MEASURE:
          display.print("PA: "); display.print(aPeak, 3); display.println("m/s2");
          display.print("RMS A: "); display.print(aRMS, 3); display.println("m/s2");
          display.print("DP: "); display.print(stdDev, 3); display.println("m/s2");
          display.print("V RMS: "); display.print(vRMS, 3); display.println("m/s");
          display.print("FD: "); display.print(freqDominant, 2); display.println("Hz");
          break;
        case VibroState::VIBRO_RESULT:
          display.println("PA: Pico de Acel.");
          display.println("RMS A: RMS de Acel.");
          display.println("DP: Desvio Padrao");
          display.println("V RMS: Vel. de RMS");
          display.println("FD: Freq. Dominante");
          break;
      }
    }else{
      display.println("Sensor indisponível");
    }
  }
  display.display();
}

// ==== Retorna nome do modo atual ====
const char* getModeName(Mode mode) {
  switch (mode) {
    case Mode::HOME: return "StroboTech";
    case Mode::FREQUENCY: return "Estroboscopio";
    case Mode::RPM: return "RPM";
    case Mode::LANTERN: return "Lanterna";
    case Mode::VIBROMETER: return "Vibrometro";
    case Mode::TEST: return "Teste";
    case Mode::ABOUT: return "Sobre";
  }
}
