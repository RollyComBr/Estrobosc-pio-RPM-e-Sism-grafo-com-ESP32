#include <Preferences.h> // Biblioteca para armazenar dados em memória flash (NVS)
#include <Wire.h> // Biblioteca para comunicação I2C
#include <Adafruit_GFX.h> // Biblioteca base para displays gráficos
#include <Adafruit_SSD1306.h> // Biblioteca específica para o display OLED SSD1306
#include <Adafruit_ADXL345_U.h> // Biblioteca para o acelerômetro ADXL345

// Definições de largura e altura do display OLED
#define SCREEN_WIDTH 128
#define SCREEN_HEIGHT 64
#define OLED_RESET -1 // Sem pino de reset

// Instância do display OLED
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);

// Instância do acelerômetro ADXL345
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345);
bool adxlAvailable = false; // Flag que indica se o acelerômetro está disponível

Preferences prefs; // Objeto para armazenar preferências na memória flash do ESP32

// Definição dos pinos utilizados
//21 Pino I2C SDA
//22 Pino I2C SCL
const int encoderPinA = 16; // Pino A do encoder rotativo
const int encoderPinB = 17; // Pino B do encoder rotativo
const int buttonDouble = 18; // Botão para dobrar FPM
const int buttonHalf = 19; // Botão para reduzir FPM pela metade
const int buttonMode = 5; // Botão para mudar o modo de operação
const int buttonSet = 15; // Botão para confirmar/ajustar
const int ledPin = 2; // Pino do LED do estroboscópio
const int sensorIRPin = 4; // Pino do sensor infravermelho (para RPM)

// Variáveis para controle do encoder rotativo
volatile long encoderValue = 0; // Valor atual do encoder
volatile unsigned long lastEncoderInterrupt = 0; // Tempo do último movimento
const unsigned long encoderDebounce = 2000; // Debounce para o encoder (em microssegundos)
const unsigned long debounceButton = 500; // Debounce para os botões (em milissegundos)
int lastEncoderValue = 0; // Valor anterior do encoder

// Controle de tempo para piscar o LED (modo estroboscópio)
unsigned long lastFlashTime = 0; // Último tempo de flash
unsigned long intervalMicros = 1000000; // Intervalo entre flashes em microssegundos

// Controle de estroboscópio
int fpm = 30; // Flashes por minuto (FPM)
int phase = 0; // Ajuste de fase
unsigned long phaseDelay = 0; // Delay da fase calculado

// Enumeração dos modos de operação
enum Mode { MODE_FREQUENCY, MODE_LANTERN, MODE_RPM, MODE_SEISMOGRAPH, MODE_TEST, MODE_ABOUT };
Mode currentMode = MODE_FREQUENCY; // Modo atual
int menuIndex = 0; // Índice do menu
bool menuActive = false; // Menu ativo ou não
bool phaseSetting = false; // Ajuste de fase ativo
bool lanternLEDState = false; // Estado do LED no modo lanterna
bool testRunning = false; // Indica se o teste está em execução

// Variáveis para medição de RPM
volatile unsigned long lastSensorTime = 0; // Último tempo do sensor IR
volatile unsigned long pulseInterval = 0; // Intervalo entre pulsos
float rpmValue = 0; // Valor calculado de RPM

// Variáveis usadas no modo sismógrafo
float amplitude = 0; // Amplitude de vibração
float frequency = 0; // Frequência da vibração
unsigned long vibStart = 0; // Início da vibração
unsigned long vibDuration = 0; // Duração da vibração
bool vibrationActive = false; // Indica se a vibração está ativa
float magnitude = 0; // Magnitude calculada

// Limites permitidos para FPM
const int minFPM = 30; // FPM mínimo
const int maxFPM = 7500; // FPM máximo

// Controle de debounce para botões
unsigned long lastButtonTimeDouble = 0; // Último clique em DOUBLE
unsigned long lastButtonTimeHalf = 0; // Último clique em HALF
unsigned long lastButtonTimeMode = 0; // Último clique em MODE
unsigned long lastButtonTimeSet = 0; // Último clique em SET

// Controle de tempo do teste automático
unsigned long testStartTime = 0; // Tempo inicial do teste

// Armazena os dados XYZ do acelerômetro para cálculo de frequência
float lastXYZ[3] = {0, 0, 0}; // Últimos valores lidos para cada eixo
unsigned long lastCrossTime[3] = {0, 0, 0}; // Último tempo de cruzamento do zero
unsigned long intervalsXYZ[3][10]; // Intervalos entre cruzamentos do zero
int intervalIndex[3] = {0, 0, 0}; // Índice de escrita atual em cada buffer
bool readyXYZ[3] = {false, false, false}; // Indica se já tem dados suficientes
float freqXYZ[3] = {0, 0, 0}; // Frequência calculada para cada eixo

// Estados possíveis para o modo sismógrafo
enum SeismoState { SEISMO_IDLE, SEISMO_CALIB, SEISMO_CONFIG, SEISMO_MEASURE, SEISMO_RESULT };
SeismoState seismoState = SEISMO_IDLE; // Estado atual do modo sismógrafo

float restAmplitude = 0.0; // Amplitude média de repouso (tara)
unsigned long seismoTimer = 0; // Timer usado nas medições
int seismoDuration = 10; // Tempo de medição (ajustável pelo usuário)
float amplitudeSum = 0.0; // Soma das amplitudes durante medição
float frequencySum = 0.0; // Soma das frequências durante medição
int seismoSamples = 0; // Contador de amostras válidas

void updateInterval() {
  fpm = constrain(fpm, minFPM, maxFPM);
  intervalMicros = 60000000UL / fpm;
  phaseDelay = (intervalMicros * phase) / 359;

  prefs.begin("config", false);
  prefs.putInt("fpm", fpm);
  prefs.end();
}

const unsigned char bitmap [] PROGMEM= {
  0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x07, 0xf8, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0x00, 0x00, 0x60, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x38, 0x07, 0x00, 0x00, 0x00, 0x00, 0x00, 0x1f, 0x00, 0x03, 0xf8, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0xc0, 0x00, 0xc0, 0x00, 0x00, 0x00, 0x00, 0x3f, 0x00, 0x06, 0x7c, 0x00, 0x00, 0x00, 0x00, 
	0x01, 0x00, 0x00, 0x60, 0x00, 0x00, 0x40, 0x00, 0x0f, 0x00, 0x0c, 0x38, 0x00, 0x00, 0x00, 0x00, 
	0x02, 0x00, 0x00, 0x10, 0x00, 0x00, 0xe0, 0x00, 0x07, 0x00, 0x18, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x04, 0x00, 0x00, 0x08, 0x00, 0x00, 0xe0, 0x00, 0x07, 0x00, 0x38, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x08, 0x00, 0x00, 0x04, 0x00, 0x01, 0xe0, 0x00, 0x07, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x10, 0x0f, 0xc0, 0x02, 0x00, 0x01, 0xf0, 0x00, 0x07, 0x00, 0x30, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x20, 0x3f, 0xe0, 0xfa, 0x00, 0x01, 0xf0, 0x00, 0x07, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x20, 0xf0, 0xf1, 0xf1, 0x00, 0x03, 0x78, 0x00, 0x07, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x41, 0xe0, 0x79, 0xf1, 0x80, 0x02, 0x78, 0x00, 0x07, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x43, 0xc0, 0x7b, 0xe0, 0x80, 0x06, 0x38, 0x00, 0x07, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x07, 0x80, 0x3d, 0xc0, 0x80, 0x06, 0x3c, 0x00, 0x07, 0x00, 0x70, 0x00, 0x00, 0x00, 0x00, 0xf0, 
	0x0f, 0x80, 0x3d, 0xc0, 0x80, 0x04, 0x3c, 0x00, 0x07, 0x03, 0xff, 0x80, 0xfc, 0x00, 0x83, 0xfc, 
	0x9f, 0x00, 0x3d, 0x80, 0x40, 0x0c, 0x1c, 0x00, 0x07, 0x00, 0x78, 0x03, 0xff, 0x0f, 0x86, 0x0e, 
	0x9e, 0x00, 0x1d, 0x80, 0x40, 0x0c, 0x1e, 0x00, 0x07, 0x00, 0x70, 0x0f, 0x07, 0x9f, 0x0c, 0x06, 
	0x3e, 0x00, 0x1f, 0x80, 0x40, 0x18, 0x1e, 0x00, 0x07, 0x00, 0x70, 0x1e, 0x07, 0xbe, 0x0c, 0x06, 
	0x3e, 0x00, 0x1f, 0x00, 0x40, 0x18, 0x0e, 0x00, 0x07, 0x00, 0x70, 0x3c, 0x03, 0xfe, 0x1c, 0x02, 
	0x3c, 0x00, 0x6f, 0x00, 0x40, 0x10, 0x0f, 0x00, 0x07, 0x00, 0x70, 0x78, 0x03, 0xdc, 0x1e, 0x00, 
	0x3c, 0x00, 0x6f, 0x00, 0x40, 0x30, 0x0f, 0x00, 0x07, 0x00, 0x70, 0xf8, 0x03, 0xd8, 0x0f, 0x00, 
	0x7c, 0x00, 0xef, 0x00, 0x00, 0x3f, 0xff, 0x00, 0x07, 0x00, 0x70, 0xf0, 0x01, 0xd8, 0x0f, 0xc0, 
	0x7c, 0x01, 0xe7, 0x80, 0x80, 0x3f, 0xff, 0x80, 0x07, 0x00, 0x71, 0xf0, 0x01, 0xf8, 0x07, 0xf0, 
	0x7c, 0x03, 0xf7, 0x80, 0x80, 0x60, 0x07, 0x80, 0x07, 0x00, 0x71, 0xe0, 0x01, 0xf0, 0x03, 0xf8, 
	0x3c, 0x07, 0xe7, 0x80, 0x80, 0x60, 0x03, 0xc0, 0x07, 0x00, 0x71, 0xe0, 0x02, 0xf0, 0x00, 0xfe, 
	0x3e, 0x0f, 0xc3, 0xc1, 0x00, 0xc0, 0x03, 0xc0, 0x07, 0x00, 0x73, 0xe0, 0x06, 0xf0, 0x00, 0x3e, 
	0x1f, 0xff, 0x03, 0xf6, 0x00, 0xc0, 0x03, 0xc0, 0x07, 0x00, 0x73, 0xc0, 0x06, 0xf0, 0x00, 0x1f, 
	0x13, 0xfc, 0x01, 0xfe, 0x00, 0xc0, 0x01, 0xe0, 0x07, 0x00, 0x73, 0xc0, 0x0e, 0x70, 0x00, 0x0f, 
	0x00, 0x00, 0x00, 0x04, 0x01, 0x80, 0x01, 0xe0, 0x07, 0x00, 0x71, 0xc0, 0x1f, 0x78, 0x10, 0x07, 
	0x00, 0x00, 0x00, 0x08, 0x01, 0x80, 0x00, 0xe0, 0x07, 0x00, 0x71, 0xe0, 0x3e, 0x78, 0x18, 0x07, 
	0x00, 0x00, 0x00, 0x10, 0x03, 0x80, 0x00, 0xf0, 0x07, 0x00, 0x71, 0xe0, 0xfc, 0x7c, 0x18, 0x06, 
	0x01, 0x00, 0x00, 0x20, 0x07, 0x80, 0x00, 0xf8, 0x0f, 0x00, 0x78, 0xfb, 0xf8, 0x3e, 0x5c, 0x0e, 
	0x00, 0xc0, 0x00, 0xc0, 0x1f, 0xf0, 0x07, 0xff, 0x3f, 0xe3, 0xff, 0x3f, 0xe0, 0x1f, 0x8f, 0x38, 
	0x00, 0x00, 0x02, 0x00, 0x00, 0x10, 0x04, 0x00, 0x20, 0x22, 0x01, 0x00, 0x00, 0x00, 0x03, 0xe0, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

const unsigned char trofeu [] PROGMEM = {
	0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x01, 0xff, 0xff, 0x80, 0x01, 0xff, 0xff, 0x80, 
	0x01, 0xff, 0xff, 0x80, 0x7f, 0xff, 0xff, 0xfe, 0xff, 0xff, 0xff, 0xfe, 0xff, 0xff, 0xff, 0xfe, 
	0xe1, 0xff, 0xff, 0x0e, 0xe1, 0xff, 0xff, 0x0e, 0x61, 0xff, 0xff, 0x0e, 0x70, 0xff, 0xff, 0x0e, 
	0x70, 0xff, 0xfe, 0x1e, 0x78, 0xff, 0xfe, 0x1c, 0x38, 0xff, 0xfe, 0x3c, 0x3c, 0x7f, 0xfe, 0x78, 
	0x1e, 0x7f, 0xfe, 0xf0, 0x0f, 0x7f, 0xfd, 0xe0, 0x07, 0xff, 0xff, 0xe0, 0x03, 0xff, 0xff, 0x80, 
	0x00, 0xff, 0xff, 0x00, 0x00, 0x3f, 0xf8, 0x00, 0x00, 0x07, 0xc0, 0x00, 0x00, 0x03, 0xc0, 0x00, 
	0x00, 0x03, 0xc0, 0x00, 0x00, 0x03, 0xc0, 0x00, 0x00, 0x7f, 0xfe, 0x00, 0x00, 0x7f, 0xfe, 0x00, 
	0x00, 0x7f, 0xfe, 0x00, 0x00, 0x7f, 0xfe, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00
};

void updateDisplay() {
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate < 50) return;
  lastUpdate = millis();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);
  display.setCursor(0, 0);
  display.print("Modo: ");
  switch (currentMode) {
    case MODE_FREQUENCY: display.print("Estrobo"); break;
    case MODE_LANTERN: display.print("Lanterna"); break;
    case MODE_RPM: display.print("RPM"); break;
    case MODE_SEISMOGRAPH: display.print("Sismografo"); break;
    case MODE_TEST: display.print("Teste"); break;
    case MODE_ABOUT: display.print("Sobre"); break;
  }
  display.drawLine(0, 10, SCREEN_WIDTH, 10, SSD1306_WHITE);

  display.setCursor(0, 14);

  if (currentMode == MODE_RPM) {
    display.print("RPM: "); display.print((int)rpmValue);
    display.setCursor(90, 14);
    display.print("S: "); display.print(pulseInterval > 0 ? "OK" : "X");
  }else if(currentMode == MODE_ABOUT){
    display.println("Grupo Alfa: ");
    display.println("Igor Gustavo");
    display.println("Nycollas luan");
    display.println("Mateus Paiva");
    display.println("Rolly Santos");
    display.drawBitmap(90, 15, trofeu, 32, 32, SSD1306_WHITE);
  } else if (currentMode == MODE_SEISMOGRAPH) {
    switch (seismoState) {
      case SEISMO_IDLE:
        display.print("Press SET para iniciar");
        break;
      case SEISMO_CALIB:
        display.print("Calibrando...");
        break;
      case SEISMO_CONFIG:
        display.print("Tempo: "); display.print(seismoDuration); display.print("s");
        display.setCursor(0, 26);
        display.print("Use encoder e SET");
        break;
      case SEISMO_MEASURE:
        display.print("Medindo... ");
        display.setCursor(0, 26);
        display.print("Faltam: ");
        display.print((seismoTimer - millis()) / 1000);
        display.print("s");
        break;
      case SEISMO_RESULT:
        display.print("Amp: "); display.print(amplitude, 2);
        display.setCursor(0, 26);
        display.print("Hz: "); display.print(frequency, 2);
        display.setCursor(0, 38);
        display.print("T: "); display.print(vibDuration); display.print("ms");
        display.setCursor(0, 50);
        display.print("Mag: "); display.print(magnitude, 2);
        break;
    }
  } else if (currentMode == MODE_TEST) {
    display.print(testRunning ? "Testando" : "Teste");
  } else if (currentMode == MODE_LANTERN) {
    display.print("LANTERNA");
    display.setCursor(90, 14);
    display.print("S: "); display.print(lanternLEDState ? "OK" : "X");
  } else {
    display.setTextSize(2);
    display.setCursor(0, 14);
    display.print(fpm);
    display.setTextSize(1);
    display.setCursor(70, 22);
    display.print("FPM");
    display.setCursor(0, 34);
    display.print("Hz:  "); display.print(fpm / 60.0, 2);
    if (currentMode == MODE_FREQUENCY) {
      display.setCursor(0, 46);
      display.print("Fase: "); display.print(phase); display.print((char)247);
    }
  }

  if (menuActive) {
    display.setCursor(0, 56);
    switch (menuIndex) {
      case 0: display.print("> Estrobo"); break;
      case 1: display.print("> Lanterna"); break;
      case 2: display.print("> RPM"); break;
      case 3: display.print("> Sismografo"); break;
      case 4: display.print("> Teste"); break;
      case 5: display.print("> Sobre"); break;
    }
  } else if (phaseSetting && currentMode == MODE_FREQUENCY) {
    display.setCursor(0, 56);
    display.print("> Ajustando fase...");
  }

  display.display();
}

void handleEncoder() {
  unsigned long now = micros();
  if (now - lastEncoderInterrupt > encoderDebounce) {
    int b = digitalRead(encoderPinB);
    if (b == LOW) encoderValue++;
    else encoderValue--;
    lastEncoderInterrupt = now;
  }
}

void setupEncoder() {
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder, FALLING);
}

void setupButtons() {
  pinMode(buttonDouble, INPUT_PULLUP);
  pinMode(buttonHalf, INPUT_PULLUP);
  pinMode(buttonMode, INPUT_PULLUP);
  pinMode(buttonSet, INPUT_PULLUP);
}

void IR_sensor_ISR() {
  unsigned long now = micros();
  pulseInterval = now - lastSensorTime;
  lastSensorTime = now;
}

void adjustFPM(float factor) {
  fpm = constrain(fpm * factor, minFPM, maxFPM);
  updateInterval();
  updateDisplay();
}

void runTestSequence() {
  static const int testSteps = 15;
  static int step = 0;
  static unsigned long lastStepTime = 0;
  static bool pulseActive = false;
  static unsigned long pulseStartMicros = 0;
  const unsigned int pulseDurationMicros = 100;

  if (step == 0) {
    testStartTime = millis();
    testRunning = true;
  }

  unsigned long now = millis();
  if (step < testSteps) {
    if (!pulseActive && now - lastStepTime >= 300) {
      int freq = map(step, 0, testSteps - 1, minFPM, maxFPM);
      int period = 60000 / freq;
      digitalWrite(ledPin, HIGH);
      pulseStartMicros = micros();
      pulseActive = true;
      lastStepTime = now;
      step++;
    }

    if (pulseActive && micros() - pulseStartMicros >= pulseDurationMicros) {
      digitalWrite(ledPin, LOW);
      pulseActive = false;
    }
  } else {
    testRunning = false;
    step = 0;
    updateDisplay();
  }
}

void setup() {
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);
  pinMode(sensorIRPin, INPUT);

  Wire.begin(21, 22); // SDA = 21, SCL = 22

  updateInterval();

  setupEncoder();
  setupButtons();
  attachInterrupt(digitalPinToInterrupt(sensorIRPin), IR_sensor_ISR, FALLING);
  
  display.begin(SSD1306_SWITCHCAPVCC, 0x3C);
  display.clearDisplay();
  display.drawBitmap(0, 0, bitmap, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
  display.display();
  delay(3000);

  adxlAvailable = accel.begin();
  if (adxlAvailable) {
    accel.setRange(ADXL345_RANGE_16_G);
  }

  updateDisplay();
}

void loop() {
  unsigned long now = millis();
  static bool pulseActive = false;
  static unsigned long pulseStartTime = 0;
  const unsigned int pulseDurationMicros = 100;

  if (digitalRead(buttonDouble) == LOW && now - lastButtonTimeDouble > debounceButton) {
    if (!menuActive && currentMode == MODE_FREQUENCY && !phaseSetting) adjustFPM(2.0);
    else if (currentMode == MODE_FREQUENCY && phaseSetting) {
      phase = (phase * 2) % 360;
      updateInterval();
      updateDisplay();
    }
    lastButtonTimeDouble = now;
  }

  if (digitalRead(buttonHalf) == LOW && now - lastButtonTimeHalf > debounceButton) {
    if (!menuActive && currentMode == MODE_FREQUENCY && !phaseSetting) adjustFPM(0.5);
    else if (currentMode == MODE_FREQUENCY && phaseSetting) {
      phase = (phase / 2) % 360;
      updateInterval();
      updateDisplay();
    }
    lastButtonTimeHalf = now;
  }

  if (digitalRead(buttonMode) == LOW && now - lastButtonTimeMode > debounceButton) {
    menuActive = !menuActive;
    phaseSetting = false;
    lastButtonTimeMode = now;
    updateDisplay();
  }

  if (digitalRead(buttonSet) == LOW && now - lastButtonTimeSet > debounceButton) {
    if (menuActive) {
      currentMode = static_cast<Mode>(menuIndex);
      menuActive = false;
    } else if (currentMode == MODE_FREQUENCY) {
      phaseSetting = !phaseSetting;
    } else if (currentMode == MODE_LANTERN) {
      lanternLEDState = !lanternLEDState;
      digitalWrite(ledPin, lanternLEDState ? HIGH : LOW);
    } else if(currentMode == MODE_ABOUT){

    } else if (currentMode == MODE_TEST && !testRunning) {
      runTestSequence();
    } else if (currentMode == MODE_SEISMOGRAPH) {
      if (seismoState == SEISMO_IDLE) {
        seismoState = SEISMO_CALIB;
        restAmplitude = 0.0;
        for (int i = 0; i < 50; i++) {
          sensors_event_t event;
          accel.getEvent(&event);
          float acc = sqrt(event.acceleration.x * event.acceleration.x +
                           event.acceleration.y * event.acceleration.y +
                           event.acceleration.z * event.acceleration.z);
          restAmplitude += acc;
          delay(10);
        }
        restAmplitude /= 50.0;
        seismoState = SEISMO_CONFIG;
      } else if (seismoState == SEISMO_CONFIG) {
        seismoState = SEISMO_MEASURE;
        seismoTimer = millis() + seismoDuration * 1000;
        amplitudeSum = 0.0;
        frequencySum = 0.0;
        seismoSamples = 0;
        vibStart = millis();
      } else if (seismoState == SEISMO_RESULT) {
        seismoState = SEISMO_IDLE;
      }
      //updateDisplay();
    }
    updateDisplay();
    lastButtonTimeSet = now;
  }

  if (encoderValue != lastEncoderValue) {
    int delta = encoderValue - lastEncoderValue;
    lastEncoderValue = encoderValue;
    if (menuActive) {
      menuIndex = (menuIndex + delta + 6) % 6;
    } else if (phaseSetting && currentMode == MODE_FREQUENCY) {
      phase = (phase + delta + 360) % 360;
      updateInterval();
    } else if (currentMode == MODE_FREQUENCY) {
      fpm += delta;
      updateInterval();
    } else if (currentMode == MODE_SEISMOGRAPH && seismoState == SEISMO_CONFIG) {
      seismoDuration = constrain(seismoDuration + delta, 10, 60);
    }
    updateDisplay();
  }

  switch (currentMode) {
    case MODE_FREQUENCY:
      seismoState = SEISMO_IDLE;
      if (!pulseActive && (micros() - lastFlashTime >= intervalMicros + phaseDelay)) {
        digitalWrite(ledPin, HIGH);
        pulseStartTime = micros();
        pulseActive = true;
        lastFlashTime = micros();
      }

      if (pulseActive && (micros() - pulseStartTime >= pulseDurationMicros)) {
        digitalWrite(ledPin, LOW);
        pulseActive = false;
      }

      break;

    case MODE_LANTERN:
      seismoState = SEISMO_IDLE;
      break;

    case MODE_RPM: {
      seismoState = SEISMO_IDLE;
      digitalWrite(ledPin, LOW);
      unsigned long intervalCopy;
      noInterrupts();
      intervalCopy = pulseInterval;
      interrupts();
      if (intervalCopy > 0) {
        rpmValue = 60000000.0 / intervalCopy;
      }
      break;
    }

    case MODE_SEISMOGRAPH:
      if (adxlAvailable) {
        sensors_event_t event;
        accel.getEvent(&event);

        float acc[3] = { event.acceleration.x, event.acceleration.y, event.acceleration.z };
        float totalAcc = sqrt(acc[0]*acc[0] + acc[1]*acc[1] + acc[2]*acc[2]);
        float adjustedAmplitude = totalAcc - restAmplitude;
        if (adjustedAmplitude < 0) adjustedAmplitude = 0;

        float freqZ = 0.0;
        if ((acc[2] > 0 && lastXYZ[2] < 0) || (acc[2] < 0 && lastXYZ[2] > 0)) {
          if (lastCrossTime[2] > 0) {
            unsigned long interval = micros() - lastCrossTime[2];
            freqZ = 1000000.0 / (interval * 2.0);
          }
          lastCrossTime[2] = micros();
        }
        lastXYZ[2] = acc[2];

        if (seismoState == SEISMO_MEASURE) {
          amplitudeSum += adjustedAmplitude;
          frequencySum += freqZ;
          seismoSamples++;
          if (millis() >= seismoTimer) {
            amplitude = amplitudeSum / max(seismoSamples, 1);
            frequency = frequencySum / max(seismoSamples, 1);
            vibDuration = millis() - vibStart;
            magnitude = log10(amplitude + 1) + 1.5 * log10(vibDuration + 1);
            seismoState = SEISMO_RESULT;
            updateDisplay();
          }
        }
      }
      break;

    case MODE_TEST:
      seismoState = SEISMO_IDLE;
      if (testRunning) {
        runTestSequence();
      } else {
        digitalWrite(ledPin, LOW);
      }
      break;

    case MODE_ABOUT:
    break;
  }
}
