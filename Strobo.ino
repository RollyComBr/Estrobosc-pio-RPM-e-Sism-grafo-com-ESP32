/**
 * @file main.cpp
 * @brief Firmware para um dispositivo multifuncional baseado em ESP32.
 * @author Grupo Alfa (Igor Gustavo, Nycollas Luan, Mateus Paiva, Rolly Santos)
 * @date 2023-10-27
 *
 * Este projeto implementa um dispositivo portátil com as seguintes funções:
 * - Estroboscópio digital com ajuste de FPM e fase.
 * - Tacômetro digital (RPM) usando um sensor IR.
 * - Sismógrafo/Vibrômetro usando um acelerômetro ADXL345.
 * - Lanterna.
 * - Modo de teste de hardware.
 * A interface é controlada por um display OLED, um encoder rotativo e quatro botões.
 */

// --- INCLUDES & BIBLIOTECAS ---
#include <Preferences.h>     // Biblioteca para armazenar dados na memória flash NVS (Non-Volatile Storage)
#include <Wire.h>            // Biblioteca para comunicação I2C (usada pelo display e acelerômetro)
#include <Adafruit_GFX.h>    // Biblioteca gráfica base para displays
#include <Adafruit_SSD1306.h> // Biblioteca específica para o display OLED SSD1306
#include <Adafruit_ADXL345_U.h> // Biblioteca para o sensor acelerômetro ADXL345

// --- DEFINIÇÕES DE HARDWARE E CONSTANTES ---
#define SCREEN_WIDTH 128 // Largura do display em pixels
#define SCREEN_HEIGHT 64 // Altura do display em pixels
#define OLED_RESET -1    // Pino de reset do OLED (-1 significa que está compartilhando o reset do ESP32)

// --- PINAGEM (MAPEAMENTO DE PINOS) ---
// Pinos I2C são definidos no Wire.begin() -> SDA=21, SCL=22
const int encoderPinA = 16;  // Pino A (CLK) do encoder rotativo
const int encoderPinB = 17;  // Pino B (DT) do encoder rotativo
const int buttonDouble = 18; // Botão para dobrar o valor de FPM
const int buttonHalf = 19;   // Botão para reduzir o FPM pela metade
const int buttonMode = 5;    // Botão para alternar entre os modos de operação (abre o menu)
const int buttonSet = 15;    // Botão para confirmar seleção ou entrar em sub-modos (ajuste de fase)
const int ledPin = 2;        // Pino do LED de alta potência para o estroboscópio/lanterna
const int sensorIRPin = 4;   // Pino do sensor infravermelho para medição de RPM

// --- INSTÂNCIAS DE OBJETOS ---
Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);
Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified(12345); // ID do sensor para a biblioteca
Preferences prefs; // Objeto para manipulação da memória NVS

// --- VARIÁVEIS GLOBAIS DE ESTADO ---
// Flags de sistema
bool adxlAvailable = false; // Flag que indica se o acelerômetro foi inicializado com sucesso
bool menuActive = false;    // Flag que indica se o menu de seleção de modo está ativo
bool phaseSetting = false;  // Flag para o sub-modo de ajuste de fase
bool lanternLEDState = false; // Estado do LED no modo lanterna (ligado/desligado)
bool testRunning = false;   // Flag que indica se a sequência de teste está em execução

// Enumeração dos modos de operação para clareza do código
enum Mode { MODE_FREQUENCY, MODE_LANTERN, MODE_RPM, MODE_SEISMOGRAPH, MODE_TEST, MODE_ABOUT };
Mode currentMode = MODE_FREQUENCY; // Modo inicial do dispositivo
int menuIndex = 0; // Índice do item selecionado no menu de modos

// Controle do Encoder Rotativo
volatile long encoderValue = 0; // Valor atual do encoder (incrementa/decrementa)
int lastEncoderValue = 0;       // Último valor lido para detectar mudança
volatile unsigned long lastEncoderInterrupt = 0; // Timestamp da última interrupção para debounce
const unsigned long encoderDebounce = 2000; // Tempo de debounce para o encoder em microssegundos

// Controle de Debounce para Botões
const unsigned long debounceButton = 500; // Tempo de debounce para botões em milissegundos
unsigned long lastButtonTimeDouble = 0;
unsigned long lastButtonTimeHalf = 0;
unsigned long lastButtonTimeMode = 0;
unsigned long lastButtonTimeSet = 0;

// Variáveis do Modo Estroboscópio
int fpm = 30; // Flashes por minuto (FPM), valor inicial
int phase = 0; // Ajuste de fase em graus (0-359)
unsigned long intervalMicros = 1000000; // Intervalo entre flashes em microssegundos, calculado a partir do FPM
unsigned long phaseDelay = 0; // Atraso de fase calculado em microssegundos
unsigned long lastFlashTime = 0; // Timestamp do último flash para controle de tempo
const int minFPM = 30; // Limite mínimo de FPM
const int maxFPM = 7500; // Limite máximo de FPM

// Variáveis do Modo RPM
volatile unsigned long lastSensorTime = 0; // Timestamp da última detecção do sensor IR
volatile unsigned long pulseInterval = 0;  // Intervalo de tempo entre pulsos do sensor IR
float rpmValue = 0;                        // Valor de RPM calculado

// Variáveis do Modo Sismógrafo
enum SeismoState { SEISMO_IDLE, SEISMO_CALIB, SEISMO_CONFIG, SEISMO_MEASURE, SEISMO_RESULT };
SeismoState seismoState = SEISMO_IDLE; // Máquina de estados interna para o modo sismógrafo
float restAmplitude = 0.0;     // Amplitude média de repouso (tara) para calibrar o sensor
int seismoDuration = 10;       // Duração da medição em segundos (ajustável)
float amplitude = 0;           // Amplitude média da vibração medida
float frequency = 0;           // Frequência média da vibração medida
unsigned long vibDuration = 0; // Duração total da vibração em milissegundos
float magnitude = 0;           // Magnitude calculada da vibração
// Variáveis auxiliares para cálculos no modo sismógrafo
unsigned long seismoTimer = 0;
float amplitudeSum = 0.0;
float frequencySum = 0.0;
int seismoSamples = 0;
unsigned long vibStart = 0;
// Variáveis para cálculo de frequência por eixo
float lastXYZ[3] = {0, 0, 0};
unsigned long lastCrossTime[3] = {0, 0, 0};

// Variáveis do Modo Teste
unsigned long testStartTime = 0; // Timestamp do início do teste

// --- BITMAPS PARA O DISPLAY OLED (ARMAZENADOS NA MEMÓRIA DE PROGRAMA) ---
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

// --- FUNÇÕES DE LÓGICA ---

/**
 * @brief Atualiza o intervalo de flash e salva o FPM na memória.
 *
 * Calcula o intervalo em microssegundos com base no valor de FPM,
 * aplica o ajuste de fase e salva o valor de FPM na memória NVS para
 * que seja recuperado na próxima inicialização.
 */
void updateInterval() {
  fpm = constrain(fpm, minFPM, maxFPM); // Garante que o FPM esteja dentro dos limites
  intervalMicros = 60000000UL / fpm; // 60 segundos * 1.000.000 micros/seg / fpm
  phaseDelay = (intervalMicros * phase) / 359; // Calcula o delay de fase

  // Salva o FPM atual na memória não-volátil
  prefs.begin("config", false); // Abre o namespace "config"
  prefs.putInt("fpm", fpm);     // Salva o valor
  prefs.end();                  // Fecha o namespace
}

/**
 * @brief Atualiza o display OLED com as informações do modo atual.
 *
 * Limpa a tela e redesenha a interface de acordo com o modo de operação
 * e o estado atual do sistema (menu, ajuste de fase, etc.).
 * @note Esta função possui um controle de tempo para não ser chamada excessivamente.
 */
void updateDisplay() {
  // Limita a taxa de atualização para evitar flickering e consumo de CPU
  static unsigned long lastUpdate = 0;
  if (millis() - lastUpdate < 50) return; // Atualiza a cada 50ms (20 FPS)
  lastUpdate = millis();

  display.clearDisplay();
  display.setTextSize(1);
  display.setTextColor(SSD1306_WHITE);

  // --- Cabeçalho Padrão ---
  display.setCursor(0, 0);
  display.print("Modo: ");
  switch (currentMode) {
    case MODE_FREQUENCY:   display.print("Estrobo"); break;
    case MODE_LANTERN:     display.print("Lanterna"); break;
    case MODE_RPM:         display.print("RPM"); break;
    case MODE_SEISMOGRAPH: display.print("Sismografo"); break;
    case MODE_TEST:        display.print("Teste"); break;
    case MODE_ABOUT:       display.print("Sobre"); break;
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

  display.display(); // Envia o buffer para o display
}


// --- INTERRUPÇÕES (ISRs) ---

/**
 * @brief ISR (Interrupt Service Routine) para o encoder rotativo.
 *
 * Chamada na borda de descida do pino A do encoder. Lê o pino B para
 * determinar a direção da rotação e atualiza `encoderValue`.
 * @note ISRs devem ser o mais rápidas possível. Evite usar `delay()` ou `Serial.print()`.
 */
void IRAM_ATTR handleEncoder() {
  unsigned long now = micros();
  // Debounce de software para a interrupção
  if (now - lastEncoderInterrupt > encoderDebounce) {
    if (digitalRead(encoderPinB) == LOW) {
      encoderValue++; // Sentido horário
    } else {
      encoderValue--; // Sentido anti-horário
    }
    lastEncoderInterrupt = now;
  }
}

/**
 * @brief ISR (Interrupt Service Routine) para o sensor de RPM.
 *
 * Chamada na borda de descida do sinal do sensor IR. Calcula o
 * intervalo de tempo desde o último pulso para determinar a velocidade.
 */
void IR_sensor_ISR() {
  unsigned long now = micros();
  pulseInterval = now - lastSensorTime;
  lastSensorTime = now;
}


// --- FUNÇÕES DE SETUP ---

/**
 * @brief Configura os pinos do encoder e anexa a interrupção.
 */
void setupEncoder() {
  pinMode(encoderPinA, INPUT_PULLUP);
  pinMode(encoderPinB, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(encoderPinA), handleEncoder, FALLING);
}

/**
 * @brief Configura os pinos dos botões como entrada com pull-up interno.
 */
void setupButtons() {
  pinMode(buttonDouble, INPUT_PULLUP);
  pinMode(buttonHalf, INPUT_PULLUP);
  pinMode(buttonMode, INPUT_PULLUP);
  pinMode(buttonSet, INPUT_PULLUP);
}


// --- FUNÇÕES DE APOIO ---

/**
 * @brief Ajusta o FPM multiplicando-o por um fator.
 * @param factor Fator de multiplicação (ex: 2.0 para dobrar, 0.5 para metade).
 */
void adjustFPM(float factor) {
  fpm = constrain(fpm * factor, minFPM, maxFPM);
  updateInterval(); // Recalcula e salva
  updateDisplay();  // Mostra o novo valor
}

/**
 * @brief Executa a sequência de teste de hardware.
 *
 * Varre o LED por uma faixa de frequências para um teste visual rápido.
 */
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


// --- FUNÇÃO SETUP PRINCIPAL ---

/**
 * @brief Função de inicialização principal.
 *
 * Executada uma vez na inicialização. Configura todos os periféricos,
 * inicializa a comunicação, carrega configurações salvas e mostra a tela de splash.
 */
void setup() {
  // Inicializa o pino do LED
  pinMode(ledPin, OUTPUT);
  digitalWrite(ledPin, LOW);

  // Inicializa o pino do sensor IR
  pinMode(sensorIRPin, INPUT);

  // Inicia a comunicação I2C com os pinos corretos
  Wire.begin(21, 22); // SDA = 21, SCL = 22

  // Recupera o último FPM salvo na memória
  prefs.begin("config", true); // Abre em modo de leitura
  fpm = prefs.getInt("fpm", 30); // Carrega 'fpm', usa 30 como padrão se não existir
  prefs.end();
  
  updateInterval(); // Calcula o intervalo inicial com base no FPM carregado

  // Configura os periféricos de entrada
  setupEncoder();
  setupButtons();
  attachInterrupt(digitalPinToInterrupt(sensorIRPin), IR_sensor_ISR, FALLING);
  
  // Inicializa o display OLED
  if(!display.begin(SSD1306_SWITCHCAPVCC, 0x3C)) { 
    // Serial.println(F("Falha ao inicializar SSD1306")); // Descomente para debug
    while(1); // Trava se o display não for encontrado
  }
  
  // Mostra a tela de splash
  display.clearDisplay();
  display.drawBitmap(0, 0, bitmap, SCREEN_WIDTH, SCREEN_HEIGHT, SSD1306_WHITE);
  display.display();
  delay(3000); // Exibe a tela de splash por 3 segundos

  // Inicializa o acelerômetro
  adxlAvailable = accel.begin();
  if (adxlAvailable) {
    accel.setRange(ADXL345_RANGE_16_G); // Configura o range de medição
  }

  updateDisplay(); // Exibe a tela do modo inicial
}

// --- LOOP PRINCIPAL ---

/**
 * @brief Loop principal do programa.
 *
 * Executado repetidamente. Gerencia a leitura dos botões, do encoder,
 * a lógica do modo de operação atual e a atualização do display.
 */
void loop() {
  unsigned long now = millis();
  static bool pulseActive = false;
  static unsigned long pulseStartTime = 0;
  const unsigned int pulseDurationMicros = 100;

  // --- LEITURA DOS BOTÕES COM DEBOUNCE ---
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
    } else if(currentMode == MODE_RPM){
      // Salva o FPM atual na memória não-volátil com o valor de RPM
      fpm = rpmValue; //FPM recebe o valor de RPM
      updateInterval(); //Atualiza o interval com novos dados
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

  // --- LEITURA DO ENCODER ---
  if (encoderValue != lastEncoderValue) {
    int delta = encoderValue - lastEncoderValue; // Calcula a variação
    lastEncoderValue = encoderValue;

    if (menuActive) {
      // Navega pelo menu
      menuIndex = (menuIndex + delta + 6) % 6; // O "+6" evita resultados negativos no módulo
    } else if (phaseSetting && currentMode == MODE_FREQUENCY) {
      // Ajusta a fase
      phase = (phase + delta + 360) % 360;
      updateInterval();
    } else if (currentMode == MODE_FREQUENCY) {
      // Ajusta o FPM
      fpm += delta;
      updateInterval();
    } else if (currentMode == MODE_SEISMOGRAPH && seismoState == SEISMO_CONFIG) {
      // Ajusta a duração da medição no sismógrafo
      seismoDuration = constrain(seismoDuration + delta, 10, 60);
    }
    updateDisplay(); // Atualiza a tela após qualquer mudança
  }

  // --- MÁQUINA DE ESTADOS PRINCIPAL (EXECUÇÃO DO MODO ATUAL) ---
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

    case MODE_RPM: { // Chaves criam um escopo local para a variável
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
	  digitalWrite(ledPin, LOW);
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
      seismoState = SEISMO_IDLE;
	  digitalWrite(ledPin, LOW);
      break;
  }
}
