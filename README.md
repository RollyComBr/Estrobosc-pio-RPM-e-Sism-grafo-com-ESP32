# Sismógrafo + Estroboscópio + RPM + Lanterna com ESP32

Este projeto é um sistema multifuncional baseado no ESP32, que combina:

- Medição de vibrações via acelerômetro ADXL345 (modo sismógrafo)
- Controle de estroboscópio com LED de alta frequência (modo FPM)
- Leitura de RPM via sensor infravermelho
- Lanterna LED
- Display OLED SSD1306 para interface

## Componentes Utilizados

- ESP32-WROOM
- ADXL345 (acelerômetro via I2C)
- Display OLED 128x64 SSD1306
- Sensor Infravermelho (reflexivo para RPM)
- LED Branco (estroboscópio e lanterna)
- Encoder rotativo com botão
- Botões extras para funções auxiliares

## Pinos Utilizados (ESP32)

| Função             | Pino ESP32 |
|--------------------|------------|
| SDA (I2C)          | GPIO 21    |
| SCL (I2C)          | GPIO 22    |
| Encoder A          | GPIO 16    |
| Encoder B          | GPIO 17    |
| Botão Dobrador     | GPIO 18    |
| Botão Divisor      | GPIO 19    |
| Botão Modo         | GPIO 5     |
| Botão Set/Ok       | GPIO 15    |
| LED Estrobo        | GPIO 2     |
| Sensor IR RPM      | GPIO 4     |

## Modos de Operação

### 1. Frequência (Estroboscópio)
- Controla a taxa de flashes por minuto (FPM)
- Permite ajustar a fase
- Botões para dobrar ou dividir a frequência

### 2. Lanterna
- Liga o LED de forma contínua

### 3. RPM
- Mede rotações por minuto com base no sensor IR
- Mostra valor no display em tempo real

### 4. Sismógrafo
- Mede vibrações em três eixos (X, Y, Z)
- Calcula frequência e amplitude média da vibração
- Mede a magnitude relativa

### 5. Teste
- Permite simulações e testes de funcionalidades

### 6. Sobre
- Mostra informações do projeto

## Armazenamento em Flash

Utiliza a biblioteca `Preferences` para salvar o valor de FPM entre reinicializações.

## Observações

- O sistema possui tratamento de debounce para botões e encoder
- As medições de frequência de vibração são baseadas em cruzamentos por zero em cada eixo
- O projeto foi adaptado de uma versão para Arduino Pro Mini, migrado para ESP32

## Bibliotecas Necessárias

Instale as seguintes bibliotecas via Arduino IDE:

- `Adafruit_ADXL345_U`
- `Adafruit_GFX`
- `Adafruit_SSD1306`
- `Preferences` (inclusa com ESP32)

## Autor

Projeto adaptado e expandido por Rolly Santos, Igor Gustavo, Mateus Paiva e Nycollas Luan.

## Worki
https://wokwi.com/projects/433161634932578305

---
