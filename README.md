# Dispositivo Multifuncional com ESP32 (Estroboscópio, Tacômetro, Sismógrafo)

Este projeto implementa um dispositivo portátil multifuncional baseado na plataforma ESP32. Ele combina diversas ferramentas úteis para análise de movimento, vibração e iluminação em um único aparelho, controlado por uma interface intuitiva com display OLED e encoder rotativo.

## ✨ Funcionalidades

O dispositivo opera em seis modos distintos:

1.  **Estroboscópio (`Estrobo`):**
    -   Gera flashes de luz com frequência ajustável (FPM - Flashes Por Minuto).
    -   Faixa de operação: **30 a 7500 FPM**.
    -   Permite o ajuste fino da **fase** (0-359°) para sincronizar o flash com o movimento do objeto.
    -   Ideal para "congelar" visualmente objetos em rotação ou vibração.

2.  **Lanterna (`Lanterna`):**
    -   Acende o LED principal de forma contínua, funcionando como uma lanterna de alta intensidade.

3.  **Tacômetro (`RPM`):**
    -   Mede a velocidade de rotação de um objeto em Rotações Por Minuto (RPM).
    -   Utiliza um sensor infravermelho (IR) para detectar passagens de uma marca reflexiva.

4.  **Sismógrafo / Vibrômetro (`Sismografo`):**
    -   Utiliza um acelerômetro ADXL345 para medir e analisar vibrações.
    -   Possui um ciclo de calibração, medição e exibição de resultados.
    -   Mede **Amplitude**, **Frequência (Hz)** e calcula uma **Magnitude** da vibração.

5.  **Modo de Teste (`Teste`):**
    -   Executa uma sequência de flashes em frequências variadas para verificar o funcionamento do LED e do sistema.

6.  **Sobre (`Sobre`):**
    -   Exibe os créditos dos desenvolvedores do projeto.

## 🛠️ Hardware Necessário

-   1x Placa de desenvolvimento **ESP32** (ex: NodeMCU-32S, DOIT DEVKIT V1)
-   1x Display **OLED 0.96" I2C SSD1306** (128x64 pixels)
-   1x Módulo Acelerômetro **ADXL345**
-   1x **Encoder Rotativo** com botão (KY-040 ou similar)
-   4x **Botões de pressão** (push-buttons)
-   1x **Sensor Infravermelho de Obstáculo/Reflexivo** (para o modo RPM)
-   1x **LED de alta potência** (com driver/transistor apropriado, se necessário)
-   Resistores, protoboard e jumpers.

## 🔌 Diagrama de Conexão (Pinout)

| Componente                    | Conexão no ESP32  |
| :---------------------------- | :---------------- |
| **Display OLED (I2C)**        |                   |
| SDA                           | GPIO 21           |
| SCL                           | GPIO 22           |
| **Acelerômetro ADXL345 (I2C)**|                   |
| SDA                           | GPIO 21           |
| SCL                           | GPIO 22           |
| **Encoder Rotativo**          |                   |
| CLK (Pino A)                  | GPIO 16           |
| DT (Pino B)                   | GPIO 17           |
| **Botões**                    |                   |
| Botão "Dobrar FPM"            | GPIO 18           |
| Botão "Metade FPM"            | GPIO 19           |
| Botão "Modo"                  | GPIO 5            |
| Botão "Set/Confirmar"         | GPIO 15           |
| **Atuadores e Sensores**      |                   |
| LED Principal                 | GPIO 2            |
| Sensor IR (Saída Digital)     | GPIO 4            |

*Nota: Todos os botões e o encoder devem ser conectados com resistores de pull-up (ou usar o `INPUT_PULLUP` interno do ESP32, como no código) para garantir leituras estáveis.*

## 📚 Bibliotecas Necessárias

Para compilar este código na IDE do Arduino, você precisará instalar as seguintes bibliotecas através do "Gerenciador de Bibliotecas":

-   `Adafruit GFX Library`
-   `Adafruit SSD1306`
-   `Adafruit ADXL345`
-   `Adafruit Unified Sensor`

A biblioteca `Preferences.h` já faz parte do core do ESP32 para Arduino.

## 🚀 Como Usar

A interface do dispositivo foi projetada para ser simples e intuitiva.

1.  **Ligar o Dispositivo:** Ao ligar, uma tela de splash é exibida por 3 segundos. O dispositivo então inicia no modo **Estroboscópio**, carregando o último valor de FPM salvo na memória.

2.  **Navegar entre Modos:**
    -   Pressione o botão **`Modo`** para abrir ou fechar o menu de seleção na parte inferior da tela.
    -   Com o menu aberto, **gire o encoder** para selecionar o modo desejado.
    -   Pressione o botão **`Set`** para confirmar e entrar no modo selecionado.

3.  **Controles Específicos:**
    -   **Modo Estroboscópio:**
        -   **Gire o encoder:** Ajusta o FPM de 1 em 1.
        -   **Botão `Dobrar`:** Multiplica o FPM por 2.
        -   **Botão `Metade`:** Divide o FPM por 2.
        -   **Botão `Set`:** Entra/sai do modo de ajuste de fase. No modo de fase, o encoder ajusta os graus.
    -   **Modo Lanterna:**
        -   **Botão `Set`:** Liga ou desliga o LED.
    -   **Modo Sismógrafo:**
        -   **Botão `Set`:** Inicia o processo (Calibrar -> Configurar -> Medir -> Mostrar Resultado).
        -   Na tela de configuração, **gire o encoder** para ajustar a duração da medição.
    -   **Modo Teste:**
        -   **Botão `Set`:** Inicia a sequência de teste.

---
**Desenvolvido pelo Grupo Alfa:**
*   Igor Gustavo
*   Nycollas Luan
*   Mateus Paiva
*   Rolly Santos
  
## Worki
https://wokwi.com/projects/433161634932578305

---
