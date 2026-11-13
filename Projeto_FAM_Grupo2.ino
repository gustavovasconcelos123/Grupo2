/*
  WALKIE TALKIE ESP32 - VERSAO REVISADA DAS SPRINTS
  Placa: RoboCore AM-032 / ESP32-WROOM-32
  Ambiente: Arduino IDE com Arduino-ESP32 Core 3.x

  Sprint 1: OLED, icones, medida de tensao, media movel.
  Sprint 2: MAC, ESP-NOW, PING/PONG, RSSI, RTT e telemetria.
  Sprint 3: pinagem do INMP441 e VU Meter via I2S RX.
  Sprint 4: REMOVIDA (sem amplificador ou alto-falante).
  Sprint 5: PTT, controle TX/RX, PCM sem fragmentacao e buffer simples.
  Sprint 6: relatorio, fora do firmware.

  IMPORTANTE:
  - Sem MAX98357A NAO E POSSIVEL OUVIR A VOZ recebida.
  - No MODO_AUDIO 2, os pacotes PCM sao recebidos e mostrados em VU remoto.
  - Os modos 1 e 2 precisam de microfone INMP441 real.
  - Modo 0 permite validar interface / PTT / ESP-NOW sem o microfone.
*/

#include <Arduino.h>
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#include <esp_idf_version.h>
#include <Wire.h>
#include <Adafruit_GFX.h>
#include <Adafruit_SSD1306.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <freertos/queue.h>

// =============================================================
// 1. PARAMETROS DO PROJETO
// =============================================================
// 0: sem microfone, ideal para validar a placa e o Wokwi.
// 1: microfone INMP441 com VU Meter local, sem enviar audio.
// 2: microfone + envio PCM via ESP-NOW + VU remoto, SEM SOM.
#define MODO_AUDIO 0
#if MODO_AUDIO < 0 || MODO_AUDIO > 2
#error "MODO_AUDIO deve estar entre 0 e 2 nesta versao"
#endif

#if MODO_AUDIO != 0
#include "driver/i2s_std.h"
#endif

#define PIN_OLED_SDA 5
#define PIN_OLED_SCL 4
#define OLED_ADDR 0x3C
#define PIN_BAT 34
#define PIN_PTT 21
#define PIN_LED 16
#define PIN_I2S_BCLK 18
#define PIN_I2S_WS 19
#define PIN_MIC_SD 23

// Se 0, o divisor esta ligado ao VCC APENAS PARA TESTE: nao
// mostramos uma porcentagem falsa para a bateria 18650.
// Mude para 1 SOMENTE quando R1 estiver ligado ao BAT+ real,
// com alimentacao/protecao adequadas e circuito conferido.
#define MEDIR_BATERIA_REAL 0

// Ajuste com os MACs REAIS, impressos no Monitor Serial.
// Na placa A: colocar o MAC da placa B; na B: MAC da A.
uint8_t MAC_OUTRO[6] = {0xAA, 0xBB, 0xCC, 0xDD, 0xEE, 0xFF};
#define CANAL_ESPNOW 1

#define TAXA_AUDIO 16000
#define AMOSTRAS_PACOTE 120
#define GANHO_MIC 4
#define FILA_AUDIO_TAMANHO 16
#define PREBUFFER_PACOTES 3
#define N_AMOSTRAS_BAT 16
#define DEBOUNCE_MS 35UL

const float R1 = 33000.0f;
const float R2 = 10000.0f;
const float FATOR_CAL = 1.00f;   // ajuste manual, nao e autocalibracao
const float BAT_MIN = 0.0f;
const float BAT_MAX = 4.2f;

Adafruit_SSD1306 display(128, 64, &Wire, -1);

// =============================================================
// 2. MENSAGENS DO RADIO E FILAS
// =============================================================
enum TipoMensagem : uint8_t {
  TIPO_PING = 1,
  TIPO_PONG = 2,
  TIPO_PTT_ON = 3,
  TIPO_PTT_OFF = 4,
  TIPO_AUDIO = 5
};

struct __attribute__((packed)) PacoteControle {
  uint8_t tipo;
  uint8_t versao;
  uint16_t seq;
  uint32_t tempo;
  float tensao;
};

struct __attribute__((packed)) PacoteAudio {
  uint8_t tipo;
  uint16_t seq;
  uint8_t quantidade;
  int16_t pcm[AMOSTRAS_PACOTE];
};

struct EventoControle {
  PacoteControle pacote;
  int8_t rssi;
};

static_assert(sizeof(PacoteControle) == 12, "Tamanho de controle inesperado");
static_assert(sizeof(PacoteAudio) == 244, "Audio deve ter 244 bytes");
static_assert(sizeof(PacoteAudio) <= 250, "Pacote supera ESP-NOW v1");

QueueHandle_t filaControle = nullptr;
#if MODO_AUDIO == 2
QueueHandle_t filaAudio = nullptr;
#endif

// =============================================================
// 3. ESTADOS DO SISTEMA
// =============================================================
uint32_t valoresBat[N_AMOSTRAS_BAT] = {};
uint64_t somaBat = 0;
uint8_t posBat = 0;
float tensaoLocal = 0.0f;
float tensaoRemota = 0.0f;

bool radioPronto = false;
uint8_t macLocal[6] = {};
volatile uint32_t ultimoRx = 0;
volatile uint32_t ultimoPTTRemoto = 0; // evita travar ESCUTANDO se PTT_OFF se perder
volatile int ultimoRSSI = -100;
uint32_t rttMs = 0;
uint16_t sequenciaControle = 0;
uint16_t seqUltimoPing = 0;
uint32_t tempoUltimoPing = 0;
uint32_t ultimoPing = 0;
uint32_t ultimoHeartbeat = 0;

// O callback de envio so libera este sinalizador.
portMUX_TYPE muxTX = portMUX_INITIALIZER_UNLOCKED;
volatile bool txOcupado = false;
volatile uint32_t falhasTX = 0;
volatile uint32_t audioDescartado = 0;
volatile uint32_t audioRecebido = 0;
volatile uint32_t estouroFila = 0;

volatile bool mudouPTT = false;
bool botaoFisico = false;
volatile bool pttLocal = false;
volatile bool pttRemoto = false;
uint32_t ultimoRebote = 0;
uint8_t comandoPTTPendente = 0;

bool pongPendente = false;
PacoteControle mensagemPong = {};
uint16_t sequenciaAudio = 0;
volatile uint16_t rmsMic = 0;
volatile uint16_t rmsRemoto = 0;
bool micPronto = false;

uint32_t ultimaBateria = 0;
uint32_t ultimaTela = 0;
uint32_t ultimoLog = 0;

#if MODO_AUDIO != 0
i2s_chan_handle_t canalRX = nullptr;
#endif

// =============================================================
// 4. MEDICAO DE TENSAO (SPRINT 1)
// =============================================================
void inicializarMediaMovel() {
  somaBat = 0;
  posBat = 0;
  for (int i = 0; i < N_AMOSTRAS_BAT; i++) {
    valoresBat[i] = analogReadMilliVolts(PIN_BAT);
    somaBat += valoresBat[i];
    delay(3);
  }
}

float lerTensao() {
  somaBat -= valoresBat[posBat];
  valoresBat[posBat] = analogReadMilliVolts(PIN_BAT);
  somaBat += valoresBat[posBat];
  posBat = (posBat + 1) % N_AMOSTRAS_BAT;
  float voltsADC = (somaBat / (float)N_AMOSTRAS_BAT) / 1000.0f;
  return voltsADC * ((R1 + R2) / R2) * FATOR_CAL;
}

int percentualBateria(float volts) {
  float percentual = 100.0f * (volts - BAT_MIN) / (BAT_MAX - BAT_MIN);
  return constrain((int)lroundf(percentual), 0, 100);
}

// =============================================================
// 5. ICONES E VU METER (SPRINTS 1 E 3)
// =============================================================
void iconeBateria(int x, int y, int percentual, bool valido) {
  display.drawRect(x, y, 17, 9, SSD1306_WHITE);
  display.fillRect(x + 17, y + 2, 2, 5, SSD1306_WHITE);
  if (valido) {
    int barras = map(constrain(percentual, 0, 100), 0, 100, 0, 13);
    if (barras > 0) display.fillRect(x + 2, y + 2, barras, 5, SSD1306_WHITE);
  }
}

void iconeSinal(int x, int y, int rssi, bool online) {
  int numero = !online ? 0 :
    rssi >= -55 ? 4 : rssi >= -65 ? 3 : rssi >= -75 ? 2 : rssi >= -85 ? 1 : 0;
  for (int i = 0; i < 4; i++) {
    int altura = 3 + 2 * i;
    int posY = y + 9 - altura;
    if (i < numero) display.fillRect(x + i * 4, posY, 3, altura, SSD1306_WHITE);
    else display.drawRect(x + i * 4, posY, 3, altura, SSD1306_WHITE);
  }
}

void iconeMicrofone(int x, int y) {
  display.fillRoundRect(x + 3, y, 6, 9, 3, SSD1306_WHITE);
  display.drawCircleHelper(x + 6, y + 6, 5, 0x6, SSD1306_WHITE);
  display.drawLine(x + 6, y + 11, x + 6, y + 14, SSD1306_WHITE);
  display.drawLine(x + 3, y + 14, x + 9, y + 14, SSD1306_WHITE);
}

void barraVU(int x, int y, int largura, uint16_t rms) {
  // Escala inicial de teste: precisa ser ajustada com o INMP441 real.
  const int RMS_MAX_TELA = 5000;
  int preenchimento = map(constrain((int)rms, 0, RMS_MAX_TELA),
                          0, RMS_MAX_TELA, 0, largura - 4);
  display.drawRect(x, y, largura, 8, SSD1306_WHITE);
  if (preenchimento > 0) {
    display.fillRect(x + 2, y + 2, preenchimento, 4, SSD1306_WHITE);
  }
}

bool radioOnline() {
  return radioPronto && ultimoRx != 0 && (millis() - ultimoRx < 3500);
}

void desenharTela() {
  bool online = radioOnline();
  display.clearDisplay();
  display.setTextColor(SSD1306_WHITE);
  display.setTextSize(1);

  iconeMicrofone(1, 0);
  iconeSinal(51, 0, ultimoRSSI, online);
  iconeBateria(105, 0, percentualBateria(tensaoLocal), true);
  display.setCursor(0, 15);
  if (pttLocal) display.print("FALANDO / TX");
  else if (pttRemoto && online) display.print("ESCUTANDO / RX");
  else {
    // Mostra o MAC Wi-Fi STA desta placa, usado pelo ESP-NOW.
    // 21 caracteres em fonte tamanho 1 = 126 pixels (cabe no OLED de 128 px).
    display.printf("MAC:%02X:%02X:%02X:%02X:%02X:%02X",
                   (unsigned)macLocal[0], (unsigned)macLocal[1],
                   (unsigned)macLocal[2], (unsigned)macLocal[3],
                   (unsigned)macLocal[4], (unsigned)macLocal[5]);
  }

  display.setCursor(0, 25);
#if MEDIR_BATERIA_REAL
  display.printf("Bat: %.2fV %d%%", tensaoLocal, percentualBateria(tensaoLocal));
#else
  display.printf("VCC: %.2fV (teste)", tensaoLocal);
#endif

  display.setCursor(0, 35);
  if (!radioPronto) display.print("Radio: ERRO");
  else if (online) display.print("Radio: ONLINE");
  else display.print("Radio: aguardando");

  display.setCursor(0, 45);
  if (online) {
    display.printf("Rem:%.1fV RSSI:%d", tensaoRemota, (int)ultimoRSSI);
  } else {
    display.print("Rem:-- RSSI:--");
  }

  display.setCursor(0, 55);
  display.printf("RTT:%lums", (unsigned long)rttMs);
#if MODO_AUDIO != 0
#if MODO_AUDIO == 2
  uint16_t volume = (online && pttRemoto) ? rmsRemoto : rmsMic;
#else
  uint16_t volume = rmsMic;
#endif
  barraVU(65, 55, 62, volume);
#else
  display.print(" PTT GPIO21");
#endif
  display.display();
}

// =============================================================
// 6. ENVIO ESP-NOW (SPRINT 2)
// =============================================================
bool enviarBytes(const void *dados, size_t tamanho) {
  if (!radioPronto || tamanho > 250) return false;

  portENTER_CRITICAL(&muxTX);
  if (txOcupado) {
    portEXIT_CRITICAL(&muxTX);
    return false;
  }
  txOcupado = true;
  portEXIT_CRITICAL(&muxTX);

  esp_err_t resultado = esp_now_send(MAC_OUTRO, (const uint8_t *)dados, tamanho);
  if (resultado != ESP_OK) {
    portENTER_CRITICAL(&muxTX);
    txOcupado = false;
    portEXIT_CRITICAL(&muxTX);
    falhasTX++;
    return false;
  }
  return true;
}

bool enviarControle(uint8_t tipo, uint16_t seq = 0, uint32_t tempo = 0) {
  PacoteControle p = {};
  p.tipo = tipo;
  p.versao = 1;
  p.tensao = tensaoLocal;
  if (tipo == TIPO_PONG) {
    p.seq = seq;
    p.tempo = tempo; // devolve a marca original de tempo
  } else {
    p.seq = ++sequenciaControle;
    p.tempo = millis();
  }
  if (enviarBytes(&p, sizeof(p))) {
    if (tipo == TIPO_PING) {
      seqUltimoPing = p.seq;
      tempoUltimoPing = p.tempo;
    }
    return true;
  }
  return false;
}

// =============================================================
// 7. CALLBACKS MINIMOS - EXIGENCIA TECNICA DO ESP-NOW
// =============================================================
// Nao sao mais uma entrega separada da Sprint 2.
// Ainda sao necessarios para receber pacotes e liberar envios.
#if ESP_IDF_VERSION >= ESP_IDF_VERSION_VAL(5, 5, 0)
void aoEnviar(const wifi_tx_info_t *info, esp_now_send_status_t status) {
#else
void aoEnviar(const uint8_t *mac, esp_now_send_status_t status) {
#endif
  if (status != ESP_NOW_SEND_SUCCESS) falhasTX++;
  portENTER_CRITICAL(&muxTX);
  txOcupado = false;
  portEXIT_CRITICAL(&muxTX);
}

void aoReceber(const esp_now_recv_info_t *info, const uint8_t *dados, int tamanho) {
  if (!info || !dados || memcmp(info->src_addr, MAC_OUTRO, 6) != 0) return;
  int rssi = (info->rx_ctrl != nullptr) ? info->rx_ctrl->rssi : -100;

  if (tamanho == (int)sizeof(PacoteControle)) {
    PacoteControle p;
    memcpy(&p, dados, sizeof(p));
    if (p.versao != 1 || p.tipo < TIPO_PING || p.tipo > TIPO_PTT_OFF) return;
    EventoControle evento = {p, (int8_t)rssi};
    if (filaControle && xQueueSend(filaControle, &evento, 0) == pdTRUE) {
      ultimoRSSI = rssi;
      ultimoRx = millis();
    }
  }
#if MODO_AUDIO == 2
  else if (tamanho == (int)sizeof(PacoteAudio)) {
    PacoteAudio p;
    memcpy(&p, dados, sizeof(p));
    if (p.tipo != TIPO_AUDIO || p.quantidade != AMOSTRAS_PACOTE) return;
    if (filaAudio && xQueueSend(filaAudio, &p, 0) == pdTRUE) {
      audioRecebido++;
      pttRemoto = true; // mesmo se o PTT_ON se perdeu
      ultimoPTTRemoto = millis();
      ultimoRSSI = rssi;
      ultimoRx = millis();
    } else {
      estouroFila++;
    }
  }
#endif
}

// =============================================================
// 8. PROCESSAR MENSAGENS E PING/PONG
// =============================================================
void processarMensagens() {
  EventoControle evento;
  while (filaControle && xQueueReceive(filaControle, &evento, 0) == pdTRUE) {
    PacoteControle p = evento.pacote;
    ultimoRSSI = evento.rssi;
    tensaoRemota = p.tensao;

    if (p.tipo == TIPO_PING) {
      // Pode estar ocupado enviando; armazena PONG para o loop tentar.
      pongPendente = true;
      mensagemPong = p;
    } else if (p.tipo == TIPO_PONG) {
      if (p.seq == seqUltimoPing && p.tempo == tempoUltimoPing) {
        rttMs = millis() - p.tempo;
        Serial.printf("PONG #%u | RTT=%lums | RSSI=%ddBm\n",
                      p.seq, (unsigned long)rttMs, (int)ultimoRSSI);
      }
    } else if (p.tipo == TIPO_PTT_ON) {
      pttRemoto = true;
      ultimoPTTRemoto = millis();
    } else if (p.tipo == TIPO_PTT_OFF) {
      pttRemoto = false;
      ultimoPTTRemoto = 0;
      rmsRemoto = 0;
#if MODO_AUDIO == 2
      if (filaAudio) xQueueReset(filaAudio);
#endif
    }
  }
}

// =============================================================
// 9. PTT COM INTERRUPCAO E DEBOUNCE (SPRINT 5)
// =============================================================
void IRAM_ATTR interrupcaoPTT() {
  mudouPTT = true;
}

// Se houver acionamento simultaneo, o menor MAC tem prioridade.
bool euTenhoPrioridade() {
  return memcmp(macLocal, MAC_OUTRO, 6) < 0;
}

void atualizarPTT() {
  uint32_t agora = millis();
  if (mudouPTT) {
    mudouPTT = false;
    ultimoRebote = agora;
  }
  if (agora - ultimoRebote < DEBOUNCE_MS) return;
  bool pressionado = (digitalRead(PIN_PTT) == LOW);
  if (pressionado != botaoFisico) {
    botaoFisico = pressionado;
    Serial.println(pressionado ? "PTT PRESSIONADO" : "PTT LIBERADO");
  }

  bool possoTransmitir = botaoFisico && (!pttRemoto || euTenhoPrioridade());
  if (possoTransmitir == pttLocal) return;

  pttLocal = possoTransmitir;
  digitalWrite(PIN_LED, pttLocal ? HIGH : LOW);
  comandoPTTPendente = pttLocal ? TIPO_PTT_ON : TIPO_PTT_OFF;
  sequenciaAudio = 0;
  if (pttLocal) ultimoHeartbeat = agora;
}

// =============================================================
// 10. RADIO ESP-NOW (SPRINT 2)
// =============================================================
void iniciarRadio() {
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();
  WiFi.setSleep(false);

  // Os dois ESP32 devem operar no mesmo canal.
  esp_wifi_set_channel(CANAL_ESPNOW, WIFI_SECOND_CHAN_NONE);
  esp_wifi_get_mac(WIFI_IF_STA, macLocal);

  Serial.print("MAC DESTA PLACA: ");
  Serial.println(WiFi.macAddress());
  if (MAC_OUTRO[0] == 0xAA && MAC_OUTRO[1] == 0xBB) {
    Serial.println("ATENCAO: troque MAC_OUTRO pelo MAC REAL do outro ESP32!");
  }

  if (esp_now_init() != ESP_OK) {
    Serial.println("ERRO ao iniciar ESP-NOW");
    return;
  }
  if (esp_now_register_send_cb(aoEnviar) != ESP_OK ||
      esp_now_register_recv_cb(aoReceber) != ESP_OK) {
    Serial.println("ERRO ao registrar callbacks internos do ESP-NOW");
    return;
  }

  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, MAC_OUTRO, 6);
  peer.channel = CANAL_ESPNOW;
  peer.ifidx = WIFI_IF_STA;
  peer.encrypt = false;
  if (esp_now_add_peer(&peer) != ESP_OK) {
    Serial.println("ERRO ao cadastrar peer");
    return;
  }
  radioPronto = true;
  Serial.println("ESP-NOW inicializado; aguardando mensagens do outro ESP32.");
}

// =============================================================
// 11. MICROFONE INMP441 E VU METER (SPRINT 3)
// =============================================================
#if MODO_AUDIO != 0
void iniciarMicrofone() {
  i2s_chan_config_t chanCfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_0, I2S_ROLE_MASTER);
  esp_err_t erro = i2s_new_channel(&chanCfg, nullptr, &canalRX);
  if (erro != ESP_OK) {
    Serial.printf("ERRO criar canal I2S RX: %d\n", (int)erro);
    return;
  }

  i2s_std_config_t cfg = {};
  cfg.clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(TAXA_AUDIO);
  cfg.slot_cfg = I2S_STD_PHILIPS_SLOT_DEFAULT_CONFIG(
    I2S_DATA_BIT_WIDTH_32BIT, I2S_SLOT_MODE_STEREO);
  cfg.slot_cfg.slot_mask = I2S_STD_SLOT_BOTH; // INMP441 L/R aterrado: canal esquerdo
  cfg.gpio_cfg.mclk = I2S_GPIO_UNUSED;
  cfg.gpio_cfg.bclk = (gpio_num_t)PIN_I2S_BCLK;
  cfg.gpio_cfg.ws = (gpio_num_t)PIN_I2S_WS;
  cfg.gpio_cfg.dout = I2S_GPIO_UNUSED;
  cfg.gpio_cfg.din = (gpio_num_t)PIN_MIC_SD;

  erro = i2s_channel_init_std_mode(canalRX, &cfg);
  if (erro != ESP_OK) {
    Serial.printf("ERRO configurar I2S RX: %d\n", (int)erro);
    return;
  }
  erro = i2s_channel_enable(canalRX);
  if (erro != ESP_OK) {
    Serial.printf("ERRO habilitar I2S RX: %d\n", (int)erro);
    return;
  }
  micPronto = true;
  Serial.println("INMP441 I2S RX pronto: 16kHz / slots 32 bits.");
}

void tarefaMicrofone(void *parametro) {
  static int32_t bruto[AMOSTRAS_PACOTE * 2]; // L/R, 32 bits por slot
  PacoteAudio pacote = {};
  pacote.tipo = TIPO_AUDIO;
  pacote.quantidade = AMOSTRAS_PACOTE;

  while (true) {
    if (!micPronto) {
      vTaskDelay(pdMS_TO_TICKS(30));
      continue;
    }
    size_t lidos = 0;
    esp_err_t erro = i2s_channel_read(
      canalRX, bruto, sizeof(bruto), &lidos, pdMS_TO_TICKS(80));
    if (erro != ESP_OK || lidos < 8) {
      vTaskDelay(pdMS_TO_TICKS(5));
      continue;
    }

    size_t quantidade = min((size_t)AMOSTRAS_PACOTE, lidos / (2 * sizeof(int32_t)));
    uint64_t somaQuadrados = 0;
    for (size_t i = 0; i < AMOSTRAS_PACOTE; i++) {
      int32_t valor = 0;
      if (i < quantidade) {
        // Primeiro slot = canal esquerdo, porque INMP441 L/R esta em GND.
        valor = (bruto[2 * i] >> 16) * GANHO_MIC;
        valor = constrain(valor, -32768L, 32767L);
      }
      pacote.pcm[i] = (int16_t)valor;
      somaQuadrados += (uint64_t)((int64_t)valor * valor);
    }
    rmsMic = (uint16_t)sqrt((double)somaQuadrados / AMOSTRAS_PACOTE);

#if MODO_AUDIO == 2
    if (pttLocal && radioPronto) {
      pacote.seq = sequenciaAudio++;
      if (!enviarBytes(&pacote, sizeof(pacote))) audioDescartado++;
    }
#endif
  }
}
#endif

// =============================================================
// 12. BUFFER SIMPLES DE RECEPCAO (SPRINT 5)
// =============================================================
// Nao fragmenta PCM: cada pacote ja tem 244 bytes (<=250).
// Nesta versao, o buffer permite VU remoto, NAO reproducao de voz.
#if MODO_AUDIO == 2
void tarefaAudioRemoto(void *parametro) {
  PacoteAudio p;
  bool iniciou = false;
  while (true) {
    if (!pttRemoto) {
      rmsRemoto = 0;
      iniciou = false;
      vTaskDelay(pdMS_TO_TICKS(10));
      continue;
    }
    if (!iniciou) {
      if (uxQueueMessagesWaiting(filaAudio) < PREBUFFER_PACOTES) {
        vTaskDelay(pdMS_TO_TICKS(3));
        continue;
      }
      iniciou = true;
    }
    if (xQueueReceive(filaAudio, &p, pdMS_TO_TICKS(35)) == pdTRUE) {
      uint64_t soma = 0;
      for (int i = 0; i < AMOSTRAS_PACOTE; i++) {
        int32_t s = p.pcm[i];
        soma += (uint64_t)((int64_t)s * s);
      }
      rmsRemoto = (uint16_t)sqrt((double)soma / AMOSTRAS_PACOTE);
      vTaskDelay(pdMS_TO_TICKS(8)); // consumo aproximado de um bloco de 7,5ms
    } else {
      rmsRemoto = 0;
      iniciou = false; // precisara novamente do prebuffer
    }
  }
}
#endif

// =============================================================
// 13. SETUP
// =============================================================
void setup() {
  Serial.begin(115200);
  delay(300);
  Serial.println("\n=== WALKIE TALKIE ESP32 | SPRINTS REVISADAS ===");
  Serial.printf("MODO_AUDIO = %d\n", MODO_AUDIO);

  pinMode(PIN_LED, OUTPUT);
  digitalWrite(PIN_LED, LOW);
  pinMode(PIN_PTT, INPUT_PULLUP);
  attachInterrupt(digitalPinToInterrupt(PIN_PTT), interrupcaoPTT, CHANGE);

  Wire.begin(PIN_OLED_SDA, PIN_OLED_SCL);
  if (!display.begin(SSD1306_SWITCHCAPVCC, OLED_ADDR)) {
    Serial.println("ERRO: OLED SSD1306 nao encontrado");
    while (true) delay(1000);
  }
  display.setRotation(2); // CORRECAO: tela da AM-032 girada em 180 graus
  display.clearDisplay();
  display.display();

  analogReadResolution(12);
  analogSetPinAttenuation(PIN_BAT, ADC_11db);
  inicializarMediaMovel();
  tensaoLocal = lerTensao();

#if !MEDIR_BATERIA_REAL
  Serial.println("ATENCAO: divisor em VCC de TESTE; nao informa carga real da 18650.");
#endif

  filaControle = xQueueCreate(16, sizeof(EventoControle));
  if (!filaControle) {
    Serial.println("ERRO: memoria insuficiente para fila de controle");
    while (true) delay(1000);
  }
#if MODO_AUDIO == 2
  filaAudio = xQueueCreate(FILA_AUDIO_TAMANHO, sizeof(PacoteAudio));
  if (!filaAudio) {
    Serial.println("ERRO: memoria insuficiente para fila de audio");
    while (true) delay(1000);
  }
#endif

  iniciarRadio();
  desenharTela();  // MAC ja foi lido antes da primeira exibicao.

#if MODO_AUDIO != 0
  iniciarMicrofone();
  if (micPronto) {
    if (xTaskCreatePinnedToCore(tarefaMicrofone, "INMP441", 4096, nullptr, 2,
                                nullptr, 1) != pdPASS) {
      micPronto = false;
      Serial.println("ERRO: tarefa de microfone");
    }
  }
#endif
#if MODO_AUDIO == 2
  if (xTaskCreatePinnedToCore(tarefaAudioRemoto, "VUremoto", 4096, nullptr, 1,
                              nullptr, 1) != pdPASS) {
    Serial.println("ERRO: tarefa de VU remoto");
  }
#endif
  Serial.println("Sistema Pronto!");
}

// =============================================================
// 14. LOOP PRINCIPAL
// =============================================================
void loop() {
  uint32_t agora = millis();
  atualizarPTT();
  processarMensagens();

  // Se o comando de liberacao se perdeu, expirar o estado remoto.
  // PTT_ON e pacotes PCM atualizam ultimoPTTRemoto; PING nao atualiza.
  if (pttRemoto && (!radioOnline() ||
      (ultimoPTTRemoto != 0 && agora - ultimoPTTRemoto > 1800))) {
    pttRemoto = false;
    rmsRemoto = 0;
  }

  // Controle pendente tem prioridade sobre telemetria e audio.
  if (radioPronto && comandoPTTPendente != 0) {
    if (enviarControle(comandoPTTPendente)) comandoPTTPendente = 0;
  }
  if (radioPronto && pongPendente && comandoPTTPendente == 0) {
    if (enviarControle(TIPO_PONG, mensagemPong.seq, mensagemPong.tempo)) {
      pongPendente = false;
    }
  }

  // Heartbeat do PTT: ajuda o receptor a acompanhar estados prolongados.
  if (radioPronto && pttLocal && comandoPTTPendente == 0 &&
      agora - ultimoHeartbeat >= 800) {
    if (enviarControle(TIPO_PTT_ON)) ultimoHeartbeat = agora;
  }

  if (radioPronto && !pttLocal && !pttRemoto &&
      comandoPTTPendente == 0 && !pongPendente &&
      agora - ultimoPing >= 1000) {
    if (enviarControle(TIPO_PING)) ultimoPing = agora;
  }

  if (agora - ultimaBateria >= 100) {
    ultimaBateria = agora;
    tensaoLocal = lerTensao();
  }
  if (agora - ultimaTela >= 200) {
    ultimaTela = agora;
    desenharTela();
  }
  if (agora - ultimoLog >= 5000) {
    ultimoLog = agora;
    Serial.printf("Vmed=%.2fV | Radio=%s | PTT=%s | Remoto=%s | RSSI=%d | RTT=%lu\n",
                  tensaoLocal, radioOnline() ? "ONLINE" : "OFFLINE",
                  pttLocal ? "TX" : "OFF", pttRemoto ? "RX" : "OFF",
                  (int)ultimoRSSI, (unsigned long)rttMs);
    Serial.printf("MIC=%s | VU=%u | VUremoto=%u | AudioRx=%lu | Desc=%lu | FilaCheia=%lu | TXfalhas=%lu\n",
                  micPronto ? "OK" : "OFF", (unsigned)rmsMic,
                  (unsigned)rmsRemoto, (unsigned long)audioRecebido,
                  (unsigned long)audioDescartado, (unsigned long)estouroFila,
                  (unsigned long)falhasTX);
  }
  delay(1);
}