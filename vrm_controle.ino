// Tulio Henrique Rozenthal Marques - TCC VRM Trifásico - UNIFEI 
//Firmware de controle PI para ESP32
#include <Arduino.h>
#include "driver/mcpwm.h"

// Pinos
const int PIN_PWM[3] = {16, 17, 23};
const int PIN_EN[3] = {19, 18, 5};
const int PIN_ISENSE[3] = {35, 34, 32};
const int PIN_VOUT = 33;

const mcpwm_io_signals_t SIG_PWM[3] = {MCPWM0A, MCPWM1A, MCPWM2A};
const mcpwm_timer_t TMR[3] = {MCPWM_TIMER_0, MCPWM_TIMER_1, MCPWM_TIMER_2};

// Parametros de hardware
const float VOUT_DIV = 1.6f;
const float ISENSE_DIV = 1.6f;
const float ACS_SENS = 0.100f; // ACS712-20A
const float DMAX_PCT = 65.0f;

const bool DUTY_EM_FRACAO = true;
const float ESC = DUTY_EM_FRACAO ? 100.0f : 1.0f;

// Ganhos do PI (0, 1F, 2F, 3F)
const float b0_tab[4] = {0.0f, 0.08545f, 0.08106f, 0.07973f};
const float b1_tab[4] = {0.0f, -0.08398f, -0.07910f, -0.07737f};

// Parametros do supervisor
const float I_ABERTA_REL = 0.25f;
const float I_MIN_MEDIA = 0.35f;
const int T_DETECTA_MS = 3;
const float I_TRIP = 8.0f;
const float I_REV = -1.5f;
const float I_LIM_FASE = 3.2f;
const float I_RET_FASE = 2.6f;
const float FOLD_V_MS = 0.0005f;
const float SOFTSTART_V_MS = 0.004f;
const float VOUT_OVP = 6.8f;
const float KI_BAL_PCT = 0.01f;
const float TRIM_MAX_PCT = 3.0f;

enum { ST_PARADO = 0, ST_NORMAL = 1, ST_DEGRADADO = 2 };
enum { CA_NENHUMA = 0, CA_ABERTA = 1, CA_OC = 2, CA_REV = 3, CA_OVP = 4 };

volatile bool rodando = false;
volatile uint8_t mask_ativa = 0x07;
volatile uint8_t mask_inj = 0;
volatile int n_ativas = 3;
volatile uint8_t estado = ST_PARADO;
volatile uint8_t causa = CA_NENHUMA;
volatile int fase_evento = 0;
volatile bool bal_on = true;
volatile float vref_cmd = 5.0f;
volatile float fold_v = 0.0f;
volatile float vref_eff = 5.0f;
volatile float vref_rampa = 0.0f;
volatile float trim_pct[3] = {0, 0, 0};
volatile float duty_pct[3] = {0, 0, 0};
volatile float i_f[3] = {0, 0, 0};
volatile float vout_f = 0;
volatile float vout_inst = 0;
volatile float vsum = 0;
volatile uint32_t vcnt = 0;
volatile uint32_t ovr = 0;
volatile uint32_t t_inj_us = 0;
volatile uint32_t t_det_us = 0;
volatile float u_pi = 0;
volatile float e_k1 = 0;

float i_off_v[3] = {1.5625f, 1.5625f, 1.5625f};
volatile float i_mv[3] = {1562.5f, 1562.5f, 1562.5f};

int cal_restante = 0;
float cal_soma[3] = {0, 0, 0};

volatile char cmd = 0;
volatile int cmd_k = 0;
volatile float cmd_val = 0;

volatile bool flag_msg_cal = false;
volatile bool flag_msg_ovp = false;
volatile bool flag_msg_iso = false;
volatile float msg_ovp_v = 0;
volatile int msg_iso_k = 0;
volatile int msg_iso_ca = 0;

TaskHandle_t hCtrl = NULL;
hw_timer_t *hTimer = NULL;

inline float clampf(float x, float lo, float hi) {
  if (x < lo) return lo;
  if (x > hi) return hi;
  return x;
}

// Buffer de captura 1 kHz
const int RING_N = 512;
const int CAP_N = 400;
const int CAP_PRE = 100;
const int CAP_POST = CAP_N - CAP_PRE;

struct Amostra {
  int16_t v;
  int16_t i[3];
};

Amostra ring[RING_N];
Amostra cap_buf[CAP_N];
uint32_t ring_cnt = 0;
uint32_t cap_fim = 0;
volatile bool cap_armada = false;
volatile bool cap_pronta = false;
uint32_t cap_ti = 0;
uint32_t cap_td = 0;
int cap_fk = 0;
float cap_vr = 5.0f;

// MCPWM 50 kHz com defasagem
void redistribui_fases() {
  uint8_t m = mask_ativa;
  int n = __builtin_popcount(m);
  if (n < 1) {
    n = 1;
  }

  mcpwm_sync_disable(MCPWM_UNIT_0, MCPWM_TIMER_0);

  int idx = 0;
  for (int k = 0; k < 3; k++) {
    int off = 0;
    if (m & (1 << k)) {
      off = (idx * 1000) / n;
      idx++;
    }
    if (k > 0) {
      mcpwm_sync_config_t sc;
      sc.sync_sig = MCPWM_SELECT_TIMER0_SYNC;
      sc.timer_val = off;
      sc.count_direction = MCPWM_TIMER_DIRECTION_UP;
      mcpwm_sync_configure(MCPWM_UNIT_0, TMR[k], &sc);
    }
  }
}

void setup_mcpwm() {
  for (int k = 0; k < 3; k++) {
    mcpwm_gpio_init(MCPWM_UNIT_0, SIG_PWM[k], PIN_PWM[k]);
  }

  mcpwm_config_t cfg;
  cfg.frequency = 50000;
  cfg.cmpr_a = 0.0;
  cfg.cmpr_b = 0.0;
  cfg.counter_mode = MCPWM_UP_COUNTER;
  cfg.duty_mode = MCPWM_DUTY_MODE_0;

  for (int k = 0; k < 3; k++) {
    mcpwm_init(MCPWM_UNIT_0, TMR[k], &cfg);
  }

  mcpwm_set_timer_sync_output(MCPWM_UNIT_0, MCPWM_TIMER_0, MCPWM_SWSYNC_SOURCE_TEZ);
  redistribui_fases();
}

// Malha de controle 10 kHz (Core 1)
void controle_passo() {
  static uint8_t sub = 0;

  float v = analogReadMilliVolts(PIN_VOUT) * 0.001f * VOUT_DIV;
  if (sub == 0) {
    i_mv[0] = (float)analogReadMilliVolts(PIN_ISENSE[0]);
  } else if (sub == 3) {
    i_mv[1] = (float)analogReadMilliVolts(PIN_ISENSE[1]);
  } else if (sub == 6) {
    i_mv[2] = (float)analogReadMilliVolts(PIN_ISENSE[2]);
  }
  sub = (sub + 1) % 10;

  vout_inst = v;
  vout_f = vout_f + 0.02f * (v - vout_f);
  vsum = vsum + v;
  vcnt = vcnt + 1;

  int n = n_ativas;
  if (!rodando || n < 1) {
    u_pi = 0;
    e_k1 = 0;
    return;
  }

  float e_k = vref_eff - v;
  float u = u_pi + b0_tab[n] * e_k + b1_tab[n] * e_k1;
  float umax = DMAX_PCT / ESC;
  if (u > umax) u = umax;
  if (u < 0.0f) u = 0.0f;
  u_pi = u;
  e_k1 = e_k;

  float base = u * ESC;
  uint8_t m = mask_ativa;
  for (int k = 0; k < 3; k++) {
    if (m & (1 << k)) {
      float d = base + (bal_on ? trim_pct[k] : 0.0f);
      d = clampf(d, 0.0f, DMAX_PCT);
      mcpwm_set_duty(MCPWM_UNIT_0, TMR[k], MCPWM_OPR_A, d);
      duty_pct[k] = d;
    } else {
      mcpwm_set_duty(MCPWM_UNIT_0, TMR[k], MCPWM_OPR_A, 0.0f);
      duty_pct[k] = 0.0f;
    }
  }
}

void tarefa_controle(void *) {
  for (;;) {
    uint32_t n = ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
    if (n > 1) {
      ovr = ovr + (n - 1);
    }
    controle_passo();
  }
}

void ARDUINO_ISR_ATTR timer_isr() {
  BaseType_t woken = pdFALSE;
  vTaskNotifyGiveFromISR(hCtrl, &woken);
  portYIELD_FROM_ISR(woken);
}

void setup_timer() {
  hTimer = timerBegin(1000000);
  timerAttachInterrupt(hTimer, &timer_isr);
  timerAlarm(hTimer, 100, true, 0); // 100 us = 10 kHz
}

// Supervisor 1 kHz (Core 0)
float le_corrente(int k) {
  float v = i_mv[k] * 0.001f;
  return (v - i_off_v[k]) * ISENSE_DIV / ACS_SENS;
}

void calibra_offsets_boot() {
  delay(300);
  (void)analogReadMilliVolts(PIN_VOUT);
  for (int k = 0; k < 3; k++) {
    uint32_t s = 0;
    for (int j = 0; j < 128; j++) {
      s += analogReadMilliVolts(PIN_ISENSE[k]);
    }
    float media_mv = s / 128.0f;
    i_mv[k] = media_mv;
    i_off_v[k] = media_mv * 0.001f;
  }
  Serial.printf("# offsets boot (V): %.3f %.3f %.3f (esperado ~1.56 com +5V ligado)\n",
                i_off_v[0], i_off_v[1], i_off_v[2]);
}

void arma_captura() {
  if (!cap_armada) {
    cap_fim = ring_cnt + CAP_POST;
    cap_armada = true;
  }
}

int cnt_ab[3] = {0, 0, 0};
int cnt_oc[3] = {0, 0, 0};
int cnt_rev[3] = {0, 0, 0};
uint32_t t_blank_ate = 0;

void zera_contadores() {
  for (int k = 0; k < 3; k++) {
    cnt_ab[k] = 0;
    cnt_oc[k] = 0;
    cnt_rev[k] = 0;
    trim_pct[k] = 0;
  }
}

void isola(int k, uint8_t motivo) {
  digitalWrite(PIN_EN[k], LOW);
  mask_ativa = mask_ativa & ~(1 << k);
  n_ativas = __builtin_popcount(mask_ativa);
  mcpwm_set_duty(MCPWM_UNIT_0, TMR[k], MCPWM_OPR_A, 0.0f);
  duty_pct[k] = 0;
  zera_contadores();

  fase_evento = k + 1;
  causa = motivo;
  t_det_us = micros();
  msg_iso_k = k + 1;
  msg_iso_ca = motivo;
  flag_msg_iso = true;
  arma_captura();

  if (n_ativas > 0) {
    redistribui_fases();
    estado = ST_DEGRADADO;
  } else {
    estado = ST_PARADO;
    rodando = false;
  }
  t_blank_ate = millis() + 25;
}

void inicia() {
  mask_ativa = 0x07;
  mask_inj = 0;
  n_ativas = 3;
  zera_contadores();
  for (int k = 0; k < 3; k++) {
    duty_pct[k] = 0;
  }

  fold_v = 0;
  causa = CA_NENHUMA;
  fase_evento = 0;
  t_inj_us = 0;
  t_det_us = 0;
  vref_rampa = 0;
  vref_eff = 0;

  redistribui_fases();
  for (int k = 0; k < 3; k++) {
    digitalWrite(PIN_EN[k], HIGH);
  }
  t_blank_ate = millis() + 1400; // espera soft-start
  estado = ST_NORMAL;
  rodando = true;
}

void para() {
  rodando = false;
  for (int k = 0; k < 3; k++) {
    digitalWrite(PIN_EN[k], LOW);
    mcpwm_set_duty(MCPWM_UNIT_0, TMR[k], MCPWM_OPR_A, 0.0f);
    duty_pct[k] = 0;
  }
  vref_eff = vref_cmd;
  estado = ST_PARADO;
}

void restaura() {
  if (!rodando) return;

  mask_inj = 0;
  mask_ativa = 0x07;
  n_ativas = 3;
  zera_contadores();
  redistribui_fases();
  for (int k = 0; k < 3; k++) {
    digitalWrite(PIN_EN[k], HIGH);
  }

  causa = CA_NENHUMA;
  fase_evento = 0;
  t_inj_us = 0;
  t_det_us = 0;
  estado = ST_NORMAL;
  t_blank_ate = millis() + 150;
}

void injeta(int k) {
  if (!rodando || k < 0 || k > 2) return;
  if (!(mask_ativa & (1 << k)) || (mask_inj & (1 << k))) return;

  t_inj_us = micros();
  t_det_us = 0;
  fase_evento = k + 1;
  t_blank_ate = 0;
  arma_captura();
  digitalWrite(PIN_EN[k], LOW);
  mask_inj = mask_inj | (1 << k);
}

void executa_comando() {
  char c = cmd;
  if (!c) return;

  switch (c) {
    case 'G':
      inicia();
      break;
    case 'X':
      para();
      break;
    case 'R':
      restaura();
      break;
    case 'F':
      injeta(cmd_k);
      break;
    case 'B':
      bal_on = (cmd_k != 0);
      break;
    case 'V':
      vref_cmd = clampf(cmd_val, 2.0f, 5.5f);
      break;
    case 'C':
      if (!rodando) {
        cal_restante = 64;
        cal_soma[0] = 0.0f;
        cal_soma[1] = 0.0f;
        cal_soma[2] = 0.0f;
      }
      break;
  }
  cmd = 0;
}

void supervisor_passo() {
  uint32_t agora = millis();
  executa_comando();

  if (cal_restante > 0) {
    for (int k = 0; k < 3; k++) {
      cal_soma[k] += i_mv[k];
    }
    cal_restante--;
    if (cal_restante == 0) {
      for (int k = 0; k < 3; k++) {
        i_off_v[k] = (cal_soma[k] / 64.0f) * 0.001f;
      }
      flag_msg_cal = true;
    }
  }

  for (int k = 0; k < 3; k++) {
    float ir = le_corrente(k);
    i_f[k] = i_f[k] + 0.4f * (ir - i_f[k]);
  }

  // buffer circular 1 kHz
  float vm = (vcnt > 0) ? (vsum / vcnt) : vout_inst;
  vsum = 0;
  vcnt = 0;
  Amostra &a = ring[ring_cnt % RING_N];
  a.v = (int16_t)(vm * 1000.0f);
  for (int k = 0; k < 3; k++) {
    a.i[k] = (int16_t)(i_f[k] * 1000.0f);
  }
  ring_cnt++;

  if (cap_armada && !cap_pronta && ring_cnt >= cap_fim) {
    for (int j = 0; j < CAP_N; j++) {
      cap_buf[j] = ring[(ring_cnt - CAP_N + j) % RING_N];
    }
    cap_ti = t_inj_us;
    cap_td = t_det_us;
    cap_fk = fase_evento;
    cap_vr = vref_eff;
    cap_pronta = true;
    cap_armada = false;
  }

  if (!rodando) return;

  // protecao OVP
  if (vout_f > VOUT_OVP) {
    causa = CA_OVP;
    msg_ovp_v = vout_f;
    flag_msg_ovp = true;
    arma_captura();
    para();
    return;
  }

  // deteccao de falha
  if (agora > t_blank_ate) {
    uint8_t m = mask_ativa;
    for (int k = 0; k < 3; k++) {
      if (!(m & (1 << k))) continue;

      float soma = 0;
      int c = 0;
      for (int j = 0; j < 3; j++) {
        if (j != k && (m & (1 << j))) {
          soma += i_f[j];
          c++;
        }
      }
      float media_out = c ? soma / c : 0.0f;

      cnt_oc[k] = (i_f[k] > I_TRIP) ? cnt_oc[k] + 1 : 0;
      cnt_rev[k] = (i_f[k] < I_REV) ? cnt_rev[k] + 1 : 0;
      bool aberta = (c >= 1) && (media_out > I_MIN_MEDIA) && (i_f[k] < I_ABERTA_REL * media_out);
      cnt_ab[k] = aberta ? cnt_ab[k] + 1 : 0;

      if (cnt_oc[k] >= 2) {
        isola(k, CA_OC);
        break;
      } else if (cnt_rev[k] >= 2) {
        isola(k, CA_REV);
        break;
      } else if (cnt_ab[k] >= T_DETECTA_MS) {
        isola(k, CA_ABERTA);
        break;
      }
    }
  }

  // foldback e soft-start
  float imax = 0;
  for (int k = 0; k < 3; k++) {
    if ((mask_ativa & (1 << k)) && i_f[k] > imax) {
      imax = i_f[k];
    }
  }
  float f = fold_v;
  if (imax > I_LIM_FASE) {
    f += FOLD_V_MS;
  } else if (imax < I_RET_FASE) {
    f -= FOLD_V_MS;
  }
  fold_v = clampf(f, 0.0f, 0.4f * vref_cmd);

  float rp = vref_rampa;
  if (rp < vref_cmd) {
    rp += SOFTSTART_V_MS;
    if (rp > vref_cmd) rp = vref_cmd;
  } else if (rp > vref_cmd) {
    rp = vref_cmd;
  }
  vref_rampa = rp;
  vref_eff = (rp > fold_v) ? (rp - fold_v) : 0.0f;

  // balanceamento de corrente
  if (bal_on && n_ativas >= 2 && agora > t_blank_ate) {
    float soma = 0;
    int c = 0;
    for (int k = 0; k < 3; k++) {
      if (mask_ativa & (1 << k)) {
        soma += i_f[k];
        c++;
      }
    }
    float media = soma / c;
    float st = 0;
    for (int k = 0; k < 3; k++) {
      if (mask_ativa & (1 << k)) {
        float t = trim_pct[k] + KI_BAL_PCT * (media - i_f[k]);
        t = clampf(t, -TRIM_MAX_PCT, TRIM_MAX_PCT);
        trim_pct[k] = t;
        st += t;
      }
    }
    float cm = st / c;
    for (int k = 0; k < 3; k++) {
      if (mask_ativa & (1 << k)) {
        trim_pct[k] = trim_pct[k] - cm;
      }
    }
  }
}

void tarefa_supervisor(void *) {
  TickType_t last = xTaskGetTickCount();
  for (;;) {
    vTaskDelayUntil(&last, 1);
    supervisor_passo();
  }
}

// Serial
void interpreta(const char *s) {
  if (!s[0]) return;
  while (cmd) delay(1);

  switch (s[0]) {
    case 'G':
    case 'g':
      cmd_k = 0;
      cmd = 'G';
      break;
    case 'X':
    case 'x':
      cmd_k = 0;
      cmd = 'X';
      break;
    case 'R':
    case 'r':
      cmd_k = 0;
      cmd = 'R';
      break;
    case 'C':
    case 'c':
      cmd_k = 0;
      cmd = 'C';
      break;
    case 'F':
    case 'f':
      cmd_k = atoi(s + 1) - 1;
      cmd = 'F';
      break;
    case 'B':
    case 'b':
      cmd_k = atoi(s + 1);
      cmd = 'B';
      break;
    case 'V':
    case 'v':
      cmd_val = atof(s + 1);
      cmd = 'V';
      break;
  }
}

void le_comandos() {
  static char buf[16];
  static int len = 0;
  while (Serial.available()) {
    char c = Serial.read();
    if (c == '\n' || c == '\r') {
      if (len) {
        buf[len] = 0;
        interpreta(buf);
        len = 0;
      }
    } else if (len < 15) {
      buf[len++] = c;
    }
  }
}

void envia_telemetria() {
  Serial.printf("{\"t\":%lu,\"rod\":%d,\"st\":%d,\"n\":%d,\"v\":%.3f,\"vr\":%.2f,\"u\":%.2f,"
                "\"i\":[%.3f,%.3f,%.3f],\"d\":[%.2f,%.2f,%.2f],\"inj\":%d,\"act\":%d,\"bal\":%d,"
                "\"ovr\":%lu,\"ti\":%lu,\"td\":%lu,\"ca\":%d,\"fk\":%d}\n",
                (unsigned long)millis(), (int)rodando, (int)estado, (int)n_ativas,
                (double)vout_f, (double)vref_eff, (double)(u_pi * ESC),
                (double)i_f[0], (double)i_f[1], (double)i_f[2],
                (double)duty_pct[0], (double)duty_pct[1], (double)duty_pct[2],
                (int)mask_inj, (int)mask_ativa, (int)bal_on,
                (unsigned long)ovr, (unsigned long)t_inj_us, (unsigned long)t_det_us,
                (int)causa, (int)fase_evento);
}

void envia_captura() {
  Serial.printf("{\"cap\":1,\"fs\":1000,\"pre\":%d,\"ti\":%lu,\"td\":%lu,\"fk\":%d,\"vr\":%.2f,\"v\":[",
                CAP_PRE, (unsigned long)cap_ti, (unsigned long)cap_td, cap_fk, (double)cap_vr);
  for (int j = 0; j < CAP_N; j++) {
    Serial.print(cap_buf[j].v);
    if (j < CAP_N - 1) Serial.print(',');
  }
  for (int k = 0; k < 3; k++) {
    Serial.printf("],\"i%d\":[", k + 1);
    for (int j = 0; j < CAP_N; j++) {
      Serial.print(cap_buf[j].i[k]);
      if (j < CAP_N - 1) Serial.print(',');
    }
  }
  Serial.println("]}");
  cap_pronta = false;
}

void setup() {
  Serial.begin(460800);
  for (int k = 0; k < 3; k++) {
    pinMode(PIN_EN[k], OUTPUT);
    digitalWrite(PIN_EN[k], LOW);
  }
  analogReadResolution(12);
  analogSetAttenuation(ADC_11db);

  setup_mcpwm();
  calibra_offsets_boot();

  xTaskCreatePinnedToCore(tarefa_controle, "ctrl", 4096, NULL, configMAX_PRIORITIES - 2, &hCtrl, 1);
  xTaskCreatePinnedToCore(tarefa_supervisor, "sup", 4096, NULL, 3, NULL, 0);
  setup_timer();

  Serial.println("# VRM pronto. Comandos: C=calibrar G=liga X=desliga F1/F2/F3=falha R=restaura");
}

void loop() {
  le_comandos();

  if (flag_msg_cal) {
    flag_msg_cal = false;
    Serial.printf("# Zero ACS712 calibrado (V): %.3f %.3f %.3f (esperado ~1.56 V)\n",
                  i_off_v[0], i_off_v[1], i_off_v[2]);
  }
  if (flag_msg_ovp) {
    flag_msg_ovp = false;
    Serial.printf("# DESARMADO POR OVP! Vout=%.2f V > %.2f V\n", (double)msg_ovp_v, (double)VOUT_OVP);
  }
  if (flag_msg_iso) {
    flag_msg_iso = false;
    Serial.printf("# Fase %d ISOLADA (causa=%d) -> N=%d\n", msg_iso_k, msg_iso_ca, (int)n_ativas);
  }

  static uint32_t prox = 0;
  uint32_t agora = millis();
  if (agora >= prox) {
    prox = agora + 10;
    envia_telemetria();
  }
  if (cap_pronta) {
    envia_captura();
  }
  delay(1);
}