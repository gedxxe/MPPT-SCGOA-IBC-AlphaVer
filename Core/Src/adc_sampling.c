#include "main.h"

enum {
    ADC_IDX_PV_VOLT = 0,
    ADC_IDX_BAT_VOLT,
    ADC_IDX_BAT_CURR,
    ADC_IDX_PV_CURR,
    ADC_IDX_TEMP,
    ADC_IDX_COUNT = ADC_CHANNEL_COUNT
};

/* Urutan kanal mengikuti konfigurasi DMA (pvV, batV, batI, pvI, suhu). */
volatile uint16_t ADC_RAW[ADC_IDX_COUNT];

/* Akumulator untuk averaging 10 sampel agar noise berkurang. */
static uint32_t sum_voltage_pv       = 0;
static uint32_t sum_voltage_bat      = 0;
static uint32_t sum_current_pv       = 0;
static uint32_t sum_current_bat      = 0;
static uint32_t sum_temperature      = 0;

/* Hasil rata-rata per batch konversi. */
static uint16_t avg_voltage_pv       = 0;
static uint16_t avg_voltage_bat      = 0;
static uint16_t avg_current_pv       = 0;
static uint16_t avg_current_bat      = 0;
static uint16_t avg_temperature      = 0;

/* Nilai fisik setelah scaling koefisien (tanpa unit display). */
static uint16_t voltage_bat   = 0;
static uint16_t voltage_pv    = 0;
static uint16_t current_bat   = 0;
static uint16_t current_pv    = 0;
static float temperature      = 0;

/* Nilai siap tampil (0.1V/0.1A dsb) dipakai state machine. */
uint16_t dis_voltage_bat   = 0;
uint16_t dis_voltage_pv    = 0;
uint16_t dis_current_bat   = 0;
uint16_t dis_current_pv    = 0;
uint16_t dis_power_bat     = 0;
uint16_t dis_power_pv      = 0;
int      dis_temperature   = 0;

uint16_t soc_battery = 0;   /* Estimasi SoC sederhana berbasis tegangan. */

static float rNTC           = 0;    /* Perhitungan resistansi NTC untuk suhu. */

static uint8_t sum_adc_count  = 0;  /* Counter sampel dalam satu batch 10 kali. */

static uint8_t flag_adc_conv  = 0;  /* Flag batch konversi siap diproses. */

void HAL_ADC_ConvCpltCallback(ADC_HandleTypeDef *hadc) {
    (void)hadc; /* Kanal callback tidak dipakai langsung; hanya DMA batch yang penting. */

    /* Toggle indikator sampling untuk debugging timing. */
    HAL_GPIO_TogglePin(GPIOA, GPIO_PIN_7);

    sum_adc_count++; /* Hitung berapa sampel yang sudah terkumpul. */

    /* Akumulasi setiap kanal untuk averaging. */
    sum_voltage_pv   += ADC_RAW[ADC_IDX_PV_VOLT];
    sum_voltage_bat  += ADC_RAW[ADC_IDX_BAT_VOLT];
    sum_current_pv   += ADC_RAW[ADC_IDX_PV_CURR];
    sum_current_bat  += ADC_RAW[ADC_IDX_BAT_CURR];
    sum_temperature  += ADC_RAW[ADC_IDX_TEMP];

    if (sum_adc_count >= 10) {
        /* Hitung rata-rata 10 sampel untuk meredam noise. */
        avg_voltage_pv   = sum_voltage_pv / 10;
        avg_voltage_bat  = sum_voltage_bat / 10;
        avg_current_pv   = sum_current_pv / 10;
        avg_current_bat  = sum_current_bat / 10;
        avg_temperature  = sum_temperature / 10;

        flag_adc_conv = 1; /* Beri tahu loop utama bahwa data siap. */

        /* Reset akumulator untuk batch berikutnya. */
        sum_voltage_pv   = 0;
        sum_voltage_bat  = 0;
        sum_current_pv   = 0;
        sum_current_bat  = 0;
        sum_temperature  = 0;

        sum_adc_count    = 0;
    }
}

void adc_sampling() {
    uint32_t tempVal; /* Buffer konversi integer sementara. */

    if (!flag_adc_conv) {
        return;
    }

    /* Konversi tegangan PV ke skala fisik lalu skala display 0.1V. */
    voltage_pv = avg_voltage_pv / PV_VOLT_COEF;
    tempVal = voltage_pv;
    tempVal *= 512;
    tempVal >>= 9;
    dis_voltage_pv = tempVal;

    /* Konversi tegangan baterai. */
    voltage_bat = avg_voltage_bat / BAT_VOLT_COEF;
    tempVal = voltage_bat;
    tempVal *= 512;
    tempVal >>= 9;
    dis_voltage_bat = tempVal;

    /* Konversi arus PV. */
    current_pv = avg_current_pv / PV_CURR_COEF;
    tempVal = current_pv;
    tempVal *= 512;
    tempVal >>= 9;
    dis_current_pv = tempVal;

    /* Konversi arus baterai. */
    current_bat = avg_current_bat / BAT_CURR_COEF;
    tempVal = current_bat;
    tempVal *= 512;
    tempVal >>= 9;
    dis_current_bat = tempVal;

    /* Konversi suhu via pembacaan NTC. */
    temperature = avg_temperature * (3.3f / 4095.0f);
    if (temperature < 0.001f) {
        /* Lindungi dari divide-by-zero ketika ADC membaca sangat rendah/noise. */
        temperature = 0.001f;
    }
    rNTC = (3.3f * 9840.0f / temperature) - 9840.0f;
    if (rNTC <= 0.0f) {
        /* Pastikan argumen log() tetap positif untuk menghindari NaN. */
        rNTC = 1.0f;
    }
    dis_temperature = ((3950 * 298.15f) / (298.15f * log(rNTC / 10000.0f) + 3950)) - 273.15f;

    /* Hitung daya PV/baterai dalam unit display untuk logika MPPT. */
    dis_power_pv = dis_current_pv * dis_voltage_pv;
    dis_power_bat = dis_current_bat * dis_voltage_bat;

    /* Estimasi SoC kasar untuk tampilan/pemilihan state awal. */
    soc_battery = (dis_voltage_bat * 100) / MAX_BATTERY_CHARGE;

    /* Tandai batch sudah dikonsumsi; charging_flow akan diizinkan. */
    flag_adc_conv = 0;
    flag_adc_done = 1;
}
