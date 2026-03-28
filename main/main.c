#include <stdio.h>
#include <string.h>
#include <math.h>
#include <freertos/FreeRTOS.h>
#include <freertos/task.h>
#include <driver/i2s_std.h>
#include <driver/uart.h>
#include <esp_log.h>

#define TAG "Haruhikage"

// I2S GPIO定义 (ESP32S3)
#define I2S_BCLK_GPIO     GPIO_NUM_5   // Bit Clock
#define I2S_WS_GPIO       GPIO_NUM_6   // Word Select (LRCK)
#define I2S_DOUT_GPIO     GPIO_NUM_7   // Data Out

// UART配置
#define UART_NUM          UART_NUM_0
#define UART_TX_GPIO      GPIO_NUM_43
#define UART_RX_GPIO      GPIO_NUM_44
#define UART_BUF_SIZE     256

// 音频参数
#define SAMPLE_RATE       44100
#define SAMPLE_BITS       16
#define DMA_BUF_COUNT     4
#define DMA_BUF_LEN       1024

// 音量控制 (0-32767)
#define VOLUME            8000

// 音符定义 (B调 1=B)
typedef struct {
    float freq;
    uint32_t duration_ms;
} Note;

// 简谱音高频率表 (B调)
#define NOTE_B3   246.94f   // 低音1 (.
#define NOTE_CS4  277.18f   // 低音2 (.
#define NOTE_DS4  311.13f   // 低音3 (.
#define NOTE_E4   329.63f   // 低音4 (.
#define NOTE_FS4  369.99f   // 低音5 (.
#define NOTE_GS4  415.30f   // 低音6 (.
#define NOTE_AS4  466.16f   // 低音7 (.
#define NOTE_B4   493.88f   // 1
#define NOTE_CS5  554.37f   // 2
#define NOTE_DS5  622.25f   // 3
#define NOTE_E5   659.25f   // 4
#define NOTE_FS5  739.99f   // 5
#define NOTE_GS5  830.61f   // 6
#define NOTE_AS5  932.33f   // 7
#define NOTE_B5   987.77f   // 高音1 ·
#define NOTE_CS6  1108.73f  // 高音2 ·
#define NOTE_DS6  1244.51f  // 高音3 ·
#define NOTE_E6   1318.51f  // 高音4 ·
#define NOTE_FS6  1479.98f  // 高音5 ·
#define NOTE_GS6  1661.22f  // 高音6 ·
#define NOTE_AS6  1864.66f  // 高音7 ·

#define REST        0.0f

// BPM = 97, 6/8拍, 四分音符 = 1拍 = 619ms (6/8拍中附点四分音符为一拍)
// 八分音符 = 1/2拍 = 309.5ms ≈ 310ms
// 十六分音符 = 1/4拍 = 155ms

#define EIGHTH_MS       310     // 八分音符
#define SIXTEENTH_MS    155     // 十六分音符
#define EIGHTH_DOT_MS   465     // 附点八分音符 = 310 + 155
#define REST_MS         2000    // 轮间休息 2秒

// 《春日影》主旋律片段 - 图片中的两小节 (B调 1=B)
// 1=B, 2=C#, 3=D#, 4=E, 5=F#, 6=G#, 7=A#
//
// 图片显示:
//   第1小节: 1̲ 2̲ | (双下划线 = 十六分音符，作为弱起)
//   第2小节: 3 3 2 4 3 2 | (六个八分音符 = 完整6/8拍小节)
//   第3小节: 2 2 1 1 4 3 2 | (七个音? 实际上是两个分组: 2 2 1 1 | 4 3 2 | = 2小节?)
//
// 仔细看图片分组:
//   3 3 2 | 是连在一起的 (3个八分)
//   4 3 2 | 是连在一起的 (3个八分)
//   2 2 1 1 | 是连在一起的 (4个八分?)
//   4 3 2 | 是连在一起的 (3个八分)

// 完整乐谱: 前两小节 + 新段落 "せかいで"
static const Note prelude_chords[] = {
    // ====== 第一段 ======
    // 弱起: 1̲ 2̲ (十六分音符，快速进入)
    {NOTE_B3, SIXTEENTH_MS}, {NOTE_CS4, SIXTEENTH_MS},

    // 第1完整小节: 3 3 2 4 3 2 | (六个八分音符)
    // "かんだこころふ"
    {NOTE_DS4, EIGHTH_MS}, {NOTE_DS4, EIGHTH_MS}, {NOTE_CS4, EIGHTH_MS},
    {NOTE_E4, EIGHTH_MS}, {NOTE_DS4, EIGHTH_MS}, {NOTE_CS4, EIGHTH_MS},

    // 第2完整小节: 2 2 1 1 4 3 2 |
    // "るえるまなざし"
    {NOTE_CS4, EIGHTH_MS}, {NOTE_CS4, EIGHTH_MS}, {NOTE_B3, EIGHTH_MS}, {NOTE_B3, EIGHTH_MS},
    {NOTE_E4, EIGHTH_MS}, {NOTE_DS4, EIGHTH_MS}, {NOTE_CS4, EIGHTH_MS},

    // ====== 第二段: "せかいで" ======
    // 2 1̲ 2̲ 3. |
    // せ   か  い  で
    {NOTE_CS4, EIGHTH_MS},       // 2 (八分音符) = せ
    {NOTE_B3, SIXTEENTH_MS},     // 1̲ (十六分音符) = か
    {NOTE_CS4, SIXTEENTH_MS},    // 2̲ (十六分音符) = い
    {NOTE_DS4, EIGHTH_DOT_MS},   // 3. (附点八分音符) = で
};

static i2s_chan_handle_t i2s_tx_chan = NULL;
static volatile bool is_playing = false;
static TaskHandle_t music_task_handle = NULL;

// 根据频率获取音名 (B调)
static const char* get_note_name(float freq) {
    if (freq < 1.0f) return "REST";
    // 低音 (·)
    if (freq > 120 && freq < 127) return "B2 (·1)";
    if (freq > 136 && freq < 141) return "C#3 (·2)";
    if (freq > 153 && freq < 158) return "D#3 (·3)";
    if (freq > 162 && freq < 167) return "E3 (·4)";
    if (freq > 182 && freq < 188) return "F#3 (·5)";
    if (freq > 205 && freq < 210) return "G#3 (·6)";
    if (freq > 230 && freq < 236) return "A#3 (·7)";
    // 中音
    if (freq > 244 && freq < 251) return "B3 (1)";
    if (freq > 274 && freq < 280) return "C#4 (2)";
    if (freq > 308 && freq < 314) return "D#4 (3)";
    if (freq > 326 && freq < 333) return "E4 (4)";
    if (freq > 366 && freq < 374) return "F#4 (5)";
    if (freq > 411 && freq < 419) return "G#4 (6)";
    if (freq > 461 && freq < 471) return "A#4 (7)";
    // 高音 (·)
    if (freq > 489 && freq < 499) return "B4 (·1)";
    if (freq > 548 && freq < 559) return "C#5 (·2)";
    if (freq > 615 && freq < 627) return "D#5 (·3)";
    if (freq > 652 && freq < 664) return "E5 (·4)";
    if (freq > 732 && freq < 744) return "F#5 (·5)";
    if (freq > 821 && freq < 836) return "G#5 (·6)";
    if (freq > 922 && freq < 938) return "A#5 (·7)";
    return "UNKNOWN";
}

// 生成正弦波样本
static inline int16_t generate_sine(float freq, uint32_t *phase) {
    if (freq < 1.0f) return 0;
    const float phase_inc = (freq * 65536.0f) / SAMPLE_RATE;
    float phase_norm = (*phase % 65536) / 65536.0f;
    int16_t sample = (int16_t)(sinf(phase_norm * 2.0f * M_PI) * VOLUME);
    *phase += (uint32_t)(phase_inc * 65536.0f / 65536.0f);
    return sample;
}

// 播放音乐任务 - 单次播放模式
static void music_task(void *pvParam) {
    int16_t *dma_buf = heap_caps_malloc(DMA_BUF_LEN * sizeof(int16_t), MALLOC_CAP_DMA);
    if (!dma_buf) {
        ESP_LOGE(TAG, "Failed to allocate DMA buffer");
        vTaskDelete(NULL);
        return;
    }

    uint32_t note_idx = 0;
    uint32_t sample_count = 0;
    uint32_t phase = 0;
    uint32_t note_samples = 0;
    float current_freq = 0;
    uint32_t total_notes = sizeof(prelude_chords) / sizeof(Note);

    ESP_LOGI(TAG, "=== Start playing, total notes: %d ===", total_notes);

    while (is_playing && note_idx < total_notes) {
        // 计算当前音符的采样数
        note_samples = (prelude_chords[note_idx].duration_ms * SAMPLE_RATE) / 1000;

        // 输出当前音名
        if (prelude_chords[note_idx].freq != current_freq) {
            current_freq = prelude_chords[note_idx].freq;
            ESP_LOGI(TAG, "Playing [%d/%d]: %s (%.2f Hz)", note_idx + 1, total_notes,
                     get_note_name(current_freq), current_freq);
        }

        // 生成DMA缓冲区数据
        for (int i = 0; i < DMA_BUF_LEN; i++) {
            if (!is_playing) break;

            // 播放当前音符
            float freq = prelude_chords[note_idx].freq;
            dma_buf[i] = generate_sine(freq, &phase);

            sample_count++;
            if (sample_count >= note_samples) {
                sample_count = 0;
                phase = 0;
                note_idx++;
                if (note_idx >= total_notes) {
                    break;  // 播放完成，退出
                }
            }
        }

        // 写入I2S
        size_t bytes_written = 0;
        if (is_playing && note_idx < total_notes) {
            i2s_channel_write(i2s_tx_chan, dma_buf, DMA_BUF_LEN * sizeof(int16_t), &bytes_written, portMAX_DELAY);
        }
    }

    // 输出静音
    memset(dma_buf, 0, DMA_BUF_LEN * sizeof(int16_t));
    for (int i = 0; i < 4; i++) {
        size_t written = 0;
        i2s_channel_write(i2s_tx_chan, dma_buf, DMA_BUF_LEN * sizeof(int16_t), &written, 100 / portTICK_PERIOD_MS);
    }

    free(dma_buf);

    // 播放完成，重置状态
    is_playing = false;
    music_task_handle = NULL;

    ESP_LOGI(TAG, "=== Playback completed ===");
    uart_write_bytes(UART_NUM, "Playback completed\r\n", 20);

    vTaskDelete(NULL);
}

// 开始播放
static void start_playback(void) {
    if (!is_playing && music_task_handle == NULL) {
        is_playing = true;
        xTaskCreate(music_task, "music_task", 4096, NULL, 5, &music_task_handle);
        ESP_LOGI(TAG, "Playback started");
    } else if (is_playing) {
        ESP_LOGW(TAG, "Already playing!");
        uart_write_bytes(UART_NUM, "Already playing! Wait for completion.\r\n", 40);
    }
}

// 停止播放
static void stop_playback(void) {
    if (is_playing) {
        is_playing = false;
        // 等待任务结束
        if (music_task_handle) {
            vTaskDelay(50 / portTICK_PERIOD_MS);
        }
        ESP_LOGI(TAG, "Playback stopped");
    }
}

// I2S初始化
static esp_err_t i2s_init(void) {
    i2s_chan_config_t chan_cfg = I2S_CHANNEL_DEFAULT_CONFIG(I2S_NUM_AUTO, I2S_ROLE_MASTER);
    chan_cfg.auto_clear = true;
    ESP_ERROR_CHECK(i2s_new_channel(&chan_cfg, &i2s_tx_chan, NULL));

    i2s_std_config_t std_cfg = {
        .clk_cfg = I2S_STD_CLK_DEFAULT_CONFIG(SAMPLE_RATE),
        .slot_cfg = I2S_STD_MSB_SLOT_DEFAULT_CONFIG(SAMPLE_BITS, I2S_SLOT_MODE_MONO),
        .gpio_cfg = {
            .mclk = I2S_GPIO_UNUSED,
            .bclk = I2S_BCLK_GPIO,
            .ws = I2S_WS_GPIO,
            .dout = I2S_DOUT_GPIO,
            .din = I2S_GPIO_UNUSED,
            .invert_flags = {
                .mclk_inv = false,
                .bclk_inv = false,
                .ws_inv = false,
            },
        },
    };

    ESP_ERROR_CHECK(i2s_channel_init_std_mode(i2s_tx_chan, &std_cfg));
    ESP_ERROR_CHECK(i2s_channel_enable(i2s_tx_chan));

    ESP_LOGI(TAG, "I2S initialized: BCLK=%d, WS=%d, DOUT=%d", I2S_BCLK_GPIO, I2S_WS_GPIO, I2S_DOUT_GPIO);
    return ESP_OK;
}

// UART初始化
static esp_err_t uart_init(void) {
    uart_config_t uart_cfg = {
        .baud_rate = 115200,
        .data_bits = UART_DATA_8_BITS,
        .parity = UART_PARITY_DISABLE,
        .stop_bits = UART_STOP_BITS_1,
        .flow_ctrl = UART_HW_FLOWCTRL_DISABLE,
        .source_clk = UART_SCLK_DEFAULT,
    };

    ESP_ERROR_CHECK(uart_driver_install(UART_NUM, UART_BUF_SIZE, UART_BUF_SIZE, 0, NULL, 0));
    ESP_ERROR_CHECK(uart_param_config(UART_NUM, &uart_cfg));
    ESP_ERROR_CHECK(uart_set_pin(UART_NUM, UART_TX_GPIO, UART_RX_GPIO, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE));

    ESP_LOGI(TAG, "UART initialized: TX=%d, RX=%d, Baud=115200", UART_TX_GPIO, UART_RX_GPIO);
    return ESP_OK;
}

void app_main(void) {
    ESP_LOGI(TAG, "=================================");
    ESP_LOGI(TAG, "   Haruhikage I2S Music Player");
    ESP_LOGI(TAG, "   ESP32S3 + MAX98357");
    ESP_LOGI(TAG, "=================================");

    // 初始化外设
    ESP_ERROR_CHECK(i2s_init());
    ESP_ERROR_CHECK(uart_init());

    uint8_t *uart_buf = malloc(UART_BUF_SIZE);

    ESP_LOGI(TAG, "Ready for UART commands:");
    ESP_LOGI(TAG, "  'p' or 'P' - Play once (auto-stop after completion)");
    ESP_LOGI(TAG, "  's' or 'S' - Stop (emergency)");
    ESP_LOGI(TAG, "  'h' or 'H' - Help");

    while (1) {
        int len = uart_read_bytes(UART_NUM, uart_buf, UART_BUF_SIZE - 1, 100 / portTICK_PERIOD_MS);

        if (len > 0) {
            uart_buf[len] = '\0';
            ESP_LOGI(TAG, "Received: %s", uart_buf);

            for (int i = 0; i < len; i++) {
                char cmd = uart_buf[i];
                switch (cmd) {
                    case 'p':
                    case 'P':
                        start_playback();
                        uart_write_bytes(UART_NUM, "Playback started\r\n", 18);
                        break;

                    case 's':
                    case 'S':
                        stop_playback();
                        uart_write_bytes(UART_NUM, "Playback stopped\r\n", 18);
                        break;

                    case 'h':
                    case 'H':
                    case '?':
                        uart_write_bytes(UART_NUM,
                            "\r\n=== Haruhikage Player Commands ===\r\n"
                            "p - Play once (wait for completion)\r\n"
                            "s - Stop (emergency)\r\n"
                            "h - Help\r\n"
                            "Note: Auto-stops after completion\r\n"
                            "===================================\r\n",
                            140);
                        break;

                    case '\r':
                    case '\n':
                    case ' ':
                        break;

                    default:
                        if (cmd >= 32 && cmd < 127) {
                            uart_write_bytes(UART_NUM, "Unknown command. Press 'h' for help.\r\n", 38);
                        }
                        break;
                }
            }
        }
    }

    free(uart_buf);
}
