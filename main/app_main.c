#include <string.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/timers.h"
#include "esp_log.h"
#include "audio_element.h"
#include "audio_pipeline.h"
#include "audio_event_iface.h"
#include "audio_mem.h"
#include "audio_common.h"
#include "fatfs_stream.h"
#include "i2s_stream.h"
#include "wav_encoder.h"
#include "wav_decoder.h"
#include "board.h"
#include "audio_sonic.h"
#include "esp_peripherals.h"
#include "periph_sdcard.h"
#include "periph_button.h"

//Component includes
#include "rotaryEncoder.h"
#include "smbus.h"
#include "i2c-lcd1602.h"
#include "generic.h"

#define SAMPLE_RATE 16000
#define CHANNEL 1
#define BITS 16

static const char *TAG = "PITCH/SPEED SHIFT";
static esp_periph_set_handle_t set;
TimerHandle_t timer_100_ms;
smbus_info_t *smbus_info = NULL;
i2c_lcd1602_info_t *lcd_info = NULL;

void writePitchAndSpeed(void);

float SONIC_PITCH = 1.0f;
float SONIC_SPEED = 1.0f;

uint16_t data = 0;
bool is_modify_speed = true;

/*
    This method checks every 100 ms if the rotation of the rotary encoder has changed.
    If it has it changes the value of the speed or the pitch, depending on which one is being changed atm
    Both speed and pitch have limits which are being protected by the if statements
*/
void timer_100_ms_callback(TimerHandle_t xTimer)
{
    RotaryEncoder_getDiff(&data);
    if (data > 0 && data < 50)
    {
        if (is_modify_speed)
        {
            if (SONIC_SPEED < 8.0)
            {
                SONIC_SPEED += 0.1;
            }
        }
        else
        {
            if (SONIC_PITCH < 4.0)
            {
                SONIC_PITCH += 0.1;
            }
        }
        //This line writes all the info on the lcd
        writePitchAndSpeed();
    }
    else if (data > 50)
    {
        if (is_modify_speed)
        {
            if (SONIC_SPEED > 0.1)
            {
                SONIC_SPEED -= 0.1;
            }
        }
        else
        {
            if (SONIC_PITCH > 0.2)
            {
                SONIC_PITCH -= 0.1;
            }
        }
        //This line writes all the info on the lcd
        writePitchAndSpeed();
    }
}

//This method writes all the info about the speed/pitch and what mode you are in, on the lcd
void writePitchAndSpeed(void)
{
    i2c_lcd1602_clear(lcd_info);

    char pitch[3];
    sprintf(&pitch[0], "%02f", SONIC_PITCH);
    char speed[3];
    sprintf(&speed[0], "%02f", SONIC_SPEED);

    //REGEL 1: Speed and Pitch control
    i2c_lcd1602_move_cursor(lcd_info, 0, 0);
    i2c_lcd1602_write_string(lcd_info, "Speed/Pitch control");

    //REGEL 2: Pitch: Value
    i2c_lcd1602_move_cursor(lcd_info, 0, 1);
    i2c_lcd1602_write_string(lcd_info, "Pitch:");
    for (int i = 0; i < 3; i++)
    {
        i2c_lcd1602_write_char(lcd_info, pitch[i]);
    }

    //REGEL 3: Speed: Value
    i2c_lcd1602_move_cursor(lcd_info, 0, 2);
    i2c_lcd1602_write_string(lcd_info, "Speed:");
    for (int i = 0; i < 3; i++)
    {
        i2c_lcd1602_write_char(lcd_info, speed[i]);
    }

    //REGEL 4: Mode:Speed/Pitch
    i2c_lcd1602_move_cursor(lcd_info, 0, 3);
    if (is_modify_speed) {
        i2c_lcd1602_write_string(lcd_info, "Mode:Speed");
    } else {
        i2c_lcd1602_write_string(lcd_info, "Mode:Pitch");
    }
}

void init_lcd(void)
{
    // Set up the SMBus
    smbus_info = smbus_malloc();
    smbus_init(smbus_info, I2C_NUM_0, LCD_ADDRESS);
    smbus_set_timeout(smbus_info, 1000 / portTICK_RATE_MS);

    // Set up the LCD1602 device with backlight off
    lcd_info = i2c_lcd1602_malloc();
    i2c_lcd1602_init(lcd_info, smbus_info, true, LCD_NUM_ROWS, LCD_NUM_COLUMNS, LCD_NUM_VIS_COLUMNS);

    // Write first line
    i2c_lcd1602_clear(lcd_info);
}

static audio_element_handle_t create_sonic()
{
    sonic_cfg_t sonic_cfg = DEFAULT_SONIC_CONFIG();
    sonic_cfg.sonic_info.samplerate = SAMPLE_RATE;
    sonic_cfg.sonic_info.channel = CHANNEL;
    sonic_cfg.sonic_info.resample_linear_interpolate = 1;
    return sonic_init(&sonic_cfg);
}

static audio_element_handle_t create_fatfs_stream(int sample_rates, int bits, int channels, audio_stream_type_t type)
{
    fatfs_stream_cfg_t fatfs_cfg = FATFS_STREAM_CFG_DEFAULT();
    fatfs_cfg.type = type;
    audio_element_handle_t fatfs_stream = fatfs_stream_init(&fatfs_cfg);
    mem_assert(fatfs_stream);
    audio_element_info_t writer_info = {0};
    audio_element_getinfo(fatfs_stream, &writer_info);
    writer_info.bits = bits;
    writer_info.channels = channels;
    writer_info.sample_rates = sample_rates;
    audio_element_setinfo(fatfs_stream, &writer_info);
    return fatfs_stream;
}

static audio_element_handle_t create_i2s_stream(int sample_rates, int bits, int channels, audio_stream_type_t type)
{
    i2s_stream_cfg_t i2s_cfg = I2S_STREAM_CFG_DEFAULT();
    i2s_cfg.type = type;
#if defined CONFIG_ESP_LYRAT_MINI_V1_1_BOARD
    if (i2s_cfg.type == AUDIO_STREAM_READER)
    {
        i2s_cfg.i2s_port = 1;
        i2s_cfg.i2s_config.channel_format = I2S_CHANNEL_FMT_ONLY_LEFT;
    }
#endif
    audio_element_handle_t i2s_stream = i2s_stream_init(&i2s_cfg);
    mem_assert(i2s_stream);
    audio_element_info_t i2s_info = {0};
    audio_element_getinfo(i2s_stream, &i2s_info);
    i2s_info.bits = bits;
    i2s_info.channels = channels;
    i2s_info.sample_rates = sample_rates;
    audio_element_setinfo(i2s_stream, &i2s_info);
    return i2s_stream;
}

static audio_element_handle_t create_wav_encoder()
{
    wav_encoder_cfg_t wav_cfg = DEFAULT_WAV_ENCODER_CONFIG();
    return wav_encoder_init(&wav_cfg);
}

static audio_element_handle_t create_wav_decoder()
{
    wav_decoder_cfg_t wav_cfg = DEFAULT_WAV_DECODER_CONFIG();
    return wav_decoder_init(&wav_cfg);
}

void record_playback_task()
{
    audio_pipeline_handle_t pipeline_rec = NULL;
    audio_pipeline_handle_t pipeline_play = NULL;
    audio_pipeline_cfg_t pipeline_cfg = DEFAULT_AUDIO_PIPELINE_CONFIG();

    ESP_LOGI(TAG, "[1.1] Initialize recorder pipeline");
    pipeline_rec = audio_pipeline_init(&pipeline_cfg);
    pipeline_play = audio_pipeline_init(&pipeline_cfg);

    ESP_LOGI(TAG, "[1.2] Create audio elements for recorder pipeline");
    audio_element_handle_t i2s_reader_el = create_i2s_stream(SAMPLE_RATE, BITS, CHANNEL, AUDIO_STREAM_READER);
    audio_element_handle_t wav_encoder_el = create_wav_encoder();
    audio_element_handle_t fatfs_writer_el = create_fatfs_stream(SAMPLE_RATE, BITS, CHANNEL, AUDIO_STREAM_WRITER);

    ESP_LOGI(TAG, "[1.3] Register audio elements to recorder pipeline");
    audio_pipeline_register(pipeline_rec, i2s_reader_el, "i2s_reader");
    audio_pipeline_register(pipeline_rec, wav_encoder_el, "wav_encoder");
    audio_pipeline_register(pipeline_rec, fatfs_writer_el, "file_writer");
    audio_pipeline_link(pipeline_rec, (const char *[]){"i2s_reader", "wav_encoder", "file_writer"}, 3);

    ESP_LOGI(TAG, "[2.2] Create audio elements for playback pipeline");
    audio_element_handle_t fatfs_reader_el = create_fatfs_stream(SAMPLE_RATE, BITS, CHANNEL, AUDIO_STREAM_READER);
    audio_element_handle_t wav_decoder_el = create_wav_decoder();
    audio_element_handle_t sonic_el = create_sonic();
    audio_element_handle_t i2s_writer_el = create_i2s_stream(SAMPLE_RATE, BITS, CHANNEL, AUDIO_STREAM_WRITER);

    ESP_LOGI(TAG, "[2.3] Register audio elements to playback pipeline");
    audio_pipeline_register(pipeline_play, fatfs_reader_el, "file_reader");
    audio_pipeline_register(pipeline_play, wav_decoder_el, "wav_decoder");
    audio_pipeline_register(pipeline_play, sonic_el, "sonic");
    audio_pipeline_register(pipeline_play, i2s_writer_el, "i2s_writer");
    audio_pipeline_link(pipeline_play, (const char *[]){"file_reader", "wav_decoder", "sonic", "i2s_writer"}, 4);

    ESP_LOGI(TAG, "[ 3 ] Set up  event listener");
    audio_event_iface_cfg_t evt_cfg = AUDIO_EVENT_IFACE_DEFAULT_CFG();
    audio_event_iface_handle_t evt = audio_event_iface_init(&evt_cfg);
    audio_event_iface_set_listener(esp_periph_set_get_event_iface(set), evt);
    ESP_LOGW(TAG, "Press [Rec] to start recording");

    //This line initiates everything needed for the lcd to work
    init_lcd();
    //This line writes all the info on the lcd
    writePitchAndSpeed();
    //This line initiates the RotaryEncoder and everything thats needed for it
    RotaryEncoder_init();

    timer_100_ms = xTimerCreate("MyTimer0.1s", pdMS_TO_TICKS(100), pdTRUE, (void *)1, &timer_100_ms_callback);
    if (xTimerStart(timer_100_ms, 10) != pdPASS)
    {
        ESP_LOGE(TAG, "Cannot start 0.1 second timer");
    }

    while (1)
    {
        audio_event_iface_msg_t msg;
        esp_err_t ret = audio_event_iface_listen(evt, &msg, portMAX_DELAY);
        if (ret != ESP_OK)
        {
            ESP_LOGE(TAG, "[ * ] Event interface error : %d", ret);
            continue;
        }
        if ((int)msg.data == get_input_mode_id())
        {
            if ((msg.cmd == PERIPH_BUTTON_LONG_PRESSED) || (msg.cmd == PERIPH_BUTTON_PRESSED))
            {
                is_modify_speed = !is_modify_speed;
                writePitchAndSpeed();
                if (is_modify_speed)
                {
                    ESP_LOGI(TAG, "You can now change the speed of the audio");
                }
                else
                {
                    ESP_LOGI(TAG, "You can now change the pitch of the audio");
                }
            }
            continue;
        }
        if ((int)msg.data == get_input_rec_id())
        {
            if (msg.cmd == PERIPH_BUTTON_PRESSED)
            {
                ESP_LOGE(TAG, "Now recording, release [Rec] to STOP");
                audio_pipeline_terminate(pipeline_play);
                audio_pipeline_reset_ringbuffer(pipeline_play);
                audio_pipeline_reset_elements(pipeline_play);

                ESP_LOGI(TAG, "Setup file path to save recorded audio");
                i2s_stream_set_clk(i2s_reader_el, SAMPLE_RATE, BITS, CHANNEL);
                audio_element_set_uri(fatfs_writer_el, "/sdcard/rec.wav");
                audio_pipeline_run(pipeline_rec);
            }
            else if (msg.cmd == PERIPH_BUTTON_RELEASE || msg.cmd == PERIPH_BUTTON_LONG_RELEASE)
            {
                ESP_LOGI(TAG, "START Playback");
                audio_pipeline_terminate(pipeline_rec);
                audio_pipeline_reset_ringbuffer(pipeline_rec);
                audio_pipeline_reset_elements(pipeline_rec);

                ESP_LOGI(TAG, "Setup file path to read the wav audio to play");
                i2s_stream_set_clk(i2s_writer_el, SAMPLE_RATE, BITS, CHANNEL);
                audio_element_set_uri(fatfs_reader_el, "/sdcard/rec.wav");

                //This line set the pitch and speed of the playback to whatever the values where changed to
                sonic_set_pitch_and_speed_info(sonic_el, SONIC_PITCH, SONIC_SPEED);

                //This line writes all the info on the lcd
                writePitchAndSpeed();

                audio_pipeline_run(pipeline_play);
            }
        }
    }

    ESP_LOGI(TAG, "[ 4 ] Stop audio_pipeline");
    audio_pipeline_terminate(pipeline_rec);
    audio_pipeline_terminate(pipeline_play);

    audio_pipeline_unregister(pipeline_play, fatfs_reader_el);
    audio_pipeline_unregister(pipeline_play, wav_decoder_el);
    audio_pipeline_unregister(pipeline_play, i2s_writer_el);

    audio_pipeline_unregister(pipeline_rec, i2s_reader_el);
    audio_pipeline_unregister(pipeline_rec, sonic_el);
    audio_pipeline_unregister(pipeline_rec, wav_encoder_el);
    audio_pipeline_unregister(pipeline_rec, fatfs_writer_el);

    /* Terminate the pipeline before removing the listener */
    audio_pipeline_remove_listener(pipeline_rec);
    audio_pipeline_remove_listener(pipeline_play);

    /* Stop all peripherals before removing the listener */
    esp_periph_set_stop_all(set);
    audio_event_iface_remove_listener(esp_periph_set_get_event_iface(set), evt);

    /* Make sure audio_pipeline_remove_listener & audio_event_iface_remove_listener are called before destroying event_iface */
    audio_event_iface_destroy(evt);

    /* Release all resources */
    audio_pipeline_deinit(pipeline_rec);
    audio_pipeline_deinit(pipeline_play);

    audio_element_deinit(fatfs_reader_el);
    audio_element_deinit(wav_decoder_el);
    audio_element_deinit(i2s_writer_el);

    audio_element_deinit(i2s_reader_el);
    audio_element_deinit(sonic_el);
    audio_element_deinit(wav_encoder_el);
    audio_element_deinit(fatfs_writer_el);
}

void app_main(void)
{
    esp_log_level_set("*", ESP_LOG_WARN);
    esp_log_level_set(TAG, ESP_LOG_INFO);

    // Initialize peripherals management
    esp_periph_config_t periph_cfg = DEFAULT_ESP_PERIPH_SET_CONFIG();
    set = esp_periph_set_init(&periph_cfg);

    // Initialize SD Card peripheral
    audio_board_sdcard_init(set);

    // Initialize Button peripheral
    audio_board_key_init(set);

    // Setup audio codec
    audio_board_handle_t board_handle = audio_board_init();
    audio_hal_ctrl_codec(board_handle->audio_hal, AUDIO_HAL_CODEC_MODE_BOTH, AUDIO_HAL_CTRL_START);

    // Start record/playback task
    record_playback_task();
    esp_periph_set_destroy(set);
}
