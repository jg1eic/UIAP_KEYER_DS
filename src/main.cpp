//==================================================================//
// UIAP_keyer_for_ch32fun
// BASE Software from : https://www.gejigeji.com/?page_id=1045
// Modified by Kimio Ohe JA9OIR/JA1AOQ
//	- port to ch32fun library
//==================================================================
#include "ch32fun.h"
#include "ch32v003_GPIO_branchless.h"
#include "keyer_hal.h"
#include <stdio.h>
#include <stdint.h>
#define SSD1306_128X64
#include "ssd1306_i2c.h"
#include "ssd1306.h"
#include "flash_eep.h"

/* ==== FLASH EEPROM ==== */
FLASH_EEP eep;  // EEPROM emulation object

//==========================================
// 常数・マクロ
//==========================================
// スクイズ状態（半チE��チE��単位�E状態機械�E�E
#define SQZ_FREE 0
#define SQZ_SPC0 1
#define SQZ_SPC 2
#define SQZ_DOT0 3
#define SQZ_DOT 4
#define SQZ_DAH_CONT0 5
#define SQZ_DAH_CONT1 6
#define SQZ_DAH_CONT 7
#define SQZ_DASH 8

// パドル状慁E
#define PDL_DOT 1
#define PDL_DASH 2
#define PDL_FREE 0

#define SQUEEZE_TYPE 0 // スクイーズモーチE
#define PDL_RATIO 4    // 短点・長点比率

#define WPM_MAX 40 // 最大速度
#define WPM_MIN 5  // 最小速度

// ト�Eン設宁E
#define TONE_DIV 3 // 周波数調整
// 2-->976Hz
// 3-->651Hz
// 4-->488Hz

// スイチE��設宁E
#define SW_SCAN_DIV 20  // 0.256ms ÁE20 ≁E5.12ms
#define SW_PRESS_TH 127 // 長押し判定！Ems ÁE128 = 0.64秒！E
#define SW_PUSH_TH 5    // 押下判定！Ems ÁE5 = 25ms�E�E
#define SW_1 (1 << 3) 
#define SW_2 (1 << 2)
#define SW_3 (1 << 1)
#define SW_4 (1 << 0)
#define SW_INFO_CLICK 0x10
#define SW_INFO_PRESS 0x20
#define SW_INFO_DOUBLE 0x40
#define SW_CLEAR() (sw_mask = 0b00001111) // 一度スイチE��を離すまでカウントをしなぁE
#define MASK_MODE 0xf0

// EDIT操作�Eリピ�Eト設定（体感調整済み�E�E
#define EDIT_REPEAT_START 15 // 15 ÁE10ms = 150ms
#define EDIT_REPEAT_SPEED 5  // 5 ÁE10ms = 50ms

// チE��チE��モーチE
#define DEBUG_MODE_PRINT 0

#ifndef PWR_CTLR_CWUF
#define PWR_CTLR_CWUF ((uint16_t)0x0004)
#define PWR_CTLR_CSBF ((uint16_t)0x0008)
#define PWR_CSR_SBF   ((uint16_t)0x0002)
#endif

// FLASH記録用
#define MSG_COUNT 4
#define PAGE_MSG1 0
#define PAGE_MSG2 1
#define PAGE_MSG3 2
#define PAGE_MSG4 3

// メチE��ージ用変数
#define MSG_NUM 4  // メモリ数�E�EW1 / SW2 / SW3 / SW4�E�E
#define MSG_LEN 64 // 1メチE��ージの最大斁E��数
#define EDIT_TABLE_LEN (sizeof(edit_table) - 1)

// チャタリング除去用
#define MIN_ON_TICKS 50 //0.256ms ÁE50 = 12.8ms
#define MIN_OFF_TICKS 50 //0.256ms ÁE50 = 12.8ms

//チE��ード用閾値
#define DIT_TICKS      250   // 0.256ms ÁE250 = 64ms (20WPM相彁E
#define DOT_MAX        (2 * dit_est)
#define CHAR_GAP_MIN   (1.5 * dit_est)
#define WORD_GAP_MIN   (5 * dit_est)

//CWバッファサイズ
#define CW_BUF_SIZE 64

//==========================================
// 構造体定義
//==========================================

// モード定義
typedef enum
{
    MODE_KEYER = 0,   // 通常キーイング
    MODE_PLAY,        // メモリ再生中
    MODE_EDIT_SELECT, // 編雁E��モリ選抁E
    MODE_EDIT,        // 編雁E��ーチE
    MODE_SETUP        // 設定（封E���E�E
} keyer_mode_t;

// 斁E��編雁E��
typedef enum
{
    EDIT_CHAR_SELECT = 0, // 斁E��選択中
    EDIT_POS_MOVE         // カーソル移動中�E�封E��拡張�E�E
} edit_state_t;

// 斁E��編雁E��にパドル検�E用
typedef struct
{
    bool dot;
    bool dash;
} paddle_release_t;

volatile paddle_release_t pad_rel = {0};

// CWイベント定義
typedef enum {
    EV_DOT,
    EV_DASH,
    EV_CHAR_GAP,
    EV_WORD_GAP
} cw_event_t;

//==========================================
// グローバル変数
//==========================================

volatile uint8_t tone_div = 0;        // ト�Eン刁E��カウンタ
volatile bool tone_on = false;        // ト�Eン出力中フラグ
volatile bool edit_tick_10ms = false; // EDIT用10msタイマ�Eフラグ

char msgs[MSG_NUM][MSG_LEN + 1]; // メチE��ージバッファ

const char default_msgs[MSG_NUM][MSG_LEN] = {
    "CQ TEST JO1YGK",
    "5NN 13M BK",
    "TEST MESSAGE 3",
    "TEST MESSAGE 4"
    }; // チE��ォルトメチE��ージ

static uint8_t cur_msg = 0;  // 編雁E��メモリ番号
static uint8_t edit_pos = 0; // カーソル位置

static const char edit_table[] =
    " ABCDEFGHIJKLMNOPQRSTUVWXYZ0123456789/?.=+-@"; // 編雁E��斁E��テーブル

#define DISP_COLS 10 // 画面に表示する斁E��数�E�編雁E��チE��ージ�E�E

static uint8_t edit_view_left = 0; // 表示ウィンドウ先頭
static uint8_t edit_len = 0;       // 現在の斁E��数
volatile bool edit_tick = false;

// キーイング用変数
int key_spd = 1000;
int key_spd_sys = 1000; // シスチE��メチE��ージ用の速度�E�固定値�E�E
int wpm = 20;
int wpm_sys = 20; // シスチE��メチE��ージ用のWPM�E�固定値�E�E
bool tone_enabled = false;
int squeeze = 0;
int paddle = PDL_FREE;
int paddle_old = PDL_FREE;

volatile uint8_t sw_div_cnt = 0;
volatile uint32_t tim1_tick256 = 0;

static uint32_t key_last_tick = 0;
static bool key_state = false; // false=OFF, true=ON

// 編雁E��カウンタ
static keyer_mode_t mode = MODE_KEYER;

// EDIT用パドル状慁E
static uint16_t edit_dot_cnt = 0;
static uint16_t edit_dash_cnt = 0;
static bool edit_dot_prev = false;  // 前フレームのDOT状慁E
static bool edit_dash_prev = false; // 前フレームのDASH状慁E
static uint8_t edit_first = 1;      // ☁E追加�E�EDIT初回フラグ

// ==== dit推宁E====
uint32_t dit_est = DIT_TICKS;        // 初期値�E�E0WPM相当！E

// モード文字�E取征E
const char *mode_to_str(keyer_mode_t m)
{
    switch (m)
    {
    case MODE_KEYER:
        return "KEYER";
    case MODE_PLAY:
        return "PLAY";
    case MODE_EDIT_SELECT:
        return "EDIT_SELECT";
    case MODE_EDIT:
        return "EDIT";
    case MODE_SETUP:
        return "SETUP";
    default:
        return "UNKNOWN";
    }
}

/* ==== 割り込み ↁEmain loop 共朁E==== */
volatile bool in_dot = false;
volatile bool in_dash = false;

// ==== 自動送信制御 ====
// 起動時は何もしなぁE
volatile bool auto_mode = false;  // 今、�E動送信中ぁE
volatile bool auto_armed = false; // SWAを一度でも押したか！Erueで “�E動送信機�Eが有効”！E
volatile bool req_start_auto = false;
volatile bool req_reset_auto = false;

// 停止琁E���E�ラチE��停止�E�E
typedef enum
{
    STOP_NONE = 0,
    STOP_PADDLE = 1, // DOT/DASHで停止�E�ラチE���E�E
    STOP_SWB = 2,    // SWBで停止�E�ラチE���E�E
} StopReason;

volatile StopReason stop_reason = STOP_NONE;

// スイチE��の判宁E
uint8_t sw_mask = 0; // スイチE��押しっぱなしをカウントしなぁE��め�Eマスク
uint8_t sw_clicked = 0;
uint8_t count_sw[4]; // スイチE��長押しとかカウンチE
volatile uint8_t sw_stat;
volatile uint8_t sw_mode;

// ==== 汎用 CW メチE��ージ再生 ====
const char *auto_msg = NULL; // 実際に再生する斁E���E
bool sys_msg_active = false; // シスチE��メチE��ージか！E
volatile bool keyout_enabled = true;       // 通常はON
volatile bool ignore_paddle_input = false; // パドル入力を無視するフラグ

// ==== キータイミング計測用 ====
uint32_t key_on_ticks = 0;
uint32_t key_off_ticks = 0;
bool key_on = false;

//CWリングバッファ
volatile cw_event_t cw_buf[CW_BUF_SIZE];
volatile uint8_t cw_w = 0;
volatile uint8_t cw_r = 0;

//モールス斁E��バチE��ァ
volatile bool flush_done = false;

//モールス符号バッファ
#define MORSE_BUF_LEN 8

char morse_buf[MORSE_BUF_LEN];
uint8_t morse_len = 0;

//volatile bool oled_need_refresh = false;
//static bool oled_refreshed_this_frame = false;

// グローバルに追加
int last_wpm = -999;

// 無操作タイマ�E
volatile uint32_t last_activity_tick = 0;

static inline uint32_t exti_line_from_pin(uint32_t pin)
{
    return 1u << (pin & 0x0f);
}

static void exti_select_port(uint8_t line, uint8_t port)
{
    const uint32_t shift = line * 2u;
    AFIO->EXTICR = (AFIO->EXTICR & ~(0x3u << shift)) | ((uint32_t)port << shift);
}

static void prepare_gpio_for_standby(void)
{
    // Put pins that do not need to wake the keyer into analog input to avoid leakage paths.
    GPIO_port_pinMode(GPIO_port_A, GPIO_pinMode_I_analog, GPIO_Speed_In);
    GPIO_port_pinMode(GPIO_port_C, GPIO_pinMode_I_analog, GPIO_Speed_In);
    GPIO_port_pinMode(GPIO_port_D, GPIO_pinMode_I_analog, GPIO_Speed_In);

    GPIO_pinMode(PIN_SW1, GPIO_pinMode_I_pullUp, GPIO_Speed_In);
    GPIO_pinMode(PIN_SW2, GPIO_pinMode_I_pullUp, GPIO_Speed_In);
    GPIO_pinMode(PIN_SW3, GPIO_pinMode_I_pullUp, GPIO_Speed_In);
    GPIO_pinMode(PIN_SW4, GPIO_pinMode_I_pullUp, GPIO_Speed_In);
    GPIO_pinMode(PIN_DOT, GPIO_pinMode_I_pullUp, GPIO_Speed_In);
    GPIO_pinMode(PIN_DASH, GPIO_pinMode_I_pullUp, GPIO_Speed_In);
    GPIO_pinMode(PIN_ST, GPIO_pinMode_I_pullUp, GPIO_Speed_In);
}

static void restore_after_standby(void)
{
    NVIC_SystemReset();
}

__attribute__((section(".noinit")))
uint32_t standby_magic;

__attribute__((section(".noinit")))
uint8_t ssd1306_buffer_backup[SSD1306_W*SSD1306_H/8];

#define STANDBY_MAGIC_VALUE 0xA5A5A5A5
#define SSD1306_W 128
#define SSD1306_H 64


//==========================================
// 関数プロトタイチE
//==========================================
static const char *morseForChar(char c);
static void printAsc(int8_t asciinumber);
static void printAscii(int8_t c);
void dump_msgs(void);
uint8_t job_paddle(void);
uint8_t job_auto(void);
void startTone(void);
void stopTone(void);
void update_speed_from_adc(void);
void start_play(uint8_t msg);
void stop_play(void);
void clear_sw_rel(void);
void edit_clear_after_cursor(void);
void adjust_edit_view(void);
void update_switch_status(void);
uint8_t sw_chatter(uint8_t sw, uint8_t *counter);
void sw_check(void);
uint8_t sw_get_info(void);
uint8_t sw_is_pressed();
void handle_keyer_mode(void);
void handle_play_mode(void);
void handle_edit_select(void);
void handle_edit_mode(void);
void draw_keyer_screen(void);
void draw_edit_screen(void);
void draw_edit_select(void);
void draw_startup_screen(void);
void loop(void);
void TIM1_UP_IRQHandler(void);
void init_flash_messages(void);
void save_current_message_to_flash(void);
void draw_sys_message(const char *msg);

//==========================================
// Morse チE�Eブル
//==========================================
static const char *morseForChar(char c)
{
    // 小文孁Eb は “BT E-...-) 扱ぁE��区刁E��用途！E
    // if (c == '=')
    //     return "-...-";
    // if (c == '+')
    //     return ".-.-.";
    if (c == 'k')
        return "-.--.";
    if (c == 'v')
        return "...-.-";
    if (c >= 'a' && c <= 'z')
        c = (char)(c - 'a' + 'A');

    switch (c)
    {
    // Letters
    case 'A':
        return ".-";
    case 'B':
        return "-...";
    case 'C':
        return "-.-.";
    case 'D':
        return "-..";
    case 'E':
        return ".";
    case 'F':
        return "..-.";
    case 'G':
        return "--.";
    case 'H':
        return "....";
    case 'I':
        return "..";
    case 'J':
        return ".---";
    case 'K':
        return "-.-";
    case 'L':
        return ".-..";
    case 'M':
        return "--";
    case 'N':
        return "-.";
    case 'O':
        return "---";
    case 'P':
        return ".--.";
    case 'Q':
        return "--.-";
    case 'R':
        return ".-.";
    case 'S':
        return "...";
    case 'T':
        return "-";
    case 'U':
        return "..-";
    case 'V':
        return "...-";
    case 'W':
        return ".--";
    case 'X':
        return "-..-";
    case 'Y':
        return "-.--";
    case 'Z':
        return "--..";

    // Digits
    case '0':
        return "-----";
    case '1':
        return ".----";
    case '2':
        return "..---";
    case '3':
        return "...--";
    case '4':
        return "....-";
    case '5':
        return ".....";
    case '6':
        return "-....";
    case '7':
        return "--...";
    case '8':
        return "---..";
    case '9':
        return "----.";

    // Punctuation (忁E��そぁE��のだぁE
    case '.':
        return ".-.-.-";
    case ',':
        return "--..--";
    case '?':
        return "..--..";
    case '/':
        return "-..-.";
    case '=':
        return "-...-";
    case '+':
        return ".-.-.";
    case '-':
        return "-....-";
    case '@':
        return ".--.-.";

    default:
        return nullptr;
    }
}

// モールス斁E��対応表
typedef struct {
    const char *morse;
    char        ch;
} MorseMap;

const MorseMap morse_table[] = {
    {".-",    'A'}, {"-...", 'B'}, {"-.-.", 'C'}, {"-..",  'D'},
    {".",     'E'}, {"..-.", 'F'}, {"--.",  'G'}, {"....", 'H'},
    {"..",    'I'}, {".---", 'J'}, {"-.-",  'K'}, {".-..", 'L'},
    {"--",    'M'}, {"-.",   'N'}, {"---",  'O'}, {".--.", 'P'},
    {"--.-",  'Q'}, {".-.",  'R'}, {"...",  'S'}, {"-",    'T'},
    {"..-",   'U'}, {"...-", 'V'}, {".--",  'W'}, {"-..-", 'X'},
    {"-.--",  'Y'}, {"--..", 'Z'},

    {"-----",'0'}, {".----",'1'}, {"..---",'2'}, {"...--",'3'},
    {"....-",'4'}, {".....",'5'}, {"-....",'6'}, {"--...",'7'},
    {"---..",'8'}, {"----.",'9'},

    {".-.-.-",'.'}, {"--..--",','}, {"..--..",'?'}, {"-..-.",'/'},
    {"-...-", '='}, {".-.-.",'+'},
};



#define FONT_WIDTH 12 //was 12
#define FONT_COLOR 1
#define LINE_HEIGHT 16
#define FONT_SCALE_16X16 fontsize_16x16
const int colums = 10; // was 10

int lcdindex = 0;
uint8_t line1[colums];
uint8_t line2[colums];
uint8_t line3[colums];
uint8_t lastChar = 0;

volatile uint8_t flg = 0;

static void reset_decoded_display(void)
{
    for (int i = 0; i < colums; i++) {
        line1[i] = ' ';
        line2[i] = ' ';
        line3[i] = ' ';
    }
    lcdindex = 0;
}

//==========================================
//	printasc : print the ascii code to the lcd
//==========================================
static void redraw_lines(void)
{
    // ---- 衁E�E�E = 16�E�E---
    for (int i = 0; i < colums; i++) {
        ssd1306_drawchar_sz(i * FONT_WIDTH, LINE_HEIGHT * 1,
                            line1[i], FONT_COLOR, FONT_SCALE_16X16);
    }

    // ---- 衁E�E�E = 32�E�E---
    for (int i = 0; i < colums; i++) {
        ssd1306_drawchar_sz(i * FONT_WIDTH, LINE_HEIGHT * 2,
                            line2[i], FONT_COLOR, FONT_SCALE_16X16);
    }

    // ---- 衁E�E�E = 48�E�E---
    for (int i = 0; i < colums; i++) {
        //ssd1306_fillRect(0, 48, 128, 16, 0); // 衁E全体消す
        ssd1306_drawchar_sz(i * FONT_WIDTH, LINE_HEIGHT * 3,
                            line3[i], FONT_COLOR, FONT_SCALE_16X16);
    }

    // ---- 右端クリア�E�保険として残す�E�E---
    int used_width = FONT_WIDTH * colums;   // 16 * 8 = 128
    if (used_width < 128) {
        int clear_w = 128 - used_width;
        ssd1306_fillRect(used_width, 16, clear_w, 16, 0);
        ssd1306_fillRect(used_width, 32, clear_w, 16, 0);
        ssd1306_fillRect(used_width, 48, clear_w, 16, 0);
    }
}

static void printAsc(int8_t asciinumber)
{
    // 行がぁE��ぱぁEↁEスクロール
    if (lcdindex >= colums) {

        // 衁E ↁE衁E
        memcpy(line1, line2, colums);

        // 衁E ↁE衁E
        memcpy(line2, line3, colums);

        // 衁E をクリア
        memset(line3, ' ', colums);

        lcdindex = 0;
    }

    // 衁E に新しい斁E��を追加
    line3[lcdindex++] = asciinumber;

    // ☁E行�E体を描画�E�部刁E��画はしなぁE��E
    redraw_lines();

    // ☁Erefresh は毎回�E�乱れ防止�E�E
    //    __disable_irq();
    ssd1306_refresh();
    // oled_refreshed_this_frame = true;
    //__enable_irq();
    //oled_refreshed_this_frame = true;
}


//==========================================
//	printascii : print the ascii code to the lcd
//==========================================
static void printAscii(int8_t c)
{
    switch (c)
    {
    case 'b': // BT
        printAsc('B');
        printAsc('T');
        break;
    case 'a': // AR
        printAsc('A');
        printAsc('R');
        break;
    case 'k': // KN
        printAsc('K');
        printAsc('N');
        break;
    case 'v': // VA
        printAsc('V');
        printAsc('A');
        break;
    default:
        printAsc(c);
        break;
    }
}

// ト�Eン出力トグル
static inline void toggle_tone_pin(void)
{
    GPIOC->OUTDR ^= (1 << 7); // PC7 の侁E
}

//==========================================
// メチE��ージ出劁E
//==========================================
void dump_msgs(void)
{
    for (int i = 0; i < MSG_NUM; i++)
    {
        //printf("MSG%d: ", i + 1);
        for (int j = 0; j < MSG_LEN; j++)
        {
            char c = msgs[i][j];
            if (c == '\0')
                break;
            //printf("%c", c);
        }
        //printf("\r\n");
    }
}

//==========================================
// フラチE��ュメモリ読み出し�E妥当性確誁E
//==========================================
static bool is_valid_message(const uint8_t *buf)
{
    bool has_null = false;

    for (int i = 0; i < MSG_LEN; i++)
    {
        uint8_t c = buf[i];

        if (c == 0xFF)
            break; // 消去領域
        if (c == '\0')
        {
            has_null = true;
            break;
        }
        if (c < 0x20 || c > 0x7E) // 非ASCII
            return false;
    }
    return has_null;
}

//==========================================
// フラチE��ュメモリ読み出し＋�E期化
//==========================================
void init_flash_messages(void)
{
    uint8_t buf[FLASH_PAGE_SIZE];

    eep.begin(MSG_COUNT);

    for (int i = 0; i < MSG_COUNT; i++)
    {
        memset(buf, 0, sizeof(buf));
        eep.read(i, buf);

        if (!is_valid_message(buf))
        {
            // Flashが空 or ゴチEↁEチE��ォルト投入
            strncpy(msgs[i], default_msgs[i], MSG_LEN - 1);
            msgs[i][MSG_LEN - 1] = '\0';
            //printf("MSG%d: default\n", i + 1);
        }
        else
        {
            memcpy(msgs[i], buf, MSG_LEN);
            msgs[i][MSG_LEN - 1] = '\0';
            //printf("MSG%d: loaded from flash\n", i + 1);
        }
    }
}

//==========================================
// フラチE��ュメモリ書き込み
//==========================================
void save_current_message_to_flash(void)
{
    uint8_t buf[FLASH_PAGE_SIZE];
    int page = cur_msg; // MSG番号 = ペ�Eジ番号

    memset(buf, 0xFF, sizeof(buf));
    strncpy((char *)buf, msgs[cur_msg], MSG_LEN);

    eep.erase(page);
    eep.write(page, buf);

    //DEBUG_PRINTF("[FLASH] save MSG%d\n", cur_msg + 1);
}

//==========================================
// 手動パドル処琁E��EWA/SWBは混ぜなぁE��E
//==========================================
uint8_t job_paddle()
{
    static uint32_t left_time = 0;
    uint8_t key_dot, key_dash;

    // パドル入力を無視する場合、状態をリセチE��して終亁E
    if (ignore_paddle_input)
    {
        paddle = PDL_FREE;
        paddle_old = PDL_FREE;
        squeeze = SQZ_FREE;
        left_time = 0;
        return 0;
    }

    key_dot = (!GPIO_digitalRead(PIN_DOT));
    key_dash = (!GPIO_digitalRead(PIN_DASH));

    if (key_dot || key_dash) {
    last_activity_tick = tim1_tick256;
    }

    if (left_time != 0)
    {
        left_time--;
    }
    else
    {
        left_time = key_spd / 2;
        if (squeeze != SQZ_FREE)
            squeeze--;
    }

    if (squeeze != SQZ_FREE)
    {
        if (paddle_old == PDL_DOT && key_dash)
            paddle = PDL_DASH;
        else if (paddle_old == PDL_DASH && key_dot)
            paddle = PDL_DOT;
    }

    if (SQUEEZE_TYPE == 0)
    {
        if (squeeze > SQZ_DASH)
            paddle = PDL_FREE;
    }
    else
    {
        if (squeeze > SQZ_SPC)
            paddle = PDL_FREE;
    }

    if (squeeze > SQZ_SPC)
        return 1;
    else if (squeeze == SQZ_SPC || squeeze == SQZ_SPC0)
        return 0;

    if (paddle == PDL_FREE)
    {
        if (key_dot)
            paddle = PDL_DOT;
        else if (key_dash)
            paddle = PDL_DASH;
    }

    if (paddle == PDL_FREE)
        return 0;

    if (paddle == PDL_DOT)
        squeeze = SQZ_DOT;
    else
    {
        uint8_t dash_len = (SQZ_SPC * PDL_RATIO + 5) / 2;
        squeeze = SQZ_SPC + dash_len;
    }

    left_time = key_spd / 2;
    paddle_old = paddle;
    paddle = PDL_FREE;
    return 1;
}

//==========================================
// 自動送信処琁E��半チE��チE��単位�E簡略版！E
//==========================================
uint8_t job_auto(void)
{
    // 半ディチE��単位�E状態機械
    typedef enum {
        AUTO_IDLE = 0,      // 次の斁E��征E��
        AUTO_ELEM_ON,       // 要素(dit/dah) ON 中
        AUTO_ELEM_OFF,      // 要素閁EOFF (1 dit)
        AUTO_CHAR_GAP,      // 斁E��間ギャチE�E (3 dit)
        AUTO_WORD_GAP       // 単語間ギャチE�E (7 dit)
    } auto_state_t;

    static auto_state_t state = AUTO_IDLE;
    static uint32_t left_time = 0;   // 0.5dit タイチE
    static uint8_t half_rem = 0;     // 残り half-dit 数
    static const char *seq = nullptr;
    static uint8_t elem = 0;
    static uint16_t pos = 0;

    // シスチE��メチE��ージ中は固定速度
    // ☁EWPM 可変対応：毎回最新の key_spd を参照
    int current_key_spd = sys_msg_active ? key_spd_sys : key_spd;

    // リセチE��要汁E
    if (req_reset_auto) {
        req_reset_auto = false;
        state = AUTO_IDLE;
        left_time = 0;
        half_rem = 0;
        seq = nullptr;
        elem = 0;
        pos = 0;
        return 0;
    }

    // 自動送信でなければ何もしなぁE
    if (!auto_mode || auto_msg == NULL) {
        return 0;
    }

    last_activity_tick = tim1_tick256;

    // ☁E0.5dit タイマ�E琁E��最新速度で毎回リセチE���E�E
    if (left_time > 0) {
        left_time--;
    } else {
        left_time = current_key_spd / 2;  // ↁEここが毎回最新になめE


        if (half_rem > 0) {
            half_rem--;
        }

        // half_rem ぁE0 になったら次の状態へ
        if (half_rem == 0) {
            switch (state) {

            case AUTO_IDLE: {
                char c = auto_msg[pos];

                // メチE��ージ終端
                if (c == '\0') {
                    auto_mode = false;
                    req_reset_auto = true;
                    auto_msg = NULL;
                    sys_msg_active = false;
                    keyout_enabled = true;
                    mode = MODE_KEYER;

                    // チE��ード系のリセチE���E��Eコード踏襲�E�E
                    morse_len = 0;
    last_activity_tick = tim1_tick256;

    cw_r = cw_w;
                    key_off_ticks = 0;
                    key_on_ticks = 0;
                    flush_done = true;

                    //draw_keyer_screen();
                    return 0;
                }

                // スペ�Eス ↁE単語間ギャチE�E
                if (c == ' ') {

                    // ☁E追加�E�スペ�EスめELCD に表示
                    if (!sys_msg_active) {
                        printAscii(' ');
                    }

                    // 連続スペ�EスをスキチE�E
                    while (auto_msg[pos] == ' ') {
                        pos++;
                    }
                    state = AUTO_WORD_GAP;
                    half_rem = 14; // 7 dit = 14 half-dit
                    break;
                }

                // 通常斁E��E
                if (!sys_msg_active) {
                    printAscii(c); // 送信斁E��を表示
                }

                seq = morseForChar(c);
                elem = 0;

                if (seq == nullptr) {
                    // 未対応文孁EↁEスペ�Eス扱ぁE
                    pos++;
                    state = AUTO_WORD_GAP;
                    half_rem = 14;
                    break;
                }

                // 最初�E要素を送信開姁E
                state = AUTO_ELEM_ON;
                half_rem = (seq[elem] == '.') ? 2 : 6; // dit=1dit=2half, dah=3dit=6half
                break;
            }

            case AUTO_ELEM_ON:
                // 要素 ON 終亁EↁE要素閁EOFF (1 dit)
                state = AUTO_ELEM_OFF;
                half_rem = 2; // 1 dit = 2 half-dit
                break;

            case AUTO_ELEM_OFF:
                // 次の要素へ
                elem++;
                if (seq[elem] == '\0') {
                    // 斁E���E最後�E要素が終わっぁE
                    char next = auto_msg[pos + 1];

                    if (next == '\0') {
                        // メチE��ージ末尾 ↁEここで終亁E
                        auto_mode = false;
                        req_reset_auto = true;
                        auto_msg = NULL;
                        sys_msg_active = false;
                        keyout_enabled = true;
                        mode = MODE_KEYER;

                        morse_len = 0;
                        cw_r = cw_w;
                        key_off_ticks = 0;
                        key_on_ticks = 0;
                        flush_done = true;

                        //draw_keyer_screen();
                        return 0;
                    } else if (next == ' ') {
                        // ☁Eここでスペ�EスめEつ表示する
                        if (!sys_msg_active) {
                            printAscii(' ');
                        }

                        // 次がスペ�Eス ↁE単語間ギャチE�E
                        pos++; // 現在の斁E��を進める
                        while (auto_msg[pos] == ' ') {
                            pos++;
                        }
                        state = AUTO_WORD_GAP;
                        half_rem = 14; // 7 dit
                    } else {
                        // 次も文孁EↁE斁E��間ギャチE�E
                        pos++; // 次の斁E��へ
                        state = AUTO_CHAR_GAP;
                        half_rem = 6; // 3 dit
                    }
                } else {
                    // まだ要素が残ってぁE�� ↁE次の要素 ON
                    state = AUTO_ELEM_ON;
                    half_rem = (seq[elem] == '.') ? 2 : 6;
                }
                break;

            case AUTO_CHAR_GAP:
                // 斁E��間ギャチE�E終亁EↁE次の斁E��へ
                state = AUTO_IDLE;
                break;

            case AUTO_WORD_GAP:
                // 単語間ギャチE�E終亁EↁE次の斁E��へ
                state = AUTO_IDLE;
                break;

            default:
                state = AUTO_IDLE;
                break;
            }
        }
    }

    // 出力：要素 ON 状態�Eときだけキー ON
    return (state == AUTO_ELEM_ON) ? 1 : 0;
}

//==========================================
// ト�Eン制御
//==========================================
void startTone()
{
    tone_on = true;

    if (keyout_enabled)
    {
        GPIO_digitalWrite(PIN_KEYOUT, high); // ☁E��常のみキーイング
    }
}

void stopTone()
{
    tone_on = false;

    GPIO_digitalWrite(PIN_TONE, low);

    if (keyout_enabled)
    {
        GPIO_digitalWrite(PIN_KEYOUT, low); // ☁E��常のみキーイング
    }
}

inline void keydown(void)
{
    if (!key_state)
    {
        key_state = true;
        key_last_tick = tim1_tick256;
    }
    startTone();
}

inline void keyup(void)
{
    if (key_state)
    {
        key_state = false;
        key_last_tick = tim1_tick256;
    }
    stopTone();
}


static inline long map(long x,
                       long in_min, long in_max,
                       long out_min, long out_max)
{
    // Arduino本家と同じくゼロ除算チェチE��はしなぁE��En_max == in_min だと未定義�E�E
    return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

static void draw_wpm_value(void)
{
    char buf[16];
    sprintf(buf, "%2d", wpm);
    ssd1306_drawstr_sz(96, 0, buf, 1, fontsize_8x8);
}

//==========================================
// ADCからスピ�Eド読み込み
//==========================================
void update_speed_from_adc()
{

    int adc = GPIO_analogRead(GPIO_Ain0_A2);
    wpm = map(adc, 0, 1023, WPM_MIN, WPM_MAX);
    dit_est = (1200UL * 1000)/(wpm * 256);
    key_spd = dit_est;

    if (wpm == last_wpm) {
        return;
    }
    last_wpm = wpm;

    // WPM描画
    draw_wpm_value();

    // メモリ再生中は printAsc() ぁErefresh する
    if (mode == MODE_PLAY) {
        return;
    }

    // ☁E衁Eを�E描画�E�Eage6/page7の上�E�E�E
    for (int i = 0; i < colums; i++) {
        ssd1306_drawchar_sz(i * FONT_WIDTH, LINE_HEIGHT * 3,
                            line3[i], FONT_COLOR, FONT_SCALE_16X16);
    }

    // ☁E右端8px�E�E=120、E27, y=56、E3�E�をクリア
    ssd1306_fillRect(120, 56, 8, 8, 0);

    // refresh
    ssd1306_refresh();
}



//==========================================
// スイチE��状態確認（各ハンドラから呼ぶ)
//==========================================
void update_switch_status(void)
{

    // スイチE��確誁E
    sw_stat = sw_get_info();
    sw_mode = sw_stat & MASK_MODE;
    sw_stat &= ~MASK_MODE;

    // チE��チE��表示
    // if (sw_stat)
    // {
    //     //printf("SW mode=%02X stat=%02X mask=%02X\n",
    //            //sw_mode, sw_stat, sw_mask);
    // }
}

/**
 * @brief  スイチE��カウンタ
 */
uint8_t sw_chatter(uint8_t sw, uint8_t *counter)
{
    uint8_t is_clicked = 0;

    if (sw)
    {
        // カウンタぁE55に達してぁE��ければ、カウンタめE増やぁE
        *counter += ((*counter != 255) ? 1 : 0);
    }
    else
    {
        is_clicked = ((SW_PUSH_TH < *counter) && (*counter < SW_PRESS_TH)) ? 1 : 0;
        *counter = 0;
    }
    return (is_clicked);
}

/**
 * @brief  スイチE��チェチE��
 * タイマ�E割り込みで呼ばれる
 */
void sw_check()
{
    uint8_t temp_pin;

    bool s1 = !GPIO_digitalRead(PIN_SW1);
    bool s2 = !GPIO_digitalRead(PIN_SW2);
    bool s3 = !GPIO_digitalRead(PIN_SW3);
    bool s4 = !GPIO_digitalRead(PIN_SW4);

    temp_pin = (s1 << 3) | (s2 << 2) | (s3 << 1) | (s4 << 0);

    if (sw_mask != 0)
    {
        count_sw[0] = count_sw[1] = count_sw[2] = count_sw[3] = 0;
        sw_clicked = 0;

        if ((temp_pin & 0x0F) == 0)
            sw_mask = 0;

        return;
    }

    sw_clicked |= (sw_chatter(temp_pin & SW_1, &count_sw[0])) ? SW_1 : 0;
    sw_clicked |= (sw_chatter(temp_pin & SW_2, &count_sw[1])) ? SW_2 : 0;
    sw_clicked |= (sw_chatter(temp_pin & SW_3, &count_sw[2])) ? SW_3 : 0;
    sw_clicked |= (sw_chatter(temp_pin & SW_4, &count_sw[3])) ? SW_4 : 0;

    if (sw_clicked) {
    last_activity_tick = tim1_tick256;
    }
}

//****************************
// スイチE��を操作したら靁E
//  メモリ再生の停止用
// ****************************
uint8_t sw_is_pressed()
{
    if (sw_mask != 0)
    {
        return (0);
    }

    // なんか押操作された�E�E
    if ((count_sw[0] > SW_PUSH_TH) || (count_sw[1] > SW_PUSH_TH) || (count_sw[2] > SW_PUSH_TH) || (count_sw[3] > SW_PUSH_TH))
    {
        return (1);
    }
    return (0);
}

/*****************************************************************************
 スイチE��状態読み込み　(8ビット�E惁E��を返す)
 xxxxdcba
            dcba sw[1,2,3,4] に対忁E
            xxxx フラグ
                0000 何もなぁE
                0001 クリチE��
                0010 長押ぁE
                0100 ダブル押ぁE
 ******************************************************************************/
uint8_t sw_get_info()
{

    volatile uint8_t count = 0;

    if (sw_mask != 0)
    {
        // 1bitでもスイチE��マスクがかかってぁE��ら、押されてなぁE��とにする
        return (0);
    }

    // 褁E��押されてる！E
    if (count_sw[0] > SW_PUSH_TH)
    {
        count += 1;
        flg |= SW_1;
    }
    if (count_sw[1] > SW_PUSH_TH)
    {
        count += 1;
        flg |= SW_2;
    }
    if (count_sw[2] > SW_PUSH_TH)
    {
        count += 1;
        flg |= SW_3;
    }
    if (count_sw[3] > SW_PUSH_TH)
    {
        count += 1;
        flg |= SW_4;
    }
    if (count >= 2)
    {
        return (SW_INFO_DOUBLE | flg);
    }

    flg = 0;
    // なんか長押しされた�E�E
    if (count_sw[0] > SW_PRESS_TH)
    {
        flg |= SW_1;
    }
    if (count_sw[1] > SW_PRESS_TH)
    {
        flg |= SW_2;
    }
    if (count_sw[2] > SW_PRESS_TH)
    {
        flg |= SW_3;
    }
    if (count_sw[3] > SW_PRESS_TH)
    {
        flg |= SW_4;
    }
    if (flg != 0)
    {
        sw_mask = 1;
        return (SW_INFO_PRESS | flg);
    }

    flg = 0;
    if (sw_clicked != 0)
    {
        // なんかクリチE��されてた！E
        flg = SW_INFO_CLICK | sw_clicked;
        sw_clicked = 0;
    }
    return (flg);
}

//==========================================
// 編雁E��の次の斁E��E
//==========================================
char next_char(char c)
{
    const char *p = strchr(edit_table, c);
    if (!p) return edit_table[0];

    p++;  // 次へ

    // 末尾なら�E頭へ
    if (*p == '\0')
        return edit_table[0];

    return *p;
}



//==========================================
// 編雁E��の前�E斁E��E
//==========================================
char prev_char(char c)
{
    const char *p = strchr(edit_table, c);
    if (!p) return edit_table[0];

    if (p == edit_table)
        p = edit_table + strlen(edit_table) - 1;
    else
        p--;

    return *p;
}



//==========================================
// 編雁E��にそれ以降�E斁E��を削除
//==========================================
// void edit_clear_after_cursor(void)
// {
//     for (uint8_t i = edit_pos; i < MSG_LEN; i++)
//     {
//         msgs[cur_msg][i] = '\0';
//     }
//     edit_len = edit_pos;
// }

//==========================================
// 編雁E��の表示を調整
//==========================================
void adjust_edit_view(void)
{
    if (edit_pos < edit_view_left)
    {
        edit_view_left = edit_pos;
    }
    else if (edit_pos >= edit_view_left + DISP_COLS)
    {
        edit_view_left = edit_pos - DISP_COLS + 1;
    }
}

//==========================================
// 汎用シスチE��メチE��ージ送信開姁E
//==========================================
void play_sys_msg(const char *msg, uint8_t wpm_val)
{
    auto_msg = msg;
    sys_msg_active = true;
    last_activity_tick = tim1_tick256;

    // シスチE��メチE��ージ用の固定WPMを設宁E
    wpm_sys = wpm_val;
    key_spd_sys = 4687 / wpm_sys;

    keyout_enabled = false; // RFキーイングしなぁE
    req_reset_auto = true;
    auto_mode = true;
    mode = MODE_PLAY;
}

void play_mem_msg(uint8_t n)
{
    auto_msg = msgs[n];
    sys_msg_active = false;
    last_activity_tick = tim1_tick256;

    keyout_enabled = true; // 通常キーイング
    req_reset_auto = true;
    auto_mode = true;
    mode = MODE_PLAY;
}

//==========================================
//  ストレートキー読み込み
//==========================================
bool read_straight_key(void)
{
    return !GPIO_digitalRead(PIN_ST); // 押されたら true
}

//==========================================
//  DIT時間推定値更新
//==========================================

void update_dit(uint32_t ticks)
{
    //TODO: DIT時間学習アルゴリズム実裁E��宁E

}

//==========================================
//  CWイベントをリングバッファにプッシュ
//==========================================
static inline void cw_push(cw_event_t ev)
{
    // 再生中�E��E動送信�E�やPLAYモード中はチE��ード用バッファに入れなぁE
    if (auto_mode || mode == MODE_PLAY) return;

    uint8_t next = (cw_w + 1) % CW_BUF_SIZE;
    if (next != cw_r) {
        cw_buf[cw_w] = ev;
        cw_w = next;
    }
}

//==========================================
//  モールス符号から斁E��へ変換
//==========================================
char morse_to_char(const char *m)
{
    for (unsigned int i = 0; i < sizeof(morse_table)/sizeof(MorseMap); i++) {
        if (strcmp(m, morse_table[i].morse) == 0) {
            return morse_table[i].ch;
        }
    }
    return '*';   // 見つからなぁE��吁E
}


//==========================================
//  ON確定時の処琁E
//==========================================
void process_on(uint32_t ticks)
{
    if (ticks < MIN_ON_TICKS) return;

    // ==== DIT ====
    if (ticks <= DOT_MAX) {
        cw_push(EV_DOT);
        //("[.] ticks=%lu  dit_est=%lu\r\n", ticks, dit_est);
        return;
    }
    // ==== DAH ====
    else {
        cw_push(EV_DASH);
        //printf("[-] ticks=%lu  dit_est=%lu\r\n", ticks, dit_est);
        return; 
    }
}

//==========================================
//  OFF確定時の処琁E
//==========================================
void process_off(uint32_t ticks)
{
    if (ticks < MIN_OFF_TICKS) return;

    //語間
    if (ticks >= WORD_GAP_MIN) {
        //printf("[7-ON] ticks=%lu  dit_est=%lu\r\n", ticks, dit_est);
        return; 
    }

    /* ==== 斁E��間 ==== */
    if (ticks >= CHAR_GAP_MIN) {
        cw_push(EV_CHAR_GAP);
        //printf("[3] ticks=%lu  dit_est=%lu\r\n", ticks, dit_est);
        return;
    }

    /* ==== 符号閁E(1dit) ==== */
    if (ticks < CHAR_GAP_MIN) {
        //printf("[ ] ticks=%lu  dit_est=%lu\r\n", ticks, dit_est);
        return;
    }
}


//==========================================
//  チE��ード�E琁E
//==========================================
void cw_decode_task(void)
{
    /*
     * Make a local snapshot of the producer index to avoid races with
     * the IRQ that pushes events (single producer in ISR, single consumer
     * here). This avoids reading cw_w multiple times while it changes.
     */
    uint8_t w = cw_w;
    while (cw_r != w)
    {
        cw_event_t ev = cw_buf[cw_r];
        cw_r = (cw_r + 1) % CW_BUF_SIZE;

        switch (ev)
        {
        case EV_DOT:
            if (morse_len < MORSE_BUF_LEN - 1) {
                morse_buf[morse_len++] = '.';
            }
            break;

        case EV_DASH:
            if (morse_len < MORSE_BUF_LEN - 1) {
                morse_buf[morse_len++] = '-';
            }
            break;

        case EV_CHAR_GAP:
            if (morse_len > 0) {
                morse_buf[morse_len] = '\0';   // 斁E���E匁E
                char c = morse_to_char(morse_buf);
                if (mode != MODE_PLAY && !auto_mode) {
                    printAscii(c); // LCD表示
                }
                //if (mode != MODE_PLAY) DEBUG_PRINTF("Decoded='%c', Buf=%s, Len=%d\r\n", c, morse_buf, morse_len);
                morse_len = 0;
            }
            break;

        case EV_WORD_GAP:
            if (morse_len > 0) {
                morse_buf[morse_len] = '\0';
                char c = morse_to_char(morse_buf);
                if (mode != MODE_PLAY && !auto_mode) {
                    printAscii(c); // LCD表示
                }
                //if (mode != MODE_PLAY) DEBUG_PRINTF("Decoded='%c', Buf=%s, Len=%d\r\n", c, morse_buf, morse_len);
                morse_len = 0;
            }
            if (mode != MODE_PLAY && !auto_mode) {
                printAscii(32); // スペ�Eス表示
            }
            break;

        }
        /* refresh local snapshot in case producer advanced while processing */
        w = cw_w;
    }
}


//==========================================
//  KEY出力�E琁E割り込みから呼ぶこと)
//==========================================
void service_keyer(void)
{
    static bool prev_on = false;
    bool on = false;

    if (mode == MODE_PLAY)
    {
        on = auto_mode ? job_auto() : false;
    }
else if (mode == MODE_KEYER)
{
    bool st = read_straight_key();  // 電鍵
    bool pd = job_paddle();         // パドル�E�状態機械�E�E
    on = st || pd;

    // ☁Eストレートキーでスリープ復帰
    if (st) {
        last_activity_tick = tim1_tick256;

        // ☁Eメモリ再生中なら強制停止
        if (mode == MODE_PLAY || auto_mode) {
            stop_play();
        }
    }
}

    /* tickカウント（唯一の場所�E�E*/
    if (on) {
        last_activity_tick = tim1_tick256;
        key_on_ticks++;
    } else {
        key_off_ticks++;
    }

    /* 状態変化検�E */
    if (on && !prev_on)
    {
        // OFF ↁEON
        process_off(key_off_ticks);        

        key_off_ticks = 0;
        key_on_ticks = 0;
        keydown();
    }
    else if (!on && prev_on)
    {
        // ON ↁEOFF
        process_on(key_on_ticks);

        key_on_ticks = 0;
        key_off_ticks = 0;
        keyup();
    }

    prev_on = on;
}


//==========================================
//  #1 KEYERモード�E琁E
//==========================================
void handle_keyer_mode(void)
{

    /* パドル入力無視フラグをクリア�E�パドルが両方離されたら�E�E*/
    bool dot = !GPIO_digitalRead(PIN_DOT);
    bool dash = !GPIO_digitalRead(PIN_DASH);
    if (ignore_paddle_input && !dot && !dash)
    {
        ignore_paddle_input = false;
    }

    // スイチE��状態取征E
    update_switch_status();

    /* 編雁E��ードへ */
    //update_switch_status();

    if (sw_mode == SW_INFO_DOUBLE && (sw_stat & (SW_1 | SW_2)) == (SW_1 | SW_2)) 
    {
        SW_CLEAR();
        mode = MODE_EDIT_SELECT;
        //printf("MODE_EDIT_SELECT\r\n");
        dump_msgs();
        draw_edit_select();
        return;
    }

    /* メモリ再生 */
    if (sw_mode == SW_INFO_CLICK && sw_stat == SW_1)
    {
        SW_CLEAR();
        //printf("START PLAY MSG1\r\n");
        mode = MODE_PLAY;
        start_play(0);
    }

    if (sw_mode == SW_INFO_CLICK && sw_stat == SW_2)
    {
        SW_CLEAR();
        //printf("START PLAY MSG2\r\n");
        mode = MODE_PLAY;
        start_play(1);
    }

        if (sw_mode == SW_INFO_CLICK && sw_stat == SW_3)
    {
        SW_CLEAR();
        //printf("START PLAY MSG3\r\n");
        mode = MODE_PLAY;
        start_play(2);
    }

    if (sw_mode == SW_INFO_CLICK && sw_stat == SW_4)
    {
        SW_CLEAR();
        //printf("START PLAY MSG4\r\n");
        mode = MODE_PLAY;
        start_play(3);
    }
}

//==========================================
//  #2 PLAYモード（�E動リピ�Eト防止�E�E
//==========================================
void handle_play_mode(void)
{
    // パドル入力を直接読み込み
    bool dot = !GPIO_digitalRead(PIN_DOT);
    bool dash = !GPIO_digitalRead(PIN_DASH);
    bool st = read_straight_key();   // ☁Eストレートキー追加

    //パドル無視フラグがONで、パドルが両方離されたらフラグをクリア
    if (ignore_paddle_input && !dot && !dash)
    {
        ignore_paddle_input = false;
    }

    // 何か操作したら止める
    if (sw_is_pressed() || dot || dash || st)
    {
        SW_CLEAR();
        stop_play();
        //printf("Interrupt Message\r\n");
        mode = MODE_KEYER;
        //draw_keyer_screen(); // 画面復帰
        return;
    }

    if (!auto_mode)
    {
        //printf("Finished Message\r\n");
        // ☁Eここで stop_play() を呼ぶのが重要E��E
        stop_play();
        mode = MODE_KEYER;
        last_activity_tick = tim1_tick256;
        // ☁E追加�E�メモリ再生終亁E��にタイムアウト関連をリセチE��
        flush_done = true;
        key_off_ticks = 0;
        key_on_ticks = 0;
    }
}

//==========================================
//  #3 編雁E��ード�E琁E��メモリ選抁E
//==========================================
void handle_edit_select(void)
{
    // スイチE��状態取征E
    update_switch_status();


    if (sw_mode == SW_INFO_CLICK) {

        if (sw_stat & SW_1) cur_msg = 0;
        if (sw_stat & SW_2) cur_msg = 1;
        if (sw_stat & SW_3) cur_msg = 2;
        if (sw_stat & SW_4) cur_msg = 3;

        // 編雁E��姁E
        edit_pos = 0;
        edit_len = strlen(msgs[cur_msg]);

        mode = MODE_EDIT;
        draw_edit_screen();
        return;
    }

    if (sw_mode == SW_INFO_DOUBLE && (sw_stat & (SW_1 | SW_2)) == (SW_1 | SW_2))
    {
        SW_CLEAR();
        mode = MODE_KEYER;
        last_activity_tick = tim1_tick256;
        draw_keyer_screen();
        ssd1306_refresh();
        //printf("BACK TO KEYER\r\n");
        return;
    }
}

//==========================================
//  #4 編雁E��ード�E琁E��文字選抁E
//==========================================
void handle_edit_mode(void)
{
    // 編雁E�E回描画
    if (edit_first)
    {
        draw_edit_screen();
        edit_first = 0;
        edit_dot_prev = false;
        edit_dash_prev = false;
    }

    // スイチE��状態取征E
    update_switch_status();

    bool dot = !GPIO_digitalRead(PIN_DOT);
    bool dash = !GPIO_digitalRead(PIN_DASH);

    /* ===== 10ms周期でのみ編雁E�E琁E===== */
    if (edit_tick_10ms)
    {
        edit_tick_10ms = false;

        /* ---- DOT�E�戻る！E--- */
        if (dot)
        {
            // 新規押下（前フレームが靟で今フレームが押されてぁE���E��E 即座に反映
            if (!edit_dot_prev)
            {
                msgs[cur_msg][edit_pos] =
                    prev_char(msgs[cur_msg][edit_pos] ? msgs[cur_msg][edit_pos] : ' ');
                draw_edit_screen();
                edit_dot_cnt = 0;
            }
            else
            {
                // 押し続け ↁEリピ�Eト壣亁E
                edit_dot_cnt++;
                if (edit_dot_cnt == EDIT_REPEAT_START ||
                    (edit_dot_cnt > EDIT_REPEAT_START &&
                     (edit_dot_cnt - EDIT_REPEAT_START) % EDIT_REPEAT_SPEED == 0))
                {

                    msgs[cur_msg][edit_pos] =
                        prev_char(msgs[cur_msg][edit_pos] ? msgs[cur_msg][edit_pos] : ' ');
                    draw_edit_screen();
                }
            }
            edit_dot_prev = true;
        }
        else
        {
            edit_dot_cnt = 0;
            edit_dot_prev = false;
        }

        /* ---- DASH�E�進む�E�E--- */
        if (dash)
        {
            // 新規押下（前フレームが靟で今フレームが押されてぁE���E��E 即座に反映
            if (!edit_dash_prev)
            {
                msgs[cur_msg][edit_pos] =
                    next_char(msgs[cur_msg][edit_pos] ? msgs[cur_msg][edit_pos] : ' ');
                draw_edit_screen();
                edit_dash_cnt = 0;
            }
            else
            {
                // 押し続け ↁEリピ�Eト壣亁E
                edit_dash_cnt++;
                if (edit_dash_cnt == EDIT_REPEAT_START ||
                    (edit_dash_cnt > EDIT_REPEAT_START &&
                     (edit_dash_cnt - EDIT_REPEAT_START) % EDIT_REPEAT_SPEED == 0))
                {

                    msgs[cur_msg][edit_pos] =
                        next_char(msgs[cur_msg][edit_pos] ? msgs[cur_msg][edit_pos] : ' ');
                    draw_edit_screen();
                }
            }
            edit_dash_prev = true;
        }
        else
        {
            edit_dash_cnt = 0;
            edit_dash_prev = false;
        }
    }

    /* ===== 以下�E従来どおり�E�即時反応でOK�E�E===== */

    // ☁ESW4 長押ぁEↁE保存して終亁E
    if (sw_mode == SW_INFO_PRESS && sw_stat == SW_4)
    {
        SW_CLEAR();
        save_current_message_to_flash();
        //printf("Message recorded\r\n");
        mode = MODE_KEYER;
        last_activity_tick = tim1_tick256;
        draw_keyer_screen();
        ssd1306_refresh();
        return;
    }

    // ☁ESW1 長押ぁEↁEカーソル以降削除
    if (sw_mode == SW_INFO_PRESS && sw_stat == SW_1)
    {
        SW_CLEAR();
        //edit_clear_after_cursor();
        msgs[cur_msg][edit_pos] = '\0';
        edit_len = strlen(msgs[cur_msg]);
        draw_edit_screen();
        return;
    }

        // ☁ESW1 ↁE前�E斁E��E
    if (sw_mode == SW_INFO_CLICK && (sw_stat & SW_1)) {
        msgs[cur_msg][edit_pos] = prev_char(msgs[cur_msg][edit_pos]);
        draw_edit_screen();
        return;
    }

    // ☁ESW2 ↁE次の斁E��E
    if (sw_mode == SW_INFO_CLICK && (sw_stat & SW_2)) {
        msgs[cur_msg][edit_pos] = next_char(msgs[cur_msg][edit_pos]);
        draw_edit_screen();
        return;
    }

    // ☁ESW4 ↁEカーソル進む
    if (sw_mode == SW_INFO_CLICK && sw_stat == SW_4)
    {
        SW_CLEAR();
        if (edit_pos < MSG_LEN - 1)
        {
            edit_pos++;
            if (edit_pos >= edit_len)
            {
                msgs[cur_msg][edit_pos] = ' ';
                edit_len = edit_pos + 1;
                msgs[cur_msg][edit_len] = '\0';
            }
            adjust_edit_view();
        }
        draw_edit_screen();
    }

    // ☁ESW3 ↁEカーソル戻ぁE
    if (sw_mode == SW_INFO_CLICK && sw_stat == SW_3 && edit_pos > 0)
    {
        SW_CLEAR();
        edit_pos--;
        adjust_edit_view();
        draw_edit_screen();
    }
}

//==========================================
//  #6 設定モーチE
//==========================================
void handle_setup_mode(void)
{
    //TODO: 封E��の拡張用
}

//==========================================
//  #1 スタート画面
//==========================================
void draw_startup_screen(void)
{
    ssd1306_drawstr_sz(0, 10, "KEYER DS", 1, fontsize_16x16);
    ssd1306_drawstr_sz(0, 30, "Powered by", 1, fontsize_8x8);
    ssd1306_drawstr_sz(40, 40, "UIAPduino", 1, fontsize_8x8);
     ssd1306_drawFastHLine(0, 50, 128, 1);
    ssd1306_drawstr_sz(0, 52, "Version 0.1", 1, fontsize_8x8);
    ssd1306_refresh();
}

//==========================================
//  #2 キーヤー画面
//==========================================

void draw_keyer_screen(void)
{
    ssd1306_setbuf(0); // 0=黁E 1=白
    ssd1306_drawstr_sz(0, 0, "KEYER", 1, fontsize_8x8);
    ssd1306_drawstr_sz(64, 0, "WPM:", 1, fontsize_8x8);
    draw_wpm_value();
    ssd1306_drawFastHLine(0, 10, 128, 1);
}

//==========================================
//  #3 録音するメモリーを選択する画面
//==========================================
void draw_edit_select(void)
{
    ssd1306_setbuf(0); // 0=黁E 1=白
    ssd1306_drawstr_sz(0, 0, "EDIT MSG", 1, fontsize_8x8);
    ssd1306_drawFastHLine(0, 10, 128, 1);
    ssd1306_drawstr(0, 16, "F1-F4:SELECT MSG", 1);
    ssd1306_drawstr(0, 25, "F1+F2:Return", 1);    
    ssd1306_fillRect(3, 48, 27, 16, 1);
    ssd1306_fillRect(35, 48, 27, 16, 1);
    ssd1306_fillRect(67, 48, 27, 16, 1);
    ssd1306_fillRect(99, 48, 27, 16, 1);    
    ssd1306_drawstr_sz(12, 48, "1", 0, fontsize_16x16);  
    ssd1306_drawstr_sz(44, 48, "2", 0, fontsize_16x16);
    ssd1306_drawstr_sz(76, 48, "3", 0, fontsize_16x16);
    ssd1306_drawstr_sz(108, 48, "4", 0, fontsize_16x16);
    //printf("Select Message\r\n");
    ssd1306_refresh();
    // oled_refreshed_this_frame = true;
}

//==========================================
//  #4 メモリー編雁E
//==========================================
void draw_edit_screen(void)
{
    char buf[32];

    ssd1306_setbuf(0);

    /* ===== タイトル ===== */
    sprintf(buf, "EDIT MSG%d", cur_msg + 1);
    ssd1306_drawstr_sz(0, 0, buf, 1, fontsize_8x8);

    /* ===== カーソル位置表示 (XX/63) ===== */
    sprintf(buf, "%02d/%d", edit_pos + 1, MSG_LEN - 1);
    ssd1306_drawstr_sz(80, 0, buf, 1, fontsize_8x8);

    ssd1306_drawFastHLine(0, 10, 128, 1);

/* ===== メチE��ージ表示用フォント幁E��義 ===== */
#define EDIT_FONT_W 12

    /* ===== メチE��ージ表示�E�E斁E��ウィンドウ�E�E==== */
    for (uint8_t i = 0; i < DISP_COLS; i++)
    {
        uint8_t idx = edit_view_left + i;
        char c = (idx < edit_len) ? msgs[cur_msg][idx] : ' ';
        ssd1306_drawchar_sz(i * EDIT_FONT_W, 14, c, 1, fontsize_16x16);
    }

/* ===== カーソル ===== */
    int cursor_col = edit_pos - edit_view_left;
    if (cursor_col < 0)
        cursor_col = 0;
    if (cursor_col >= DISP_COLS)
        cursor_col = DISP_COLS - 1;

    int x = cursor_col * EDIT_FONT_W+EDIT_FONT_W/2 - 4; // カーソルを文字�E中央に配置
    ssd1306_drawchar_sz(x, 32, '^', 1, fontsize_8x8);

    /* ===== 操作説昁E===== */

    ssd1306_fillRect(3, 40, 27, 8, 1);
    ssd1306_fillRect(35, 40, 27, 8, 1);
    ssd1306_fillRect(67, 40, 27, 8, 1);
    ssd1306_fillRect(99, 40, 27, 8, 1);

    const char arrow_up[]    = { 0x04, '\0' };
    const char arrow_down[]  = { 0x01, '\0' };
    const char arrow_left[]  = { 0x02, '\0' };
    const char arrow_right[] = { 0x03, '\0' };
    ssd1306_drawstr_sz(12, 40, (char*)arrow_up, 0, fontsize_8x8);  // 0はスペ�Eス�E��E�E�表示
    ssd1306_drawstr_sz(44, 40, (char*)arrow_down, 0, fontsize_8x8);  // 1はスペ�Eス�E��E�E�表示
    ssd1306_drawstr_sz(76, 40, (char*)arrow_left, 0, fontsize_8x8);  // 2はスペ�Eス�E��E�E�表示
    ssd1306_drawstr_sz(108, 40, (char*)arrow_right, 0, fontsize_8x8); // 3はスペ�Eス�E��E�E�表示

    ssd1306_drawstr_sz(3, 48, "Press >1sec", 1, fontsize_8x8);
    ssd1306_fillRect(3, 56, 27, 8, 1);
    ssd1306_fillRect(35, 56, 27, 8, 1);
    ssd1306_fillRect(67, 56, 27, 8, 1);
    ssd1306_fillRect(99, 56, 27, 8, 1);

    ssd1306_drawstr_sz(7, 56, "DEL", 0, fontsize_8x8);
    ssd1306_drawstr_sz(103, 56, "END", 0, fontsize_8x8);
    ssd1306_refresh();
    // oled_refreshed_this_frame = true;
}

//==========================================
//  #5 シスチE��メチE��ージ表示
//==========================================
void draw_sys_message(const char *msg)
{
    ssd1306_setbuf(0); // ↁEこれだけでOK
    ssd1306_drawstr_sz(0, 16, (char *)msg, 1, fontsize_16x16);
     ssd1306_refresh();
    // oled_refreshed_this_frame = true;
}

//==========================================
//  タイマ�E割り込み
//==========================================
void TIM1_UP_IRQHandler(void)
{
    /* 割り込みフラグクリア */
    TIM1->INTFR &= (uint16_t)~TIM_IT_Update;

    /* キー処琁E��ここで1回だけ！E*/
    service_keyer();

    /* スイチE��処琁E��刁E�� */
    if (++sw_div_cnt >= SW_SCAN_DIV)
    {
        sw_div_cnt = 0;
        sw_check();
    }

    /* IRQ tick�E�デバッグ用ならOK�E�E*/
    tim1_tick256++;

    /* ト�Eン制御 */
    if (tone_on)
    {
        if (++tone_div >= TONE_DIV)
        {
            tone_div = 0;
            toggle_tone_pin();
        }
    }
    else
    {
        GPIO_digitalWrite(PIN_TONE, low);
    }
}

//==========================================
//  スタンバイモードへ入めE
//==========================================
void enter_standby(void)
{
    const uint32_t wake_lines =
        exti_line_from_pin(PIN_SW1) | exti_line_from_pin(PIN_SW2) |
        exti_line_from_pin(PIN_SW3) | exti_line_from_pin(PIN_SW4) |
        exti_line_from_pin(PIN_DOT) | exti_line_from_pin(PIN_DASH) |
        exti_line_from_pin(PIN_ST); 

    tone_on = false;
    GPIO_digitalWrite(PIN_TONE, low);
    GPIO_digitalWrite(PIN_KEYOUT, low);

    // OLEDバッファをバチE��アチE�Eに保孁E
    memcpy(ssd1306_buffer_backup, ssd1306_buffer, sizeof(ssd1306_buffer));

    TIM1->DMAINTENR &= ~TIM_IT_Update;
    TIM1->CTLR1 &= ~TIM_CEN;
    // 割り込みを無効化（有効化ではなく無効化！E
    NVIC_DisableIRQ(TIM1_UP_IRQn);
    NVIC_ClearPendingIRQ(TIM1_UP_IRQn);

    GPIO_ADC_set_power(0);
    ADC1->CTLR1 = 0;
    ADC1->CTLR2 = 0;

    // --- OLED OFF�E�最重要E��E---
    ssd1306_cmd(SSD1306_DISPLAYOFF);   // あなた�ESSD1306ライブラリに合わせて
    I2C1->CTLR1 &= ~I2C_CTLR1_PE;

    prepare_gpio_for_standby();

    RCC->APB1PCENR &= ~(RCC_APB1Periph_I2C1 | RCC_APB1Periph_TIM2);
    RCC->APB2PCENR &= ~(RCC_APB2Periph_ADC1 | RCC_APB2Periph_TIM1 | RCC_APB2Periph_USART1);

    // --- PWR クロチE��有効匁E---
    RCC->APB1PCENR |= RCC_APB1Periph_PWR;
    RCC->APB2PCENR |= RCC_APB2Periph_AFIO;

    // --- SLEEPDEEP = 1 ---
    exti_select_port(PIN_SW1 & 0x0f, 3);   // PD0
    exti_select_port(PIN_SW2 & 0x0f, 2);   // PC3
    exti_select_port(PIN_SW3 & 0x0f, 3);   // PD2
    exti_select_port(PIN_SW4 & 0x0f, 2);   // PC4
    exti_select_port(PIN_DOT & 0x0f, 2);   // PC5
    exti_select_port(PIN_DASH & 0x0f, 2);  // PC6
    exti_select_port(PIN_ST & 0x0f, 0);    // PA1

    EXTI->INTENR &= ~wake_lines;
    EXTI->EVENR = (EXTI->EVENR & ~0x7fu) | wake_lines;
    EXTI->RTENR &= ~wake_lines;
    EXTI->FTENR = (EXTI->FTENR & ~0x7fu) | wake_lines;
    EXTI->INTFR = wake_lines;

    NVIC->SCTLR |= (1 << 2);

    // --- Standby モード選抁E---
    PWR->CTLR |= PWR_CTLR_PDDS;

    // --- 復帰イベント設定！EW1〜SW4, DOT, DASH, ST�E�E---
    // 例：PA1, PA2, PA3, PA4, PC1, PC2, PC3 など
    // --- Standby へ ---
    __WFE();   // ↁEIRQ ではなぁEEVT で復帰
    restore_after_standby();
}

//==========================================
//  ループ�E琁E
//==========================================
void loop(void)
{
    //oled_refreshed_this_frame = false;   // ☁Eフレーム開姁E
    static uint16_t sec_cnt = 0;
    static uint32_t last_tick = 0;

    /* ==== 10ms周朁E(IRQ基溁E 256us ÁE40 ≁E10ms) ==== */
    if ((tim1_tick256 - last_tick) >= 40)
    {
        last_tick += 40; // advance by 40 ticks
        edit_tick_10ms = true;

        /* ==== EDIT中以外�EWPM更新 ==== */
        if (mode != MODE_EDIT)
        {
            update_speed_from_adc();
        }

        /* ==== 1秒周朁E==== */
        if (++sec_cnt >= 100)
        { // 100 ÁE10ms = 1s
            sec_cnt = 0;
#if DEBUG_MODE_PRINT
            //printf("[MODE] %s\r\n", mode_to_str(mode));
#endif
        }
    }

    /* ==== モード�E琁E==== */
    switch (mode)
    {
    case MODE_KEYER:
        handle_keyer_mode();
        break;
    case MODE_PLAY:
        handle_play_mode();
        break;
    case MODE_EDIT_SELECT:
        handle_edit_select();
        break;
    case MODE_EDIT:
        handle_edit_mode();
        break;
    case MODE_SETUP:
        handle_setup_mode();
        break;
    }

    cw_decode_task();

    // ==== 無音タイムアウト（最後�E斁E��確定用�E�E====
    // ☁Eメモリ再生中は自動デコード確定を無効匁E
    if (!key_state && !flush_done && !auto_mode) {

        if (key_off_ticks >= WORD_GAP_MIN) {
            cw_push(EV_WORD_GAP);
            flush_done = true;
        }

    }
    else if (key_state) {
        flush_done = false;
    }

    // ☁Eフレームの最後に refresh めE回だぁE
    //ssd1306_refresh();

    // ☁E無操佁E0秒でスリーチE
    uint32_t now = tim1_tick256;
    if (!key_state &&
        !auto_mode &&
        mode != MODE_PLAY &&
        mode != MODE_EDIT_SELECT &&
        mode != MODE_EDIT &&
        (now - last_activity_tick) > 39062)   // 10私E
    {

        standby_magic = STANDBY_MAGIC_VALUE;   // ☁E復帰フラグ
        enter_standby();
        // // OLED OFF
        // ssd1306_cmd(SSD1306_DISPLAYOFF);

        // wake_flag = false;

        // // ☁EWFI ループ（割り込みで wake_flag が立つ�E�E
        // while (!wake_flag) {
        //     __WFI();
        // }

        // // 復帰処琁E
        // ssd1306_cmd(SSD1306_DISPLAYON);
        // ssd1306_refresh();

        // last_activity_tick = tim1_tick256;
    }

}

/* ===============================
 * PLAY / SAVE 仮実裁E��ダミ�E�E�E
 * =============================== */

void start_play(uint8_t msg)
{
    cur_msg = msg;
    last_activity_tick = tim1_tick256;

    auto_msg = msgs[msg]; // ☁E追加�E��E生する文字�Eを指宁E
    sys_msg_active = false;
    keyout_enabled = true;

    // 再生開始時に未確定データめE��イマをクリアして、前回�E残りで誤チE��ードされなぁE��ぁE��する
    morse_len = 0;    // 未確定�Eモールス符号を破棁E
    cw_r = cw_w;      // CWイベントバチE��ァをクリア
    key_off_ticks = 0;
    key_on_ticks = 0;
    flush_done = true; // 無音タイムアウト�E琁E��一時的に抑止

    auto_mode = true;
    req_reset_auto = true;
    mode = MODE_PLAY;
}

void stop_play(void)
{
    auto_mode = false;     // 自動送信OFF
    req_reset_auto = true; // job_auto 冁E��状態を初期匁E

    // パドル入力を無視（停止操作が音声にならなぁE��ぁE��するため�E�E
    ignore_paddle_input = true;

    // スイチE��マスクをセチE���E�停止操作�E入力を無視！E
    sw_mask = 1;

    // ===== 追加 =====
    morse_len = 0;         // ↁE未確定文字を破棁E

    // ☁Eここで「次の1回�E忁E��描き直せ」と持E��
    last_wpm = -999;
    update_speed_from_adc();
    ssd1306_refresh();

}

//==========================================
//  main loop
//==========================================
int main()
{

    // 初期化（最初に実行すめE
    SystemInit();
    // RCC->APB1PCENR |= RCC_APB1Periph_PWR; // PWR クロチE��有効匁E
    // bool from_standby = (PWR->CSR & PWR_CSR_SBF); // 復帰允E��スタンバイかどぁE��
    // PWR->CTLR |= PWR_CTLR_CWUF | PWR_CTLR_CSBF; // 復帰フラグクリア
    ssd1306_i2c_init();
    ssd1306_init();
    //ssd1306_cmd(SSD1306_DISPLAYOFF);
    //__WFI(); // 割り込み征E��
    GPIO_setup(); // gpio Setup;
    GPIO_ADCinit();
    tim1_int_init(); //
    init_flash_messages();
    // tim2_pwm_init();             // TIM2 PWM Setup

    bool resumed_from_standby = (standby_magic == STANDBY_MAGIC_VALUE);

    //スタンバイからの復帰かどぁE��を判宁E
    if (resumed_from_standby) {
        standby_magic = 0;  // 次回�Eためにクリア
        reset_decoded_display();

        // スタンバイ復帰時�EOLEDをリセチE��して完�Eにクリアしてから再描画
        ssd1306_cmd(SSD1306_DISPLAYOFF);
        ssd1306_init();
        for (int i = 0; i < 4; i++) {     // 褁E��回クリアして確実に初期匁E
            ssd1306_setbuf(0);
            ssd1306_refresh();
        }
        draw_keyer_screen();
    }
    else {
        // 通常起動時はフラグを�E期化
        standby_magic = 0;

        reset_decoded_display();

        Delay_Ms(1000);
        draw_startup_screen(); // スタート画面
        play_sys_msg("OK", 20);
        Delay_Ms(1000);
        draw_keyer_screen(); // キーヤー画面
    }
    // ☁E起動直後に WPM を強制描画して refresh
    update_speed_from_adc();
    ssd1306_refresh();

    // ループ�E琁E
    while (1)
    {
        loop();
    }
}
