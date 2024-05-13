#include <stdarg.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "FreeRTOS.h"
#include "task.h"
#include "timers.h"
#include "semphr.h"
#include "event_groups.h"
#include "ht32.h"
//#include "main.h"
#include "console.h"
#include "MFRC522_conf.h"
#include "MFRC522.h"

#define max(a, b)   (((a) > (b)) ? (a) : (b))
#define min(a, b)   (((a) > (b)) ? (b) : (a))

#define STACK_SIZE              200         // u32 type
#define dbguSTACK_SIZE			300         // u32 type
#define dbguQUEUE_SIZE			200         // u8 type
#define dbguDELAY			    ((portTickType) 20 / portTICK_RATE_MS)
#define STR_USAGE	"Usage: "
#define BIT0        1
#define BIT1        2
#define BIT2        4
#define BIT3        8
#define LR_ASCII 	0x0A
#define BUF_SIZE 	512

// Debug message control defition (verbose)
#define DEBUG_TIMER     1
#define DEBUG_SONIC     2
#define DEBUG_MOTOR     4
#define DEBUG_RFID      8
#define DEBUG_LCD       0x10
#define DEBUG_IR        0x20
#define DEBUG_SERVO     0x40
#define DEBUG_KEYPAD    0x80

enum {
    INFRA_INVALID,
    INFRA_CHM, INFRA_CH, INFRA_CHP, INFRA_PREV, INFRA_NEXT, INFRA_PLAY, INFRA_VOLM, INFRA_VOLP, INFRA_VOLEQ,
    INFRA_100P, INFRA_200P, INFRA_0, INFRA_1, INFRA_2, INFRA_3, INFRA_4, INFRA_5, INFRA_6, INFRA_7, INFRA_8, INFRA_9,
};

enum {
    LCDMSG_RFID,
    LCDMSG_IRKEY,
    LCDMSG_BALLDROPPING,        // indicate how many balls have been dropped.
    LCDMSG_BALLDROPPED,         // indicate all balls have been dropped.
    LCDMSG_KEYPAD,
    LCDMSG_KEYRESULT,
    LCDMSG_UPDATESTATUS,
    LCDMSG_PLAY,
    LCDMSG_PAUSE,
    LCDMSG_STOP,
};

enum {
    SERVO_OPENDOOR,     // open the door if the input secret key is correct from the keypad.
    SERVO_CLOSEDOOR,    // close the door using the IR control.
    
    SERVO_DROPBALL,
    SERVO_RESET,        // the command is used to stop dropping down the ball.
};

typedef struct {
    int id;
    uint8_t rfid[5];
    int key;            // valid if id is LCDMSG_IRKEY, LCDMSG_KEYPAD or LCDMSG_BALLDROPPING
} LCDMsg;

QueueHandle_t LCDQueue, ServoQueue, MusicQueue;

static portTASK_FUNCTION_PROTO(vDbguQueueConsumer, pvParameters);
static portTASK_FUNCTION_PROTO(vDbguCmdLine, pvParameters);
static xQueueHandle xDbguQueue;

const char banner[] = "\n\r=== ESK32-30501 ===\n\r";
static const char erase_seq[] = "\b \b";
static const char PROMPT_STRING[] = "Console> ";

typedef struct cmds_s {
	char *name;
	int (*func)(int argc, char *argv[], const struct cmds_s *cmd_p);
	char *argc_errmsg;
} cmds_t;

char LogBuf[BUF_SIZE];
volatile int log_rpos = 0;
volatile int log_wpos = 0;

static EventGroupHandle_t xEventGroup;
uint32_t verbose;
int manual_mode = 0;    // set for controlling via command line.

// define motor operations
#define FORWARD     0
#define BACKWARD    1

static int motor_dir = FORWARD;

static const struct {
    char *str;
    uint32_t debug;
} verbose_tbl[] = {
    { "timer", DEBUG_TIMER },
    { "sonic", DEBUG_SONIC },
    { "motor", DEBUG_MOTOR },
    { "rfid", DEBUG_RFID },
    { "lcd", DEBUG_LCD },
    { "ir", DEBUG_IR },
    { "servo", DEBUG_SERVO },
    { "keypad", DEBUG_KEYPAD },
    { "all", DEBUG_TIMER | DEBUG_SONIC | DEBUG_MOTOR | DEBUG_RFID | DEBUG_LCD | DEBUG_IR | DEBUG_SERVO | DEBUG_KEYPAD },
    { NULL, 0 }
};

void xPortSysTickHandler(void);
void SysTick_Handler()
{
    if (xTaskGetSchedulerState() != taskSCHEDULER_NOT_STARTED) {
        xPortSysTickHandler();
    }
}

void UART_Transmit(const char buf[], int n)
{
    for (int i = 0; i < n; i++) {
        while (USART_GetFlagStatus(HT_USART1, USART_FLAG_TXC) == RESET);
        USART_SendData(HT_USART1, buf[i]);
    }
}

int UART_Receive(uint8_t *data)
{
    if (USART_GetFlagStatus(HT_USART1, USART_FLAG_RXDR) == SET) {
        *data = USART_ReceiveData(HT_USART1);
    } else {
        return 0;
    }
    return 1;
}

static portTASK_FUNCTION(MonitorTask, pvParameters)
{
    while (1) {
        xEventGroupWaitBits(xEventGroup, BIT0, pdFALSE, pdFALSE, portMAX_DELAY);
        Printf("Hello\n");
        vTaskDelay(1000);
    }
}

#if 0
static const uint8_t motor_seq[][4] = {     // full-step sequence
    { 1, 0, 1, 0 }, { 0, 1, 1, 0 }, { 0, 1, 0, 1 }, { 1, 0, 0, 1}
};
#endif

static const uint8_t motor_seq[][4] = {     // half-step control
    { 0, 0, 0, 1 }, { 0, 0, 1, 1 }, { 0, 0, 1, 0 }, { 0, 1, 1, 0 },
    { 0, 1, 0, 0 }, { 1, 1, 0, 0 }, { 1, 0, 0, 0 }, { 1, 0, 0, 1 }
};

// PIN Mapping
// PC11 -> INA (Blue); PC12 -> INC (Yellow)
// PC14 -> INB (Pink); PC15 -> IND (Orange)
static void MotorPulse()
{
    if (motor_dir == FORWARD) {
        for (int i = 0; i < 8 ; i++) {
            GPIO_WriteOutBits(HT_GPIOC, GPIO_PIN_11, (motor_seq[i][0] == 0) ? RESET : SET);
            GPIO_WriteOutBits(HT_GPIOC, GPIO_PIN_12, motor_seq[i][1]);
            GPIO_WriteOutBits(HT_GPIOC, GPIO_PIN_14, motor_seq[i][2]);
            GPIO_WriteOutBits(HT_GPIOC, GPIO_PIN_15, motor_seq[i][3]);
            vTaskDelay(5);
        }
    } else {
        for (int i = 7; i >= 0; i--) {
            GPIO_WriteOutBits(HT_GPIOC, GPIO_PIN_11, motor_seq[i][0]);
            GPIO_WriteOutBits(HT_GPIOC, GPIO_PIN_12, motor_seq[i][1]);
            GPIO_WriteOutBits(HT_GPIOC, GPIO_PIN_14, motor_seq[i][2]);
            GPIO_WriteOutBits(HT_GPIOC, GPIO_PIN_15, motor_seq[i][3]);
            vTaskDelay(5);
        }
    }
}
            
#if 1
static portTASK_FUNCTION(MotorTask, pvParameters)
{
    while (1) {
        if (manual_mode) {
            GPIO_WriteOutBits(HT_GPIOC, GPIO_PIN_11, 0);
            GPIO_WriteOutBits(HT_GPIOC, GPIO_PIN_12, 0);
            GPIO_WriteOutBits(HT_GPIOC, GPIO_PIN_14, 0);
            GPIO_WriteOutBits(HT_GPIOC, GPIO_PIN_15, 0);
            xEventGroupWaitBits(xEventGroup, BIT1, pdFALSE, pdFALSE, portMAX_DELAY);
        }

        MotorPulse();
    }
}
#endif

void PutIntoLogBuf(char* pSrc, int len)
{
	int cnt = 0;

	while (cnt < len && ((log_wpos+1) % BUF_SIZE) != log_rpos) {
		LogBuf[log_wpos] = pSrc[cnt];

		cnt++;
		log_wpos = (log_wpos + 1) % BUF_SIZE;

		if (LogBuf[(log_wpos + BUF_SIZE - 1) % BUF_SIZE] == '\n' && ((log_wpos + 1) % BUF_SIZE) != log_rpos) {
			LogBuf[log_wpos] = '\r';
            log_wpos = (log_wpos + 1) % BUF_SIZE;
		}
	}
}

char qbuf[BUF_SIZE];
static portTASK_FUNCTION(vDbguQueueConsumer, pvParameters)
{
    while (1) {
        taskENTER_CRITICAL();
        int n = ((log_wpos + BUF_SIZE) - log_rpos) % BUF_SIZE;
        int curpos = log_rpos, i;
        for (i = 0; i < n; i++) {
            qbuf[i] = LogBuf[log_rpos];
            log_rpos = (log_rpos + 1) % BUF_SIZE;
        }
        taskEXIT_CRITICAL();

        if (n != 0) {
            UART_Transmit(qbuf, n);

            taskENTER_CRITICAL();
            if (log_rpos != log_wpos) {
                taskEXIT_CRITICAL();
                continue;
            }
            taskEXIT_CRITICAL();
        }

        vTaskDelay(5);
	}
}

void Printf(char *msg, ...)
{
	char buf[dbguQUEUE_SIZE];

	va_list arg;
	va_start(arg, msg);
    vsnprintf(buf, dbguQUEUE_SIZE, msg, arg);
    taskENTER_CRITICAL();
	PutIntoLogBuf(buf, strlen(buf));
    taskEXIT_CRITICAL();
	va_end(arg);
}

static void __Printf(const char *msg, ...)
{
	char buf[dbguQUEUE_SIZE];

	va_list arg;
	va_start(arg, msg);
    vsnprintf(buf, dbguQUEUE_SIZE, msg, arg);
    taskENTER_CRITICAL();
	PutIntoLogBuf(buf, strlen(buf));
    taskEXIT_CRITICAL();
	va_end(arg);
}

void dprintf(int dbglevel, char *msg, ...)
{
	char buf[dbguQUEUE_SIZE];

    if (verbose & dbglevel) {
        va_list arg;
        va_start(arg, msg);
        vsnprintf(buf, dbguQUEUE_SIZE, msg, arg);
        taskENTER_CRITICAL();
        PutIntoLogBuf(buf, strlen(buf));
        taskEXIT_CRITICAL();
        va_end(arg);
    }
}

void PrintfFromISR(char *msg, ...)
{
    char buf[64];

	va_list arg;
	va_start(arg, msg);
	vsnprintf(buf, 64, msg, arg);
	PutIntoLogBuf(buf, strlen(buf));
	va_end(arg);
}

#define CMD_NAME_SIZE		16
static int cmd_show(const cmds_t *cmdp, char *buf, int buf_size, int *ii)
{
	int ii1 = (*ii) * CMD_NAME_SIZE;

	strncpy(buf+ii1, cmdp->name, strlen(cmdp->name) > CMD_NAME_SIZE ? CMD_NAME_SIZE : strlen(cmdp->name));
	if (*ii == 3) {
		Printf("%s\n", buf);
		memset(buf, ' ', buf_size - 1);
		buf[buf_size-1] = '\0';
	}
	return 0;
}

#define SHOW_BUF_SIZE		65
static int cmd_show_valid(const cmds_t	cmds[])
{
	char	buf[SHOW_BUF_SIZE];
	const cmds_t	*cmdp;
	int ii;

	memset(buf, ' ', SHOW_BUF_SIZE-1);
	buf[SHOW_BUF_SIZE-1] = '\0';
	for (ii = 0, cmdp = cmds; cmdp->name != NULL; cmdp++, ii = (ii+1) % 4)
		cmd_show(cmdp, buf, SHOW_BUF_SIZE, &ii);

	if (ii != 0)
		Printf ("%s\n", buf);
	return 0;
}

static int doShow(int argc, char *argv[], const cmds_t *cmd_p)
{
     Printf("ESK32-30501\n");

     return 0;
}

static int doStartMonitor(int argc, char *argv[], const cmds_t *cmd_p)
{
    xEventGroupSetBits(xEventGroup, BIT0);
    return 0;
}

static int doStopMonitor(int argc, char *argv[], const cmds_t *cmd_p)
{
    xEventGroupClearBits(xEventGroup, BIT0);
    return 0;
}

static int MotorRun(int argc, char *argv[], const cmds_t *cmd_p)
{
    xEventGroupSetBits(xEventGroup, BIT1);
    return 0;
}

static int MotorStop(int argc, char *argv[], const cmds_t *cmd_p)
{
    xEventGroupClearBits(xEventGroup, BIT1);
    return 0;
}

static int MotorForward(int argc, char *argv[], const cmds_t *cmd_p)
{
    motor_dir = FORWARD;
    return 0;
}

static int MotorBackward(int argc, char *argv[], const cmds_t *cmd_p)
{
    motor_dir = BACKWARD;
    return 0;
}

static int ServoRun(int argc, char *argv[], const cmds_t *cmd_p)
{
    xEventGroupSetBits(xEventGroup, BIT2);
    return 0;
}

static int ServoStop(int argc, char *argv[], const cmds_t *cmd_p)
{
    xEventGroupClearBits(xEventGroup, BIT2);
    return 0;
}

typedef struct {
    u16 note;
    u16 freq;
} FreqTbl;

const FreqTbl Octave3[] = {
    { 'C', 131 },
    { 'C' | ('#' << 8), 139 },
    { 'D' | ('-' << 8), 139 },
    { 'D', 147},
    { 'D' | ('#' << 8), 156 },
    { 'E' | ('-' << 8), 156 },
    { 'E', 165 },
    { 'F', 175 },
    { 'F' | ('#' << 8), 185 },
    { 'G' | ('-' << 8), 185 },
    { 'G', 196 },
    { 'G' | ('#' << 8), 208 },
    { 'A' | ('-' << 8), 208 },
    { 'A', 220 },
    { 'A' | ('#' << 8), 233 },
    { 'B' | ('-' << 8), 233 },
    { 'B', 247 },
};

const FreqTbl Octave4[] = {
    { 'C', 262 },
    { 'C' | ('#' << 8), 277 },
    { 'D' | ('-' << 8), 277 },
    { 'D', 294 },
    { 'D' | ('#' << 8), 311 },
    { 'E' | ('-' << 8), 311 },
    { 'E', 330 },
    { 'F', 349 },
    { 'F' | ('#' << 8), 370 },
    { 'G' | ('-' << 8), 370 },
    { 'G', 392 },
    { 'G' | ('#' << 8), 415 },
    { 'A' | ('-' << 8), 415 },
    { 'A', 440 },
    { 'A' | ('#' << 8), 466 },
    { 'B' | ('-' << 8), 466 },
    { 'B', 494 },
};

const FreqTbl Octave5[] = {
    { 'C', 523 },
    { 'C' | ('#' << 8), 554 },
    { 'D' | ('-' << 8), 554 },
    { 'D', 587 },
    { 'D' | ('#' << 8), 622 },
    { 'E' | ('-' << 8), 622 },
    { 'E', 659 },
    { 'F', 698 },
    { 'F' | ('#' << 8), 740 },
    { 'G' | ('-' << 8), 740 },
    { 'G', 784 },
    { 'G' | ('#' << 8), 831 },
    { 'A' | ('-' << 8), 831 },
    { 'A', 880 },
    { 'A' | ('#' << 8), 932 },
    { 'B' | ('-' << 8), 932 },
    { 'B', 988 },
};

const FreqTbl Octave6[] = {
    { 'C', 1047 },
    { 'C' | ('#' << 8), 1109 },
    { 'D' | ('-' << 8), 1109 },
    { 'D', 1175 },
    { 'D' | ('#' << 8), 1245 },
    { 'E' | ('-' << 8), 1245 },
    { 'E', 1319 },
    { 'F', 1397 },
    { 'F' | ('#' << 8), 1480 },
    { 'G' | ('-' << 8), 1480 },
    { 'G', 1568 },
    { 'G' | ('#' << 8), 1661 },
    { 'A' | ('-' << 8), 1661 },
    { 'A', 1760 },
    { 'A' | ('#' << 8), 1865 },
    { 'B' | ('-' << 8), 1865 },
    { 'B', 1976 },
};

// Totoro
const char music0[] =
    "F8G8A8B-8O5C4O4A8F4O5C4O4B-4G8P4.B-4G8E4B-4A4F4"
    "L4PD-FB-AO5C8O4F4.P8A8B-8A8B-8A8B-8A8F8G4.P4"
    "F8G8A8B-8O5C4O4A8F4O5C4O4B-4G8P4.B-4G8E4B-4A4F4"
    "P4D4O5D4C8O4B-8A8B-8O5C4.O4F8F4.A8B-8A8F8B-8A8F8O5D8C P4O4C8C8B-8A8G8A8F4."
;//

const char music1[] = "O3CDEFGABP2O4CDEFGABP2O5CDEFGAB";

// https://m.tintinpiano.com/sheetmusic/98498
const char music2[] = 
    "T110 L8 G O5CCCO4GO5CEG4 FED4EDCO4G O5CCCO4GO5CEG4"
    "FEDO4BO5C4.O4G O5CCEDC4CO4G O5CEGGC4.D16E16 FFFFF8.E16D4"
    "DCDEO4G4.G O5CCEDC4CO4G O5CEGGC4.E FFFDEDCE"
    "DO4GBO5DC4P4 P2D4E4 F4F4F4ED EEEDE4CC"
    "DDDCO4BBBG O5C4D4E4DE F4F4F4ED EEEDC4"
    //"DDD8.E16FEDG4";
    ;

// Canon in D: http://www.tom163.net/yuepuku/waiguogepu/200707/23823.html
const char music3[] =
    "T96 L2 O5ED CO4B AG AB"
    "O5CO4B AG FE FD"
    "L8 O5CO4BO5CO4C O3BO4GDE CO5CO4BABO5EGA FEDFEDCO4B AGFEDFED"
    "CDEFGDGF EAGFGFED CO4AO4ABO5CO4BAG FEDAGAGF"
    "L4 EO5ED2 C2O4B2 O5CEDF"// D2 O5CO4BO5CO4A"
    "L16 G8EFG8EFGO4GABO5CDEF E8CDE8O4EFGAGFGEFG F8AGF8EDEDCDEFGA"
    "F8AGA8BO5CO4GABO5CDEFG E8CDE8DCDO4BO5CDEDCO4B O5C8O4ABO5C8O4CDEFEDEO5CO4BO5C"
    "T96 L16 O4A8O5CO4BA8GFGFEFGABO5C O4A8O5CO4BO5C8O4BABO5CDCO4BO5CO4AB"
    //"L8 O5EO4EFEDO5DED CO4ECAGO3GFG AO4ABABO3GFG AO4AGABBAB"
    "L8 CO5CDCO4BO3BO4CO3B AO4AGABO3BO4ED CO5CDFEO4EGO5E CFEFDO4GFG"
    "L16 E8O5CO4BO5C8O4E8G8GAB8G8 E8O5CDE8C8E8EDC8O4B A8AGA8B8O5C8EDC8E8"

    "L8 O4FC16B16AAGDGG E4.O5EEFED C4.CCDCO4B A2O5C2"
    "CO4B-AB-G4.G G4.GGAGF E4.EEFED O5CO4B-AB-G4.G"
    "L4 FO5CO4B.B O5G2O4G4.F8 O5E2E4.D8 C2.O4C"
    "O5C2O4B2 O5CO4CO3BO4B AO3AGO4G FO5FEO4E"
    "DADO5D EO4EDO5D CO4CO3BO4B AO5AGO4G"
    "F4.O5D8O4GO5D E2";

const char music4[] = 
    "O5F#2E2D2C#2 O4B2A2B2O5C#2 D4F#4A4G4F#4D4F#EDO4BO5DF# GBAG"
;

// Spirited Away: Always with me
const char music5[] =
    "O4F8G8A8F8O5C4.O4A8GO5CO4GF8D8A4.F8E2EDEF8G8CFG8A8"
    "B-B-8A8G8F8G2 F8G8A8F8O5C4.O4A8GO5CO4GF8D8D4E8F8C2C"
    "DEF8G8CFG8A8B-B-8A8G8F8F1"
;

// Spirited Away: One summer's day
const char music6[] = 
    "O5EFG8GG8G8F8E8D8D8E16C2P16 E16F16G8G8G8G8G8F8E8D8D8E8E2 C8D8E4.O4A8O5C2"
    "A8B8O6C8C8D8C8O5B4. E16G16A8A8G8F8G2P16 G8F8F8E-8F4P16 C8F8G4.A-8G4."
;

// Castle in the Sky
const char music7[] =
    "T150 O5C8D8E4.D8EGD2. O4GO5C4.O4B8O5CEE1 O4A8B8O5CO4B8O5C8DC4.O4G8G2"
    "O5FEDCE2. EA4.A8G4.G8E8D8C2CDC8D4.GE2. EA4.A8G4.G8E8D8C2CDC8D4.O4BA2."
;

// An Island
const char music8[] =
    "O4G8O5C8C8E8D8C C8O4G8O5C8E8G8G8C4. D16E16F8F8F8F8F8.E16DD8C8D8E8O4G4."
    "G8O5C8C8E8D8C C8O4G8O5C8E8G8G8C4. E8F8F8F8D8E8D8C8E8D8O4G8B8O5D8C2"
;

static const char *mcoll[] = {
     music0, music2, music5, music6, music7, music3, 
};

enum {
    MUSIC_PLAY,
    MUSIC_STOP,
    MUSIC_PAUSE,
    MUSIC_NEXT,
    MUSIC_PREV,
};

static int music_state, music_idx;

static void InitSound(int freq)
{
    u32 reload = SystemCoreClock / 4 / freq - 1;
    u32 compare = (reload + 1) * (100 - 33) / 100;
    TM_SetCounterReload(HT_SCTM1, reload);
    TM_SetCaptureCompare(HT_SCTM1, TM_CH_0, compare);
    TM_ChannelConfig(HT_SCTM1, TM_CH_0, TM_CHCTL_ENABLE);
    TM_Cmd(HT_SCTM1, ENABLE);
}

static portTASK_FUNCTION(MusicTask, pvParameters)
{
    int freq, period;
    int ledtoggle;
    u16 note;
    int cmd;
    const char *p = mcoll[music_idx];
    FreqTbl *ptbl = (FreqTbl *) Octave4;
    int rhythm = 4, setrhythm;
    int T = 375, setT;

    
    music_state = MUSIC_STOP;
    while (1) {
        if (xQueueReceive(MusicQueue, &cmd, 0) == pdPASS) {
            switch (music_state) {
            case MUSIC_PLAY:
                if (cmd == INFRA_PLAY) {
                    music_state = MUSIC_PAUSE;
                } else if (cmd == INFRA_VOLEQ) {    // this key is translated as "STOP" command.
                    music_state = MUSIC_STOP;
                } else if (cmd == INFRA_NEXT) {
                    if (music_idx < sizeof(mcoll)/sizeof(mcoll[0]) - 1) {
                        p = mcoll[++music_idx];
                        ptbl = (FreqTbl *) Octave4;
                        rhythm = 4;
                    }
                } else if (cmd == INFRA_PREV) {
                    if (music_idx != 0) {
                        p = mcoll[--music_idx];
                        ptbl = (FreqTbl *) Octave4;
                        rhythm = 4;
                    }
                }
                continue;
            case MUSIC_STOP:
                if (cmd == INFRA_PLAY) {
                    p = mcoll[music_idx];
                    ptbl = (FreqTbl *) Octave4;
                    music_state = MUSIC_PLAY;
                } else if (cmd == INFRA_NEXT) {
                    if (music_idx < sizeof(mcoll)/sizeof(mcoll[0]) - 1) {
                        music_idx++;
                    }              
                } else if (cmd == INFRA_PREV) {
                    if (music_idx != 0)
                        music_idx--;
                }
                continue;
            case MUSIC_PAUSE:
                if (cmd == INFRA_PLAY) {
                    music_state = MUSIC_PLAY;
                } else if (cmd == INFRA_VOLEQ) {
                    music_state = MUSIC_STOP;
                }
                continue;
            }
                
        } else {
            if (music_state == MUSIC_PAUSE || music_state == MUSIC_STOP) {
                GPIO_WriteOutBits(HT_GPIOC, GPIO_PIN_14, ledtoggle ? 1 : 0);
                GPIO_WriteOutBits(HT_GPIOC, GPIO_PIN_15, ledtoggle ? 1 : 0);
                ledtoggle = !ledtoggle;
                vTaskDelay(500);
                continue;
            }
        }
        
        if (*p == 0) {
            p = mcoll[music_idx];
            vTaskDelay(2000);
            continue;
        }
        
        setT = setrhythm = 0;
        int duration = rhythm;
        if (*p >= 'A' && *p <= 'G') {
            note = *p++;
            if (*p == '#' || *p == '-') {
                note |= *p << 8;
                p++;
            }

            for (int i = 0; i < 17; i++) {
                if (ptbl[i].note == note) {
                    freq = ptbl[i].freq;
                    break;
                }
            }
        } else if (*p == 'P') {
            p++;
            freq = 0;
        } else if (*p == 'O') {
            p++;
            switch (*p++) {
            case '3':
                ptbl = (FreqTbl *) Octave3;
                break;
            case '4':
                ptbl = (FreqTbl *) Octave4;
                break;
            case '5':
                ptbl = (FreqTbl *) Octave5;
                break;
            case '6':
                ptbl = (FreqTbl *) Octave6;
                break;
            }
            continue;
        } else if (*p == 'L') {
            p++;
            setrhythm = 1;
        } else if (*p == 'T') {
            setT = 1;
            p++;
        } else {
            p++;
            continue;
        }
        
        if (*p >= '0' && *p <= '9') {
            duration = 0;
            for (duration = 0; *p >= '0' && *p <= '9'; p++) {
                duration = duration * 10 + *p - '0';
            }
        }
        
        if (setT) {
            T = 60000 / duration;       // units ms/beat
            continue;
        }
        
        if (setrhythm) {
            rhythm = duration;
            continue;
        }
        
        period = T * 4 / duration;
        if (*p == '.') {
            duration <<= 1;
            period += T * 4 / duration;
            p++;
        }
        
        GPIO_WriteOutBits(HT_GPIOC, GPIO_PIN_14, ledtoggle ? 1 : 0);
        GPIO_WriteOutBits(HT_GPIOC, GPIO_PIN_15, ledtoggle ? 0 : 1);
        ledtoggle = !ledtoggle;
        InitSound(freq);
        vTaskDelay(period * 80 / 100);
        TM_ChannelConfig(HT_SCTM1, TM_CH_0, TM_CHCTL_DISABLE);
        vTaskDelay(period * 20 / 100);
    }
}

u8 _backlightval;
u8 _displaycontrol;
u8 _displayfunction;

void I2C_Write(HT_I2C_TypeDef* I2Cx, u16 slave_address, u8* buffer, u8 BufferSize)
{
	u8 Tx_Index = 0;
	
  /* Send I2C START & I2C slave address for write                                                           */
  I2C_TargetAddressConfig(I2Cx, slave_address, I2C_MASTER_WRITE);

  /* Check on Master Transmitter STA condition and clear it                                                 */
  while (!I2C_CheckStatus(I2Cx, I2C_MASTER_SEND_START));
	
  /* Check on Master Transmitter ADRS condition and clear it                                                */
  while (!I2C_CheckStatus(I2Cx, I2C_MASTER_TRANSMITTER_MODE));
	
  /* Send data                                                                                              */
  while (Tx_Index < BufferSize)
  {
    /* Check on Master Transmitter TXDE condition                                                           */
    while (!I2C_CheckStatus(I2Cx, I2C_MASTER_TX_EMPTY));
    /* Master Send I2C data                                                                                 */
    I2C_SendData(I2Cx, buffer[Tx_Index ++]);
  }
  /* Send I2C STOP condition                                                                                */
  I2C_GenerateSTOP(I2Cx);
  /*wait for BUSBUSY become idle                                                                            */
  while (I2C_ReadRegister(I2Cx, I2C_REGISTER_SR)&0x80000);
}

void I2C_Read(HT_I2C_TypeDef* I2Cx, u16 slave_address, u8* buffer, u8 BufferSize)
{
	u8 Rx_Index = 0;
	
	/* Send I2C START & I2C slave address for read                                                            */
  I2C_TargetAddressConfig(I2Cx, slave_address, I2C_MASTER_READ);

  /* Check on Master Transmitter STA condition and clear it                                                 */
  while (!I2C_CheckStatus(I2Cx, I2C_MASTER_SEND_START));

  /* Check on Master Transmitter ADRS condition and clear it                                                */
  while (!I2C_CheckStatus(I2Cx, I2C_MASTER_RECEIVER_MODE));

  I2C_AckCmd(I2Cx, ENABLE);
  /* Send data                                                                                              */
  while (Rx_Index < BufferSize)
  {

    /* Check on Slave Receiver RXDNE condition                                                              */
    while (!I2C_CheckStatus(I2Cx, I2C_MASTER_RX_NOT_EMPTY));
    /* Store received data on I2C1                                                                          */
    buffer[Rx_Index ++] = I2C_ReceiveData(I2Cx);
    if (Rx_Index == (BufferSize-1))
    {
      I2C_AckCmd(I2Cx, DISABLE);
    }
  }
  /* Send I2C STOP condition                                                                                */
  I2C_GenerateSTOP(I2Cx);
  /*wait for BUSBUSY become idle                                                                            */
  while (I2C_ReadRegister(I2Cx, I2C_REGISTER_SR)&0x80000);
}

void LCD_I2C_1602_4bit_Write(u8 data)
{
	data |= _backlightval;
	
	I2C_Write(LCD_I2C_CH,I2C_SLAVE_ADDRESS,&data,1);

	data |= I2C_1062_E;
	I2C_Write(LCD_I2C_CH,I2C_SLAVE_ADDRESS,&data,1);
	vTaskDelay(1);
	
	data &= ~I2C_1062_E;
	I2C_Write(LCD_I2C_CH,I2C_SLAVE_ADDRESS,&data,1);
	vTaskDelay(1);
}

void LCD_command(u8 command)
{
	u8 high_4b = command & 0xF0;
	u8 low_4b = (command<<4) & 0xF0;
	
	LCD_I2C_1602_4bit_Write(high_4b);
	LCD_I2C_1602_4bit_Write(low_4b);
}

void LCD_setCursor(uint8_t col, uint8_t row)
{
	int row_offsets[] = { 0x00, 0x40, 0x14, 0x54 };
	if (row > LCD_MAX_ROWS) {
		row = LCD_MAX_ROWS-1;    // we count rows starting w/0
	}
	LCD_command(LCD_SETDDRAMADDR | (col + row_offsets[row]));
}

void LCD_Write(u8 Data){
	u8 high_4b = (Data & 0xF0) | I2C_1062_RS;
	u8 low_4b = ((Data<<4) & 0xF0) | I2C_1062_RS;
	
	LCD_I2C_1602_4bit_Write(high_4b);
	LCD_I2C_1602_4bit_Write(low_4b);
}

void LCDInit(void)
{
	_backlightval=LCD_BACKLIGHT;

	_displayfunction = LCD_4BITMODE | LCD_2LINE | LCD_5x8DOTS;
	vTaskDelay(1);

	LCD_I2C_1602_4bit_Write(0x30);
	vTaskDelay(1);
	LCD_I2C_1602_4bit_Write(0x30);
	vTaskDelay(1);
	LCD_I2C_1602_4bit_Write(0x30);
	vTaskDelay(1);

	LCD_I2C_1602_4bit_Write(LCD_FUNCTIONSET | LCD_4BITMODE);
	LCD_command(LCD_FUNCTIONSET | _displayfunction); 
	_displaycontrol = LCD_DISPLAYOFF | LCD_CURSOROFF | LCD_BLINKOFF;
	LCD_command(LCD_DISPLAYCONTROL | _displaycontrol);
	LCD_command(LCD_CLEARDISPLAY);
	vTaskDelay(1);
	
	LCD_command(LCD_ENTRYMODESET | LCD_ENTRYLEFT | LCD_ENTRYSHIFTDECREMENT);
	LCD_command(LCD_RETURNHOME);
	vTaskDelay(1);
	
	_displaycontrol = LCD_DISPLAYON | LCD_CURSOROFF | LCD_BLINKOFF;
	LCD_command(LCD_DISPLAYCONTROL | _displaycontrol);
}

void LCD_Backlight(u8 enable)
{
	u8 data = 0;
	if(enable)	_backlightval=LCD_BACKLIGHT;
	else				_backlightval=LCD_NOBACKLIGHT;
	data = _backlightval;
	I2C_Write(LCD_I2C_CH,I2C_SLAVE_ADDRESS,&data,1);
}

static char lcm_line0[17], lcm_line1[17];
void LCDLines()
{
    int eol;
    LCD_setCursor(0, 0);
    eol = 0;
    for (int i = 0; i < 16; i++) {
        if (!eol && lcm_line0[i] == 0)
            eol = 1;
        LCD_Write(eol ? ' ' : lcm_line0[i]);
    }

    LCD_setCursor(0, 1);
    eol = 0;
    for(int i = 0; i < 16; i++) {
        if (!eol && lcm_line1[i] == 0)
            eol = 1;
        LCD_Write(eol ? ' ' : lcm_line1[i]);
    }
}

// define the LCD display state for the rfid and IR's number.
enum {
    CARD_INVALID,
    CARD_VALID,
    CARD_BALLDROPPING,       // number of balls have been specified.
    CARD_BALLDROPPED,
};

// LCD 2nd line is used to show either music status or a secet key.
enum {
    KEYMUSIC_KEY,
    KEYMUSIC_MUSIC,
};

enum {
    LCDMUSIC_PLAY,
    LCDMUSIC_STOP,
    LCDMUSIC_PAUSE,
};

enum {
    LCDKEY_KEY,
    LCDKEY_CORRECT_INPUT,
    LCDKEY_INCORRECT_INPUT,
};

const static struct {
    int irkey;
    int val;
} irkeymap[] = {
    { INFRA_1, 1 }, { INFRA_2, 2 }, { INFRA_3, 3 }, { INFRA_4, 4 }, { INFRA_5, 5 },
    { INFRA_6, 6 }, { INFRA_7, 7 }, { INFRA_8, 8 }, { INFRA_9, 9 }
};

static uint8_t rfid[5];
static char keystr[5];
static int keycnt;
static int keymusicstate, keystate, musicstate;
static int cardstate;
static int twinkle_play, twinkle_keyresult;
static int playtime;
static TimerHandle_t droppedtimer, inputtimer, playtimer, statustimer;

static void BallsTimerCallback(TimerHandle_t xTimer)
{
    LCDMsg msg;
    msg.id = LCDMSG_BALLDROPPED;
    xQueueSend(LCDQueue, &msg, 0);
}

static void InputTimerCallback(TimerHandle_t xTimer)
{
    LCDMsg msg;
    if (keymusicstate == KEYMUSIC_KEY && (keystate == LCDKEY_CORRECT_INPUT || keystate == LCDKEY_INCORRECT_INPUT)) {
        if (twinkle_keyresult <= 6) {
            xTimerStart(xTimer, 0);
            twinkle_keyresult++;
            msg.id = LCDMSG_KEYRESULT;
            xQueueSend(LCDQueue, &msg, 0);
        } else {
            keymusicstate = KEYMUSIC_MUSIC;
            msg.id = LCDMSG_UPDATESTATUS;
            xQueueSend(LCDQueue, &msg, 0);
        }
    }
}

static void PlayTimerCallback(TimerHandle_t xTimer)
{
    LCDMsg msg;
    if (musicstate == LCDMUSIC_PLAY) {
        xTimerStart(xTimer, 0);
        twinkle_play++;
        playtime++;
    }
    msg.id = LCDMSG_UPDATESTATUS;
    xQueueSend(LCDQueue, &msg, 0);
}

static void StatusTimerCallback(TimerHandle_t xTimer)
{
    LCDMsg msg;
    msg.id = LCDMSG_UPDATESTATUS;
    xQueueSend(LCDQueue, &msg, 0);
}

static portTASK_FUNCTION(LCDTask, pvParameters)
{   
    int i, balls, val;
    LCDMsg msg;
    LCDInit();
    snprintf(lcm_line0, 17, "----------   0/0");
    strcpy(lcm_line1, "");
    LCDLines();
    
    droppedtimer = xTimerCreate("balls", 2000, pdFALSE, (void *) 0, BallsTimerCallback);
    inputtimer = xTimerCreate("input", 500, pdFALSE, (void *) 0, InputTimerCallback);
    playtimer = xTimerCreate("play", 500, pdFALSE, (void *) 0, PlayTimerCallback);
    statustimer = xTimerCreate("status", 1500, pdFALSE, (void *) 0, StatusTimerCallback);
    
    cardstate = CARD_INVALID;
    keymusicstate = KEYMUSIC_MUSIC;
    keystate = LCDKEY_KEY;
    musicstate = LCDMUSIC_STOP;
    while (1) {
        if (xQueueReceive(LCDQueue, &msg, portMAX_DELAY) == pdPASS) {
            if (msg.id == LCDMSG_RFID || msg.id == LCDMSG_IRKEY || msg.id == LCDMSG_BALLDROPPING || msg.id == LCDMSG_BALLDROPPED) {
                switch (cardstate) {
                case CARD_INVALID:
                    if (msg.id == LCDMSG_RFID) {
                        for (i = 0; i < 5; i++)
                            rfid[i] = msg.rfid[i];
                        snprintf(lcm_line0, 17, "%02x%02x%02x%02x%02x", rfid[0], rfid[1], rfid[2], rfid[3], rfid[4]);
                        cardstate = CARD_VALID;
                    }
                    break;
                case CARD_VALID:
                    if (msg.id == LCDMSG_IRKEY) {
                        balls = 1;
                        for (i = 0; i < sizeof(irkeymap)/sizeof(irkeymap[0]); i++) {
                            if (msg.key == irkeymap[i].irkey) {
                                balls = irkeymap[i].val;
                                break;
                            }
                        }
                        if (i == sizeof(irkeymap)/sizeof(irkeymap[0])) {
                            strcpy(lcm_line1, "  Invalid Key");
                            xTimerStart(statustimer, 0);
                        } else {
                            snprintf(lcm_line0, 17, "%02x%02x%02x%02x%02x   0/%d", rfid[0], rfid[1], rfid[2], rfid[3], rfid[4], balls);
                            cardstate = CARD_BALLDROPPING;
                            val = SERVO_DROPBALL | (balls << 16);
                            xQueueSend(ServoQueue, (void *) &val, 0);
                        }
                    } else if (msg.id == LCDMSG_RFID) {
                        if (memcmp(rfid, msg.rfid, 5) == 0)
                            continue;
                        for (i = 0; i < 5; i++)
                            rfid[i] = msg.rfid[i];
                        snprintf(lcm_line0, 17, "%02x%02x%02x%02x%02x", rfid[0], rfid[1], rfid[2], rfid[3], rfid[4]);
                        strcpy(lcm_line1, "  Card Changed");
                        xTimerStart(statustimer, 0);
                    } else {
                        continue;
                    }
                    break;
                case CARD_BALLDROPPING:
                    if (msg.id == LCDMSG_BALLDROPPING) {
                        snprintf(lcm_line0, 17, "%02x%02x%02x%02x%02x   %d/%d", rfid[0], rfid[1], rfid[2], rfid[3], rfid[4], msg.key, balls);
                    } else if (msg.id == LCDMSG_BALLDROPPED) {
                        snprintf(lcm_line0, 17, "%02x%02x%02x%02x%02x   %d/%d", rfid[0], rfid[1], rfid[2], rfid[3], rfid[4], balls, balls);
                        cardstate = CARD_BALLDROPPED;
                        xTimerStart(droppedtimer, 0);
                    }
                    break;
                case CARD_BALLDROPPED:
                    snprintf(lcm_line0, 17, "----------   0/0");
                    cardstate = CARD_INVALID;
                    break;
                }
            } else if (msg.id == LCDMSG_KEYPAD) {
                keymusicstate = KEYMUSIC_KEY;
                if (msg.key >= '0' && msg.key <= '9' && keycnt < 4) {
                    keystr[keycnt++] = msg.key;
                    keystr[keycnt] = 0;
                    snprintf(lcm_line1, 16, "%s", keystr);
                } else if (msg.key == '*') {    // erase key
                    if (keycnt >= 1)
                        keystr[--keycnt] = 0;
                    snprintf(lcm_line1, 16, "%s", keystr);
                    if (keycnt == 0) {
                        LCDMsg m;
                        keymusicstate = KEYMUSIC_MUSIC;
                        m.id = LCDMSG_UPDATESTATUS;
                        xQueueSend(LCDQueue, &m, NULL);
                    }
                } else if (msg.key == '#') {
                    if (strcmp(keystr, "1234") == 0) {
                        val = SERVO_OPENDOOR;
                        xQueueSend(ServoQueue, (void *) &val, 0);
                        strcpy(lcm_line1, "    Allowed");
                        keystate = LCDKEY_CORRECT_INPUT;
                    } else {
                        strcpy(lcm_line1, "     Denied");
                        keystate = LCDKEY_INCORRECT_INPUT;
                    }
                    twinkle_keyresult = 0;
                    xTimerStart(inputtimer, 0);
                    keycnt = 0;
                    keystr[0] = 0;
                }
            } else if (msg.id == LCDMSG_KEYRESULT) {
                char *p = (keystate == LCDKEY_CORRECT_INPUT) ? "    Allowed" : "     Denied";
                strcpy(lcm_line1, ((twinkle_keyresult % 2) == 0) ? "" : p);
            } else if (msg.id == LCDMSG_UPDATESTATUS) {
                if (keymusicstate == KEYMUSIC_MUSIC) {
                    switch (musicstate) {
                    case LCDMUSIC_PLAY:
                        strcpy(lcm_line1, ((twinkle_play % 2) == 0) ? "" : "Playing");
                        break;
                    case LCDMUSIC_PAUSE:
                        strcpy(lcm_line1, "Paused");
                        break;
                    case LCDMUSIC_STOP:
                        strcpy(lcm_line1, "Stopped");
                        break;
                    }
                } else {
                    snprintf(lcm_line1, 16, "%s", keystr);
                }
            } else if (msg.id == LCDMSG_PLAY) {
                musicstate = LCDMUSIC_PLAY;
                if (keymusicstate == KEYMUSIC_MUSIC) {
                    xTimerStart(playtimer, 0);
                }
            } else if (msg.id == LCDMSG_STOP) {
                playtime = 0;
                musicstate = LCDMUSIC_STOP;
                strcpy(lcm_line1, "Stopped");
            } else if (msg.id == LCDMSG_PAUSE) {
                musicstate = LCDMUSIC_PAUSE;
                strcpy(lcm_line1, "Paused");
            }
                    
        }
        LCDLines();
	}
}

static portTASK_FUNCTION(RFIDTask, pvParameters)
{
    LCDMsg msg;

    MFRC522_Init();
    while (1) {
        if (!Get_Card_Number())	{
            dprintf(DEBUG_RFID, "Card No.: %02x%02x%02x%02x%02x\n", serNum[0], serNum[1], serNum[2], serNum[3], serNum[4]);
            for (int i = 0; i < 5; i++) {
                msg.rfid[i] = serNum[i];
            }
            msg.id = LCDMSG_RFID;
            xQueueSend(LCDQueue, (void *) &msg, 0);
        }
        vTaskDelay(200);
    }
}

static const uint16_t row[4] = { GPIO_PIN_3, GPIO_PIN_2, GPIO_PIN_10, GPIO_PIN_9 };
static const uint16_t col[4] = { GPIO_PIN_0, GPIO_PIN_1, GPIO_PIN_2, GPIO_PIN_3 };
static const char keymap[4][4]={
    {'1', '2', '3', 'A'},
	{'4', '5', '6', 'B'},
	{'7', '8', '9', 'C'},
	{'*', '0', '#', 'D'}
};

typedef struct {
    int key_pressed;            // indicate the corresponding key is pressed or not.
    int wait_ticks;
    long pressed, released;
} KeyStatus;

KeyStatus key_status[4][4];

static portTASK_FUNCTION(KeypadTask, pvParameters)
{
    int i, j;
    LCDMsg msg;
    long ticks;
    KeyStatus *p;

	for (i = 0; i < 4; i++)
        GPIO_WriteOutBits(HT_GPIOD, col[i], 1);

    while (1) {
        ticks = xTaskGetTickCount();
		for (i = 0; i < 4; i++) {
			for (j = 0; j < 4; j++) {
				GPIO_WriteOutBits(HT_GPIOD, col[j], 0);
                p = &key_status[i][j];
				if (GPIO_ReadInBit(HT_GPIOA, row[i]) == 0) {    // keymap[i][j] pressed
                    if (!p->key_pressed) {                      // first time
                        p->key_pressed = 1;
                        p->pressed = ticks;
                        p->wait_ticks = 500;
                        dprintf(DEBUG_KEYPAD, "Keypad: %c\n", keymap[i][j]);
                        msg.id = LCDMSG_KEYPAD;
                        msg.key = keymap[i][j];
                        xQueueSend(LCDQueue, (void *) &msg, 0);
					} else {                                    // pressed already
                        if (ticks - p->pressed >= p->wait_ticks) {
                            p->pressed = ticks;
                            p->wait_ticks = 200;
                            dprintf(DEBUG_KEYPAD, "Keypad: %c\n", keymap[i][j]);
                            msg.id = LCDMSG_KEYPAD;
                            msg.key = keymap[i][j];
                            xQueueSend(LCDQueue, (void *) &msg, 0);
                        }
                    }
				} else {                                        // keymap[i][j] released;
                    p->key_pressed = 0;
                }
				GPIO_WriteOutBits(HT_GPIOD, col[j], 1);
			}
		}
        vTaskDelay(5);
    }
}

float AngletoDuty(int angle)
{
	float duty_cycle;
    duty_cycle = 2.5 + 2.0 / 36 * angle;
	return duty_cycle;
}

enum {
    DOOR_CLOSED,
    DOOR_OPEN,
};

enum {
    VALVE_CC,       // top: closed; bottom: closed
    VALVE_OC,       // top: open; bottom: closed
    VALVE_CO,       // top: closed; bottom: open
    VALVE_CC2,      // same as VALVE_CC except that a ball stays between the 2 valves.
};

static int state_door;
static int state_valve, ballsdropped; 
// Ch0 is used to control the machine door.
// Ch2/Ch3 are used to throttle the ball to drop down.
void ServoControl(int ch, float duty)
{
    TM_OutputInitTypeDef OutInit;
    u16 ccr = SystemCoreClock / 100 * duty / 0x10000;    // (SystemCoreClock / 0x10000) * (duty / 100);
    OutInit.Channel = ch;
    OutInit.OutputMode = TM_OM_PWM1;
    OutInit.Control = TM_CHCTL_ENABLE;
    OutInit.Polarity = TM_CHP_NONINVERTED;
    OutInit.Compare = ccr ;
    OutInit.AsymmetricCompare = 0;
    TM_OutputInit(HT_MCTM0, &OutInit);
}

static portTASK_FUNCTION(ServoTask, pvParameters)
{
    int i;
    uint32_t cmd, balls = 0;
    LCDMsg msg;
    ServoControl(TM_CH_0, 0);
    ServoControl(TM_CH_2, 0);
    ServoControl(TM_CH_3, 0);
    TM_Cmd(HT_MCTM0, ENABLE);
    MCTM_CHMOECmd(HT_MCTM0, ENABLE);
	
    state_door = DOOR_CLOSED;
    state_valve = VALVE_CC;
	while (1) {
        if (xQueueReceive(ServoQueue, &cmd, 0) == pdPASS) {
            switch (cmd & 0xffff) {
            case SERVO_OPENDOOR:
                if (state_door == DOOR_CLOSED) {
                    for(i = 0; i <= 90; i += 3) {
                        ServoControl(TM_CH_0, AngletoDuty(i));
                        vTaskDelay(20);
                    }
                    state_door = DOOR_OPEN;
                }
                break;
            case SERVO_CLOSEDOOR:
                if (state_door == DOOR_OPEN) {
                    for (i = 90; i >= 0; i -= 3) {
                        ServoControl(TM_CH_0, AngletoDuty(i));
                        vTaskDelay(20);
                    }
                    state_door = DOOR_CLOSED;
                }
                break;
            case SERVO_DROPBALL:
                state_valve = VALVE_CC;
                balls = cmd >> 16;
                ballsdropped = 0;
                break;
            }
        }
        
        switch (state_valve) {
        case VALVE_CC:
            if (balls > ballsdropped) {
                for (i = 0; i <= 90; i += 3) {
                    ServoControl(TM_CH_2, AngletoDuty(i));
                    vTaskDelay(20);
                }
                state_valve = VALVE_OC;
            }
            break;

        case VALVE_OC:
            for (i = 90; i >= 0; i -= 3) {
                ServoControl(TM_CH_2, AngletoDuty(i));
                vTaskDelay(20);
            }
            state_valve = VALVE_CC2;
            break;
        case VALVE_CC2:
            for (i = 0; i <= 90; i += 3) {
                ServoControl(TM_CH_3, AngletoDuty(i));
                vTaskDelay(20);
            }            
            state_valve = VALVE_CO;
            msg.id = LCDMSG_BALLDROPPING;
            msg.key = ++ballsdropped;
            xQueueSend(LCDQueue, (void *) &msg, 0);
            state_valve = VALVE_CO;
            break;
        case VALVE_CO:
            for (i = 90; i >= 0; i -= 3) {
                ServoControl(TM_CH_3, AngletoDuty(i));
                vTaskDelay(20);
            }
            if (balls <= ballsdropped) {
                msg.id = LCDMSG_BALLDROPPED;
                xQueueSend(LCDQueue, (void *) &msg, 0);
                balls = ballsdropped = 0;
            }
            state_valve = VALVE_CC;
            break;
        }
        
        vTaskDelay(100);
	}
}

// Infrared state definition
enum {
    IR_IDLE, IR_LOSTART, IR_HISTART, IR_DATA_LOW, IR_DATA_HIGH
};

#define TIME_LOSTART    9200
#define TIME_HISTART    4400
#define TIME_LODATA     600
#define TIME_HIDATA0    550
#define TIME_HIDATA1    1650

typedef struct
{
    u32 OverflowCounter;
    u32 StartValue;
    u32 CapturePulse;
    TM_CHP_Enum ChannelPolarity;
    bool DataValid;
    u32 state;
    u32 bitcount;
    u32 data;
} CAPINFO;

CAPINFO CapInfo;

int TimeAround(u32 time1, u32 time2)
{
    if ((time1 * 90 < time2 * 100) && (time1 * 110 > time2 * 100))
        return 1;
    return 0;
}
    
void GPTM0_IRQHandler(void)
{
    bool update_flag = FALSE;

    u32 status = HT_GPTM0->INTSR;
    u32 cnt = HT_GPTM0->CNTR;
    HT_GPTM0->INTSR = ~status;

    if (status & TM_INT_UEV) {
        update_flag = TRUE;
        if (CapInfo.OverflowCounter != 0xFFFF)
            CapInfo.OverflowCounter++;
    }

    if (status & TM_INT_CH1CC) {
        u32 cap_value = TM_GetCaptureCompare(HT_GPTM0, TM_CH_1);
        bool isCapBeforeUpdate = (update_flag && (cap_value > cnt))? TRUE : FALSE;
        if (isCapBeforeUpdate)
            CapInfo.OverflowCounter--;
    
        // Pulse width clock cycles = (overflow counter * (CounterReload + 1)) - rising capture + falling capture
        CapInfo.CapturePulse = (CapInfo.OverflowCounter << 16) - CapInfo.StartValue + cap_value;

        CapInfo.StartValue = cap_value;
        CapInfo.OverflowCounter = (isCapBeforeUpdate) ? 1: 0;

        u32 us = (int)CapInfo.CapturePulse / (SystemCoreClock / 1000000ul);
        switch (CapInfo.state) {
        case IR_IDLE: 
            CapInfo.state = (TimeAround(us, TIME_LOSTART)) ? IR_LOSTART : IR_IDLE;
            break;
        case IR_LOSTART:
            CapInfo.state = (TimeAround(us, TIME_HISTART)) ? IR_HISTART : IR_IDLE;
            break;
        case IR_HISTART:
            if (TimeAround(us, TIME_LODATA)) {
                CapInfo.state = IR_DATA_LOW;
                CapInfo.data = 0;
                CapInfo.bitcount = 0;
            } else {
                CapInfo.state = IR_IDLE;
            }
            break;
        case IR_DATA_LOW:
            if (TimeAround(us, TIME_HIDATA0)) {
                CapInfo.state = IR_DATA_HIGH;
                CapInfo.data <<= 1;
                CapInfo.bitcount++;
            } else if (TimeAround(us, TIME_HIDATA1)) {
                CapInfo.state = IR_DATA_HIGH;
                CapInfo.data = (CapInfo.data << 1) | 1;
                CapInfo.bitcount++;
            } else {
                CapInfo.state = IR_IDLE;
            }
            
            if (CapInfo.bitcount == 32) {
                CapInfo.DataValid = TRUE;
                //PrintfFromISR("====== Completed ======\n");
                CapInfo.state = IR_IDLE;
            }
            break;
        case IR_DATA_HIGH:
            CapInfo.state = (TimeAround(us, TIME_LODATA)) ? IR_DATA_LOW : IR_IDLE;
            break;
        }
        #if 0
        PrintfFromISR("%s: %d us, %d\n",
            (CapInfo.ChannelPolarity == TM_CHP_NONINVERTED) ? "Low" : "High",
            (int)CapInfo.CapturePulse / (SystemCoreClock / 1000000ul), CapInfo.bitcount);
        #endif
        CapInfo.ChannelPolarity = (CapInfo.ChannelPolarity == TM_CHP_NONINVERTED) ? TM_CHP_INVERTED : TM_CHP_NONINVERTED;
        TM_ChPolarityConfig(HT_GPTM0, TM_CH_1, CapInfo.ChannelPolarity);
    }
}

typedef struct {
    uint32_t code;
    int id;
    char *name;
} Infrared_t;
    
const Infrared_t InfraredTbl[] = {
    { 0xffa25d, INFRA_CHM, "CH-" },
    { 0xff629d, INFRA_CH, "CH" },
    { 0xffe21d, INFRA_CHP, "CH+" },
    { 0xff22dd, INFRA_PREV, "PREV" },
    { 0xff02fd, INFRA_NEXT, "NEXT" },
    { 0xffc23d, INFRA_PLAY, "PLAY/PAUSE" },
    { 0xffe01f, INFRA_VOLM, "-" },
    { 0xffa857, INFRA_VOLP, "+" },
    { 0xff906f, INFRA_VOLEQ, "EQ" },
    { 0xff6897, INFRA_0, "0" },
    { 0xff9867, INFRA_100P, "100+" },
    { 0xffb04f, INFRA_200P, "200+" },
    { 0xff30cf, INFRA_1, "1" },
    { 0xff18e7, INFRA_2, "2" },
    { 0xff7a85, INFRA_3, "3" },
    { 0xff10ef, INFRA_4, "4" },
    { 0xff38c7, INFRA_5, "5" },
    { 0xff5aa5, INFRA_6, "6" },
    { 0xff42bd, INFRA_7, "7" },
    { 0xff4ab5, INFRA_8, "8" },
    { 0xff52ad, INFRA_9, "9" },
};
    
int InfraGetID(uint32_t code)
{
    for (int i = 0; i < sizeof(InfraredTbl)/sizeof(Infrared_t); i++) {
        if (code == InfraredTbl[i].code) {
            dprintf(DEBUG_IR, "Infrared Key: %s\n", InfraredTbl[i].name);
            return InfraredTbl[i].id;
        }
    }
    
    return INFRA_INVALID;
}

static portTASK_FUNCTION(InfraredTask, pvParameters)
{
    uint32_t val;
    LCDMsg msg;
    while (1) {
        if (CapInfo.DataValid) {
            dprintf(DEBUG_IR, "IR Code: %x\n", CapInfo.data);
            uint32_t key = InfraGetID(CapInfo.data);
            switch (key) {
            case INFRA_PLAY:
                msg.id = (music_state == MUSIC_PLAY) ? LCDMSG_PAUSE : LCDMSG_PLAY;
                xQueueSend(LCDQueue, (void *) &msg, 0);
                val = key;
                xQueueSend(MusicQueue, (void *) &val, 0);
                break;
            case INFRA_VOLEQ:       // viewed as "STOP" command.
                msg.id = LCDMSG_STOP;
                xQueueSend(LCDQueue, (void *) &msg, 0);
                val = key;
                xQueueSend(MusicQueue, (void *) &val, 0);
                break;
            case INFRA_PREV:
            case INFRA_NEXT:
                val = key;
                xQueueSend(MusicQueue, (void *) &val, 0);
                break;
            case INFRA_CHM:         // used to close the machine door.
                val = SERVO_CLOSEDOOR;
                xQueueSend(ServoQueue, (void *) &val, 0);
                break;
            default:    // remaining keys are redirected to the LCD task.
                msg.id = LCDMSG_IRKEY;
                msg.key = key;
                xQueueSend(LCDQueue, (void *) &msg, 0);
                break;
            }
            CapInfo.DataValid = FALSE;
        }
        vTaskDelay(50);
    }
}

typedef struct
{
    u32 OverflowCounter;
    u32 StartValue;
    u32 CapturePulse;
    TM_CHP_Enum ChannelPolarity;
    bool DataValid;
} ECHOINFO;

ECHOINFO EchoInfo;

void GPTM1_IRQHandler(void)
{
    bool update_flag = FALSE;

    u32 status = HT_GPTM1->INTSR;
    u32 cnt = HT_GPTM1->CNTR;
    HT_GPTM1->INTSR = ~status;

    if (status & TM_INT_UEV) {
        update_flag = TRUE;
        if (EchoInfo.OverflowCounter != 0xFFFF)
            EchoInfo.OverflowCounter++;
    }

    if (status & TM_INT_CH0CC) {
        u32 cap_value = TM_GetCaptureCompare(HT_GPTM1, TM_CH_0);
        bool isCapBeforeUpdate = (update_flag && (cap_value > cnt))? TRUE : FALSE;
        
        if (EchoInfo.ChannelPolarity == TM_CHP_NONINVERTED) {
            EchoInfo.OverflowCounter = (isCapBeforeUpdate) ? 1 : 0;
            EchoInfo.StartValue = cap_value;
        } else {
            // Compute pulse width in PCLK unit when falling edge occurred
            if (isCapBeforeUpdate)
                EchoInfo.OverflowCounter--;
    
            // Pulse width clock cycles = (overflow counter * (CounterReload + 1)) - rising capture + falling capture
            EchoInfo.CapturePulse = (EchoInfo.OverflowCounter << 16) - EchoInfo.StartValue + cap_value;
            
            #if 0
            PrintfFromISR("%s: %d, %d us\n",
                (EchoInfo.ChannelPolarity == TM_CHP_NONINVERTED) ? "Low" : "High",
                EchoInfo.CapturePulse, (int) EchoInfo.CapturePulse / (SystemCoreClock / 1000000ul));
            #endif
            EchoInfo.DataValid = TRUE;
        }
        
        EchoInfo.ChannelPolarity = (EchoInfo.ChannelPolarity == TM_CHP_NONINVERTED) ? TM_CHP_INVERTED : TM_CHP_NONINVERTED;
        TM_ChPolarityConfig(HT_GPTM1, TM_CH_0, EchoInfo.ChannelPolarity);
    }
}

static portTASK_FUNCTION(UltrasonicTask, pvParameters)
{
    while (1) {
        if (EchoInfo.DataValid) {
            dprintf(DEBUG_SONIC, "Ultrasonic: %d, %d us\n",
                EchoInfo.CapturePulse, (int) EchoInfo.CapturePulse / (SystemCoreClock / 1000000ul));
            
            dprintf(DEBUG_SONIC, "Distance = %d cm\n", EchoInfo.CapturePulse / (SystemCoreClock / 1000000ul) / 58);
            EchoInfo.DataValid = FALSE;
        }
        vTaskDelay(50);
    }
}

static int SetVerbose(int argc, char *argv[], const cmds_t *cmd_p)
{
    if (argc == 1) {
        Printf("usage: vs [timer|motor|all]\n");
        return 0;
    }

    int i;
    for (i = 0; verbose_tbl[i].str != NULL; i++) {
        if (strcmp(argv[1], verbose_tbl[i].str) == 0) {
            verbose |= verbose_tbl[i].debug;
            break;
        }
    }

    if (verbose_tbl[i].str == NULL) {
        Printf("usage: vs [timer|motor|all]\n");
    }

    return 0;
}

static int ClearVerbose(int argc, char *argv[], const cmds_t *cmd_p)
{
    if (argc == 1) {
        Printf("usage: vc [timer|motor|all]\n");
        return 0;
    }

    int i;
    for (i = 0; verbose_tbl[i].str != NULL; i++) {
        if (strcmp(argv[1], verbose_tbl[i].str) == 0) {
            verbose &= ~verbose_tbl[i].debug;
            break;
        }
    }

    if (verbose_tbl[i].str == NULL) {
        Printf("usage: vc [timer|motor|all]\n");
    }

    return 0;
}

void StartTasks(unsigned portBASE_TYPE uxPriority)
{
    UART_Transmit(banner, sizeof(banner));

    LCDQueue = xQueueCreate(6, sizeof(LCDMsg));
    ServoQueue = xQueueCreate(6, sizeof(uint32_t));
    MusicQueue = xQueueCreate(6, sizeof(uint32_t));
    
    xEventGroup = xEventGroupCreate();
    //xTaskCreate(MonitorTask, "Monitor", STACK_SIZE, NULL, uxPriority, (xTaskHandle *) NULL);
    //xTaskCreate(MotorTask, "Motor", STACK_SIZE, NULL, uxPriority, (xTaskHandle *) NULL);
    xTaskCreate(LCDTask, "LCD", STACK_SIZE, NULL, uxPriority, (xTaskHandle *) NULL);
    xTaskCreate(KeypadTask, "Keypad", STACK_SIZE, NULL, uxPriority, (xTaskHandle *) NULL);
    xTaskCreate(ServoTask, "Servo", STACK_SIZE, NULL, uxPriority, (xTaskHandle *) NULL);
    xTaskCreate(InfraredTask, "Infrared", STACK_SIZE, NULL, uxPriority, (xTaskHandle *) NULL);
    xTaskCreate(UltrasonicTask, "Ultrasonic", STACK_SIZE, NULL, uxPriority, (xTaskHandle *) NULL);
    xTaskCreate(RFIDTask, "RFID", STACK_SIZE, NULL, uxPriority, (xTaskHandle *) NULL);
    xTaskCreate(MusicTask, "Music", STACK_SIZE, NULL, uxPriority, (xTaskHandle *) NULL);
    
	xDbguQueue = xQueueCreate(dbguQUEUE_SIZE, (unsigned portBASE_TYPE) sizeof(unsigned portCHAR));
	xTaskCreate(vDbguQueueConsumer, "DBGU", dbguSTACK_SIZE, (void *) &xDbguQueue, uxPriority , (xTaskHandle *) NULL);
	xTaskCreate(vDbguCmdLine, "CmdLine", dbguSTACK_SIZE, (void *) &xDbguQueue, uxPriority, (xTaskHandle *) NULL);
}

const cmds_t Cmds[] = {
	{ "show",		doShow, "Show" },
    { "v",          doStartMonitor, "Start monitor" },
    { "q",          doStopMonitor, "Stop monitor" },
    { "mr",         MotorRun, "Run Motor" },
    { "ms",         MotorStop, "Stop Motor" },
    { "for",        MotorForward, "Forward" },
    { "back",       MotorBackward, "Backward" },
    { "sr",         ServoRun, "Run Servo" },
    { "ss",         ServoStop, "Stop Servo" },
    { "vs",         SetVerbose, "Set Verbose" },
    { "vc",         ClearVerbose, "Clear Verbose" },
    { NULL,		NULL }
};

int do_cmd(int argc, char *argv[])
{
	const cmds_t *cmdp;

	if (argc == 0)
		return 0;

	for (cmdp = Cmds; cmdp->name != NULL; cmdp++) {
		if (strcmp(argv[0], cmdp->name) == 0) {
			if ( ((argv[1] != NULL ) && (!strcmp(argv[1], "?"))) ||
		 		((argv[1] != NULL ) && (!strcmp(argv[1], "help"))) ) {
				if (cmdp->argc_errmsg != NULL)
					Printf(STR_USAGE "%s\n", cmdp->argc_errmsg);
				else
					Printf(STR_USAGE "no help\n");
				return -1;
			}

			if (cmdp->func == NULL)	{
				Printf("Can't find execution function for command: %s\n", cmdp->name);
				return -2;
			}

			return (*cmdp->func)(argc, argv, cmdp);
		}
	}

	Printf("Valid commands are:\n");
	cmd_show_valid(Cmds);
	return 1;
}

#define CFG_MAXARGS	10
static int parse_line(char *line, char *argv[])
{
	int nargs = 0;

	while (nargs < CFG_MAXARGS)	{
		while (*line == ' ' || *line == '\t')
			line++;

		if (*line == 0)
			return nargs;

		argv[nargs++] = line;

		while (*line && *line != ' ' && *line != '\t')
			line++;

		if (*line == 0) {
			argv[nargs] = NULL;
			return nargs;
		}

		*line++ = 0;
	}

	return nargs;
}

static portTASK_FUNCTION(vDbguCmdLine, pvParameters)
{
	int n;
	uint8_t data;
	char cmdline[80];
	int nargs;
	char *argv[CFG_MAXARGS];

	n = 0;
	Printf((char*) PROMPT_STRING);
	for ( ; ; )	{
		if (USART_GetFlagStatus(HT_USART1, USART_FLAG_RXDR) == RESET) {
			vTaskDelay(10);
            continue;
        }

        if (!UART_Receive(&data)) {
            taskYIELD();
            continue;
        }
        switch (data)	{
        case 0x08:	// backspace
        case 0x7f:	// del
            if (n != 0)	{
                cmdline[n]= 0;
                __Printf(erase_seq);
                n--;
            }
            break;
        case '\r':	// Enter
        case '\n':
            cmdline[n] = 0;
            nargs = parse_line(cmdline, argv);
            Printf("\n");
            do_cmd(nargs, argv);
            Printf((char*) PROMPT_STRING);
            n = 0;
            break;
        default:
            if (n >= (sizeof(cmdline)-1))  // 1 byte for Enter
                break;
            cmdline[n++] = data;
            __Printf("%c", data);
        }
	}
}
