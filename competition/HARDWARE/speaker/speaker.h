#ifndef _SPEAKER_H_
#define _SPEAKER_H_

#include <stdint.h>
#include <string.h>
#include "stm32f4xx.h"

// 外部变量声明
extern volatile uint8_t Speaker_Playing;  // 语音播放忙标志（1=播放中，0=空闲）

// 编码格式定义
typedef enum {
    SPEAKER_ENCODE_GB2312 = 0x00,
    SPEAKER_ENCODE_GBK    = 0x01,
    SPEAKER_ENCODE_UNICODE_LE = 0x03,  // 小头存储
    SPEAKER_ENCODE_UNICODE_BE = 0x04,  // 大头存储
    SPEAKER_ENCODE_UTF8   = 0x05
} Speaker_EncodeTypeDef;

// 发音人选择
typedef enum {
    SPEAKER_VOICE_XIAOLING = 3,   // 女声"晓玲" [m3]
    SPEAKER_VOICE_YINXIAOJIAN = 51, // 男声"尹小坚" [m51]
    SPEAKER_VOICE_YIXIAOQIANG = 52, // 男声"易小强" [m52]
    SPEAKER_VOICE_TIANBEIBEI = 53,  // 女声"田蓓蓓" [m53]
    SPEAKER_VOICE_DONALDDUCK = 54,  // 效果器"唐老鸭" [m54]
    SPEAKER_VOICE_XIAOYANZI = 55,   // 女童声"小燕子" [m55]
    SPEAKER_VOICE_BEITONG = 56,     // 男童声"贝童" [m56]
    SPEAKER_VOICE_XIAOKE = 57       // 男童声"晓可" [m57]
} Speaker_VoiceTypeDef;

// 芯片状态
typedef enum {
    SPEAKER_STATE_IDLE = 0,
    SPEAKER_STATE_BUSY,
    SPEAKER_STATE_ERROR,
    SPEAKER_STATE_SLEEP
} Speaker_StateTypeDef;

// 初始化语音模块
void UART_Speaker_Init(void);

// 基础语音合成（GB2312编码）
void UART_Speaker(char *speech_data);

// 指定编码格式进行语音合成
void UART_Speaker_WithEncode(char *speech_data, Speaker_EncodeTypeDef encode);

// 控制命令
void Speaker_Stop(void);           // 停止合成
void Speaker_Pause(void);          // 暂停合成
void Speaker_Resume(void);         // 恢复合成
void Speaker_Sleep(void);          // 进入Deep Sleep模式
void Speaker_WakeUp(void);         // 唤醒芯片
void Speaker_QueryState(void);     // 查询芯片状态

// 参数设置（通过控制标记）
void Speaker_SetVolume(uint8_t level);     // 音量设置 0-10，默认5
void Speaker_SetSpeed(uint8_t level);      // 语速设置 0-30，默认5，值越小越慢
void Speaker_SetTone(uint8_t level);       // 语调设置 0-10，默认5
void Speaker_SetVoice(Speaker_VoiceTypeDef voice); // 切换发音人

// 高级功能
uint8_t Speaker_WaitForIdle(void);            // 等待当前播放完成（阻塞式）
uint8_t Speaker_IsBusy(void);              // 获取忙状态
Speaker_StateTypeDef Speaker_GetState(void); // 获取详细状态
void Speaker_SendRawFrame(uint8_t *data, uint16_t len); // 发送原始帧

#endif