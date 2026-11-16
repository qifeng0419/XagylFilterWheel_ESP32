// ============================================================================
//  Console.h —— 服务台命令行（UART0 / USB CDC，给人用，不参与 Xagyl 协议）
//
//  为什么单独占一路串口：
//    Xagyl 协议要求「命令无结束符 + 应答一问一答」，设备端任何主动输出都可能
//    被驱动当成应答读走，导致命令-应答错位。所以调试信息一律打到这一路，
//    协议口（默认 Serial2）只跑协议。
//
//  与「初始位置可调」直接相关的命令：
//    home <deg>      设槽位 1 的参考角（初始位置的核心参数）
//    space <deg>     设槽间距（0 = 自动 360/N）
//    set <n>         示教：把当前实测角定为槽位 n 的中心
//    offset <n> <u>  直接写第 n 槽的偏移单位数
//    boot <0|1|2>    上电定位策略
//    cal             跑一次逐槽自动校准
//  全部改动即刻写入 flash，掉电不丢。
// ============================================================================
#pragma once

#include <Arduino.h>

class Console {
 public:
  void begin();
  void tick();                 // 非阻塞逐字节收行，遇 CR/LF 执行
  void banner();

 private:
  void execute(char *line);
  void helpText();
  void printStatus();
  void printDiag();
  void printSlots();
  void teach(int slot);
  void manualMove(int argc, char **argv);

  char     _line[96];
  uint16_t _len = 0;
};

extern Console g_console;
