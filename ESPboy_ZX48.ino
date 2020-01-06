//v1.2 05.01.2020 bug fixes, keyboard module support (allows to use spectrum basic but slows emulator overall)
//v1.1 23.12.2019  z80 format v3 support, improved frameskip, screen and controls config files
//v1.0 20.12.2019 initial version, with sound
//by Shiru
//shiru@mail.ru
//https://www.patreon.com/shiru8bit
//uses Z80 core by Ketmar

#include <Adafruit_MCP23017.h>
#include <Adafruit_MCP4725.h>
#include <ESP8266WiFi.h>
#include <TFT_eSPI.h>
#include <sigma_delta.h>

/*
   IMPORTANT: the project consumes a lot of RAM, to allow enough set
    SSL Support to Basic
    IwIP to Lower memory (no features)

   Use 160 MHz for much better performance.

   Make sure all the display driver and pin comnenctions are correct by
   editting the User_Setup.h file in the TFT_eSPI library folder.

   Set SPI_FREQUENCY to 39000000 for best performance.

   Games should be uploaded into SPIFFS as 48K .z80 snapshots (v1,v2,v3)
   You can create such snapshots using ZXSPIN emulator

   You can also put a 6912 byte screen with the exact same name to be displayed before game

   You can provide an optional controls configuration file to provide convinient way to
   control a game. Such file has extension of .cfg, it is a plain text file that contains
   a string 8 characters long. The characters may represent a key A-Z, 0-9, _ for space,
   $ for Enter, @ for CS, # for SS
   Warning! This file overrides manual controls selection in file manager

   The order of characters is UP,DOWN,LEFT,RIGHT,ACT,ESC,LFT,RGT
   for example: QAOPM_0$

   I.e. a game file set may look like:

   game.z80 - snapshot
   game.scr - splash screen in native ZX Spectrum format
*/

#include "zymosis.h"
#include "glcdfont.c"
#include "gfx/espboy.h"
#include "gfx/keyboard.h"
#include "rom/rom.h"

#define MCP23017address 0 // actually it's 0x20 but in <Adafruit_MCP23017.h> lib there is (x|0x20) :)

//PINS
#define LEDPIN         D4
#define SOUNDPIN       D3

//SPI for LCD
#define csTFTMCP23017pin 8

Adafruit_MCP23017 mcp;
Adafruit_MCP23017 mcpKeyboard;

TFT_eSPI tft = TFT_eSPI();

#define MCP4725address 0x60
Adafruit_MCP4725 dac;

uint8_t pad_state;
uint8_t pad_state_prev;
uint8_t pad_state_t;
uint8_t keybModuleExist;

#define PAD_LEFT        0x01
#define PAD_UP          0x02
#define PAD_DOWN        0x04
#define PAD_RIGHT       0x08
#define PAD_ACT         0x10
#define PAD_ESC         0x20
#define PAD_LFT         0x40
#define PAD_RGT         0x80
#define PAD_ANY         0xff

uint16_t line_buffer[128];

uint8_t line_change[32 + 1]; //bit mask to updating each line, the extra bit is border update flag

uint8_t memory[49152];

uint8_t port_fe;  //keyboard, tape, sound, border
uint8_t port_1f;  //kempston joystick

Z80Info cpu;

#define ZX_CLOCK_FREQ   3500000
#define ZX_FRAME_RATE   50

#define SAMPLE_RATE     48000   //more is better, but emulations gets slower

#define MAX_FRAMESKIP   8

enum {
  K_CS = 0,
  K_Z,
  K_X,
  K_C,
  K_V,

  K_A,
  K_S,
  K_D,
  K_F,
  K_G,

  K_Q,
  K_W,
  K_E,
  K_R,
  K_T,

  K_1,
  K_2,
  K_3,
  K_4,
  K_5,

  K_0,
  K_9,
  K_8,
  K_7,
  K_6,

  K_P,
  K_O,
  K_I,
  K_U,
  K_Y,

  K_ENTER,
  K_L,
  K_K,
  K_J,
  K_H,

  K_SPACE,
  K_SS,
  K_M,
  K_N,
  K_B,
  
  K_DEL,
  K_LED,
};


constexpr uint8_t keybCurrent[7][5] PROGMEM = {
    {K_Q, K_E, K_R, K_U, K_O}, 
    {K_W, K_S, K_G, K_H, K_L},  
    {255, K_D, K_T, K_Y, K_I}, 
    {K_A, K_P, K_SS, K_ENTER, K_DEL}, 
    {K_SPACE, K_Z, K_C, K_N, K_M}, 
    {K_CS, K_X, K_V, K_B, K_6}, 
    {K_LED, K_SS, K_F, K_J, K_K}
};

constexpr uint8_t keybCurrent2[7][5] PROGMEM = {
    {K_Q, K_2, K_3, K_U, K_O}, 
    {K_1, K_4, K_G, K_H, K_L},  
    {255, K_5, K_T, K_Y, K_I}, 
    {K_A, K_P, K_SS, K_ENTER, K_DEL}, 
    {K_SPACE, K_7, K_9, K_N, K_M}, 
    {K_CS, K_8, K_V, K_B, K_6}, 
    {K_0, K_SS, K_6, K_J, K_K}
};

uint8_t key_matrix[40];

char filename[32];

uint8_t control_type;

uint8_t control_pad_l;
uint8_t control_pad_r;
uint8_t control_pad_u;
uint8_t control_pad_d;
uint8_t control_pad_act;
uint8_t control_pad_esc;
uint8_t control_pad_lft;
uint8_t control_pad_rgt;


enum {
  CONTROL_PAD_KEYBOARD,
  CONTROL_PAD_KEMPSTON
};

volatile uint8_t sound_dac;

#define SOUND_BUFFER_SIZE   (SAMPLE_RATE / ZX_FRAME_RATE * 2)
#define SOUND_MIN_GAP       SOUND_BUFFER_SIZE/10

volatile uint8_t* sound_buffer;

volatile uint16_t sound_wr_ptr;
volatile uint16_t sound_rd_ptr;



static void mem_wr(Z80Info *z80, uint16_t addr, uint8_t value, Z80MemIOType mio)
{
  uint16_t line;

  if (addr >= 0x4000)
  {
    memory[addr - 0x4000] = value;

    if (addr < 0x5800)
    {
      addr -= 0x4000;
      line = ((addr / 256) & 7) + ((addr / 32) & 7) * 8 + addr / 2048 * 64;
      line_change[line / 8] |= (1 << (line & 7));
    }
    else if (addr < 0x5b00)
    {
      line_change[(addr - 0x5800) / 32] = 255;
    }
  }
}



static uint8_t mem_rd(Z80Info *z80, uint16_t addr, Z80MemIOType mio)
{
  if (addr < 0x4000)
  {
    return pgm_read_byte(&rom[addr]);
  }
  else
  {
    return memory[addr - 0x4000];
  }
}



static uint8_t port_in(Z80Info *z80, uint16_t port, Z80PIOType pio)
{
  uint8_t val;
  uint16_t off;

  val = 0xff;

  if (!(port & 0x01)) //port #fe
  {
    off = 0;

    if (!(port & 0x0100)) off = 5 * 0;
    if (!(port & 0x0200)) off = 5 * 1;
    if (!(port & 0x0400)) off = 5 * 2;
    if (!(port & 0x0800)) off = 5 * 3;
    if (!(port & 0x1000)) off = 5 * 4;
    if (!(port & 0x2000)) off = 5 * 5;
    if (!(port & 0x4000)) off = 5 * 6;
    if (!(port & 0x8000)) off = 5 * 7;

    if (key_matrix[off + 0]) val &= ~0x01;
    if (key_matrix[off + 1]) val &= ~0x02;
    if (key_matrix[off + 2]) val &= ~0x04;
    if (key_matrix[off + 3]) val &= ~0x08;
    if (key_matrix[off + 4]) val &= ~0x10;
  }
  else
  {
    if ((port & 0xff) == 0x1f) val = port_1f;
  }

  return val;
}



static void port_out(Z80Info *z80, uint16_t port, uint8_t value, Z80PIOType pio)
{
  if (!(port & 0x01))
  {
    if ((port_fe & 7) != (value & 7)) line_change[32] = 1; //update border

    port_fe = value;
  }
}



void zx_init()
{
  memset(memory, 0, sizeof(memory));
  memset(key_matrix, 0, sizeof(key_matrix));
  memset(line_change, 0xff, sizeof(line_change));

  port_fe = 0;
  port_1f = 0;

  memset(&cpu, 0, sizeof(cpu));

  Z80_ResetCallbacks(&cpu);

  cpu.memReadFn = mem_rd;
  cpu.memWriteFn = mem_wr;
  cpu.portInFn  = port_in;
  cpu.portOutFn = port_out;
  cpu.evenM1    = false;

  Z80_Reset(&cpu);
}



void z80_unrle(uint8_t* mem, int sz)
{
  int i, ptr, len;
  uint8_t val;

  ptr = 0;

  while (ptr < sz - 4)
  {
    if (mem[ptr] == 0xed && mem[ptr + 1] == 0xed)
    {
      len = mem[ptr + 2];
      val = mem[ptr + 3];

      memmove(&mem[ptr + len], &mem[ptr + 4], sz - (ptr + len));  //not memcpy, because of the overlapping

      for (i = 0; i < len; ++i) mem[ptr++] = val;
    }
    else
    {
      ++ptr;
    }
  }
}



uint8_t zx_load_z80(const char* filename)
{
  uint8_t header[30];
  int sz, len, ptr;
  uint8_t rle;

  fs::File f = SPIFFS.open(filename, "r");

  if (!f) return 0;

  sz = f.size();
  f.readBytes((char*)header, sizeof(header));
  sz -= sizeof(header);

  Z80_Reset(&cpu);

  cpu.af.a = header[0];
  cpu.af.f = header[1];
  cpu.bc.c = header[2];
  cpu.bc.b = header[3];
  cpu.hl.l = header[4];
  cpu.hl.h = header[5];
  cpu.pc = header[6] + header[7] * 256;
  cpu.sp.l = header[8];
  cpu.sp.h = header[9];
  cpu.regI = header[10];
  cpu.regR = header[11];

  if (header[12] == 255) header[12] = 1;

  rle = header[12] & 0x20;
  port_fe = (header[12] >> 1) & 7;

  cpu.de.e = header[13];
  cpu.de.d = header[14];

  cpu.bcx.c = header[15];
  cpu.bcx.b = header[16];
  cpu.dex.e = header[17];
  cpu.dex.d = header[18];
  cpu.hlx.l = header[19];
  cpu.hlx.h = header[20];
  cpu.afx.a = header[21];
  cpu.afx.f = header[22];

  cpu.ix.l = header[23];
  cpu.ix.h = header[24];
  cpu.iy.l = header[25];
  cpu.iy.h = header[26];

  if (!(header[27] & 1)) //di
  {
    cpu.iff1 = 0;
  }
  else
  {
    cpu.iff1 = 1;
    cpu.prev_was_EIDDR = 1;
  }

  cpu.iff2 = header[28];
  cpu.im = (header[29] & 3);

  if (cpu.pc) //v1 format
  {
    f.readBytes((char*)memory, sz);

    if (rle) z80_unrle(memory, 16384 * 3);
  }
  else  //v2 or v3 format, features an extra header
  {
    //read actual PC from the extra header, skip rest of the extra header

    f.readBytes((char*)header, 4);
    sz -= 4;

    len = header[0] + header[1] * 256 + 2 - 4;
    cpu.pc = header[2] + header[3] * 256;

    f.seek(len, fs::SeekCur);
    sz -= len;

    //unpack 16K pages

    while (sz > 0)
    {
      f.readBytes((char*)header, 3);
      sz -= 3;

      len = header[0] + header[1] * 256;

      switch (header[2])
      {
        case 4: ptr = 0x8000; break;
        case 5: ptr = 0xc000; break;
        case 8: ptr = 0x4000; break;
        default:
          ptr = 0;
      }

      if (ptr)
      {
        ptr -= 0x4000;

        f.readBytes((char*)&memory[ptr], len);
        sz -= len;

        if (len < 0xffff) z80_unrle(&memory[ptr], 16384);
      }
      else
      {
        f.seek(len, fs::SeekCur);
        sz -= len;
      }
    }
  }

  f.close();

  return 1;
}



uint8_t zx_load_scr(const char* filename)
{
  fs::File f = SPIFFS.open(filename, "r");

  if (!f) return 0;

  f.readBytes((char*)memory, 6912);
  f.close();

  memset(line_change, 0xff, sizeof(line_change));

  return 1;
}



void zx_emulate_frame()
{
  int n, ticks, sacc, sout;

  sacc = 0;
  sout = 0;
  ticks = Z80_Interrupt(&cpu);

  while (ticks < (ZX_CLOCK_FREQ / ZX_FRAME_RATE))
  {
    n = Z80_ExecuteTS(&cpu, 8);

    sacc += n;

    if (port_fe & 0x10) sout += 127 * n;

    if (sacc >= (ZX_CLOCK_FREQ / SAMPLE_RATE))
    {
      sound_buffer[sound_wr_ptr] = sout / sacc;

      if (sound_wr_ptr != sound_rd_ptr)
      {
        ++sound_wr_ptr;

        if (sound_wr_ptr >= SOUND_BUFFER_SIZE) sound_wr_ptr = 0;
      }

      sacc -= ZX_CLOCK_FREQ / SAMPLE_RATE;
      sout = 0;
    }

    ticks += n;
  }
}



#define RGB565Q(r,g,b)    ( ((((r)>>5)&0x1f)<<11) | ((((g)>>4)&0x3f)<<5) | (((b)>>5)&0x1f) )
//#define LHSWAP(w)         ( (((w)>>8)&0x00ff) | (((w)<<8)&0xff00) )
#define LHSWAP(w)         ( ((w)>>8) | ((w)<<8) )

void zx_render_frame()
{
  uint16_t i, j, ch, ln, px, row, aptr, optr, attr, pptr1, pptr2, line1, line2, bright;
  uint16_t col, ink, pap;

  const uint16_t palette[16] = {
    RGB565Q(0, 0, 0),
    RGB565Q(0, 0, 128),
    RGB565Q(128, 0, 0),
    RGB565Q(128, 0, 128),
    RGB565Q(0, 128, 0),
    RGB565Q(0, 128, 128),
    RGB565Q(128, 128, 0),
    RGB565Q(128, 128, 128),
    RGB565Q(0, 0, 0),
    RGB565Q(0, 0, 255),
    RGB565Q(255, 0, 0),
    RGB565Q(255, 0, 255),
    RGB565Q(0, 255, 0),
    RGB565Q(0, 255, 255),
    RGB565Q(255, 255, 0),
    RGB565Q(255, 255, 255),
  };

  if (line_change[32])
  {
    line_change[32] = 0;

    col = LHSWAP(palette[port_fe & 7] << 2);
    for (i = 0; i < 128; ++i) line_buffer[i] = col; 
    for (i = 0; i < 16; ++i)
    {
      tft.pushImage(0, i, 128, 1, line_buffer);
      tft.pushImage(0, 112 + i, 128, 1, line_buffer);
    }
  }

  row = 16;

  for (ln = 0; ln < 192; ln += 2)
  {
    if (!(line_change[ln / 8] & (3 << (ln & 7))))
    {
      ++row;
      continue;
    }

    line_change[ln / 8] &= ~(3 << (ln & 7));

    pptr1 = (ln & 7) * 256 + ((ln / 8) & 7) * 32 + (ln / 64) * 2048;
    pptr2 = pptr1 + 256;
    aptr = 6144 + ln / 8 * 32;
    optr = 0;

    for (ch = 0; ch < 32; ++ch)
    {
      attr = memory[aptr++];
      bright = (attr & 0x40) ? 8 : 0;
      ink = palette[(attr & 7) + bright];
      pap = palette[((attr >> 3) & 7) + bright];

      line1 = memory[pptr1++];
      line2 = memory[pptr2++];

      for (px = 0; px < 8; px += 2)
      {
        col = (line1 & 0x80) ? ink : pap;
        col += (line1 & 0x40) ? ink : pap;
        col += (line2 & 0x80) ? ink : pap;
        col += (line2 & 0x40) ? ink : pap;

        line_buffer[optr++] = LHSWAP(col);

        line1 <<= 2;
        line2 <<= 2;
      }
    }

    tft.pushImage(0, row++, 128, 1, line_buffer);
  }
}


int check_key()
{
  pad_state_prev = pad_state;
  pad_state = ~mcp.readGPIOAB() & 255;
  pad_state_t = pad_state ^ pad_state_prev & pad_state;
  return pad_state;
}



//0 no timeout, otherwise timeout in ms

void wait_any_key(int timeout)
{
  timeout /= 100;

  while (1)
  {
    check_key();

    if (pad_state_t&PAD_ANY) break;

    if (timeout)
    {
      --timeout;

      if (timeout <= 0) break;
    }

    delay(100);
  }
}


//render part of a 8-bit uncompressed BMP file
//no clipping
//uses line buffer to draw it much faster than through writePixel

void drawBMP8Part(int16_t x, int16_t y, const uint8_t bitmap[], int16_t dx, int16_t dy, int16_t w, int16_t h)
{
  uint16_t i, j, col, c16;
  uint32_t bw, bh, wa, off, rgb;

  bw = pgm_read_dword(&bitmap[0x12]);
  bh = pgm_read_dword(&bitmap[0x16]);
  wa = (bw + 3) & ~3;

  if (w >= h)
  {
    for (i = 0; i < h; ++i)
    {
      off = 54 + 256 * 4 + (bh - 1 - (i + dy)) * wa + dx;

      for (j = 0; j < w; ++j)
      {
        col = pgm_read_byte(&bitmap[off++]);
        rgb = pgm_read_dword(&bitmap[54 + col * 4]);
        c16 = ((rgb & 0xf8) >> 3) | ((rgb & 0xfc00) >> 5) | ((rgb & 0xf80000) >> 8);
        line_buffer[j] = LHSWAP(c16);
      }

      tft.pushImage(x, y + i, w, 1, line_buffer);
    }
  }
  else
  {
    for (i = 0; i < w; ++i)
    {
      off = 54 + 256 * 4 + (bh - 1 - dy) * wa + i + dx;

      for (j = 0; j < h; ++j)
      {
        col = pgm_read_byte(&bitmap[off]);
        rgb = pgm_read_dword(&bitmap[54 + col * 4]);
        c16 = ((rgb & 0xf8) >> 3) | ((rgb & 0xfc00) >> 5) | ((rgb & 0xf80000) >> 8);
        line_buffer[j] = LHSWAP(c16);
        off -= wa;
      }

      tft.pushImage(x + i, y, 1, h, line_buffer);
    }
  }
}



void drawCharFast(uint16_t x, uint16_t y, uint16_t c, uint16_t color, uint16_t bg)
{
  uint16_t i, j, c16, line;

  for (i = 0; i < 5; ++i)
  {
    line = pgm_read_byte(&font[c * 5 + i]);

    for (j = 0; j < 8; ++j)
    {
      c16 = (line & 1) ? color : bg;
      line_buffer[j * 5 + i] = LHSWAP(c16);
      line >>= 1;
    }
  }

  tft.pushImage(x, y, 5, 8, line_buffer);
}



void printFast(int x, int y, char* str, int16_t color)
{
  char c;

  while (1)
  {
    c = *str++;

    if (!c) break;

    drawCharFast(x, y, c, color, 0);
    x += 6;
  }
}



bool espboy_logo_effect(int out)
{
  int16_t i, j, w, h, sx, sy, off, st, anim;

  sx = 32;
  sy = 28;
  w = 64;
  h = 72;
  st = 8;

  for (anim = 0; anim < st; ++anim)
  {
    if (check_key()&PAD_ANY) return false;

    //if (!out) set_speaker(200 + anim * 50, 5);

    for (i = 0; i < w / st; ++i)
    {
      for (j = 0; j < st; ++j)
      {
        off = anim - (7 - j);

        if (out) off += 8;

        if (off < 0 || off >= st) off = 0; else off += i * st;

        drawBMP8Part(sx + i * st + j, sy, g_espboy, off, 0, 1, h);
      }
    }

    delay(1000 / 30);
  }

  return true;
}



#define CONTROL_TYPES   5

const char* const layout_name[] = {
  "KEMP",
  "QAOP",
  "ZXse",
  "SINC",
  "CURS"
};

const int8_t layout_scheme[] = {
  -1, 0, 0, 0, 0, 0, 0, 0,
  K_O, K_P, K_Q, K_A, K_SPACE, K_M, K_0, K_1,
  K_Z, K_X, K_Q, K_A, K_SPACE, K_ENTER, K_0, K_1,
  K_6, K_7, K_9, K_8, K_0, K_ENTER, K_SPACE, K_1,
  K_5, K_8, K_8, K_7, K_0, K_ENTER, K_SPACE, K_1
};



#define FILE_HEIGHT    14
#define FILE_FILTER   "z80"

int16_t file_cursor;

uint8_t file_browser_ext(const char* name)
{
  while (1) if (*name++ == '.') break;

  return (strcasecmp(name, FILE_FILTER) == 0) ? 1 : 0;
}



void file_browser(String path, const char* header, char* fname, uint16_t fname_len)
{
  int16_t i, j, sy, pos, off, frame, file_count, control_type;
  uint8_t change, filter;
  fs::Dir dir;
  fs::File entry;
  char name[19 + 1];
  const char* str;

  memset(fname, 0, fname_len);
  memset(name, 0, sizeof(name));

  tft.fillScreen(TFT_BLACK);

  dir = SPIFFS.openDir(path);

  file_count = 0;
  control_type = 0;

  while (dir.next())
  {
    entry = dir.openFile("r");

    filter = file_browser_ext(entry.name());

    entry.close();

    if (filter) ++file_count;
  }

  if (!file_count)
  {
    printFast(24, 60, (char*)"No files found", TFT_RED);

    while (1) delay(1000);
  }

  printFast(4, 4, (char*)header, TFT_GREEN);
  tft.fillRect(0, 12, 128, 1, TFT_WHITE);

  change = 1;
  frame = 0;

  while (1)
  {
    if (change)
    {
      printFast(100, 4, (char*)layout_name[control_type], TFT_WHITE);

      pos = file_cursor - FILE_HEIGHT / 2;

      if (pos > file_count - FILE_HEIGHT) pos = file_count - FILE_HEIGHT;
      if (pos < 0) pos = 0;

      dir = SPIFFS.openDir(path);
      i = pos;
      while (dir.next())
      {
        entry = dir.openFile("r");

        filter = file_browser_ext(entry.name());

        entry.close();

        if (!filter) continue;

        --i;
        if (i < 0) break;
      }

      sy = 14;
      i = 0;

      while (1)
      {
        entry = dir.openFile("r");

        filter = file_browser_ext(entry.name());

        if (filter)
        {
          str = entry.name() + 1;

          for (j = 0; j < sizeof(name) - 1; ++j)
          {
            if (*str != 0 && *str != '.') name[j] = *str++; else name[j] = ' ';
          }

          printFast(8, sy, name, TFT_WHITE);

          drawCharFast(2, sy, ' ', TFT_WHITE, TFT_BLACK);

          if (pos == file_cursor)
          {
            strncpy(fname, entry.name(), fname_len);

            if (!(frame & 128)) drawCharFast(2, sy, 0xda, TFT_WHITE, TFT_BLACK);
          }
        }

        entry.close();

        if (!dir.next()) break;

        if (filter)
        {
          sy += 8;
          ++pos;
          ++i;
          if (i >= FILE_HEIGHT) break;
        }
      }

      change = 0;
    }

    check_key();

    if (pad_state_t & PAD_UP)
    {
      --file_cursor;

      if (file_cursor < 0) file_cursor = file_count - 1;

      change = 1;
      frame = 0;

    }

    if (pad_state_t & PAD_DOWN)
    {
      ++file_cursor;

      if (file_cursor >= file_count) file_cursor = 0;

      change = 1;
      frame = 0;
    }

    if (pad_state_t&PAD_ACT)
    {
      ++control_type;
      if (control_type >= CONTROL_TYPES) control_type = 0;
      change = 1;
    }

    if (pad_state_t & PAD_ESC) break;

    if ((pad_state&PAD_LFT) || (pad_state&PAD_RGT)) {
      fname[0]=0;
      break;
    }
    
    delay(1);
    ++frame;

    if (!(frame & 127)) change = 1;
  }

  off = control_type * 8;

  if (layout_scheme[off + 0] >= 0)
  {
    control_type = CONTROL_PAD_KEYBOARD;
    control_pad_l = layout_scheme[off + 0];
    control_pad_r = layout_scheme[off + 1];
    control_pad_u = layout_scheme[off + 2];
    control_pad_d = layout_scheme[off + 3];
    control_pad_act = layout_scheme[off + 4];
    control_pad_esc = layout_scheme[off + 5];
    control_pad_lft = layout_scheme[off + 6];
    control_pad_rgt = layout_scheme[off + 7];
  }
  else
  {
    control_type = CONTROL_PAD_KEMPSTON;
  }

  tft.fillScreen(TFT_BLACK);
}



void ICACHE_RAM_ATTR sound_ISR()
{
  int gap;

  sigmaDeltaWrite(0, sound_dac);

  sound_dac = sound_buffer[sound_rd_ptr];

  if (sound_rd_ptr < sound_wr_ptr) gap = sound_wr_ptr - sound_rd_ptr; else gap = SOUND_BUFFER_SIZE - sound_rd_ptr + sound_wr_ptr;

  if (gap < SOUND_MIN_GAP)
  {
    ++sound_rd_ptr;

    if (sound_rd_ptr >= SOUND_BUFFER_SIZE) sound_rd_ptr = 0;
  }
}



void setup()
{
  //serial init

  //Serial.begin(115200);
  //Serial.println(ESP.getFreeHeap());

  //disable wifi to save some battery power

  WiFi.mode(WIFI_OFF);

//I2C to 1mHz
  Wire.setClock(1000000);
  
  //DAC init, LCD backlit off

  dac.begin(MCP4725address);
  delay(50);
  dac.setVoltage(0, false);
  delay(50);
  
  //mcp23017 and buttons init, should preceed the TFT init
  mcp.begin(MCP23017address);
  delay(100);

  for (int i = 0; i < 8; ++i)
  {
    mcp.pinMode(i, INPUT);
    mcp.pullUp(i, HIGH);
  }

  pad_state = 0;
  pad_state_prev = 0;
  pad_state_t = 0;

  //TFT init

  mcp.pinMode(csTFTMCP23017pin, OUTPUT);
  mcp.digitalWrite(csTFTMCP23017pin, LOW);

  tft.begin();
  tft.setRotation(0);
  tft.fillScreen(TFT_BLACK);

  dac.setVoltage(4095, true);

  //keybModule init
  Wire.begin();    
  Wire.beginTransmission(0x27); //check for MCP23017Keyboard at address 0x27
  if (!Wire.endTransmission()) {
    keybModuleExist = 1;
    mcpKeyboard.begin(7);
    for (uint8_t i = 0; i < 7; i++){
      mcpKeyboard.pinMode(i, OUTPUT);
      mcpKeyboard.digitalWrite(i, HIGH);}
    for (uint8_t i = 0; i < 5; i++){
      mcpKeyboard.pinMode(i+8, INPUT);
      mcpKeyboard.pullUp(i+8, HIGH);}
    mcpKeyboard.pinMode(7, OUTPUT); 
    mcpKeyboard.digitalWrite(7, HIGH); //backlit on
   }
  else keybModuleExist = 0;
 
  //filesystem init
  SPIFFS.begin();

  //Serial.println(ESP.getFreeHeap());
  delay(300);
}



void sound_init(void)
{
  uint16_t i;

  sound_buffer = (uint8_t*)malloc(SOUND_BUFFER_SIZE);

  for (i = 0; i < SOUND_BUFFER_SIZE; ++i) sound_buffer[i] = 0;

  sound_dac = 0;
  sound_rd_ptr = 0;
  sound_wr_ptr = 0;

  noInterrupts();
  sigmaDeltaSetup(0, F_CPU / 256);
  sigmaDeltaAttachPin(SOUNDPIN);
  sigmaDeltaEnable();
  timer1_attachInterrupt(sound_ISR);
  timer1_enable(TIM_DIV1, TIM_EDGE, TIM_LOOP);
  timer1_write(ESP.getCpuFreqMHz() * 1000000 / SAMPLE_RATE);
  interrupts();
}



void change_ext(char* fname, const char* ext)
{
  while (1)
  {
    if (!*fname) break;
    if (*fname++ == '.')
    {
      fname[0] = ext[0];
      fname[1] = ext[1];
      fname[2] = ext[2];
      break;
    }
  }
}



uint8_t zx_layout_code(char c)
{
  if (c >= 'a' && c <= 'z') c -= 32;
  switch (c)
  {
    case 'A': return K_A;
    case 'B': return K_B;
    case 'C': return K_C;
    case 'D': return K_D;
    case 'E': return K_E;
    case 'F': return K_F;
    case 'G': return K_G;
    case 'H': return K_H;
    case 'I': return K_I;
    case 'J': return K_J;
    case 'K': return K_K;
    case 'L': return K_L;
    case 'M': return K_M;
    case 'N': return K_N;
    case 'O': return K_O;
    case 'P': return K_P;
    case 'Q': return K_Q;
    case 'R': return K_R;
    case 'S': return K_S;
    case 'T': return K_T;
    case 'U': return K_U;
    case 'V': return K_V;
    case 'W': return K_W;
    case 'X': return K_X;
    case 'Y': return K_Y;
    case 'Z': return K_Z;
    case '0': return K_0;
    case '1': return K_1;
    case '2': return K_2;
    case '3': return K_3;
    case '4': return K_4;
    case '5': return K_5;
    case '6': return K_6;
    case '7': return K_7;
    case '8': return K_8;
    case '9': return K_9;
    case '_': return K_SPACE;
    case '$': return K_ENTER;
    case '@': return K_CS;
    case '#': return K_SS;
  }

  return 0;
}



void zx_load_layout(char* filename)
{
  char cfg[8];

  fs::File f = SPIFFS.open(filename, "r");

  if (!f) return;

  f.readBytes(cfg, 8);
  f.close();
  
  control_type = CONTROL_PAD_KEYBOARD;
  control_pad_u = zx_layout_code(cfg[0]);
  control_pad_d = zx_layout_code(cfg[1]);
  control_pad_l = zx_layout_code(cfg[2]);
  control_pad_r = zx_layout_code(cfg[3]);
  control_pad_act = zx_layout_code(cfg[4]);
  control_pad_esc = zx_layout_code(cfg[5]);
  control_pad_lft = zx_layout_code(cfg[6]);
  control_pad_rgt = zx_layout_code(cfg[7]);
}



void loop()
{
  uint32_t t_prev, t_new;
  uint8_t frames;

  file_cursor = 0;

  control_type = CONTROL_PAD_KEYBOARD;
  control_pad_l = K_Z;
  control_pad_r = K_X;
  control_pad_u = K_Q;
  control_pad_d = K_A;
  control_pad_act = K_SPACE;
  control_pad_esc = K_ENTER;
  control_pad_lft = K_0;
  control_pad_rgt = K_1;

  //logo (skippable)

  if (espboy_logo_effect(0))
  {
    wait_any_key(1000);
    espboy_logo_effect(1);
  }

  file_browser("/", "Load .Z80:", filename, sizeof(filename));

  zx_init();

  if (filename != "")
  {
    change_ext(filename, "cfg");
    zx_load_layout(filename); 
  
    change_ext(filename, "scr");
    if (zx_load_scr(filename))
    {
      zx_render_frame();
      wait_any_key(3 * 1000);
    }

    change_ext(filename, "z80");
    zx_load_z80(filename);
  }

  SPIFFS.end();
  
  memset(line_change, 0xff, sizeof(line_change));
  sound_init();

  //main loop

  t_prev = micros();

  while (1)
  {
    check_key();

    switch (control_type)
    {  
      case CONTROL_PAD_KEYBOARD:
        memset(key_matrix, 0, sizeof(key_matrix));
        key_matrix[control_pad_l] |= (pad_state & PAD_LEFT) ? 1 : 0;
        key_matrix[control_pad_r] |= (pad_state & PAD_RIGHT) ? 1 : 0;
        key_matrix[control_pad_u] |= (pad_state & PAD_UP) ? 1 : 0;
        key_matrix[control_pad_d] |= (pad_state & PAD_DOWN) ? 1 : 0;
        key_matrix[control_pad_act] |= (pad_state & PAD_ACT) ? 1 : 0;
        key_matrix[control_pad_esc] |= (pad_state & PAD_ESC) ? 1 : 0;
        key_matrix[control_pad_lft] |= (pad_state & PAD_LFT) ? 1 : 0;
        key_matrix[control_pad_rgt] |= (pad_state & PAD_RGT) ? 1 : 0;
        break;

      case CONTROL_PAD_KEMPSTON:
        port_1f = 0;
        if (pad_state & PAD_LEFT) port_1f |= 0x02;
        if (pad_state & PAD_RIGHT) port_1f |= 0x01;
        if (pad_state & PAD_UP) port_1f |= 0x08;
        if (pad_state & PAD_DOWN) port_1f |= 0x04;
        if (pad_state & PAD_ACT) port_1f |= 0x10;
        key_matrix[K_SPACE] = (pad_state & PAD_ESC) ? 1 : 0;
        key_matrix[K_0] = (pad_state & PAD_LFT) ? 1 : 0;
        key_matrix[K_1] = (pad_state & PAD_RGT) ? 1 : 0;
        break;
    }

//check keyboard module
    if (keybModuleExist){
      static uint8_t keysReaded[7];
      static uint8_t row, col;
      static uint8_t keykeyboardpressed;
      static uint8_t symkeyboardpressed;
      symkeyboardpressed = 0;
      for (row = 0; row < 7; row++){
        mcpKeyboard.digitalWrite(row, LOW);
        keysReaded [row] = ((mcpKeyboard.readGPIOAB()>>8) & 31);
        mcpKeyboard.digitalWrite(row, HIGH);
      }
      if (!(keysReaded[2]&1)) symkeyboardpressed = 1; // if "sym" key is pressed
      for (row = 0; row < 7; row++)
        for (col = 0; col < 5; col++)
          if (!((keysReaded[row] >> col) & 1))
          {
            if (!symkeyboardpressed) keykeyboardpressed = pgm_read_byte(&keybCurrent[row][col]);
            else keykeyboardpressed = pgm_read_byte(&keybCurrent2[row][col]);
            if (keykeyboardpressed < 40) key_matrix[keykeyboardpressed] |= 1;
            else {
              if (keykeyboardpressed == K_DEL){
                key_matrix[K_0]|=1;
                key_matrix[K_CS]|=1;}
              if (keykeyboardpressed == K_LED){ 
                mcpKeyboard.digitalWrite(7, !mcpKeyboard.digitalRead(7));
                delay(100);}
            }   
          }   
    }
    
    t_new = micros();
    frames = ((t_new - t_prev) / (1000000 / ZX_FRAME_RATE));
    if (frames < 1) frames = 1;
    t_prev = t_new;

    if (frames > MAX_FRAMESKIP) frames = MAX_FRAMESKIP;

    while (frames--) zx_emulate_frame();

    zx_render_frame();

    delay(1);
  }
}
