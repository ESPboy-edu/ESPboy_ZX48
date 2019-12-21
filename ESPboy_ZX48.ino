//v1.0 20.12.2019 initial version, with sound
//by Shiru
//shiru@mail.ru
//https://www.patreon.com/shiru8bit

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
*/

#include "zymosis.h"
#include "glcdfont.c"

#include "gfx/espboy.h"
#include "rom/rom.h"


#define MCP23017address 0 // actually it's 0x20 but in <Adafruit_MCP23017.h> lib there is (x|0x20) :)
#define SPIFFS_CACHE 0

//PINS
#define LEDPIN         2//D4
#define SOUNDPIN       0//D3

//SPI for LCD
#define csTFTMCP23017pin 8
#define TFT_RST       -1
#define TFT_DC        15//D8
#define TFT_CS        -1

Adafruit_MCP23017 mcp;

TFT_eSPI tft = TFT_eSPI();

#define MCP4725address 0x60
Adafruit_MCP4725 dac;

uint8_t pad_state;
uint8_t pad_state_prev;
uint8_t pad_state_t;

#define PAD_LEFT        0x01
#define PAD_UP          0x02
#define PAD_DOWN        0x04
#define PAD_RIGHT       0x08
#define PAD_A           0x10
#define PAD_B           0x20
#define PAD_ANY         (PAD_UP|PAD_DOWN|PAD_LEFT|PAD_RIGHT|PAD_A|PAD_B)

uint16_t line_buffer[128];

uint8_t line_change[32 + 1]; //bit mask to updating each line, the extra bit is border update flag

uint8_t memory[49152];

uint8_t port_fe;  //keyboard, tape, sound, border
uint8_t port_1f;  //kempston joystick

Z80Info cpu;

#define ZX_CLOCK_FREQ   3500000
#define ZX_FRAME_RATE   50

#define SAMPLE_RATE     32000   //more is better, but emulations gets slower

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
};

uint8_t key_matrix[40];


uint8_t control_type;

uint8_t control_pad_l;
uint8_t control_pad_r;
uint8_t control_pad_u;
uint8_t control_pad_d;
uint8_t control_pad_a;
uint8_t control_pad_b;

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



void zx_load_z80(const char* filename)
{
  uint8_t header[30];
  int i, sz, len, ptr, rle, val;

  fs::File f = SPIFFS.open(filename, "r");

  if (!f) return;

  sz = f.size();
  f.readBytes((char*)header, sizeof(header));
  f.readBytes((char*)memory, sz - sizeof(header));
  f.close();

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

  if (rle)  //unpack RLE'd snapshot without using extra RAM, most of snapshots are packed
  {
    ptr = 0;

    while (ptr < sizeof(memory) - 4)
    {
      if (memory[ptr] == 0xed && memory[ptr + 1] == 0xed)
      {
        len = memory[ptr + 2];
        val = memory[ptr + 3];

        memmove(&memory[ptr + len], &memory[ptr + 4], sizeof(memory) - ptr - len);	//not memcpy, because of the overlapping

        for (i = 0; i < len; ++i) memory[ptr++] = val;
      }
      else
      {
        ++ptr;
      }
    }
  }
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
#define LHSWAP(w)         ( (((w)>>8)&0x00ff) | (((w)<<8)&0xff00) )

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
  pad_state = 0;

  for (uint16_t i = 0; i < 8; i++)
  {
    if (!mcp.digitalRead(i)) pad_state |= (1 << i);
  }

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

const uint8_t layout_scheme[] = {
  -1, 0, 0, 0, 0, 0,
  K_O, K_P, K_Q, K_A, K_SPACE, K_M,
  K_Z, K_X, K_Q, K_A, K_SPACE, K_ENTER,
  K_6, K_7, K_9, K_8, K_0, K_ENTER,
  K_5, K_8, K_8, K_7, K_0, K_ENTER,
};



#define FILE_HEIGHT    14
#define FILE_FILTER   "z80"

int file_cursor;

bool file_browser_ext(const char* name)
{
  while (1) if (*name++ == '.') break;

  return (strcasecmp(name, FILE_FILTER) == 0) ? true : false;
}



void file_browser(String path, const char* header, char* filename, int filename_len)
{
  int16_t i, j, sy, pos, off, frame, file_count, control_type;
  bool change, filter;
  fs::Dir dir;
  fs::File entry;
  char name[19 + 1];
  const char* str;

  memset(filename, 0, filename_len);
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
    printFast(24, 60, "No files found", TFT_RED);

    while (1) delay(1000);
  }

  printFast(4, 4, (char*)header, TFT_GREEN);
  tft.fillRect(0, 12, 128, 1, TFT_WHITE);

  change = true;
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
        if (i <= 0) break;
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
            strncpy(filename, entry.name(), filename_len);

            if (frame & 32) drawCharFast(2, sy, 0xda, TFT_WHITE, TFT_BLACK);
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

      change = false;
    }

    check_key();

    if (pad_state_t & PAD_UP)
    {
      --file_cursor;

      if (file_cursor < 0) file_cursor = file_count - 1;

      change = true;
      frame = 32;

    }

    if (pad_state_t & PAD_DOWN)
    {
      ++file_cursor;

      if (file_cursor >= file_count) file_cursor = 0;

      change = true;
      frame = 32;
    }

    if (pad_state_t&PAD_A)
    {
      ++control_type;
      if (control_type >= CONTROL_TYPES) control_type = 0;
      change = true;
    }

    if (pad_state_t & PAD_B) break;

    delay(1);

    ++frame;

    if (!(frame & 31)) change = true;
  }

  off = control_type * 6;

  if (layout_scheme[off + 0] >= 0)
  {
    control_type = CONTROL_PAD_KEYBOARD;
    control_pad_l = layout_scheme[off + 0];
    control_pad_r = layout_scheme[off + 1];
    control_pad_u = layout_scheme[off + 2];
    control_pad_d = layout_scheme[off + 3];
    control_pad_a = layout_scheme[off + 4];
    control_pad_b = layout_scheme[off + 5];
  }
  else
  {
    control_type = CONTROL_PAD_KEMPSTON;
  }
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



  //serial init
  Serial.begin(115200);
  //disable wifi to save some battery power
  WiFi.mode(WIFI_OFF);
 // WiFi.forceSleepBegin();

  //DAC init, LCD backlit off
  dac.begin(MCP4725address);
  delay(100);
  dac.setVoltage(0, false);

  //mcp23017 and buttons init, should preceed the TFT init
  mcp.begin(MCP23017address);
  delay(100);

  for (int i = 0; i < 8; i++){
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

  //filesystem init
  SPIFFS.begin();
//  Serial.println(ESP.getFreeHeap() );
//  delay(500);
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



void loop()
{ 
  unsigned long t_prev, t_new;
  int frames;
  char filename[64];

  file_cursor = 0;

  control_type = CONTROL_PAD_KEYBOARD;
  control_pad_l = K_Z;
  control_pad_r = K_X;
  control_pad_u = K_Q;
  control_pad_d = K_A;
  control_pad_a = K_SPACE;
  control_pad_b = K_ENTER;

  //logo (skippable)

  if (espboy_logo_effect(0))
  {
    wait_any_key(1000);
    espboy_logo_effect(1);
  }

  file_browser("/", "Load .Z80:", filename, sizeof(filename));
  //strcpy(filename, "/Dizzy.z80");

  zx_init();
  zx_load_z80(filename);

  SPIFFS.end();
  
  sound_init();

  //main loop

  t_prev = micros();

  while (1)
  {
    check_key();

    switch (control_type)
    {
      case CONTROL_PAD_KEYBOARD:
        key_matrix[control_pad_l] = (pad_state & PAD_LEFT) ? 1 : 0;
        key_matrix[control_pad_r] = (pad_state & PAD_RIGHT) ? 1 : 0;
        key_matrix[control_pad_u] = (pad_state & PAD_UP) ? 1 : 0;
        key_matrix[control_pad_d] = (pad_state & PAD_DOWN) ? 1 : 0;
        key_matrix[control_pad_a] = (pad_state & PAD_A) ? 1 : 0;
        key_matrix[control_pad_b] = (pad_state & PAD_B) ? 1 : 0;
        break;

      case CONTROL_PAD_KEMPSTON:
        port_1f = 0;
        if (pad_state & PAD_LEFT) port_1f |= 0x02;
        if (pad_state & PAD_RIGHT) port_1f |= 0x01;
        if (pad_state & PAD_UP) port_1f |= 0x08;
        if (pad_state & PAD_DOWN) port_1f |= 0x04;
        if (pad_state & PAD_A) port_1f |= 0x10;
        key_matrix[K_SPACE] = (pad_state & PAD_A) ? 1 : 0;
        break;
    }

    t_new = micros();
    frames = ((t_new - t_prev) / (1000000 / ZX_FRAME_RATE)) + 1;
    t_prev = t_new;

    if (frames > MAX_FRAMESKIP) frames = MAX_FRAMESKIP;

    while (frames--) zx_emulate_frame();

    zx_render_frame();

    delay(1);
  }
}
