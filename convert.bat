del gfx\*.h
del rom\*.h

bin2h.py gfx\espboy.bmp g_espboy > gfx\espboy.h
bin2h.py rom\1982.ROM rom > rom\rom.h