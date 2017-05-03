
# objs
snd-soc-tfa98xx-objs    := tfa98xx.o
snd-soc-tfa98xx-objs    += tfa_container.o
snd-soc-tfa98xx-objs    += tfa_dsp.o
snd-soc-tfa98xx-objs    += tfa9896_init.o
ifdef TFA_DEBUG
snd-soc-tfa98xx-objs    += tfa_debug.o
endif


# CFLAGS
CFLAGS_tfa98xx.o       += $(TFA98XX_FLAGS)
CFLAGS_tfa_container.o += $(TFA98XX_FLAGS)
CFLAGS_tfa_dsp.o       += $(TFA98XX_FLAGS)
CFLAGS_tfa9872_init.o  += $(TFA98XX_FLAGS)
ifdef TFA_DEBUG
CFLAGS_tfa_debug.o     += $(TFA98XX_FLAGS)
endif


# built-in driver
obj-y	+= snd-soc-tfa98xx.o
