#
# SPDX-License-Identifier: Apache-2.0
#

ifeq (${SSID},)
SSID:=cc3220
endif
ifeq (${PSK},)
PSK:=12345678
endif


KBUILD_CFLAGS += -DAP_SSID=\"${SSID}\"
KBUILD_CFLAGS += -DAP_PSK=\"${PSK}\"

$(info SSID is ${SSID})
$(info PSK is ${PSK})

obj-y += main.o
obj-y += wlan.o

