#
# SPDX-License-Identifier: Apache-2.0
#

KBUILD_CFLAGS += -DAP_SSID=\"${SSID}\"
KBUILD_CFLAGS += -DAP_PSK=\"${PSK}\"

$(info SSID is $(SSID))
$(info PSK is $(PSK))

obj-y += main.o
obj-y += wlan.o
obj-y += http_client.o
obj-y += http_message.o
obj-y += http_parser.o
obj-y += ss.o
obj-y += tcp_client.o
