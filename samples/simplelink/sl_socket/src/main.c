#ifdef CONFIG_HAS_CC3220SDK
#include <kernel.h>
#include <misc/printk.h>
#include <ti/drivers/net/wifi/simplelink.h>
#include <ti/drivers/net/wifi/bsd/sys/socket.h>
#include <ti/drivers/net/wifi/bsd/netinet/in.h>
#include <ti/drivers/net/wifi/bsd/arpa/inet.h>
#define printf printk
#define UART_PRINT printk
#define perror(str) printk("perror %s+%d: %s", __FILE__, __LINE__, str)

/* =========================== SPI Driver Fxns ========================= */
/* TODO: put in a CC3220SF_LAUNCHXL.c board module in ext porting dir */

#include <ti/devices/cc32xx/inc/hw_ints.h>
#include <ti/devices/cc32xx/inc/hw_memmap.h>
#include <ti/devices/cc32xx/inc/hw_types.h>

#include <ti/devices/cc32xx/driverlib/rom_map.h>
#include <ti/devices/cc32xx/driverlib/prcm.h>
#include <ti/devices/cc32xx/driverlib/spi.h>
#include <ti/devices/cc32xx/driverlib/udma.h>
typedef enum CC3220SF_LAUNCHXL_SPIName {
    CC3220SF_LAUNCHXL_SPI0 = 0,
    CC3220SF_LAUNCHXL_SPICOUNT
} CC3220SF_LAUNCHXL_SPIName;

#include <ti/drivers/SPI.h>
#include <ti/drivers/spi/SPICC32XXDMA.h>

SPICC32XXDMA_Object spiCC3220SDMAObjects[CC3220SF_LAUNCHXL_SPICOUNT];

__attribute__ ((aligned (32)))
uint32_t spiCC3220SDMAscratchBuf[CC3220SF_LAUNCHXL_SPICOUNT];

const SPICC32XXDMA_HWAttrsV1 spiCC3220SDMAHWAttrs[CC3220SF_LAUNCHXL_SPICOUNT] = {
    /* index 0 is reserved for LSPI that links to the NWP */
    {
	.baseAddr = LSPI_BASE,
	.intNum = INT_LSPI,
	.intPriority = (~0),
	.spiPRCM = PRCM_LSPI,
	.csControl = SPI_SW_CTRL_CS,
	.csPolarity = SPI_CS_ACTIVEHIGH,
	.pinMode = SPI_4PIN_MODE,
	.turboMode = SPI_TURBO_OFF,
	.scratchBufPtr = &spiCC3220SDMAscratchBuf[CC3220SF_LAUNCHXL_SPI0],
	.defaultTxBufValue = 0,
	.rxChannelIndex = UDMA_CH12_LSPI_RX,
	.txChannelIndex = UDMA_CH13_LSPI_TX,
	.minDmaTransferSize = 100,
	.mosiPin = SPICC32XXDMA_PIN_NO_CONFIG,
	.misoPin = SPICC32XXDMA_PIN_NO_CONFIG,
	.clkPin = SPICC32XXDMA_PIN_NO_CONFIG,
	.csPin = SPICC32XXDMA_PIN_NO_CONFIG
    }
};

const SPI_Config SPI_config[CC3220SF_LAUNCHXL_SPICOUNT] = {
    {
	.fxnTablePtr = &SPICC32XXDMA_fxnTable,
	.object = &spiCC3220SDMAObjects[CC3220SF_LAUNCHXL_SPI0],
	.hwAttrs = &spiCC3220SDMAHWAttrs[CC3220SF_LAUNCHXL_SPI0]
    }
};

const uint_least8_t SPI_count = CC3220SF_LAUNCHXL_SPICOUNT;


/*
 *  =============================== DMA ===============================
 */
#include <ti/drivers/dma/UDMACC32XX.h>

__attribute__ ((aligned (1024)))
static tDMAControlTable dmaControlTable[64];

/*
 *  ======== dmaErrorFxn ========
 *  This is the handler for the uDMA error interrupt.
 */
static void dmaErrorFxn(uintptr_t arg)
{
    int status = MAP_uDMAErrorStatusGet();
    MAP_uDMAErrorStatusClear();

    /* Suppress unused variable warning */
    (void)status;

    while (1);
}

UDMACC32XX_Object udmaCC3220SObject;

const UDMACC32XX_HWAttrs udmaCC3220SHWAttrs = {
    .controlBaseAddr = (void *)dmaControlTable,
    .dmaErrorFxn = (UDMACC32XX_ErrorFxn)dmaErrorFxn,
    .intNum = INT_UDMAERR,
    .intPriority = (~0)
};

const UDMACC32XX_Config UDMACC32XX_config = {
    .object = &udmaCC3220SObject,
    .hwAttrs = &udmaCC3220SHWAttrs
};



#define DEVICE_ERROR		("Device error, please refer \"DEVICE ERRORS CODES\" section in errors.h")
#define WLAN_ERROR		("WLAN error, please refer \"WLAN ERRORS CODES\" section in errors.h")
#define BSD_SOCKET_ERROR	("BSD Socket error, please refer \"BSD SOCKET ERRORS CODES\" section in errors.h")
#define SL_SOCKET_ERROR		("Socket error, please refer \"SOCKET ERRORS CODES\" section in errors.h")
#define NETAPP_ERROR		("Netapp error, please refer \"NETAPP ERRORS CODES\" section in errors.h")
#define OS_ERROR		("OS error, please refer \"NETAPP ERRORS CODES\" section in errno.h")
#define CMD_ERROR		("Invalid option/command.")

#define CHANNEL_MASK_ALL	    (0x1FFF)
#define RSSI_TH_MAX		    (-95)
#define SL_STOP_TIMEOUT		(200)


/* Configure the device to a default state, resetting previous parameters .*/
int32_t ConfigureSimpleLinkToDefaultState()
{
     uint8_t				  ucConfigOpt;
     uint8_t				  ucPower;
     int32_t				  RetVal = -1;
     int32_t				  Mode = -1;
     uint32_t				  IfBitmap = 0;
     SlWlanScanParamCommand_t		  ScanDefault = {0};
     SlWlanRxFilterOperationCommandBuff_t RxFilterIdMask = {{0}};

     /* Turn NWP on */
     Mode = sl_Start(0, 0, 0);
     ASSERT_ON_ERROR(Mode, DEVICE_ERROR);

     if(Mode != ROLE_STA)
     {
	   /* Set NWP role as STA */
	   Mode = sl_WlanSetMode(ROLE_STA);
	   ASSERT_ON_ERROR(Mode, WLAN_ERROR);

	 /* For changes to take affect, we restart the NWP */
	 RetVal = sl_Stop(SL_STOP_TIMEOUT);
	 ASSERT_ON_ERROR(RetVal, DEVICE_ERROR);

	 Mode = sl_Start(0, 0, 0);
	 ASSERT_ON_ERROR(Mode, DEVICE_ERROR);
     }

     if(Mode != ROLE_STA)
     {
	 printk("Failed to configure device to it's default state");
	 return -1;
     }

     /* Set policy to auto only */
     RetVal = sl_WlanPolicySet(SL_WLAN_POLICY_CONNECTION, SL_WLAN_CONNECTION_POLICY(1,0,0,0), NULL ,0);
     ASSERT_ON_ERROR(RetVal, WLAN_ERROR);

     /* Disable Auto Provisioning */
     RetVal = sl_WlanProvisioning(SL_WLAN_PROVISIONING_CMD_STOP, 0xFF, 0, NULL, 0x0);
     ASSERT_ON_ERROR(RetVal, WLAN_ERROR);

     /* Delete existing profiles */
     RetVal = sl_WlanProfileDel(0xFF);
     ASSERT_ON_ERROR(RetVal, WLAN_ERROR);

     /* enable DHCP client */
     RetVal = sl_NetCfgSet(SL_NETCFG_IPV4_STA_ADDR_MODE, SL_NETCFG_ADDR_DHCP, 0, 0);
     ASSERT_ON_ERROR(RetVal, NETAPP_ERROR);

     /* Disable ipv6 */
     IfBitmap = !(SL_NETCFG_IF_IPV6_STA_LOCAL | SL_NETCFG_IF_IPV6_STA_GLOBAL);
     RetVal = sl_NetCfgSet(SL_NETCFG_IF, SL_NETCFG_IF_STATE, sizeof(IfBitmap),(const unsigned char *)&IfBitmap);
     ASSERT_ON_ERROR(RetVal, NETAPP_ERROR);

     /* Configure scan parameters to default */
     ScanDefault.ChannelsMask = CHANNEL_MASK_ALL;
     ScanDefault.RssiThershold = RSSI_TH_MAX;

     RetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, SL_WLAN_GENERAL_PARAM_OPT_SCAN_PARAMS, sizeof(ScanDefault), (uint8_t *)&ScanDefault);
     ASSERT_ON_ERROR(RetVal, WLAN_ERROR);

     /* Disable scans */
     ucConfigOpt = SL_WLAN_SCAN_POLICY(0, 0);
     RetVal = sl_WlanPolicySet(SL_WLAN_POLICY_SCAN , ucConfigOpt, NULL, 0);
     ASSERT_ON_ERROR(RetVal, WLAN_ERROR);

     /* Set TX power lvl to max */
     ucPower = 0;
     RetVal = sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, SL_WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (uint8_t *)&ucPower);
     ASSERT_ON_ERROR(RetVal, WLAN_ERROR);

     /* Set NWP Power policy to 'normal' */
     RetVal = sl_WlanPolicySet(SL_WLAN_POLICY_PM, SL_WLAN_NORMAL_POLICY, NULL, 0);
     ASSERT_ON_ERROR(RetVal, WLAN_ERROR);

     /* Unregister mDNS services */
     RetVal = sl_NetAppMDNSUnRegisterService(0, 0, 0);
     ASSERT_ON_ERROR(RetVal, NETAPP_ERROR);

     /* Remove all 64 RX filters (8*8) */
     memset(RxFilterIdMask.FilterBitmap , 0xFF, 8);

     RetVal = sl_WlanSet(SL_WLAN_RX_FILTERS_ID, SL_WLAN_RX_FILTER_REMOVE, sizeof(SlWlanRxFilterOperationCommandBuff_t),(uint8_t *)&RxFilterIdMask);
     ASSERT_ON_ERROR(RetVal, WLAN_ERROR);

     /* Set NWP role as STA */
     RetVal = sl_WlanSetMode(ROLE_STA);
     ASSERT_ON_ERROR(RetVal, WLAN_ERROR);

     /* For changes to take affect, we restart the NWP */
     RetVal = sl_Stop(0xFF);
     ASSERT_ON_ERROR(RetVal, DEVICE_ERROR);

     Mode = sl_Start(0, 0, 0);
     ASSERT_ON_ERROR(Mode, DEVICE_ERROR);

     if(ROLE_STA != Mode)
     {
	 printk("Failed to configure device to it's default state");
	 return -1 ;
     }
     else
     {
//	 app_CB.Role = ROLE_STA;
//	 SET_STATUS_BIT(app_CB.Status, STATUS_BIT_NWP_INIT);
     }

     return 0;
}

#else // CONFIG_HAS_CC3220SDK

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <stdio.h>
#include <string.h>

#endif // CONFIG_HAS_CC3220SDK

#define HOST "example.com"
#define PORT 80

const char* CRLF = "\r\n";
const char* HTTP_EOH = "\r\n\r\n"; // End of Header
const char* CONTENT_LENGTH = "Content-Length:";
const char* REQUEST = "GET / HTTP/1.1\r\n"
		       "Host: " HOST "\r\n"
		       "User-Agent: curl/7.47.0\r\n"
		       "Accept: */*\r\n"
		       "\r\n";
char response[4096];

static void wlan_init(void);

uint32_t resolve_host_addr(const char *host)
{
	uint32_t dest = 0;
#ifdef CONFIG_HAS_CC3220SDK
	sl_NetAppDnsGetHostByName((_i8 *)host, strlen(host), (_u32 *)&dest,
				  SL_AF_INET);
	dest = sl_Htonl(dest);
#else  // CONFIG_HAS_CC3220SDK
	struct hostent *he;

	he = gethostbyname(host);
	if (he && he->h_addr_list && he->h_addr_list[0]) {
		dest = ((struct in_addr *)(he->h_addr_list[0]))->s_addr;
	}
#endif // CONFIG_HAS_CC3220SDK
	return dest;
}


void hexdump(void* ptr, int size) {
	int i;
	const char* c = (char*) ptr;
	for (i = 0; i < size; i++) {
		printf("%02X%s", c[i], (i+1) % 8 == 0 ? "\n" : " ");
	}
}

int main(int argc, char *argv[])
{
	int sock, ret, count, content_length;
	struct sockaddr_in server_addr;
	char *p;
	int EOH, CL;

	wlan_init();

	sock = socket(AF_INET, SOCK_STREAM, 0);
	if (sock < 0) {
		perror("socket");
	}

	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_port = htons(PORT);
	server_addr.sin_addr.s_addr = resolve_host_addr(HOST);

#if !defined(CONFIG_HAS_CC3220SDK)
	inet_ntop(AF_INET, &server_addr.sin_addr, response, sizeof(response));
	printf("server: %s\n", response);
#endif

	printf("connect to server...\n");
	ret = connect(sock, (const struct sockaddr *)&server_addr,
		      sizeof(server_addr));
	if (ret < 0) {
		perror("connect");
	}

	printf("send request...\n");
	ret = send(sock, REQUEST, strlen(REQUEST), 0);
	if (ret < 0) {
		perror("send");
	}
	printf("%d bytes sent\n", ret);

	printf("recv response...\n");
	count = 0;
	CL = EOH = 0;
	content_length = 0;
	while ((ret = recv(sock, &response[count], sizeof(response) - count, 0)) > 0) {
		printf("received %d bytes...\n", ret);
		//hexdump(&response[count], ret);
		count += ret;
		if ((p = strstr(response, HTTP_EOH)) != NULL) {
			EOH = p - response;
			printf("EOH at %d\n", EOH);
		}
		if ((p = strstr(response, CONTENT_LENGTH)) != NULL) {
			CL = p - response;
			content_length = atoi(p + strlen(CONTENT_LENGTH));
			printf("CL at %d, value: %d\n", CL, content_length);
		}
		if (CL && count - EOH - strlen(HTTP_EOH) == content_length) {
			break;
		}
	}
	response[count] = '\0';
	printf("response: %s\n", response);
	if (ret < 0) {
		perror("recv");
	}

	close(sock);

	printf("done\n");

	return 0;
}

#ifdef CONFIG_HAS_CC3220SDK

#if !defined(AP_SSID) || !defined(AP_PSK)
#error AP_SSID or AP_PSK not defined
#endif

#define SSID_NAME AP_SSID
#define SECURITY_KEY AP_PSK
#define SECURITY_TYPE SL_WLAN_SEC_TYPE_WPA_WPA2

#define CLR_STATUS_BIT_ALL(status_variable) (status_variable = 0)
#define SET_STATUS_BIT(status_variable, bit) (status_variable |= (1 << (bit)))
#define CLR_STATUS_BIT(status_variable, bit) (status_variable &= ~(1 << (bit)))
#define GET_STATUS_BIT(status_variable, bit) (status_variable & (1 << (bit)))

#define SSID_LEN_MAX 32
unsigned long g_wlan_status;
unsigned char g_connected_ssid[SSID_LEN_MAX + 1];
unsigned char g_connected_bssid[SL_WLAN_BSSID_LENGTH];

enum {
	STATUS_BIT_NWP_INIT = 0,
	STATUS_BIT_CONNECTION,
};

void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
	if (pWlanEvent == NULL) {
		return;
	}

	switch (pWlanEvent->Id) {
	case SL_WLAN_EVENT_CONNECT: {
		SET_STATUS_BIT(g_wlan_status, STATUS_BIT_CONNECTION);

		// Copy new connection SSID and BSSID to global parameters
		memcpy(g_connected_ssid,
			   pWlanEvent->Data.Connect.SsidName,
			   pWlanEvent->Data.Connect.SsidLen);
		memcpy(g_connected_bssid, pWlanEvent->Data.Connect.Bssid, SL_WLAN_BSSID_LENGTH);

		UART_PRINT("[WLAN EVENT] STA Connected to the AP: %s ,"
			   " BSSID: %x:%x:%x:%x:%x:%x\n",
			   g_connected_ssid, g_connected_bssid[0],
			   g_connected_bssid[1], g_connected_bssid[2],
			   g_connected_bssid[3], g_connected_bssid[4],
			   g_connected_bssid[5]);
	} break;

	case SL_WLAN_EVENT_DISCONNECT: {
		CLR_STATUS_BIT(g_wlan_status, STATUS_BIT_CONNECTION);

		UART_PRINT(
			"[WLAN EVENT]Device disconnected from the AP: %s,"
			"BSSID: %x:%x:%x:%x:%x:%x, ReasonCode: %d\r\n",
			    g_connected_ssid, g_connected_bssid[0],
			    g_connected_bssid[1], g_connected_bssid[2],
			    g_connected_bssid[3], g_connected_bssid[4],
			    g_connected_bssid[5], pWlanEvent->Data.Disconnect.ReasonCode);
		memset(g_connected_ssid, 0, sizeof(g_connected_ssid));
		memset(g_connected_bssid, 0, sizeof(g_connected_bssid));
	} break;

	case SL_WLAN_EVENT_P2P_CONNECT: {
		UART_PRINT("[WLAN EVENT] P2P client connection event\n");
	} break;

	case SL_WLAN_EVENT_P2P_DISCONNECT: {
		UART_PRINT("[WLAN EVENT] P2P client disconnection event\n");
	} break;

	default: {
		UART_PRINT("[WLAN EVENT] Unknow event [0x%lx]\n\r",
			   pWlanEvent->Id);
	} break;
	}
}

#define WLAN_STACK_SIZE 4096
char wlan_thread_stack[WLAN_STACK_SIZE];
atomic_t wlan_thread_started;
void wlan_thread_work(void *arg1, void *arg2, void *arg3)
{
	printf("%s start\n", __func__);
	while (1) {
		atomic_set(&wlan_thread_started, 1);
		// _SlNonOsMainLoopTask();
	}
}

void wlan_init(void)
{
	int mode = 0;
	static SlWlanSecParams_t security_params = {0};

	SPI_init();
#if 0
	k_thread_spawn(wlan_thread_stack, WLAN_STACK_SIZE, wlan_thread_work,
		       NULL, NULL, NULL, K_PRIO_PREEMPT(1), 0, K_NO_WAIT);
	while (atomic_get(&wlan_thread_started) == 0) {
		k_sleep(100);
	}
#endif

#define EXPECT_OK(expr)                                                        \
	({                                                                     \
		long ret = (expr);                                             \
		printf(#expr ": %s\n", (ret < 0) ? "FAIL" : "OK");             \
		ret;                                                           \
	})

    // ConfigureSimpleLinkToDefaultState();

	printf("wlan start\n");
	mode = EXPECT_OK(sl_Start(NULL, NULL, NULL));
	if (mode != ROLE_STA) {
		printf("wlan stop\n");
		EXPECT_OK(sl_Stop(200));
		printf("set mode to STA\n");
		EXPECT_OK(sl_WlanSetMode(ROLE_STA));
		printf("wlan start\n");
		EXPECT_OK(sl_Start(NULL, NULL, NULL));
	}
	printf("set mode successed\n");

	// AUTO policy
	EXPECT_OK(sl_WlanPolicySet(SL_WLAN_POLICY_CONNECTION,
				   SL_WLAN_CONNECTION_POLICY(1, 0, 0, 0), NULL,
				   0));

	// delete all profiles
	EXPECT_OK(sl_WlanProfileDel(0xFF));

	security_params.Key = (signed char *)SECURITY_KEY;
	security_params.KeyLen = strlen(SECURITY_KEY);
	security_params.Type = SECURITY_TYPE;
	EXPECT_OK(sl_WlanProfileAdd(SSID_NAME, strlen(SSID_NAME), NULL,
				    &security_params, NULL, 1, 0));

	printf("try to connect to AP %s %s\r\n", SSID_NAME, SECURITY_KEY);
	EXPECT_OK(sl_WlanConnect(SSID_NAME, strlen(SSID_NAME), NULL,
				 &security_params, NULL));
	while (GET_STATUS_BIT(g_wlan_status, STATUS_BIT_CONNECTION) == 0) {
		printf("connecting...\r\n");
		k_sleep(1000);
	}
}

void SimpleLinkFatalErrorEventHandler(SlDeviceFatal_t *pSlFatalErrorEvent) {
	UART_PRINT("[FATAL ERROR] Id: %x\r\n", pSlFatalErrorEvent->Id);
}

void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pSlDeviceEvent) {
	UART_PRINT("[EVENT] Id: %x\r\n", pSlDeviceEvent->Id);
}

void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pSlNetAppEvent) {
	UART_PRINT("[NETAPP] Id: %x\r\n", pSlNetAppEvent->Id);
}

void SimpleLinkSockEventHandler(SlSockEvent_t *pSlSockEvent) {
	UART_PRINT("[SOCKET] Event: %x\r\n", pSlSockEvent->Event);
}

void SimpleLinkHttpServerEventHandler(SlNetAppHttpServerEvent_t *pSlHttpServerEvent, SlNetAppHttpServerResponse_t *pSlHttpServerResponse) {
	UART_PRINT("[HTTP SERVER] Event: %x\r\n", pSlHttpServerEvent->Event);
}

void SimpleLinkNetAppRequestEventHandler(SlNetAppRequest_t *pNetAppRequest, SlNetAppResponse_t *pNetAppResponse) {
	UART_PRINT("[NETAPPREQ] AppId %x\r\n", pNetAppRequest->AppId);
}

void SimpleLinkNetAppRequestMemFreeEventHandler(_u8 *buffer) {
	UART_PRINT("[MEMFREE] buffer: %x\r\n", buffer);
}


#else  // CONFIG_HAS_CC3220SDK
void wlan_init(void) {}
#endif // CONFIG_HAS_CC3220SDK
