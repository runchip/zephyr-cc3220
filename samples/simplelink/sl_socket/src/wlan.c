#include <kernel.h>
#include <misc/printk.h>
#include <ti/drivers/net/wifi/simplelink.h>
#include <ti/drivers/net/wifi/bsd/sys/socket.h>
#include <ti/drivers/net/wifi/bsd/netinet/in.h>
#include <ti/drivers/net/wifi/bsd/arpa/inet.h>
#define printf printk
#define UART_PRINT printk
#define perror(str) printk("perror %s+%d: %s", __FILE__, __LINE__, str)

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

K_SEM_DEFINE(g_sem_connected, 0, 1);
K_SEM_DEFINE(g_sem_ip_aquired, 0, 1);


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
			   
		SET_STATUS_BIT(g_wlan_status, STATUS_BIT_CONNECTION);
        k_sem_give(&g_sem_connected);
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

void wlan_init(void)
{
	int mode = 0;
	static SlWlanSecParams_t security_params = {0};

	SPI_init();

#define EXPECT_OK(expr)                                                        \
	({                                                                     \
		long ret = (expr);                                             \
		printf(#expr ": %s\n", (ret < 0) ? "FAIL" : "OK");             \
		ret;                                                           \
	})

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

    printf("wait for connected...\n");
	k_sem_take(&g_sem_connected, K_FOREVER);
	
	printf("acquire IP address...\n");
	k_sem_take(&g_sem_ip_aquired, K_FOREVER);
	printf("IP address aquired.\n");
}

void SimpleLinkFatalErrorEventHandler(SlDeviceFatal_t *pSlFatalErrorEvent) {
	UART_PRINT("[FATAL ERROR] Id: %x\r\n", pSlFatalErrorEvent->Id);
}

void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pSlDeviceEvent) {
	UART_PRINT("[EVENT] Id: %x\r\n", pSlDeviceEvent->Id);
}

void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent) {
	switch (pNetAppEvent->Id) {
	    case SL_NETAPP_EVENT_IPV4_ACQUIRED:
	        UART_PRINT("[NETAPP] IPv4 aquired: %d.%d.%d.%d, Gateway: %d.%d.%d.%d, DNS: %d.%d.%d.%d\n",
	            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,3),
		        SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,2),
		        SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,1),
		        SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Ip,0),
		        
	            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Gateway,3),
		        SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Gateway,2),
		        SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Gateway,1),
		        SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Gateway,0),

	            SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Dns,3),
		        SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Dns,2),
		        SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Dns,1),
		        SL_IPV4_BYTE(pNetAppEvent->Data.IpAcquiredV4.Dns,0));
	        k_sem_give(&g_sem_ip_aquired);
    	    break;
	    
	    case SL_NETAPP_EVENT_IPV6_ACQUIRED:
	        UART_PRINT("[NETAPP] IPv4 aquired: ");
	        k_sem_give(&g_sem_ip_aquired);
	        break;

	    default:
	    	UART_PRINT("[NETAPP] Id: %x\r\n", pNetAppEvent->Id);
	}
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


