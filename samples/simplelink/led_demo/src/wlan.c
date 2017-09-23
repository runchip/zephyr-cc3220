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

#ifndef AP_CHANNEL
#define AP_CHANNEL 9
#endif

#define SSID_NAME AP_SSID
#define SECURITY_KEY AP_PSK
#define SECURITY_TYPE SL_WLAN_SEC_TYPE_WPA_WPA2
#define CHANNEL_MASK_ALL            (0x1FFF)
#define RSSI_TH_MAX                 (-95)
#define SL_STOP_TIMEOUT         (2000)

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


typedef struct StartApCmd
{
    uint8_t                 hidden;          /* Determine if AP has hidden SSID */
    uint8_t                 channel;         /* 802.11 WLAN channel [1-11] */
    uint8_t                 tx_pow;          /* The AP's TX power */
    uint8_t                 sta_limit;       /* Limits the number of stations that the AP's has */
    uint8_t                 *ssid;           /* Ap's SSID */
    SlWlanSecParams_t       secParams;       /* Security parameters - Security Type and Password */
}StartApCmd_t;

static StartApCmd_t StartApParams;


enum {
	STATUS_BIT_NWP_INIT = 0,
	STATUS_BIT_CONNECTION,
};

#define EXPECT_OK(expr)                                                        \
	({                                                                     \
		long ret = (expr);                                             \
		printf(#expr ": %s\n", (ret < 0) ? "FAIL" : "OK");             \
		ret;                                                           \
	})

static void fill_init_params()
{
	memset(&StartApParams, 0, sizeof(StartApParams));

	StartApParams.ssid = SSID_NAME;
	StartApParams.secParams.KeyLen = strlen(SECURITY_KEY);
	StartApParams.secParams.Key = SECURITY_KEY;
	StartApParams.secParams.Type = SECURITY_TYPE;

	StartApParams.channel = AP_CHANNEL;
	StartApParams.sta_limit = 4;
	StartApParams.tx_pow = 0; // 0 will set MAX power

	UART_PRINT("ssid: %s\n", StartApParams.ssid);
	UART_PRINT("Key: %s\n", StartApParams.secParams.Key);

}

int32_t ConfigureSimpleLinkToDefaultState();

void wlan_init_ap()
{
	int mode = -1;
	fill_init_params();

	SPI_init();  // init NWP control interface

	ConfigureSimpleLinkToDefaultState();

    // /* Restart the NWP so the new configuration will take affect */
    // EXPECT_OK((mode = sl_Start(0, 0, 0)) >= 0);
    // UART_PRINT("mode: %d\n", mode);

    // /* Set device role as AP */
    // EXPECT_OK(sl_WlanSetMode(ROLE_AP));
    // /* Restart the NWP so the new configuration will take affect */
    // EXPECT_OK(sl_Stop(1000));
    // if(sl_Start(0, 0, 0) != ROLE_AP)
    // {
    //     UART_PRINT("\n\r[wlan ap start] : Unable to configure AP role.\n\r");
    // }

    /* set SSID name */
    EXPECT_OK(sl_WlanSet(SL_WLAN_CFG_AP_ID, SL_WLAN_AP_OPT_SSID, strlen((const char *)(StartApParams.ssid)), (unsigned char*)(StartApParams.ssid)));

    /* Set security type */
    EXPECT_OK(sl_WlanSet(SL_WLAN_CFG_AP_ID, SL_WLAN_AP_OPT_SECURITY_TYPE, 1, (unsigned char *)(&StartApParams.secParams.Type)));

    /* Set password (if needed) */
    if(StartApParams.secParams.Type != SL_WLAN_SEC_TYPE_OPEN)
    {
        EXPECT_OK(sl_WlanSet(SL_WLAN_CFG_AP_ID, SL_WLAN_AP_OPT_PASSWORD, StartApParams.secParams.KeyLen, (unsigned char *)(StartApParams.secParams.Key)));
    }

    /* Set AP's SSID as hidden (if needed) */
    if(StartApParams.hidden)
    {
        EXPECT_OK(sl_WlanSet(SL_WLAN_CFG_AP_ID, SL_WLAN_AP_OPT_HIDDEN_SSID, sizeof(StartApParams.hidden), (unsigned char *)(&StartApParams.hidden)));
    }

    /* Set channel number */
    if(StartApParams.channel)
    {
        EXPECT_OK(sl_WlanSet(SL_WLAN_CFG_AP_ID, SL_WLAN_AP_OPT_CHANNEL, sizeof(StartApParams.channel), (unsigned char *)(&StartApParams.channel)));
    }

    /* Set STA connection limit */
    EXPECT_OK(sl_WlanSet(SL_WLAN_CFG_AP_ID, SL_WLAN_AP_OPT_MAX_STATIONS, sizeof(StartApParams.sta_limit), (unsigned char *)(&StartApParams.sta_limit)));

    /* Set TX power */
    EXPECT_OK(sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, SL_WLAN_GENERAL_PARAM_OPT_AP_TX_POWER, 1,(unsigned char *)(&StartApParams.tx_pow)));

    /* Set device role as AP */
    EXPECT_OK(sl_WlanSetMode(ROLE_AP));

    /* Restart the NWP so the new configuration will take affect */
    EXPECT_OK(sl_Stop(1000));
    if(sl_Start(0, 0, 0) != ROLE_AP)
    {
        UART_PRINT("\n\r[wlan ap start] : Unable to configure AP role.\n\r");
    }
}

static const char* GetModeName(int mode) {
	switch (mode) {
		case ROLE_STA: return "STA";
		case ROLE_P2P: return "P2P";
		case ROLE_AP: return "AP";
		default: break;
	}
	return "Unknow";
}

int32_t ConfigureSimpleLinkToDefaultState()
{
     uint8_t                              ucConfigOpt;
     uint8_t                              ucPower;
     int32_t                              RetVal = -1;
     int32_t                              Mode = -1;
     uint32_t                             IfBitmap = 0;
     SlWlanScanParamCommand_t             ScanDefault = {0};
     SlWlanRxFilterOperationCommandBuff_t RxFilterIdMask = {{0}};

     /* Turn NWP on */
     EXPECT_OK(Mode = sl_Start(0, 0, 0));
     printf("mode: %s, %d\n", GetModeName(Mode), Mode);
     if (Mode != ROLE_STA)
     {
           /* Set NWP role as STA */
           EXPECT_OK(sl_WlanSetMode(ROLE_STA) >= 0);

         /* For changes to take affect, we restart the NWP */
         EXPECT_OK(sl_Stop(SL_STOP_TIMEOUT));

         EXPECT_OK(sl_Start(0, 0, 0) >= 0);
     }

     if(Mode != ROLE_STA)
     {
         UART_PRINT("Failed to configure device to it's default state");
         return -1;
     }

     /* Set policy to auto only */
     EXPECT_OK(sl_WlanPolicySet(SL_WLAN_POLICY_CONNECTION, SL_WLAN_CONNECTION_POLICY(1,0,0,0), NULL ,0));

     /* Disable Auto Provisioning */
     EXPECT_OK(sl_WlanProvisioning(SL_WLAN_PROVISIONING_CMD_STOP, 0xFF, 0, NULL, 0x0));

     /* Delete existing profiles */
     EXPECT_OK(sl_WlanProfileDel(0xFF));

     /* enable DHCP client */
     EXPECT_OK(sl_NetCfgSet(SL_NETCFG_IPV4_STA_ADDR_MODE, SL_NETCFG_ADDR_DHCP, 0, 0));

     /* Disable ipv6 */
     IfBitmap = !(SL_NETCFG_IF_IPV6_STA_LOCAL | SL_NETCFG_IF_IPV6_STA_GLOBAL);
     EXPECT_OK(sl_NetCfgSet(SL_NETCFG_IF, SL_NETCFG_IF_STATE, sizeof(IfBitmap),(const unsigned char *)&IfBitmap));

     /* Configure scan parameters to default */
     ScanDefault.ChannelsMask = CHANNEL_MASK_ALL;
     ScanDefault.RssiThershold = RSSI_TH_MAX;

     EXPECT_OK(sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, SL_WLAN_GENERAL_PARAM_OPT_SCAN_PARAMS, sizeof(ScanDefault), (uint8_t *)&ScanDefault));

     /* Disable scans */
     ucConfigOpt = SL_WLAN_SCAN_POLICY(0, 0);
     EXPECT_OK(sl_WlanPolicySet(SL_WLAN_POLICY_SCAN , ucConfigOpt, NULL, 0));

     /* Set TX power lvl to max */
     ucPower = 0;
     EXPECT_OK(sl_WlanSet(SL_WLAN_CFG_GENERAL_PARAM_ID, SL_WLAN_GENERAL_PARAM_OPT_STA_TX_POWER, 1, (uint8_t *)&ucPower));

     /* Set NWP Power policy to 'normal' */
     EXPECT_OK(sl_WlanPolicySet(SL_WLAN_POLICY_PM, SL_WLAN_NORMAL_POLICY, NULL, 0));

     /* Unregister mDNS services */
     EXPECT_OK(sl_NetAppMDNSUnRegisterService(0, 0, 0));

     /* Remove all 64 RX filters (8*8) */
     memset(RxFilterIdMask.FilterBitmap , 0xFF, 8);

     EXPECT_OK(sl_WlanSet(SL_WLAN_RX_FILTERS_ID, SL_WLAN_RX_FILTER_REMOVE, sizeof(SlWlanRxFilterOperationCommandBuff_t),(uint8_t *)&RxFilterIdMask));

     /* Set NWP role as STA */
     EXPECT_OK(sl_WlanSetMode(ROLE_STA));

     /* For changes to take affect, we restart the NWP */
     EXPECT_OK(sl_Stop(0xFF));

     EXPECT_OK(sl_Start(0, 0, 0));

     if(ROLE_STA != Mode)
     {
         UART_PRINT("Failed to configure device to it's default state");
         return -1 ;
     }

     return 0;
}


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

	case SL_WLAN_EVENT_STA_ADDED: {
		UART_PRINT("[WLAN EVENT] AP accept new STA: %x:%x:%x:%x:%x:%x\n",
				pWlanEvent->Data.STAAdded.Mac[0],
				pWlanEvent->Data.STAAdded.Mac[1],
				pWlanEvent->Data.STAAdded.Mac[2],
				pWlanEvent->Data.STAAdded.Mac[3],
				pWlanEvent->Data.STAAdded.Mac[4],
				pWlanEvent->Data.STAAdded.Mac[5]);
	} break;

	case SL_WLAN_EVENT_STA_REMOVED: {
		UART_PRINT("[WLAN EVENT] AP lost a STA: %x:%x:%x:%x:%x:%x\n",
				pWlanEvent->Data.STARemoved.Mac[0],
				pWlanEvent->Data.STARemoved.Mac[1],
				pWlanEvent->Data.STARemoved.Mac[2],
				pWlanEvent->Data.STARemoved.Mac[3],
				pWlanEvent->Data.STARemoved.Mac[4],
				pWlanEvent->Data.STARemoved.Mac[5]);
	} break;

	default: {
		UART_PRINT("[WLAN EVENT] other event [0x%lx]\n\r",
			   pWlanEvent->Id);
	} break;
	}
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
	        UART_PRINT("[NETAPP] IPv6 aquired\n");
	        k_sem_give(&g_sem_ip_aquired);
	        break;

	    case SL_NETAPP_EVENT_DHCPV4_LEASED:
	        UART_PRINT("[NETAPP] AP/P2P DHCP lease a IP: %d.%d.%d.%d for MAC: %x:%x:%x:%x:%x:%x, lease time: %d\n",
	        	SL_IPV4_BYTE(pNetAppEvent->Data.IpLeased.IpAddress, 3),
	        	SL_IPV4_BYTE(pNetAppEvent->Data.IpLeased.IpAddress, 2),
	        	SL_IPV4_BYTE(pNetAppEvent->Data.IpLeased.IpAddress, 1),
	        	SL_IPV4_BYTE(pNetAppEvent->Data.IpLeased.IpAddress, 0),
	        	pNetAppEvent->Data.IpLeased.Mac[0],
	        	pNetAppEvent->Data.IpLeased.Mac[1],
	        	pNetAppEvent->Data.IpLeased.Mac[2],
	        	pNetAppEvent->Data.IpLeased.Mac[3],
	        	pNetAppEvent->Data.IpLeased.Mac[4],
	        	pNetAppEvent->Data.IpLeased.Mac[5],
	        	pNetAppEvent->Data.IpLeased.LeaseTime);
	        break;

	    case SL_NETAPP_EVENT_DHCPV4_RELEASED:
	        UART_PRINT("[NETAPP] AP/P2P DHCP lease a IP: %d.%d.%d.%d for MAC: %x:%x:%x:%x:%x:%x, reson: %d\n",
	        	SL_IPV4_BYTE(pNetAppEvent->Data.IpReleased.IpAddress, 3),
	        	SL_IPV4_BYTE(pNetAppEvent->Data.IpReleased.IpAddress, 2),
	        	SL_IPV4_BYTE(pNetAppEvent->Data.IpReleased.IpAddress, 1),
	        	SL_IPV4_BYTE(pNetAppEvent->Data.IpReleased.IpAddress, 0),
	        	pNetAppEvent->Data.IpReleased.Mac[0],
	        	pNetAppEvent->Data.IpReleased.Mac[1],
	        	pNetAppEvent->Data.IpReleased.Mac[2],
	        	pNetAppEvent->Data.IpReleased.Mac[3],
	        	pNetAppEvent->Data.IpReleased.Mac[4],
	        	pNetAppEvent->Data.IpReleased.Mac[5],
	        	pNetAppEvent->Data.IpReleased.Reason);
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


