/*
 * Copyright (c) 2016-2017, Texas Instruments Incorporated
 *
 * SPDX-License-Identifier: Apache-2.0
 *
 * Console shell to test SimpleLink WiFi host driver APIs.
 * Inspired by the Zephyr shell and irc-bot.
 */

#include <stdlib.h>
#include <string.h>
#include <zephyr.h>
#include <misc/printk.h>
#include <console.h>
#include <stdint.h>

#include <ti/drivers/net/wifi/simplelink.h>


#define ARGC_MAX 10
#define COMMAND_MAX_LEN 50

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

/* =========================== SimpleLink Support Fxns ========================= */

#undef ASSERT_ON_ERROR
#define ASSERT_ON_ERROR(ret, errortype) __ASSERT_NO_MSG(ret >= 0)

#define SET_STATUS_BIT(status_variable, bit) status_variable |= (1<<(bit))

#define CLR_STATUS_BIT(status_variable, bit) status_variable &= ~(1<<(bit))

#define GET_STATUS_BIT(status_variable, bit)	\
				(0 != (status_variable & (1<<(bit))))

#define IS_CONNECTED(status_variable)	     GET_STATUS_BIT(status_variable,\
							    STATUS_BIT_CONNECTION)

#define IS_NWP_INITIALIZED(status_variable)   GET_STATUS_BIT(status_variable,\
							    STATUS_BIT_NWP_INIT);

#define CMD_BUFFER_LEN		(256)
#define MAX_SCAN_TRIALS		(10)
#define MAX_CMD_NAME_LEN	(32)
#define APPLICATION_NAME	("Network Terminal")
#define APPLICATION_VERSION	("1.0.1.0")
#define TASK_STACK_SIZE		(2048)
#define SPAWN_TASK_PRIORITY	(9)
#define SSID_LEN_MAX		(32)
#define BSSID_LEN_MAX		(6)
#define PASSWD_LEN_MAX		(63)
#define PASSWD_LEN_MIN		(8)
#define WLAN_SCAN_COUNT		(30)
#define MAX_FILE_NAME_LEN	(32)
#define MAX_TEXT_PAD_SIZE	(256)
#define MAX_FILE_LIST		(20)
#define MAX_BUF_SIZE		(1400)
#define SL_STOP_TIMEOUT		(200)
#define DEV_TYPE_LEN		(17)
#define IPV6_ADDR_LEN		(16)
#define IPV4_ADDR_LEN		(4)
#define DEVICE_ERROR		("Device error, please refer \"DEVICE ERRORS CODES\" section in errors.h")
#define WLAN_ERROR		("WLAN error, please refer \"WLAN ERRORS CODES\" section in errors.h")
#define BSD_SOCKET_ERROR	("BSD Socket error, please refer \"BSD SOCKET ERRORS CODES\" section in errors.h")
#define SL_SOCKET_ERROR		("Socket error, please refer \"SOCKET ERRORS CODES\" section in errors.h")
#define NETAPP_ERROR		("Netapp error, please refer \"NETAPP ERRORS CODES\" section in errors.h")
#define OS_ERROR		("OS error, please refer \"NETAPP ERRORS CODES\" section in errno.h")
#define CMD_ERROR		("Invalid option/command.")

#define CHANNEL_MASK_ALL	    (0x1FFF)
#define RSSI_TH_MAX		    (-95)

/* Application defines */
#define QUERY_CONTINUOUS_MAX_TEXT_LENGTH	(120)
#define QUERY_CONTINUOUS_MAX_SERVICES		(3)
#define QUERY_CONTINUOUS_MAX_CHARACTERS		(1480)

typedef enum{

    STATUS_BIT_NWP_INIT = 0,	      /* This bit is set: Network Processor is powered up */


    STATUS_BIT_CONNECTION,	      /* This bit is set: the device is connected
					 to the AP or client is connected to device (AP) */

    STATUS_BIT_IP_LEASED,	      /* This bit is set: the device has leased IP to
					 any connected client */

    STATUS_BIT_IP_ACQUIRED,	      /* This bit is set: the device has acquired an IP */


    STATUS_BIT_P2P_DEV_FOUND,	      /* If this bit is set: the device (P2P mode)
					 found any p2p-device in scan */

    STATUS_BIT_P2P_REQ_RECEIVED,      /* If this bit is set: the device (P2P mode)
					 found any p2p-negotiation request */

    STATUS_BIT_CONNECTION_FAILED,     /* If this bit is set: the device(P2P mode)
					 connection to client(or reverse way) is failed */

    STATUS_BIT_PING_STARTED,	      /* This bit is set: device is undergoing ping operation */


    STATUS_BIT_SCAN_RUNNING,	      /* This bit is set: Scan is running is background */


    STATUS_BIT_IPV6_ACQUIRED,	      /* If this bit is set: the device has acquired
					 an IPv6 address */

    STATUS_BIT_IPV6_GLOBAL_ACQUIRED,  /* If this bit is set: the device has acquired
					 an IPv6 address */

    STATUS_BIT_IPV6_LOCAL_ACQUIRED,   /* If this bit is set: the device has acquired
					an IPv6 address */

    STATUS_BIT_AUTHENTICATION_FAILED, /* If this bit is set: Authentication with ENT AP failed. */


    STATUS_BIT_RESET_REQUIRED,


    STATUS_BIT_TX_STARED

}e_StatusBits;

#define IS_PING_RUNNING(status_variable)    \
		GET_STATUS_BIT(status_variable, STATUS_BIT_PING_STARTED)
#define IS_IP_ACQUIRED(status_variable)	    \
		GET_STATUS_BIT(status_variable, STATUS_BIT_IP_ACQUIRED)
typedef union
{
    uint8_t		       nwData[MAX_BUF_SIZE];
    int8_t		       textPad[MAX_TEXT_PAD_SIZE];
    SlWlanNetworkEntry_t       netEntries[WLAN_SCAN_COUNT];
}gDataBuffer_t;

typedef struct connectionControlBlock_t
{
    struct k_sem     connectEventSyncObj;
    struct k_sem     ip4acquireEventSyncObj;
    struct k_sem     ip6acquireEventSyncObj;
    struct k_sem     eventCompletedSyncObj;
    uint32_t GatewayIP;
    uint8_t  ConnectionSSID[SSID_LEN_MAX+1];
    uint8_t  ConnectionBSSID[BSSID_LEN_MAX];
    uint32_t DestinationIp;
    uint32_t IpAddr;
    uint32_t StaIp;
    uint32_t Ipv6Addr[4];
}connection_CB;

typedef struct appControlBlock_t
{
    /* Status Variables */
    uint32_t	    Status;		/* This bit-wise status variable shows the state of the NWP */
    uint32_t	    Role;		/* This field keeps the device's role (STA, P2P or AP) */
    uint32_t	    PingAttempts;	/* Sets the number of Ping attempts to send */
    /* Data & Network entry Union */
    gDataBuffer_t     gDataBuffer;

    /* STA/AP mode CB */
    connection_CB    CON_CB;
}appControlBlock;

appControlBlock	   app_CB;

void initAppVariables(void)
{
    app_CB.Status = 0 ;
    app_CB.Role = ROLE_RESERVED;

    memset(&app_CB.gDataBuffer, 0x0, sizeof(app_CB.gDataBuffer));
    memset(&app_CB.CON_CB, 0x0, sizeof(app_CB.CON_CB));

    k_sem_init(&app_CB.CON_CB.connectEventSyncObj,    0, 1);
    k_sem_init(&app_CB.CON_CB.eventCompletedSyncObj,  0, 1);
    k_sem_init(&app_CB.CON_CB.ip4acquireEventSyncObj, 0, 1);
    k_sem_init(&app_CB.CON_CB.ip6acquireEventSyncObj, 0, 1);
}


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
	 app_CB.Role = ROLE_STA;
	 SET_STATUS_BIT(app_CB.Status, STATUS_BIT_NWP_INIT);
     }

     return 0;
}

/*****************************************************************************
		  Callback Functions
*****************************************************************************/

/*!
    \brief	    SimpleLinkWlanEventHandler

    This handler gets called whenever a WLAN event is reported
    by the host driver / NWP. Here user can implement he's own logic
    for any of these events. This handler is used by 'network_terminal'
    application to show case the following scenarios:

    1. Handling connection / Disconnection.
    2. Handling Addition of station / removal.
    3. RX filter match handler.
    4. P2P connection establishment.

    \param	    pWlanEvent	     -	 pointer to Wlan event data.

    \return	    void

    \note	    For more information, please refer to: user.h in the porting
		    folder of the host driver and the  CC3120/CC3220 NWP programmer's
		    guide (SWRU455) sections 4.3.4, 4.4.5 and 4.5.5.

    \sa		    cmdWlanConnectCallback, cmdEnableFilterCallback, cmdWlanDisconnectCallback,
		    cmdP2PModecallback.

*/
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    if(!pWlanEvent)
    {
	return;
    }

    switch(pWlanEvent->Id)
    {
	case SL_WLAN_EVENT_CONNECT:
	{
	    SET_STATUS_BIT(app_CB.Status, STATUS_BIT_CONNECTION);

	    /* Copy new connection SSID and BSSID to global parameters */
	    memcpy(app_CB.CON_CB.ConnectionSSID, pWlanEvent->Data.Connect.SsidName, pWlanEvent->Data.Connect.SsidLen);
	    memcpy(app_CB.CON_CB.ConnectionBSSID, pWlanEvent->Data.Connect.Bssid, BSSID_LEN_MAX);

	    printk("\n\r[WLAN EVENT] STA Connected to the AP: %s , "
		"BSSID: %x:%x:%x:%x:%x:%x\n\r",
		      app_CB.CON_CB.ConnectionSSID, app_CB.CON_CB.ConnectionBSSID[0],
		      app_CB.CON_CB.ConnectionBSSID[1],app_CB.CON_CB.ConnectionBSSID[2],
		      app_CB.CON_CB.ConnectionBSSID[3],app_CB.CON_CB.ConnectionBSSID[4],
		      app_CB.CON_CB.ConnectionBSSID[5]);

	    k_sem_give(&app_CB.CON_CB.connectEventSyncObj);
	}
	break;

	case SL_WLAN_EVENT_DISCONNECT:
	{
	    SlWlanEventDisconnect_t  *pEventData = NULL;

	    CLR_STATUS_BIT(app_CB.Status, STATUS_BIT_CONNECTION);
	    CLR_STATUS_BIT(app_CB.Status, STATUS_BIT_IP_ACQUIRED);
	    CLR_STATUS_BIT(app_CB.Status, STATUS_BIT_IPV6_ACQUIRED);

	    /* If ping operation is running, release it. */
	    if(IS_PING_RUNNING(app_CB.Status))
	    {
		k_sem_give(&app_CB.CON_CB.eventCompletedSyncObj);
		printk("\n\rPing failed, since device is no longer connected.\n\r");
	    }

	    pEventData = &pWlanEvent->Data.Disconnect;

	    /* If the user has initiated 'Disconnect' request,
	      'reason_code' is SL_WLAN_DISCONNECT_USER_INITIATED */
	    if(SL_WLAN_DISCONNECT_USER_INITIATED == pEventData->ReasonCode)
	    {
		printk("\n\r[WLAN EVENT] Device disconnected from the AP: %s,\n\r"
		"BSSID: %x:%x:%x:%x:%x:%x on application's request \n\r",
		  app_CB.CON_CB.ConnectionSSID, app_CB.CON_CB.ConnectionBSSID[0],
		  app_CB.CON_CB.ConnectionBSSID[1],app_CB.CON_CB.ConnectionBSSID[2],
		  app_CB.CON_CB.ConnectionBSSID[3],app_CB.CON_CB.ConnectionBSSID[4],
		  app_CB.CON_CB.ConnectionBSSID[5]);
	    }
	    else
	    {
		printk("\n\r[WLAN ERROR] Device disconnected from the AP: %s,\n\r"
		"BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!! ReasonCode: %d \n\r",
		  app_CB.CON_CB.ConnectionSSID, app_CB.CON_CB.ConnectionBSSID[0],
		  app_CB.CON_CB.ConnectionBSSID[1],app_CB.CON_CB.ConnectionBSSID[2],
		  app_CB.CON_CB.ConnectionBSSID[3],app_CB.CON_CB.ConnectionBSSID[4],
		  app_CB.CON_CB.ConnectionBSSID[5], pEventData->ReasonCode);
	    }
	    memset(&(app_CB.CON_CB.ConnectionSSID), 0x0, sizeof(app_CB.CON_CB.ConnectionSSID));
	    memset(&(app_CB.CON_CB.ConnectionBSSID), 0x0, sizeof(app_CB.CON_CB.ConnectionBSSID));
	}
	break;

	case SL_WLAN_EVENT_PROVISIONING_STATUS:
	{
	    /* Do nothing, this suppress provisioning event is because simplelink is configured to default state. */
	}
	break;

	case SL_WLAN_EVENT_STA_ADDED:
	{
	    memcpy(&(app_CB.CON_CB.ConnectionBSSID), pWlanEvent->Data.STAAdded.Mac, SL_WLAN_BSSID_LENGTH);
	    printk("\n\r[WLAN EVENT] STA was added to AP: BSSID: %x:%x:%x:%x:%x:%x\n\r",
		    app_CB.CON_CB.ConnectionBSSID[0],app_CB.CON_CB.ConnectionBSSID[1],
		    app_CB.CON_CB.ConnectionBSSID[2],app_CB.CON_CB.ConnectionBSSID[3],
		    app_CB.CON_CB.ConnectionBSSID[4],app_CB.CON_CB.ConnectionBSSID[5]);
	}
	break;

	case SL_WLAN_EVENT_STA_REMOVED:
	{
	    memcpy(&(app_CB.CON_CB.ConnectionBSSID), pWlanEvent->Data.STAAdded.Mac, SL_WLAN_BSSID_LENGTH);
	    printk("\n\r[WLAN EVENT] STA was removed from AP: BSSID: %x:%x:%x:%x:%x:%x\n\r",
		    app_CB.CON_CB.ConnectionBSSID[0],app_CB.CON_CB.ConnectionBSSID[1],
		    app_CB.CON_CB.ConnectionBSSID[2],app_CB.CON_CB.ConnectionBSSID[3],
		    app_CB.CON_CB.ConnectionBSSID[4],app_CB.CON_CB.ConnectionBSSID[5]);

	    memset(&(app_CB.CON_CB.ConnectionBSSID), 0x0, sizeof(app_CB.CON_CB.ConnectionBSSID));
	}
	break;

	case SL_WLAN_EVENT_RXFILTER:
	{
	    SlWlanEventRxFilterInfo_t  *triggred_filter = NULL;

	    triggred_filter = &(pWlanEvent->Data.RxFilterInfo) ;

	    printk("\n\r[WLAN EVENT] Rx filter match triggered. Set filters in filter bitmap :0x%x.\n\r", triggred_filter->UserActionIdBitmap);

	    /*
	     *	   User can write he's / her's rx filter match handler here.
	     *	   Be advised, you can use the 'triggred_filter' structure info to determine which filter
	     *	   has received a match. (Bit X is set if user action id X was passed to a filter that matched a packet.)
	     */
	}
	break;

	default:
	{
	    printk("\n\r[WLAN EVENT] Unexpected event [0x%x]\n\r", pWlanEvent->Id);
	}
	break;
    }
}

/*!
    \brief	    SimpleLinkNetAppEventHandler

    This handler gets called whenever a Netapp event is reported
    by the host driver / NWP. Here user can implement he's own logic
    for any of these events. This handler is used by 'network_terminal'
    application to show case the following scenarios:

    1. Handling IPv4 / IPv6 IP address acquisition.
    2. Handling IPv4 / IPv6 IP address Dropping.

    \param	    pNetAppEvent     -	 pointer to Netapp event data.

    \return	    void

    \note	    For more information, please refer to: user.h in the porting
		    folder of the host driver and the  CC3120/CC3220 NWP programmer's
		    guide (SWRU455) section 5.7

*/
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    if(!pNetAppEvent)
    {
	return;
    }

    switch(pNetAppEvent->Id)
    {
	case SL_DEVICE_EVENT_DROPPED_NETAPP_IPACQUIRED:
	{
	    SlIpV4AcquiredAsync_t *pEventData = NULL;

	    SET_STATUS_BIT(app_CB.Status, STATUS_BIT_IP_ACQUIRED);

	    /* Ip Acquired Event Data */
	    pEventData = &pNetAppEvent->Data.IpAcquiredV4;
	    app_CB.CON_CB.IpAddr = pEventData->Ip ;

	    /* Gateway IP address */
	    app_CB.CON_CB.GatewayIP = pEventData->Gateway;

	    printk("\n\r[NETAPP EVENT] IP set to: IPv4=%d.%d.%d.%d , "
		    "Gateway=%d.%d.%d.%d\n\r",

		    SL_IPV4_BYTE(app_CB.CON_CB.IpAddr,3),
		    SL_IPV4_BYTE(app_CB.CON_CB.IpAddr,2),
		    SL_IPV4_BYTE(app_CB.CON_CB.IpAddr,1),
		    SL_IPV4_BYTE(app_CB.CON_CB.IpAddr,0),

		    SL_IPV4_BYTE(app_CB.CON_CB.GatewayIP,3),
		    SL_IPV4_BYTE(app_CB.CON_CB.GatewayIP,2),
		    SL_IPV4_BYTE(app_CB.CON_CB.GatewayIP,1),
		    SL_IPV4_BYTE(app_CB.CON_CB.GatewayIP,0));

	    k_sem_give(&(app_CB.CON_CB.ip4acquireEventSyncObj));
	}
	break;

	case SL_DEVICE_EVENT_DROPPED_NETAPP_IPACQUIRED_V6:
	{
	    uint32_t i = 0;

	    SET_STATUS_BIT(app_CB.Status, STATUS_BIT_IPV6_ACQUIRED);

	    for(i = 0 ; i < 4 ; i++)
	    {
		app_CB.CON_CB.Ipv6Addr[i] = pNetAppEvent->Data.IpAcquiredV6.Ip[i];
	    }

	    printk("\n\r[NETAPP EVENT] IP Acquired: IPv6=");

	    for(i = 0; i < 3 ; i++)
	    {
		printk("%04x:%04x:", ((app_CB.CON_CB.Ipv6Addr[i]>>16) & 0xffff), app_CB.CON_CB.Ipv6Addr[i] & 0xffff);
	    }

	    printk("%04x:%04x", ((app_CB.CON_CB.Ipv6Addr[3]>>16) & 0xffff), app_CB.CON_CB.Ipv6Addr[3] & 0xffff);
	    k_sem_give(&app_CB.CON_CB.ip6acquireEventSyncObj);
	}
	break;

	case SL_DEVICE_EVENT_DROPPED_NETAPP_IP_LEASED:
	{
	    SET_STATUS_BIT(app_CB.Status, STATUS_BIT_IP_LEASED);
	    SET_STATUS_BIT(app_CB.Status, STATUS_BIT_IP_ACQUIRED);

	    app_CB.CON_CB.StaIp = pNetAppEvent->Data.IpLeased.IpAddress;
	    printk("\n\r[NETAPP EVENT] IP Leased to Client: IP=%d.%d.%d.%d \n\r",
			SL_IPV4_BYTE(app_CB.CON_CB.StaIp ,3), SL_IPV4_BYTE(app_CB.CON_CB.StaIp ,2),
			SL_IPV4_BYTE(app_CB.CON_CB.StaIp ,1), SL_IPV4_BYTE(app_CB.CON_CB.StaIp ,0));

	    k_sem_give(&(app_CB.CON_CB.ip4acquireEventSyncObj));
	}
	break;

	case SL_DEVICE_EVENT_DROPPED_NETAPP_IP_RELEASED:
	{
	    printk("\n\r[NETAPP EVENT] IP is released.\n\r");
	}
	break;

	default:
	{
	    printk("\n\r[NETAPP EVENT] Unexpected event [0x%x] \n\r", pNetAppEvent->Id);
	}
	break;
    }
}

void SimpleLinkHttpServerEventHandler(SlNetAppHttpServerEvent_t *pHttpEvent,
				      SlNetAppHttpServerResponse_t *pHttpResponse)
{
    /* Unused in this application */
}


/*!
    \brief	    SimpleLinkGeneralEventHandler

    This handler gets called whenever a general error is reported
    by the NWP / Host driver. Since these errors are not fatal,
    application can handle them.

    \param	    pDevEvent	 -   pointer to device error event.

    \return	    void

    \note	    For more information, please refer to: user.h in the porting
		    folder of the host driver and the  CC3120/CC3220 NWP programmer's
		    guide (SWRU455) section 17.9.

*/
void SimpleLinkGeneralEventHandler(SlDeviceEvent_t *pDevEvent)
{
    if(!pDevEvent)
    {
	return;
    }
    /*
      Most of the general errors are not FATAL are are to be handled
      appropriately by the application
    */
    printk("\n\r[GENERAL EVENT] - ID=[%d] Sender=[%d]\n\n",
	       pDevEvent->Data.Error.Code,
	       pDevEvent->Data.Error.Source);
}

void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    /* Unused in this application */
}

/*!
    \brief	    SimpleLinkFatalErrorEventHandler

    This handler gets called whenever a socket event is reported
    by the NWP / Host driver. After this routine is called, the user's
    application must restart the device in order to recover.
*/
void SimpleLinkFatalErrorEventHandler(SlDeviceFatal_t *slFatalErrorEvent)
{

    switch (slFatalErrorEvent->Id)
    {
	case SL_DEVICE_EVENT_FATAL_DEVICE_ABORT:
	{
	    printk("\n\r[ERROR] - FATAL ERROR: Abort NWP event detected: "
			"AbortType=%d, AbortData=0x%x\n\r",
			slFatalErrorEvent->Data.DeviceAssert.Code,
			slFatalErrorEvent->Data.DeviceAssert.Value);
	}
	break;

	case SL_DEVICE_EVENT_FATAL_DRIVER_ABORT:
	{
	    printk("\n\r[ERROR] - FATAL ERROR: Driver Abort detected. \n\r");
	}
	break;

	case SL_DEVICE_EVENT_FATAL_NO_CMD_ACK:
	{
	    printk("\n\r[ERROR] - FATAL ERROR: No Cmd Ack detected "
			"[cmd opcode = 0x%x] \n\r",
					slFatalErrorEvent->Data.NoCmdAck.Code);
	}
	break;

	case SL_DEVICE_EVENT_FATAL_SYNC_LOSS:
	{
	    printk("\n\r[ERROR] - FATAL ERROR: Sync loss detected n\r");
	}
	break;

	case SL_DEVICE_EVENT_FATAL_CMD_TIMEOUT:
	{
	    printk("\n\r[ERROR] - FATAL ERROR: Async event timeout detected "
			"[event opcode =0x%x]  \n\r",
				    slFatalErrorEvent->Data.CmdTimeout.Code);
	}
	break;

	default:
	    printk("\n\r[ERROR] - FATAL ERROR: Unspecified error detected \n\r");
	break;
    }
}

/*!
    \brief	    SimpleLinkNetAppRequestEventHandler

    This handler gets called whenever a NetApp event is reported
    by the NWP / Host driver. User can write he's logic to handle
    the event here.

*/
void SimpleLinkNetAppRequestEventHandler(SlNetAppRequest_t *pNetAppRequest, SlNetAppResponse_t *pNetAppResponse)
{
    /* Unused in this application */
}

/*!
    \brief	    SimpleLinkNetAppRequestMemFreeEventHandler

    This handler gets called whenever the NWP is done handling with
    the buffer used in a NetApp request. This allows the use of
    dynamic memory with these requests.
*/
void SimpleLinkNetAppRequestMemFreeEventHandler(uint8_t *buffer)
{
    /* Unused in this application */
}



static size_t line2argv(char *str, char *argv[], size_t size)
{
	size_t argc = 0;

	if (!strlen(str)) {
		return 0;
	}

	while (*str && *str == ' ') {
		str++;
	}

	if (!*str) {
		return 0;
	}

	argv[argc++] = str;

	while ((str = strchr(str, ' '))) {
		*str++ = '\0';

		while (*str && *str == ' ') {
			str++;
		}

		if (!*str) {
			break;
		}

		argv[argc++] = str;

		if (argc == size) {
			printk("Too many parameters (max %zu)\n", size - 1);
			return 0;
		}
	}

	/* keep it POSIX style where argv[argc] is required to be NULL */
	argv[argc] = NULL;

	return argc;
}

char SSID_NAME[32];
char SECURITY_KEY[64];

#if 0

#ifdef NET_SSID
#define SSID_NAME	STRINGIFY(NET_SSID)
#else
#error "Pass CFLAGS="-DNET_SSID="<SSID_Name>"" to make"
#endif

#ifdef NET_PASS
#define SECURITY_KEY	STRINGIFY(NET_PASS)
#else
#error "Pass CFLAGS="-DNET_PASS="<password>"" to make"
#endif

#endif

#define SECURITY_TYPE	    SL_WLAN_SEC_TYPE_WPA_WPA2

static long WlanConnect()
{
    SlWlanSecParams_t secParams = {0};
    long lRetVal = 0;

    secParams.Key = (signed char*)SECURITY_KEY;
    secParams.KeyLen = strlen(SECURITY_KEY);
    secParams.Type = SECURITY_TYPE;

    lRetVal = sl_WlanConnect((signed char*)SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);

    printk("Wait for WLAN connect...\n");
    //k_sem_take(&app_CB.CON_CB.connectEventSyncObj, K_FOREVER);
    
	printk("Wait for IP4 address...\n");
    //k_sem_take(&app_CB.CON_CB.ip4acquireEventSyncObj, K_FOREVER);

    return 0;
}

void printBorder(char ch, int n)
{
    int	       i = 0;

    for(i=0; i<n; i++) {
      printk("%c",ch);
    }
    printk("%c",'\n');
}

void printScanResults(uint32_t res_num)
{

    uint32_t	index;
    uint32_t	sub_index;
    uint8_t	ssid_len;
    char WPA_str[]		  = "WPA";
    char WPA2_str[]		  = "WPA2";
    char WEP_str[]		  = "WEP";
    char OPEN_str[]		  = "OPEN";
    char WPAWPA2_str[]		  = "WPA/WPA2";

    /* Print table column headers */
    printBorder('-', 73);
    printk(" No |    SSID    |	   BSSID    | RSSI | Ch |Hidden| Security \n");
    printBorder('-', 73);

    /* Print the table */
    for(index = 0; index < res_num; index++)
    {
	printk(" %-2d ", index+1);

	/* In case the SSID length is 32 characters (the maximum valid size),
	 * the NWP sends the SSID field without NULL terminating character.
	 * In order to avoid printing a string which has no NULL terminated character,
	 * print each character individually.
	 */
	ssid_len = app_CB.gDataBuffer.netEntries[index].SsidLen;

	if (ssid_len < SL_WLAN_SSID_MAX_LENGTH)
	{
	    printk("| %-32s | ", app_CB.gDataBuffer.netEntries[index].Ssid);
	}

	for(sub_index = 0; sub_index < 5 ; sub_index++)
	{
	    printk("%02x:", app_CB.gDataBuffer.netEntries[index].Bssid[sub_index]);
	}

	printk("%02x |", app_CB.gDataBuffer.netEntries[index].Bssid[5]);

	printk(" %-5d |", app_CB.gDataBuffer.netEntries[index].Rssi);

	printk(" %-2d |", app_CB.gDataBuffer.netEntries[index].Channel);

	sub_index = app_CB.gDataBuffer.netEntries[index].SecurityInfo;

	printk(" %s |", SL_WLAN_SCAN_RESULT_HIDDEN_SSID(sub_index)==0?"NO    ":"YES   ");

	if(SL_WLAN_SCAN_RESULT_SEC_TYPE_BITMAP(sub_index) == 0x6)
	{
	    printk(" %-4s |", WPAWPA2_str);
	}
	else
	{
	    printk(" %-4s     |",  SL_WLAN_SCAN_RESULT_SEC_TYPE_BITMAP(sub_index)==0?OPEN_str:SL_WLAN_SCAN_RESULT_SEC_TYPE_BITMAP(sub_index)==4?WPA2_str:
					     SL_WLAN_SCAN_RESULT_SEC_TYPE_BITMAP(sub_index)==2?WPA_str:WEP_str);
	}

	printk("\n");
    }

    printBorder('-', 73);
    printk("\n");

    return;
}

void SimpleLinkPingReport(SlNetAppPingReport_t *pPingReport)
{
    if(IS_PING_RUNNING(app_CB.Status) && (pPingReport->PacketsSent > 0))
    {
	printk("\n");
	printk("[Ping report]: Packet Sent: %d, Packet Received: %d \n\r",
	       pPingReport->PacketsSent , pPingReport->PacketsReceived);
	printk("Max Round Time: %dms , Min Round Time: %dms,"
	       "Average Round Time: %dms\n\r",
	       pPingReport->MaxRoundTime , pPingReport->MinRoundTime,
	       pPingReport->AvgRoundTime);

	/* Release the ping semaphore */
	if(app_CB.PingAttempts == pPingReport->PacketsSent)
	{
	    CLR_STATUS_BIT(app_CB.Status, STATUS_BIT_PING_STARTED);
	    app_CB.PingAttempts = 0;
	    printk("\n");
	    /* This post releases the Ping Callback */
	    k_sem_give(&app_CB.CON_CB.eventCompletedSyncObj);
	}
    }
}

static void on_cmd_connect(size_t argc, char **argv)
{
    if (argc < 3) {
        printk("usage: SSID PSK\n");
        return;
    }
   
	printk("connect to `%s` `%s`\n", argv[1], argv[2]);
    strncpy(SSID_NAME, argv[1], strlen(argv[1]));
    strncpy(SECURITY_KEY, argv[2], strlen(argv[2]));
    
	if(!IS_CONNECTED(app_CB.Status)) {
		WlanConnect();
	}
	else{
		printk("Already connected to %s\n", app_CB.CON_CB.ConnectionSSID);
	}
}

static void on_cmd_disconnect(size_t argc, char **argv)
{
	int ret;

	if(IS_CONNECTED(app_CB.Status)) {
		ret = sl_WlanDisconnect();
		ASSERT_ON_ERROR(ret, WLAN_ERROR);
	}
	else {
		printk("Already disconnected\n");
	}
}

static void on_cmd_scan(size_t argc, char **argv)
{
	int32_t	 ret = -1;
	uint8_t	 trials = 0;

	/* Clear the results buffer */
	memset(&app_CB.gDataBuffer, 0x0, sizeof(app_CB.gDataBuffer));

	/*
	 * Retrieve and print scan results:
	 * Note: If scan policy isn't set, invoking 'sl_WlanGetNetworkList()'
	 * for the first time triggers 'one shot' scan.
	 */

	/* Get scan results from NWP - results will be placed
	 * inside the provided buffer */
	ret = sl_WlanGetNetworkList(0, WLAN_SCAN_COUNT,
				    &app_CB.gDataBuffer.netEntries[0]);

	if (SL_ERROR_WLAN_GET_NETWORK_LIST_EAGAIN == ret) {
	    while(trials < MAX_SCAN_TRIALS) {
		/* Wait for one second for the NWP to complete the
		 * initiated scan and collect results */
		k_sleep(1000);

		/* Collect results form one-shot scans.*/
		ret = sl_WlanGetNetworkList(0, WLAN_SCAN_COUNT,
					    &app_CB.gDataBuffer.netEntries[0]);

		if(ret > 0) {
		  break;
		}
		else {
		  /* If NWP results aren't ready, try 'MAX_SCAN_TRAILS'
		   *  attempts to get results */
		  trials++ ;
		}
	    }
	}

	if(ret <= 0) {
	  printk("\n\r[scan] : Unable to retrieve the network list\n\r");
	  return;
	}

	/* Print the result table */
	printScanResults(ret);
}

void PrintIPAddress(unsigned char ipv6, void *ip)
{
    uint32_t	    *pIPv4;
    uint8_t	    *pIPv6;
    int32_t	     i=0;

    if(!ip) {
	return;
    }

    if(ipv6) {
	pIPv6 = (uint8_t*) ip;

	for(i = 0; i < 14; i+=2)
	{
	    printk("%02x%02x:", pIPv6[i], pIPv6[i+1]);
	}

	printk("%02x%02x", pIPv6[i], pIPv6[i+1]);
    }
    else {
	pIPv4 = (uint32_t*)ip;
	printk("%d.%d.%d.%d", SL_IPV4_BYTE(*pIPv4,3), SL_IPV4_BYTE(*pIPv4,2),
	       SL_IPV4_BYTE(*pIPv4,1), SL_IPV4_BYTE(*pIPv4,0));
    }
    return;
}

typedef struct PingCmd
{
    uint8_t		  *host;	 /* Host name or address */
    SlNetAppPingCommand_t pingCmd;	 /* SimpleLink Ping command structure */
}PingCmd_t;

static void on_cmd_ping(size_t argc, char **argv)
{
    int32_t		      ret = -1;
    SlNetAppPingReport_t      pingReport = {0};
    PingCmd_t		      pingParams;
    int32_t		      ulIpAddr = 0;

    if (argc != 2) {
      printk("Usage: ping <host_name>\n");
      return;
    }
    else {
      pingParams.pingCmd.PingIntervalTime = 1000;
      pingParams.pingCmd.PingSize = 56;
      pingParams.pingCmd.PingRequestTimeout = 3000;
      pingParams.pingCmd.TotalNumberOfAttempts = 10;
      pingParams.pingCmd.Flags = 1; /* Report after each ping */
      pingParams.host = argv[1];
    }

    /* Check if the device has IP and is connected. */
    if((!IS_IP_ACQUIRED(app_CB.Status)) && (!IS_CONNECTED(app_CB.Status)))
    {
	printk("Cannot ping host when IP address isn't leased.\n");
	return;
    }

    /* Clear the Sync object.
     * This semaphore along with the Ping report event, signals
     * The main context if sending ping packets has been completed.
     * For further info, please refer to: SimpleLinkPingReport()
     */
    k_sem_reset(&app_CB.CON_CB.eventCompletedSyncObj);
    SET_STATUS_BIT(app_CB.Status, STATUS_BIT_PING_STARTED);

    /* If the user inserted a name rather than IP address,
     * we get host address via DNS query, assuming Host address is IPV4.
     * In case the user provided an address rather than host name,
     * this API returns the IP address.
     */
    ret = sl_NetAppDnsGetHostByName((signed char*)pingParams.host,
				    strlen((const char*)pingParams.host),
				    (unsigned long*)&ulIpAddr, SL_AF_INET);

    ASSERT_ON_ERROR(ret, NETAPP_ERROR);

    /* Replace the ping address to match host name's IP address */
    pingParams.pingCmd.Ip = ulIpAddr;

    printk("\nPinging  %s (", pingParams.host );
    PrintIPAddress(0,(void *)&ulIpAddr);
    printk(") with %d bytes of data, for %d attempts.\n",
	   pingParams.pingCmd.PingSize,
	   pingParams.pingCmd.TotalNumberOfAttempts);

    /* Switch to network byte ordering*/
    ulIpAddr = sl_Htonl(ulIpAddr);

    /* Update Ping attempts global */
    app_CB.PingAttempts = pingParams.pingCmd.TotalNumberOfAttempts ;

    /* Try to ping HOST_NAME */
    ret = sl_NetAppPing((SlNetAppPingCommand_t*)&pingParams.pingCmd,
			SL_AF_INET,
			(SlNetAppPingReport_t*)&pingReport,
			SimpleLinkPingReport);
    ASSERT_ON_ERROR(ret, NETAPP_ERROR);

    /* Wait for ping to complete - SimpleLinkPingReport would
     * post this semaphore when Ping would complete or in case of error.*/
    k_sem_take(&app_CB.CON_CB.eventCompletedSyncObj, K_FOREVER);

    /* Calling ping API with all zeros, stops any ongoing ping command */
    ret = sl_NetAppPing(0, 0, 0, 0);
    ASSERT_ON_ERROR(ret, NETAPP_ERROR);
}

typedef union
{
    uint32_t    ipv4;       /* Ipv4 Address */
    uint8_t     ipv6[16];   /* Ipv6 Address */
}ip_t;

/* Generated by http://www.lipsum.com/
 * 3 paragraphs, 176 words, 1230 bytes of Lorem Ipsum
 */
static char *lorem_ipsum =
        "Lorem ipsum dolor sit amet, consectetur adipiscing elit. "
        "Vestibulum id cursus felis, sit amet suscipit velit. Integer "
        "facilisis malesuada porta. Nunc at accumsan mauris. Etiam vehicula, "
        "arcu consequat feugiat venenatis, tellus velit gravida ligula, quis "
        "posuere sem leo eget urna. Curabitur condimentum leo nec orci "
        "mattis, nec faucibus dui rutrum. Ut mollis orci in iaculis "
        "consequat. Nulla volutpat nibh eu velit sagittis, a iaculis dui "
        "aliquam."
        "\n"
        "Quisque interdum consequat eros a eleifend. Fusce dapibus nisl "
        "sit amet velit posuere imperdiet. Quisque accumsan tempor massa "
        "sit amet tincidunt. Integer sollicitudin vehicula tristique. Nulla "
        "sagittis massa turpis, ac ultricies neque posuere eu. Nulla et "
        "imperdiet ex. Etiam venenatis sed lacus tincidunt hendrerit. In "
        "libero nisl, congue id tellus vitae, tincidunt tristique mauris. "
        "Nullam sed porta massa. Sed condimentum sem eu convallis euismod. "
        "Suspendisse lobortis purus faucibus, gravida turpis id, mattis "
        "velit. Maecenas eleifend sapien eu tincidunt lobortis. Sed elementum "
        "sapien id enim laoreet consequat."
        "\n"
        "Aenean et neque aliquam, lobortis lectus in, consequat leo. Sed "
        "quis egestas nulla. Quisque ac risus quis elit mollis finibus. "
        "Phasellus efficitur imperdiet metus."
        "\n";

typedef union{
    SlSockAddrIn6_t     in6;   /* Socket info for Ipv6 */
    SlSockAddrIn_t      in4;   /* Socket info for Ipv4 */
}sockAddr_t;

static void on_cmd_udpserver(size_t argc, char **argv)
{
    uint16_t portNumber;
    uint32_t numberOfPackets;
    uint8_t nb = TRUE;
    uint8_t tx = FALSE;
    int32_t         sock;
    int32_t         status;
    uint32_t        i = 0;
    int32_t         nonBlocking;
    int32_t         buflen;
    SlSockAddr_t    *sa;  /* Contains the local ip address and port */
    SlSockAddr_t    *csa; /* Contains the ip address and port of the peer. */
    sockAddr_t      sAddr;
    int32_t         addrSize;

    if (argc != 4) {
      printk("Usage: udpserver <port_num> <num_packets> <tx: 0|1>\n");
      return;
    }
    else {
      portNumber = atoi(argv[1]);
      numberOfPackets = atoi(argv[2]);
      tx = atoi(argv[3]);
    }

    /* clear the global data buffer */
    memset(&app_CB.gDataBuffer.nwData, 0x0 , sizeof(app_CB.gDataBuffer.nwData));

    /* fill the buffer with data */
    buflen = strlen(lorem_ipsum);
    for (i = 0 ; i < buflen ; i++)
    {
           app_CB.gDataBuffer.nwData[i] = (char)lorem_ipsum[i];
    }

    /* filling the addresses and ports for the server's side.
     * Note: Since this is the server's side, we do not require to
     * fill the IP address field of the SlSockAddr_t structure */
    sAddr.in4.sin_family = SL_AF_INET;
    sAddr.in4.sin_port = sl_Htons((uint16_t)portNumber);
    sAddr.in4.sin_addr.s_addr = SL_INADDR_ANY;

    sa = (SlSockAddr_t*)&sAddr.in4;
    csa = (SlSockAddr_t*)&sAddr.in4;
    addrSize = sizeof(SlSockAddrIn_t);

    /* Get UDP sock descriptor - This call opens the socket. */
    sock = sl_Socket(sa->sa_family, SL_SOCK_DGRAM, 0);
    ASSERT_ON_ERROR(sock, SL_SOCKET_ERROR);

    /* Bind socket to port -
     * Since this is the server's side, we bind the socket descriptor we just
     * Opened, to a specific port, on which we'll get traffic from. */
    status = sl_Bind(sock, sa, addrSize);

    if(status < 0)
    {
        printk("[line:%d, error:%d] %s\n", __LINE__, status, SL_SOCKET_ERROR);
        sl_Close(sock);
        return;
    }
    /* Set socket as non-blocking socket (if needed):
     * Non-blocking sockets allows user to handle other tasks rather than block
     * on socket API calls. If an API call using the Non-blocking socket descriptor
     * returns 'SL_ERROR_BSD_EAGAIN' - this indicate that the user should try the API again later.
     */
    if(TRUE == nb){
        nonBlocking = TRUE;
        status = sl_SetSockOpt(sock, SL_SOL_SOCKET, SL_SO_NONBLOCKING,
			       &nonBlocking, sizeof(nonBlocking));
        if(status < 0) {
            printk("[line:%d, error:%d] %s\n", __LINE__, status,
		       SL_SOCKET_ERROR);
            sl_Close(sock);
            return;
        }
    }

    i = 0;

    if(tx) {
        ((SlSockAddrIn_t*)csa)->sin_addr.s_addr =
	  sl_Htonl(SL_IPV4_VAL(10,0,1,37));
        while(i < numberOfPackets) {
            while(TRUE) {
                /*  Calling 'sl_SendTo' until we send all the packets
                 *  we requested to send.
                 */
                status = sl_SendTo(sock, &app_CB.gDataBuffer.nwData, buflen,
				   0, csa, addrSize);

                if((status == SL_ERROR_BSD_EAGAIN) && (TRUE == nb)) {
                    k_sleep(1000);
                    continue;
                }
                else if(status < 0) {
                    printk("[line:%d, error:%d] %s\n", __LINE__, status,
			       SL_SOCKET_ERROR);
                    sl_Close(sock);
                    return;
                }
                break;
            }
            printk("  Packet: %d, bytes: %d\n", i, status);
            i++;
        }
        printk("Sent %u packets successfully\n",numberOfPackets);
    }
    else
    {
        while(i < numberOfPackets) {
            /* If user doesn't wish to transmit data, sl_recvFrom is invoked
	     * in order to receive UDP datagrams here.
             */
            status = sl_RecvFrom(sock, &app_CB.gDataBuffer.nwData, buflen, 0,
				 csa, (SlSocklen_t*)&addrSize);
            if((status == SL_ERROR_BSD_EAGAIN) && (TRUE == nb))  {
                k_sleep(1000);
                continue;
            }
            else if(status < 0) {
                printk("[line:%d, error:%d] %s\n", __LINE__, status,
			   SL_SOCKET_ERROR);
                sl_Close(sock);
                return;
            }
            printk("  Packet: %d, bytes: %d\n", i, status);
            i++;
        }

        printk("Received %u packets successfully\n", numberOfPackets);
    }

    /* Calling 'close' with the socket descriptor,
     * once operation is finished.
     */
    status = sl_Close(sock);
    ASSERT_ON_ERROR(status, SL_SOCKET_ERROR);

    return;
}

static void on_cmd_http_test(int argc, char** argv) {

}

#define CMD(c) { \
	.cmd = #c, \
	.cmd_len = sizeof(#c) - 1, \
	.func = on_cmd_ ## c \
}

static const struct {
  const char *cmd;
  size_t cmd_len;
  void (*func)(size_t argc, char **argv);
} commands[] = {
    CMD(connect),
    CMD(disconnect),
    CMD(scan),
    CMD(ping),
    CMD(udpserver),
//    CMD(http_test),
};

void main(void)
{
	int done = false;
	char *cmdline;
	int i;
	char *argv[ARGC_MAX + 1];
	size_t argc;
	int32_t	 retval;

	console_getline_init();

	/* Init SimpleLink: */
	MAP_PRCMLPDSWakeupSourceEnable(PRCM_LPDS_HOST_IRQ);
	SPI_init();
	initAppVariables();
	retval = ConfigureSimpleLinkToDefaultState();

	while (!done) {
		printk("cc3220sf wifi> ");
		cmdline = console_getline();
		argc = line2argv(cmdline, argv, ARRAY_SIZE(argv));
		if (!argc) {
		  continue;
		}

		for (i = 0; i < ARRAY_SIZE(commands); i++) {
		  if (!strncmp(argv[0], commands[i].cmd, commands[i].cmd_len)) {
		    printk("Executing: %s\n", (char *)cmdline);
		    commands[i].func(argc, argv);
		  }
		}
	}
}
