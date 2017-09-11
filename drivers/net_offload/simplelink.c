/*
 * Copyright (c) 2017 Texas Instruments
 *
 * SPDX-License-Identifier: Apache-2.0
 */

#if defined(CONFIG_SIMPLELINK_DEBUG)
#define SYS_LOG_DOMAIN "simplelink"
#define SYS_LOG_LEVEL SYS_LOG_LEVEL_DEBUG
#define NET_LOG_ENABLED 1
#include <logging/sys_log.h>
#include <stdio.h>
#endif

#include <zephyr.h>

#include <stdbool.h>
#include <errno.h>
#include <stddef.h>
#include <net/buf.h>
#include <net/nbuf.h>
#include <net/net_if.h>
#include <net/net_core.h>
#include <net/dhcpv4.h>
#include <net/ethernet.h>
#include <net/net_mgmt.h>
#include <net/net_offload.h>

#include <simplelink.h>

/* TBD: These taken from simplelink samples, to be better integrated here: */
#include <example/common/common.h>
#include <example/common/network_if.h>
#include <example/common/uart_if.h>
#include <example/common/udma_if.h>

/* This should be defined by simplelink.h somewhere, but it isn't. */
#define SIMPLELINK_MAX_TCP_IPV4_DATA_SIZE    1460
/* Max L2 packet size, per SimpleLink User's Guide */
#define SIMPLELINK_MTU	1472


/* TBD: Move these to the global device driver data, see if protection needed:
 * SimpleLink Async Callbacks do not carry a client token,
 * so we assume there is only one instance of the WiFi device.
 */
unsigned char  volatile g_status = 0;
unsigned long  volatile g_gatewayIP = 0;
unsigned char  u_connectionSSID[SSID_LEN_MAX+1]; //Connection SSID
unsigned char  u_connectionBSSID[BSSID_LEN_MAX]; //Connection BSSID

SlFdSet_t read_fds;

struct simplelink_dev_data {
	struct net_if *iface;
	bool init_done;
	struct in_addr dns;   /* Store here, since net_if has no dns field */
};

static struct simplelink_dev_data simplelink_dev_data =
{
	NULL, false,
};

//*****************************************************************************
//
//! \brief This function handles network events such as IP acquisition, IP
//!	      leased, IP released etc.
//!
//! \param[in]	pNetAppEvent - Pointer to NetApp Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkNetAppEventHandler(SlNetAppEvent_t *pNetAppEvent)
{
    switch(pNetAppEvent->Event)
    {
	/* This can also be called later when lease expires, and new IP reassigned. */
	case SL_NETAPP_IPV4_IPACQUIRED_EVENT:
	{
	    SlIpV4AcquiredAsync_t *pEventData = NULL;
	    struct net_if *iface;
	    uint32_t in_addr;
	    uint32_t lease_time = 0;
	    unsigned long tmp;
	    unsigned long subnet_mask;

	    SET_STATUS_BIT(g_status, STATUS_BIT_IP_AQUIRED);

	    //Ip Acquired Event Data
	    pEventData = &pNetAppEvent->EventData.ipAcquiredV4;

	    //Gateway IP address
	    g_gatewayIP = pEventData->gateway;

	    NET_INFO("[NETAPP EVENT] IP Acquired: \n\r\tIP=%ld.%ld.%ld.%ld, "
	    "Gateway=%ld.%ld.%ld.%ld, DNS =%ld.%ld.%ld.%ld",
	    SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,3),
	    SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,2),
	    SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,1),
	    SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.ip,0),
	    SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,3),
	    SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,2),
	    SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,1),
	    SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.gateway,0),
	    SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.dns,3),
	    SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.dns,2),
	    SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.dns,1),
	    SL_IPV4_BYTE(pNetAppEvent->EventData.ipAcquiredV4.dns,0));

	    NET_ASSERT(simplelink_dev_data.iface != NULL);
	    iface = simplelink_dev_data.iface;

	    iface->ipv4.gw.s_addr[0] = ntohl(pNetAppEvent->EventData.ipAcquiredV4.gateway);
	    simplelink_dev_data.dns.s_addr[0] = ntohl(pNetAppEvent->EventData.ipAcquiredV4.dns);

	    /* Get subnet mask, as that is not sent along in this event: */
	    Network_IF_IpConfigGet(&tmp, &subnet_mask, &tmp, &tmp);
	    iface->ipv4.netmask.s_addr[0] = ntohl(subnet_mask);

	    /* 
	     * Add our IPV4 IP address to the interface, and
	     * send a NET_EVENT_IPV4_ADDR_ADD event, to notify DHCP clients
	     * Note: There is no SimpleLink API to get lease time in STATION mode.
	     */
	    in_addr = ntohl(pNetAppEvent->EventData.ipAcquiredV4.ip);
	    if(!net_if_ipv4_addr_add(iface, (struct in_addr *)&in_addr,
				     NET_ADDR_DHCP, lease_time)) {
		    NET_DBG("Failed to add IPv4 addr to iface %p", iface);
	    }
	    /* Now, the interface is up, and IP address acquired: */
	    net_mgmt_event_notify(NET_EVENT_IF_UP, iface);
	}
	break;

	default:
	{
	    NET_INFO("[NETAPP EVENT] Unexpected event [0x%x]",
		   (unsigned int)pNetAppEvent->Event);
	}
	break;
    }
}

//*****************************************************************************
//
//! \brief The Function Handles WLAN Events
//!
//! \param[in]	pWlanEvent - Pointer to WLAN Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkWlanEventHandler(SlWlanEvent_t *pWlanEvent)
{
    switch(pWlanEvent->Event)
    {
	case SL_WLAN_CONNECT_EVENT:
	{
	    SET_STATUS_BIT(g_status, STATUS_BIT_CONNECTION);

	    //
	    // Information about the connected AP (like name, MAC etc) will be
	    // available in 'slWlanConnectAsyncResponse_t'-Applications
	    // can use it if required
	    //
	    //	slWlanConnectAsyncResponse_t *pEventData = NULL;
	    // pEventData = &pWlanEvent->EventData.STAandP2PModeWlanConnected;
	    //

	    // Copy new connection SSID and BSSID to global parameters
	    memcpy(u_connectionSSID, pWlanEvent->EventData.
		   STAandP2PModeWlanConnected.ssid_name,
		   pWlanEvent->EventData.STAandP2PModeWlanConnected.ssid_len);
	    memcpy(u_connectionBSSID,
		   pWlanEvent->EventData.STAandP2PModeWlanConnected.bssid,
		   SL_BSSID_LENGTH);

	    NET_INFO("[WLAN EVENT] STA Connected to the AP: %s ,"
			"BSSID: %x:%x:%x:%x:%x:%x",
		      u_connectionSSID,u_connectionBSSID[0],
		      u_connectionBSSID[1],u_connectionBSSID[2],
		      u_connectionBSSID[3],u_connectionBSSID[4],
		      u_connectionBSSID[5]);
	}
	break;

	case SL_WLAN_DISCONNECT_EVENT:
	{
	    slWlanConnectAsyncResponse_t*  pEventData = NULL;

	    CLR_STATUS_BIT(g_status, STATUS_BIT_CONNECTION);
	    CLR_STATUS_BIT(g_status, STATUS_BIT_IP_AQUIRED);

	    pEventData = &pWlanEvent->EventData.STAandP2PModeDisconnected;

	    // If the user has initiated 'Disconnect' request,
	    //'reason_code' is SL_WLAN_DISCONNECT_USER_INITIATED_DISCONNECTION
	    if(SL_WLAN_DISCONNECT_USER_INITIATED_DISCONNECTION == pEventData->reason_code)
	    {
		NET_INFO("[WLAN EVENT]Device disconnected from the AP: %s,"
		"BSSID: %x:%x:%x:%x:%x:%x on application's request",
			   u_connectionSSID,u_connectionBSSID[0],
			   u_connectionBSSID[1],u_connectionBSSID[2],
			   u_connectionBSSID[3],u_connectionBSSID[4],
			   u_connectionBSSID[5]);
	    }
	    else
	    {
		NET_INFO("[WLAN ERROR]Device disconnected from the AP AP: %s,"
		"BSSID: %x:%x:%x:%x:%x:%x on an ERROR..!!",
			   u_connectionSSID,u_connectionBSSID[0],
			   u_connectionBSSID[1],u_connectionBSSID[2],
			   u_connectionBSSID[3],u_connectionBSSID[4],
			   u_connectionBSSID[5]);
	    }
	    memset(u_connectionSSID,0,sizeof(u_connectionSSID));
	    memset(u_connectionBSSID,0,sizeof(u_connectionBSSID));
	}
	break;

	default:
	{
	    NET_INFO("[WLAN EVENT] Unexpected event [0x%x]",
		   (unsigned int)pWlanEvent->Event);
	}
	break;
    }
}

//*****************************************************************************
//
//! This function handles socket events indication
//!
//! \param[in]	    pSock - Pointer to Socket Event Info
//!
//! \return None
//!
//*****************************************************************************
void SimpleLinkSockEventHandler(SlSockEvent_t *pSock)
{
    //
    // This application doesn't work w/ socket - Events are not expected
    //
    switch( pSock->Event )
    {
	case SL_SOCKET_TX_FAILED_EVENT:
	    switch( pSock->socketAsyncEvent.SockTxFailData.status)
	    {
		case SL_ECLOSE:
		    NET_INFO("[SOCK ERROR] - close socket (%d) operation "
				"failed to transmit all queued packets\n\n",
				    pSock->socketAsyncEvent.SockTxFailData.sd);
		    break;
		default:
		    NET_INFO("[SOCK ERROR] - TX FAILED	:  socket %d , reason "
				"(%d) \n\n",
				pSock->socketAsyncEvent.SockTxFailData.sd, pSock->socketAsyncEvent.SockTxFailData.status);
		  break;
	    }
	    break;

	default:
		NET_INFO("[SOCK EVENT] - Unexpected Event [%lx0x]\n\n",pSock->Event);
	  break;
    }

}

//****************************************************************************
//
//! \brief Connecting to a WLAN Accesspoint
//!
//!  This function connects to the required AP (SSID_NAME) with Security
//!  parameters specified in te form of macros at the top of this file
//!
//! \param  None
//!
//! \return  None
//!
//! \warning	If the WLAN connection fails or we don't aquire an IP
//!	       address, It will be stuck in this function forever.
//
//****************************************************************************
static long WlanConnect()
{
    SlSecParams_t secParams = {0};
    long lRetVal = 0;

    secParams.Key = (signed char*)SECURITY_KEY;
    secParams.KeyLen = strlen(SECURITY_KEY);
    secParams.Type = SECURITY_TYPE;

    lRetVal = sl_WlanConnect((signed char*)SSID_NAME, strlen(SSID_NAME), 0, &secParams, 0);
    ASSERT_ON_ERROR(lRetVal);

    /* TBD: use a semaphore here: */
    while((!IS_CONNECTED(g_status)) || (!IS_IP_ACQUIRED(g_status)))
    {
	    k_sleep(1);
    }

    return SUCCESS;
}

static int simplelink_dev_init(struct device *dev)
{
	struct simplelink_dev_data *data = dev->driver_data;
	long retval;

	SYS_LOG_DBG("[%p] dev %p", data, dev);

	/* Initialize the uDMA for TCP/UDP data transfer: */
	UDMAInit();

	/*
	   Start up the SimpleLink NWP (NetWork Processor),
	   Set the mode to STATION
	   Configure connection policy to Auto and AutoSmartConfig
	   Delete all the stored profiles
	   Enable DHCP
	   Disable Scan policy
	   Set Tx power to maximum
	   Set power policy to normal
	   Unregister mDNS services
	   Remove all filters
	*/
	retval = Network_IF_InitDriver(ROLE_STA);

	return (int)retval;
}

/*
 * Socket (fd) to Callback map, for use by simplelink_recv() and
 * simplelink_server()
 */
struct sl_callbacks {
	/* Callback to be triggered when socket receives data */
	net_context_recv_cb_t cb;

	/* Net context associated with the socket */
	struct net_context *context;

	/* user data to pass back to the callback */
	void *user_data;
};

static struct sl_callbacks sl_callback[SL_MAX_SOCKETS] = { 0, };

/* Tell if it's time to regenerate the FD_SET array, and re-invoke select: */
bool sl_gen_new_fds = false;

void sl_callback_register(short int sd, net_context_recv_cb_t cb,
			  struct net_context *context, void *user_data)
{
	NET_ASSERT(sd < SL_MAX_SOCKETS);
	sl_callback[sd].cb = cb;
	sl_callback[sd].user_data = user_data;
	sl_callback[sd].context = context;
	SL_FD_SET(sd, &read_fds);
	sl_gen_new_fds = true;
}

void sl_callback_deregister(short int sd)
{
	NET_ASSERT(sd < SL_MAX_SOCKETS);
	sl_callback[sd].cb = 0;
	sl_callback[sd].user_data = 0;
	sl_callback[sd].context = 0;
	SL_FD_CLR(sd, &read_fds);
	sl_gen_new_fds = true;
}

void sl_callback_fire(short int sd, struct net_buf *buf, int status)
{
	net_context_recv_cb_t cb;
	struct net_context *context;
	void *user_data;

	NET_ASSERT(sd < SL_MAX_SOCKETS);
	cb = sl_callback[sd].cb;
	context = sl_callback[sd].context;
	user_data = sl_callback[sd].user_data;

	cb(context, buf, status, user_data);
}

struct net_context *sl_callback_context(short int sd)
{
	NET_ASSERT(sd < SL_MAX_SOCKETS);
	return sl_callback[sd].context;
}


#define STACKSIZE 1048
char __noinit __stack sl_server_thread_stack[STACKSIZE];

unsigned char recv_buf[SIMPLELINK_MTU];

/*
 * Call sl_Select() in a loop, waiting for registered file descriptors
 * to get signalled; if so, call their registered recv callbacks
 */
void sl_server(void)
{
	SlFdSet_t active_fds;
	SlTimeval_t timeout;
	short int size;
	short int sock = 0;  /* TBD: get from FD_SET */
	struct net_buf *recv_net_buf;
	int status;

	timeout.tv_sec = 10;
	timeout.tv_usec = 0;

	SL_FD_ZERO(&read_fds);
	active_fds = read_fds;

	while(1) {
		/*
		 * TBD: With timeout > 0 this is racy: may miss a UDP packet while
		 * sitting in sl_Select with stale fds.
		 * Burn one socket, use that for signalling new fds, and then wait forever.
		 */
		status = sl_Select(sock + 1, &active_fds, NULL, NULL, &timeout);
		if (status > 0 && SL_FD_ISSET(sock, &active_fds)) {
			/* Retreive the buffer recieved on this socket: */
			size = sl_Recv(sock, recv_buf, SIMPLELINK_MTU, 0);
			if( size <= 0 ) {
				sl_Close(sock);
			}

			/* Allocate a network buffer, and copy data: */
			recv_net_buf = net_nbuf_get_rx(sl_callback_context(sock));
			net_nbuf_append(recv_net_buf, size, recv_buf);
			net_nbuf_set_appdata(recv_net_buf, (uint8_t *)recv_buf);
			net_nbuf_set_appdatalen(recv_net_buf, (uint16_t)size);

			/* Fire the app callback, which unref's the recv_net_buf: */
			sl_callback_fire(sock, recv_net_buf, size);
		}
		/* This will pickup a change in the fd set: */
		if(sl_gen_new_fds) {
			active_fds = read_fds;
			sl_gen_new_fds = false;
		}
	}
}

/* SimpleLink Offload functions: similar to net_context APIs */
int simplelink_get(sa_family_t family,
		   enum net_sock_type type,
		   enum net_ip_protocol ip_proto,
		   struct net_context **context)
{
	short int sock;
	short int sock_type;

	/* Context allocated by client: */
	NET_ASSERT(*context);
	NET_ASSERT(type == SOCK_DGRAM || type == SOCK_STREAM);

	/* WARNING: Zephyr SOCK_DGRAM, SOCK_STREAM are swapped from POSIX values! */
	/* TBD: fix Zephyr to be POSIX compliant. */
	/* Map Zephyr to POSIX values: */
	sock_type = (type == SOCK_DGRAM? SL_SOCK_DGRAM : SL_SOCK_STREAM);

	/* Allocate socket and store in passed net_context struct: */
	sock = sl_Socket(family, sock_type, ip_proto);
	if (sock >= 0) {
		(*context)->conn_handler = (struct net_conn_handle *)(int)sock;
	}
	NET_INFO("sock(family:%d, sock_type:%d, ip_proto:%d): %d",
		 family, sock_type, ip_proto, sock);
	return(sock);
}

int simplelink_bind(struct net_context *context,
	 const struct sockaddr *addr,
	 socklen_t addrlen)
{
	short int sock;
	short int retval;
	SlSockAddrIn_t local_addr;
	struct sockaddr_in *in_addr = (struct sockaddr_in *)addr;

	NET_ASSERT(context);

	sock = (short int)(int)context->conn_handler;
	local_addr.sin_family = in_addr->sin_family;
	local_addr.sin_port = in_addr->sin_port;
	/* SimpleLink sl_Bind seems to require local_addr to be 0!? */
	/* Also, we are forced to use S_un.S_addr field, as Zephyr defines s_addr
	   which SimpleLink also defines */
	local_addr.sin_addr.S_un.S_addr = 0;
	//local_addr.sin_addr.S_un.S_addr = in_addr->sin_addr.s_addr[0];

	retval = sl_Bind(sock, (SlSockAddr_t *)&local_addr, sizeof(SlSockAddrIn_t));

	NET_INFO("bind(%d): %d", sock, retval);
	return retval;
}


int simplelink_listen(struct net_context *context, int backlog)
{
	return 0;
}

int simplelink_connect(struct net_context *context,
		       const struct sockaddr *addr,
		       socklen_t addrlen,
		       net_context_connect_cb_t cb,
		       int32_t timeout,
		       void *user_data)
{
	return 0;
}


int simplelink_accept(struct net_context *context,
	   net_tcp_accept_cb_t cb,
	   int32_t timeout,
	   void *user_data)
{
	return 0;
}


int simplelink_send(struct net_buf *buf,
	    net_context_send_cb_t cb,
	    int32_t timeout,
	    void *token,
	    void *user_data)
{
	return 0;
}

int simplelink_sendto(struct net_buf *send_buf,
	      const struct sockaddr *dst_addr,
	      socklen_t addrlen,
	      net_context_send_cb_t cb,
	      int32_t timeout,
	      void *token,
	      void *user_data)
{
	short int sock;
	short int retval;
	short int buf_len;
	struct net_context *context;
	SlSockAddrIn_t	dest_addr;
	struct sockaddr_in *in_addr = net_sin(dst_addr);
	void *data;
	struct net_buf *buf = send_buf;

	NET_ASSERT(buf);
	context = net_nbuf_context(buf);
	NET_ASSERT(context);
	/* Turns out, addrlen is not used for SimpleLink */
	//NET_ASSERT(addrlen == sizeof(SlSockAddrIn_t));   // This assert fails

	sock = (short int)(int)context->conn_handler;

	buf = buf->frags;
	buf_len = buf->len;

	/* Verify data len within range: 1-1460 bytes allowed by SimpleLink */
	if (buf_len < 1 || buf_len > SIMPLELINK_MAX_TCP_IPV4_DATA_SIZE) {
		retval = (-1);
		goto done;
	}

	data = (void *)buf->data;

	dest_addr.sin_family = in_addr->sin_family;
	dest_addr.sin_port = in_addr->sin_port;
	dest_addr.sin_addr.S_un.S_addr = in_addr->sin_addr.s_addr[0];

	/* TBD: Handle timeout, and involve the SimpleLink Socket Handler for errors */
	retval = sl_SendTo(sock, data, buf_len, 0, (SlSockAddr_t *)&dest_addr,
			   sizeof(SlSockAddrIn_t));

	if (retval >= 0) {
		net_nbuf_unref(send_buf);
	}

 done:
	/* Fire user's callback, if any. */
	if (cb) {
		cb(context, (retval >= 0 ? 0 : retval),
		   UINT_TO_POINTER((uint32_t)retval), user_data);
	}

	NET_INFO("sendto(%d): %d", sock, retval);

	return retval;
}

/* This function must work in modes, per net_context_recv() definition:
   See net_context.h
   Mode 1: Timeout = 0:	  NOTE: Most used case (and most complex):
	 - Register callback with context (socket), and return;
	 - Later, when data received on this socket, callback will be called.
   Mode 2: Timeout > 0 && !K_FOREVER:  Only used with CONFIG_NET_CONTEXT_SYNC_RECV.
	 - Wait for buffer per timeout;
	 - Don't call callback if timed out; call callback otherwise
   Mode 3: Timeout == K_FOREVER:
	 - Wait for the network buffer;
	 - Call callback when received.
   Mode 4: CONFIG_NET_CONTEXT_SYNC_RECV:  Not used much yet, but need to
	  #ifdef the timeout handling code to signal a semaphore.
	  Note: only samples/net/zperf uses this, otherwise, timeout ignored.
*/
int simplelink_recv(struct net_context *context,
	    net_context_recv_cb_t cb,
	    int32_t timeout,
	    void *user_data)
{
	short int sock;
	int status;
	long non_blocking = 1;

	NET_ASSERT(context);
	/* TBD: handle other net_context_recv() modes: */
	NET_ASSERT(timeout == K_NO_WAIT);

	sock = (short int)(int)context->conn_handler;

	status = sl_SetSockOpt(sock, SL_SOL_SOCKET, SL_SO_NONBLOCKING,
			       &non_blocking, sizeof(non_blocking));
	if (status >= 0) {
		/* Register callback and user_data with context (socket) */
		sl_callback_register(sock, cb, context, user_data);
	}
	return status;
}

int simplelink_put(struct net_context *context)
{
	short int sock;

	NET_ASSERT(context);

	sock = (short int)(int)context->conn_handler;

	sl_callback_deregister(sock);

	return 0;
}

static struct net_offload simplelink_offload_fxns = {
	.get = simplelink_get,
	.put = simplelink_put,
	.bind = simplelink_bind,
	.listen = simplelink_listen,
	.connect = simplelink_connect,
	.accept = simplelink_accept,
	.send = simplelink_send,
	.sendto = simplelink_sendto,
	.recv = simplelink_recv,
};

static void simplelink_iface_init(struct net_if *iface)
{
	struct simplelink_dev_data *data = net_if_get_device(iface)->driver_data;
	long lRetVal = 0;

	/*
	 *  We assume that net_buf data size can accomodate a SimpleLink data packet
	 *  without having to traverse net_buf fragments (at least on receive).
	*/
	NET_ASSERT(CONFIG_NET_NBUF_DATA_SIZE == SIMPLELINK_MAX_TCP_IPV4_DATA_SIZE);

	/* Declare the net_context APIs being offloaded to the SimpleLink NWP */
	iface->offload_ip = &simplelink_offload_fxns;

	/* Save for the SimpleLink Event Handlers, which allow no user tokens */
	data->iface = iface;

	/* Connect to our AP:  The Wlan Handler will signal Zephyr IPv4 added */
	/*
	 * TBD: All this Wlan connect, and DHCP will be moved to new connection
	 * management APIs, and we can remove SimpleLink DHCP once Zephyr DHCP working.
	 */
	lRetVal = WlanConnect();
	if (lRetVal >= 0) {
		NET_INFO("Connection established w/ AP and IP is aquired");
		data->init_done = true;
	}
	else {
		NET_ASSERT_INFO(false, "We should not be here");
		data->init_done = false;
	}

	/* Start the SimpleLink Socket receive server: */
	k_thread_spawn(&sl_server_thread_stack[0], STACKSIZE,
		       (k_thread_entry_t)sl_server,
		       NULL, NULL, NULL, K_PRIO_COOP(7), 0, 0);

}

static int simplelink_iface_send(struct net_if *iface, struct net_buf *buf)
{
	NET_ASSERT_INFO(false, "We should not be here");

	return 0;
}


static struct net_if_api simplelink_api = {
	/* Mandatory net_if apis */
	.init = simplelink_iface_init,
	.send = simplelink_iface_send,
};

/* TBD: NULL is for the cfg_info argument:  what is this used for? */
/* Note: must be after osi_zephyr_init priority */
NET_DEVICE_INIT(simplelink, CONFIG_SIMPLELINK_DRV_NAME,
		simplelink_dev_init, &simplelink_dev_data,
		NULL, CONFIG_SIMPLELINK_INIT_PRIO, &simplelink_api,
		OFFLOAD_IP_L2, OFFLOAD_IP_L2_CTX_TYPE, SIMPLELINK_MTU);
