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
#define sleep(s) k_sleep((s) * 1000)
#else // CONFIG_HAS_CC3220SDK

#include <arpa/inet.h>
#include <netdb.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>

#include <math.h>
#include <stdio.h>
#include <string.h>

#endif // CONFIG_HAS_CC3220SDK

#define HOST "emdroid.org"
#define PORT 8080

const char* CRLF = "\r\n";
const char* HTTP_EOH = "\r\n\r\n"; // End of Header
const char* CONTENT_LENGTH = "Content-Length:";
char RESPONSE[4096];
char REQUEST[4096];

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


#ifdef CONFIG_HAS_CC3220SDK
void wlan_init(void);
#else  // CONFIG_HAS_CC3220SDK
void wlan_init(void) {}
#endif // CONFIG_HAS_CC3220SDK

int connect_to_server(const char* name, uint16_t port)
{
    int ret, host;
    struct sockaddr_in server_addr;

    int fd = socket(AF_INET, SOCK_STREAM, 0);
    if (fd < 0) {
        perror("socket");
    }

    host = resolve_host_addr(name);

    memset(&server_addr, 0, sizeof(server_addr));
    server_addr.sin_family = AF_INET;
    server_addr.sin_port = htons(port);
    server_addr.sin_addr.s_addr = host;

    ret = connect(fd, (const struct sockaddr*)&server_addr,
                  sizeof(server_addr));
    if (ret < 0) {
        perror("connect");
    }
    return fd;
}

int send_request(int sock, const char* request, int reqlen)
{
	int ret = 0;
	printf("send request %d...\n", reqlen);
	ret = send(sock, request, reqlen, 0);
	if (ret < 0) {
		perror("send");
	}
	printf("%d bytes sent\n", ret);
	return ret;
}

int recv_response(int sock, char* buffer, int buflen)
{
	char *p;
	int EOH, CL;
	int ret, count, content_length;

	CL = EOH = 0;
	count = 0;
	content_length = 0;
	while ((ret = recv(sock, &buffer[count], buflen - count, 0)) > 0) {
		printf("received %d bytes...\n", ret);
		//hexdump(&buffer[count], ret);
		count += ret;
		if ((p = strstr(buffer, HTTP_EOH)) != NULL) {
			EOH = p - buffer;
			printf("EOH at %d\n", EOH);
		}
		if ((p = strstr(buffer, CONTENT_LENGTH)) != NULL) {
			CL = p - buffer;
			content_length = atoi(p + strlen(CONTENT_LENGTH));
			printf("CL at %d, value: %d\n", CL, content_length);
		}
		if (CL && count - EOH - strlen(HTTP_EOH) >= content_length) {
			break;
		}
	}
	if (ret < 0) {
		perror("recv");
	}
	buffer[count] = '\0';
	return count;
}

int prepare_request(char* buffer, int buflen, const char* host, int port, const char* path, int temp)
{
	char line[128];

	sprintf(buffer, "POST %s HTTP/1.1\r\n", path);
	sprintf(line, "Host: %s:%d\r\n", host, port);
	strcat(buffer, line);
	strcat(buffer, "User-Agent: CC3220\r\n"
					"Accept: */*\r\n"
	               "Content-Type: application/x-www-form-urlencoded\r\n");
	sprintf(line, "Content-Length: %d\r\n", sprintf(line, "temp=%d", temp));
	strcat(buffer, line);
	strcat(buffer, "\r\n");
	sprintf(line, "temp=%d", temp);
	strcat(buffer, line);

	return strlen(buffer);
}

int main(int argc, char *argv[])
{
	int sock;

	wlan_init();

	printf("connect to server...\n");
	sock = connect_to_server(HOST, PORT);

	for (int i = 0; i < 10; i++) {
		int reqlen, temperature;

		temperature = 15 + i; // 20 + 5*sin(M_PI*2/10*i);

		reqlen = prepare_request(REQUEST, sizeof(REQUEST), HOST, PORT, "/temp", temperature);

		printf("send request...\n");
		send_request(sock, REQUEST, reqlen);

		printf("recv response...\n");
		recv_response(sock, RESPONSE, sizeof(RESPONSE));
		printf("RESPONSE: %s\n", RESPONSE);

		sleep(1);
	}

	close(sock);

	printf("done\n");

	return 0;
}


