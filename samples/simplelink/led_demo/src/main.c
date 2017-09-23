#ifdef CONFIG_HAS_CC3220SDK
#include <kernel.h>
#include <misc/printk.h>
#include <board.h>
#include <device.h>
#include <gpio.h>
#include <ti/drivers/net/wifi/simplelink.h>
#include <ti/drivers/net/wifi/bsd/sys/socket.h>
#include <ti/drivers/net/wifi/bsd/netinet/in.h>
#include <ti/drivers/net/wifi/bsd/arpa/inet.h>
#define printf printk
#define UART_PRINT printk
#define perror(str) printk("perror %s +%d: %s\n", __FILE__, __LINE__, str)
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
#include <pthread.h>

#endif // CONFIG_HAS_CC3220SDK

#define PORT 9090
#define LISTEN_BACK 5
#define MAX_ACTIVE_CONNECTIONS 5

char BUFFER[4096];
int active_connections = 0;

typedef struct client_info {
	int connfd;
	socklen_t addrlen;
	struct sockaddr_in addr;
} client_info_t;

client_info_t clients[MAX_ACTIVE_CONNECTIONS] = {0};

void hexdump(void* ptr, int size) {
	int i;
	const char* c = (char*) ptr;
	for (i = 0; i < size; i++) {
		printf("%02X%c", c[i], (i+1) % 8 == 0 ? '\n' : ' ');
	}
}

#ifdef CONFIG_HAS_CC3220SDK
void wlan_init_ap(void);
#else  // CONFIG_HAS_CC3220SDK
void wlan_init_ap(void) {}
#endif // CONFIG_HAS_CC3220SDK

int listen_on_port(uint16_t port, int backlogs)
{
    int sock = socket(AF_INET, SOCK_STREAM, 0);
    if (sock < 0) {
        perror("socket");
    }

	struct sockaddr_in server_addr;
	memset(&server_addr, 0, sizeof(server_addr));
	server_addr.sin_family = AF_INET;
	server_addr.sin_addr.s_addr = htonl(INADDR_ANY);
	server_addr.sin_port = htons(port);

	int rc = bind(sock, (const struct sockaddr*)&server_addr, sizeof(server_addr));
	if (rc < 0) {
		perror("bind");
	}

	rc = listen(sock, backlogs);
	if (rc < 0) {
		perror("listen");
	}
	printf("listen on port: %d...\n", (int)port);

	return sock;
}


int accept_new_connection(int sock, struct sockaddr *client_addr, socklen_t* socklen)
{
	int connfd = accept(sock, client_addr, socklen);
	if (connfd < 0) {
		perror("accept");
		return connfd;
	}
	active_connections++;
	return connfd;
}


typedef struct header {
	int length;
	int magic;
} __attribute__((packed)) header_t;

#define MAGIC 0x1a2b3c4d

#define MSG_CONTENT(h) (((char*)h) + sizeof(header_t))
#define MSG_LENGTH(h)  (((header_t*)h)->length)
#define MSG_MAGIC(h)   (((header_t*)h)->magic)
#define IS_VALID(h)    (((header_t*)h)->magic == MAGIC)

int pack_message(char* buffer, int buflen, char* content, int contlen)
{
	int msglen = contlen + sizeof(header_t);
	if (msglen > buflen) {
		printf("ERROR: buffer not large enough!\n");
		return 0;
	}
	MSG_LENGTH(buffer) = contlen;
	MSG_MAGIC(buffer) = MAGIC;
	strncpy(MSG_CONTENT(buffer), content, contlen);
	return msglen;
}

int recv_message(int sock, char* buffer, int buflen)
{
	int ret, count = 0;
	while ((ret = recv(sock, &buffer[count], buflen - count, 0)) > 0) {
		printf("send %d bytes\n", send(sock, &buffer[count], ret, 0));
		count += ret;
		if (count >= MSG_LENGTH(buffer)) {
			break;
		}
	}
	if (ret < 0) {
		perror("recv");
	}
	buffer[count] = '\0';
	printf("receviced: ");
	hexdump(buffer, count);
	printf("length: %08X, magic: %08X\n", MSG_LENGTH(buffer), MSG_MAGIC(buffer));
	if (!IS_VALID(buffer)) {
		printf("ERROR: magic is %08X\n", MSG_MAGIC(buffer));
		return -1;
	}
	return count;
}

#ifdef __ZEPHYR__

/* Change this if you have an LED connected to a custom port */
#define LED_PORT	LED0_GPIO_PORT

/* Change this if you have an LED connected to a custom pin */
#define LED0_PIN	LED0_GPIO_PIN
#define LED1_PIN    2 /* GPIO_10 */
#define LED2_PIN    1 /* GPIO_9 */


#define NLEDS 3
volatile char frequencies[NLEDS] = {0};

const int LED_PINS[] = { LED0_PIN, LED1_PIN, LED2_PIN };

#define LED_THREAD_STACK_SIZE 512
K_THREAD_STACK_DEFINE(led_thread_stack, LED_THREAD_STACK_SIZE);
struct k_thread led_thread;

void led_thread_work(void* arg1, void* arg2, void* arg3)
{
	struct device* dev = (struct device*)arg1;
	while (1) {
		for (int i = 0; i < NLEDS; i++) {
			if (0 == frequencies[i]) {
				gpio_pin_write(dev, LED_PINS[i], 1); k_sleep(200);
			} else {
				int period = 1000/2 / (int)frequencies[i];
				gpio_pin_write(dev, LED_PINS[i], 1); k_sleep(period);
				gpio_pin_write(dev, LED_PINS[i], 0); k_sleep(period);
			}
		}
	}
}

void start_led_thread()
{
	struct device *dev;

	dev = device_get_binding(LED_PORT);
	/* Set LED pin as output */
	gpio_pin_configure(dev, LED0_PIN, GPIO_DIR_OUT);
	gpio_pin_configure(dev, LED1_PIN, GPIO_DIR_OUT);
	gpio_pin_configure(dev, LED2_PIN, GPIO_DIR_OUT);

	k_thread_create(&led_thread, led_thread_stack, LED_THREAD_STACK_SIZE,
					led_thread_work, dev, NULL, NULL,
					0, 0, 0);
}

void process_message(const char* msg, int msglen)
{
	if (msglen >= NLEDS) memcpy(frequencies, msg, NLEDS);
}

#define WORKER_THREAD_STACK_SIZE 1024
K_THREAD_STACK_DEFINE(worker_thread_stack, WORKER_THREAD_STACK_SIZE);
struct k_thread worker_threads[MAX_ACTIVE_CONNECTIONS];

void connection_thread_work(void* arg1, void* arg2, void* arg3)
#else  // __ZEPHYR__

void process_message(const char* msg, int msglen) {}

pthread_t worker_threads[MAX_ACTIVE_CONNECTIONS];
void* connection_thread_work(void* arg1)
#endif // __ZEPHYR__
{
	char* txt = NULL;
	int connfd;
	client_info_t* info = (client_info_t*) arg1;

	connfd = info->connfd;
	
	recv_message(connfd, BUFFER, sizeof(BUFFER));

	txt = MSG_CONTENT(BUFFER);
	printf("content: %s\n", txt);

	process_message(MSG_CONTENT(BUFFER), MSG_LENGTH(BUFFER));

	close(connfd); // close connection

	info->addrlen = 0; // mark this client slot as inactive
#ifndef __ZEPHYR__
	return NULL;
#endif
}

void start_worker_thread(int id)
{
#ifdef __ZEPHYR__
	k_thread_create(&worker_threads[id], worker_thread_stack, WORKER_THREAD_STACK_SIZE,
					connection_thread_work, &clients[id], NULL, NULL,
					0, 0, 0);
#else  // __ZEPHYR__
	pthread_create(&worker_threads[id], NULL, connection_thread_work, &clients[id]);
#endif // __ZEPHYR__
}

int main(int argc, char *argv[])
{
	int sock;

	wlan_init_ap();

	sock = listen_on_port(PORT, LISTEN_BACK);

#ifdef __ZEPHYR__
	start_led_thread();
#endif

	while (sock >= 0) {
		int i;
		for (i = 0; i < MAX_ACTIVE_CONNECTIONS; i++) {
			if (0 == clients[i].addrlen) { // find a inactive client slot
				break;
			}
		}
		if (MAX_ACTIVE_CONNECTIONS == i) {
			printf("concurrent connections %d archive max value...\n", active_connections);
			sleep(1);
			continue;
		}

		printf("wait for new connection...\n");
		clients[i].connfd = accept_new_connection(sock, (struct sockaddr*)&clients[i].addr, &clients[i].addrlen);

		start_worker_thread(i);
		// sleep(1);
	}

	printf("done\n");

	return 0;
}


