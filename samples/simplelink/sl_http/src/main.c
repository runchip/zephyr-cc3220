#include "http_client.h"

void hexdump(void* ptr, int size) {
	int i;
	const char* c = (char*) ptr;
	for (i = 0; i < size; i++) {
		printf("%02X%s", c[i], (i+1) % 8 == 0 ? "\n" : " ");
	}
}

static char request_buffer[4096];
static char response_buffer[4096];


void http_post_test()
{
    http_client client;
    http_message request, response;

    http_message_init(&request);
    http_message_init(&response);

    http_client_init(&client, &request);
    http_client_set_request_buffer(&client, buffer, sizeof(buffer));
    http_client_set_response_buffer(&client, buffer, sizeof(buffer));

    http_message_set_url(&request, "http://emdroid.org:9090/");
    http_message_set_method(&request, HTTP_POST);
    http_message_add_parameter(&request, "temp", "23");

    http_client_execute(&client, &response);

    printf("==== response header ====\n");
    for (int i = 0; i < response.nheaders; i++) {
        static char key[64], val[64];
        ss_strcopy(key, sizeof(key), response.header_names[i]);
        ss_strcopy(val, sizeof(val), response.header_values[i]);
        printf("%s: %s\n", key, val);
    }

    static char body[4096];
    ss_strcopy(body, sizeof(body), response.body);
    printf("==== response body ====\n%s", body);
    printf("=======================\n");
}


#ifdef CONFIG_HAS_CC3220SDK
extern void wlan_init(void);
#else  // CONFIG_HAS_CC3220SDK
static void wlan_init(void) {}
#endif // CONFIG_HAS_CC3220SDK

int main(int argc, char *argv[])
{
	wlan_init();

	http_post_test();

	printf("done\n");

	return 0;
}



