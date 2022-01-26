#ifndef HTTP_MAIN_H
#define HTTP_MAIN_H

#include <string.h>
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
// #include "nvs_flash.h"
// #include "protocol_examples_common.h"

#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include "lwip/netdb.h"
#include "lwip/dns.h"

/* Constants that aren't configurable in menuconfig */
#define WEB_SERVER "doanphd.online"
#define WEB_PORT "80"

// #include "sdkconfig.h"
// #include "FreeRTOSConfig.h"
// #include "portmacro.h"


// void HTTP_init();
void HTTP_send_data(int DATA_NUM_MOVE, int DATA_INTENSITY_MOVE, int DATA_HEARTRATE);

#endif