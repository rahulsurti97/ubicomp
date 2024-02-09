#include "config.h"

#include "AdafruitIO_WiFi.h"

#define IO_USERNAME "rahulsurti97"
#define IO_KEY "aio_xhMi29FhzqGfvc6NxmxjiRdS5Xlx"
AdafruitIO_WiFi io(IO_USERNAME, IO_KEY, WIFI_SSID, WIFI_PASSWORD);

AdafruitIO_Feed *_adafruitIoFeed= io.feed("stepcount");
