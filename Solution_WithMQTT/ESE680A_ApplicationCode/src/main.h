/**
 * \file
 *
*/

#ifndef MAIN_H_INCLUDED
#define MAIN_H_INCLUDED

#ifdef __cplusplus
extern "C" {
#endif
/* MQTT Settings */
/* Max size of UART buffer. */
#define MAIN_CHAT_BUFFER_SIZE 64

/* Max size of MQTT buffer. */
#define MAIN_MQTT_BUFFER_SIZE 128

/* Limitation of user name. */
#define MAIN_CHAT_USER_NAME_SIZE 64

/* Limitation of password. */
#define MAIN_CHAT_PASSWORD_SIZE 64

/* Chat MQTT topic. */
#define MAIN_CHAT_TOPIC "chat"
#define SENSOR_TOPIC    "sensor"
#define ACTUATOR_TOPIC  "actuator"
#define VERSION_TOPIC	"version"
#define VERSIONREADOUT_TOPIC "versionreadout"
#define UPGRADE_TOPIC	"upgrade"
#define MQTT_PORT       18312
/*
 * A MQTT broker server which was connected.
 * m2m.eclipse.org is public MQTT broker.
 */
static const char main_mqtt_broker[] = "m13.cloudmqtt.com";

/** Wi-Fi AP Settings. */
#define MAIN_WLAN_SSID                       "hp-setup-1" /**< Destination SSID */
#define MAIN_WLAN_AUTH                       M2M_WIFI_SEC_WPA_PSK /**< Security manner */
#define MAIN_WLAN_PSK                        "WelcomeHal" /**< Password for Destination SSID */

/** IP address parsing. */
#define IPV4_BYTE(val, index)                ((val >> (index * 8)) & 0xFF)

/** Content URI for download. */
#define MAIN_HTTP_FILE_URL                   "https://www.seas.upenn.edu/~nichins/button_led2.bin" 
#define MAIN_HTTP_CRC_URL					"https://www.seas.upenn.edu/~nichins/button_led2_crc.bin"

/** Maximum size for packet buffer. */
#define MAIN_BUFFER_MAX_SIZE                 (1024)
/** Maximum file name length. */
#define MAIN_MAX_FILE_NAME_LENGTH            (250)
/** Maximum file extension length. */
#define MAIN_MAX_FILE_EXT_LENGTH             (8)
/** Output format with '0'. */
#define MAIN_ZERO_FMT(SZ)                    (SZ == 4) ? "%04d" : (SZ == 3) ? "%03d" : (SZ == 2) ? "%02d" : "%d"

typedef enum {
	NOT_READY = 0, /*!< Not ready. */
	STORAGE_READY = 0x01, /*!< Storage is ready. ~NOT used for flash IC*/
	WIFI_CONNECTED = 0x02, /*!< Wi-Fi is connected. */
	GET_REQUESTED = 0x04, /*!< GET request is sent. */
	DOWNLOADING = 0x08, /*!< Running to download. */
	COMPLETED = 0x10, /*!< Download completed. */
	CANCELED = 0x20 /*!< Download canceled. */
} download_state;

#ifdef __cplusplus
}
#endif

#endif /* MAIN_H_INCLUDED */
