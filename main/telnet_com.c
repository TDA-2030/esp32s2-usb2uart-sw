/* BSD Socket API Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <string.h>
#include <sys/param.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "esp_system.h"
#include "esp_wifi.h"
#include "esp_event.h"
#include "esp_log.h"
#include "nvs_flash.h"
#include "esp_netif.h"
#include "esp_heap_caps.h"
#include "driver/gpio.h"
#include "hal/gpio_ll.h"
#include "lwip/err.h"
#include "lwip/sockets.h"
#include "lwip/sys.h"
#include <lwip/netdb.h>
#include "serial.h"
#include "io.h"
#include "driver/uart.h"


#define PORT                        4000
#define KEEPALIVE_IDLE              5
#define KEEPALIVE_INTERVAL          5
#define KEEPALIVE_COUNT             3
#define CONFIG_EXAMPLE_IPV4 1

static const char *TAG = "telnet";

// telnet protocol characters
#define SE 0xf0    // Subnegotiation End
#define NOP 0xf1   // No Operation
#define DM 0xf2    // Data Mark
#define BRK 0xf3   // Break
#define IP 0xf4    // Interrupt process
#define AO 0xf5    // Abort output
#define AYT 0xf6   // Are You There
#define EC 0xf7    // Erase Character
#define EL 0xf8    // Erase Line
#define GA 0xf9    // Go Ahead
#define SB 0xfa    // Subnegotiation Begin
#define WILL 0xfb
#define WONT 0xfc
#define DO 0xfd
#define DONT 0xfe
#define IAC 0xff   // Interpret As Command
#define IAC_DOUBLED 0xffff

// selected telnet options
#define BINARY 0x00    // 8-bit data path
#define ECHO 0x01      // echo
#define SGA 0x03       // suppress go ahead

// RFC2217
#define COM_PORT_OPTION 44 // COM port options

#define SET_BAUDRATE 0x01
#define SET_DATASIZE 0x02
#define SET_PARITY 0x03
#define SET_STOPSIZE 0x04
#define SET_CONTROL 0x05
#define NOTIFY_LINESTATE 0x06
#define NOTIFY_MODEMSTATE 0x07
#define FLOWCONTROL_SUSPEND 0x08
#define FLOWCONTROL_RESUME 0x09
#define SET_LINESTATE_MASK 0x0a
#define SET_MODEMSTATE_MASK 0x0b
#define PURGE_DATA 0x0c

#define SERVER_SET_BAUDRATE 0x65
#define SERVER_SET_DATASIZE 0x66
#define SERVER_SET_PARITY 0x67
#define SERVER_SET_STOPSIZE 0x68
#define SERVER_SET_CONTROL 0x69
#define SERVER_NOTIFY_LINESTATE 0x6a
#define SERVER_NOTIFY_MODEMSTATE 0x6b
#define SERVER_FLOWCONTROL_SUSPEND 0x6c
#define SERVER_FLOWCONTROL_RESUME 0x6d
#define SERVER_SET_LINESTATE_MASK 0x6e
#define SERVER_SET_MODEMSTATE_MASK 0x6f
#define SERVER_PURGE_DATA 0x70

#define PURGE_RECEIVE_BUFFER 0x01      // Purge access server receive data buffer
#define PURGE_TRANSMIT_BUFFER 0x02     // Purge access server transmit data buffer
#define PURGE_BOTH_BUFFERS 0x03        // Purge both the access server receive data
// buffer and the access server transmit data buffer

enum {
    DATASIZE_GET = 0,
    DATASIZE_5BIT = 5,
    DATASIZE_6BIT,
    DATASIZE_7BIT,
    DATASIZE_8BIT,
};

enum {
    PARITY_GET = 0,
    PARITY_NONE,
    PARITY_ODD,
    PARITY_EVEN,
    PARITY_MARK,
    PARITY_SPACE,
};

enum {
    STOPSIZE_GET = 0,
    STOPSIZE_1BIT,
    STOPSIZE_2BIT,
    STOPSIZE_1_5BIT,
};

enum {
    CONTROL_GET = 0,
    CONTROL_NONE, //Use No Flow Control (outbound/both)
    CONTROL_USE_XON_XOFF, //Use XON/XOFF Flow Control (outbound/both)
    CONTROL_USE_HARDWARE_FLOW, //Use HARDWARE Flow Control (outbound/both)
    CONTROL_BREAK_REQ,  // request current BREAK state
    CONTROL_BREAK_ON,   // set BREAK (TX-line to LOW)
    CONTROL_BREAK_OFF,  // Set BREAK State OFF
    CONTROL_DTR_REQ, // Request DTR Signal State
    CONTROL_DTR_ON,   // Set DTR Signal State ON
    CONTROL_DTR_OFF,  // Set DTR Signal State OFF
    CONTROL_RTS_REQ, // Request RTS Signal State
    CONTROL_RTS_ON,  // Set RTS Signal State ON
    CONTROL_RTS_OFF,  // Set RTS Signal State OFF
};

// telnet state machine states
typedef enum {
    TN_STATE_NORMAL,
    TN_STATE_IAC,
    // TN_STATE_WILL,
    TN_STATE_START,
    TN_STATE_END,
    TN_STATE_NEGOTIATE,
    TN_STATE_COMPORT,
    TN_STATE_SETCONTROL,
    TN_STATE_SETBAUD,
    TN_STATE_SETDATASIZE,
    TN_STATE_SETPARITY,
    TN_STATE_SETSTOPSIZE,
    TN_STATE_PURGEDATA
} telnet_state_t;

typedef struct {
    int listen_socket;
    int client_socket;
    telnet_state_t state;
    uint8_t command;
    uint16_t txbufferlen;   // length of data in txbuffer
    uint8_t *rx_buffer;
    size_t rx_buffer_len;
} telnet_data_t;

static char tn_baudCnt;
static uint32_t tn_baud;     // shared across all sockets, thus possible race condition
static uint8_t tn_break = 0; // 0=BREAK-OFF, 1=BREAK-ON
static telnet_data_t *g_telnet_data = NULL;
static uint32_t in_mcu_flashing;

static void send_data(int sock, const uint8_t *data, size_t length)
{
    int to_write = length;
    while (to_write > 0) {
        int written = send(sock, data + (length - to_write), to_write, 0);
        if (written < 0) {
            ESP_LOGE(TAG, "Error occurred during sending: errno %d", errno);
        }
        to_write -= written;
    }
}

esp_err_t telnet_send_date(const uint8_t *data, size_t length)
{
    // printf("urev %d : ", length);
    // for (size_t i = 0; i < length; i++) {
    //     printf("%x, ", data[i]);
    // } printf("\n");

    send_data(g_telnet_data->client_socket, data, length);
    return ESP_OK;
}

static void uart_write_char(uint8_t c)
{
    uint8_t *buf = &c;
    size_t rx_size = 1;

    const int transferred = uart_write_bytes(UART_NUM_1, buf, rx_size);
    if (transferred != rx_size) {
        ESP_LOGW(TAG, "uart_write_bytes transferred %d bytes only!", transferred);
    }
    // esp_err_t err = uart_wait_tx_done(UART_NUM_1, pdMS_TO_TICKS(1000));
}

static void telnet_send_subnegotiation(const uint8_t *option, size_t option_len, const uint8_t *value, size_t value_len)
{
    // value = value.replace(IAC, IAC_DOUBLED)
    if (option_len + value_len + 5 > 64) {
        ESP_LOGE(TAG, "option too long");
        return;
    }

    uint8_t buf[64] = {IAC, SB, COM_PORT_OPTION };
    uint8_t *p = buf + 3;
    memcpy(p, option, option_len);
    p += option_len;
    memcpy(p, value, value_len);
    p += value_len;
    *p++ = IAC;
    *p++ = SE;
    send_data(g_telnet_data->client_socket, buf, p - buf);
}

static void telnet_parse(telnet_data_t *telnet, uint8_t *inBuf, int len)
{
    telnet_state_t state = telnet->state;

    for (int i = 0; i < len; i++) {
        uint8_t c = inBuf[i];
        switch (state) {
        case TN_STATE_NORMAL:
            if (c == IAC) {
                state = TN_STATE_IAC;
            } else {
                uart_write_char(c); // regular char
            }
            break;
        case TN_STATE_IAC:
            switch (c) {
            case IAC: // second IAC -> write one to outbuf and go normal again
                state = TN_STATE_NORMAL;
                uart_write_char(c);
                break;
            case SB: // command sequence begin
                state = TN_STATE_START;
                break;
            case SE: // command sequence end
                state = TN_STATE_NORMAL;
                break;
            case DO:
            case DONT:
            case WILL:
            case WONT:
                state = TN_STATE_NEGOTIATE;
                telnet->command = c;
                break;
            default:
                ESP_LOGW(TAG, "unsupported command [%d], drop it", c);
                state = TN_STATE_END;
                break;
            }
            break;

        case TN_STATE_NEGOTIATE: {
            // client announcing it will send telnet cmds, try to respond
            uint8_t respBuf[3] = {IAC, DONT, c};
            ESP_LOGI(TAG, "NEGOTIATE cmd=%x, op=%x", telnet->command, c);

            switch (telnet->command) {
            case WILL:
                respBuf[1] = DO;
                break;
            case DO:
                respBuf[1] = WILL;
                break;
            case WONT:
                respBuf[1] = DONT;
                break;

            default:
                break;
            }

            // if (c == COM_PORT_OPTION || c == SGA) {
            //     respBuf[1] = DO;
            // } else {
            //     ESP_LOGI(TAG, "rejecting WILL %d", c);
            // }
            send_data(telnet->client_socket, respBuf, 3);
            state = TN_STATE_NORMAL; // go back to normal
            break;
        }
        case TN_STATE_START: // in command seq, now comes the type of cmd
            if (c == COM_PORT_OPTION) {
                state = TN_STATE_COMPORT;
            } else {
                state = TN_STATE_END;    // an option we don't know, skip 'til the end seq
            }
            break;
        case TN_STATE_END: // wait for end seq
            if (c == IAC) {
                state = TN_STATE_IAC;    // simple wait to accept end or next escape seq
            }
            break;
        case TN_STATE_COMPORT:
            switch (c) {
            case SET_CONTROL:
                state = TN_STATE_SETCONTROL;
                break;
            case SET_DATASIZE:
                state = TN_STATE_SETDATASIZE;
                break;
            case SET_PARITY:
                state = TN_STATE_SETPARITY;
                break;
            case SET_STOPSIZE:
                state = TN_STATE_SETSTOPSIZE;
                break;
            case SET_BAUDRATE:
                state = TN_STATE_SETBAUD;
                tn_baudCnt = 0;
                tn_baud = 0;
                break;
            case PURGE_DATA:
                state = TN_STATE_PURGEDATA;
                break;
            default:
                state = TN_STATE_END;
                break;
            }
            break;
        case TN_STATE_PURGEDATA: {// purge FIFO-buffers
            switch (c) {
            case PURGE_RECEIVE_BUFFER:
                // TODO: flush TX buffer
                break;
            }
            uint8_t respBuf[7] = {IAC, SB, COM_PORT_OPTION, SERVER_PURGE_DATA, c, IAC, SE};
            send_data(telnet->client_socket, respBuf, 7);
            state = TN_STATE_END;
        } break;
        case TN_STATE_SETCONTROL: { // switch control line
            static bool dtr = 1, rts = 1;
            switch (c) {
            case CONTROL_DTR_ON:
                dtr = 0;
                break;
            case CONTROL_DTR_OFF:
                dtr = 1;
                break;
            case CONTROL_RTS_ON:
                rts = 0;
                break;
            case CONTROL_RTS_OFF:
                rts = 1;
                break;
            case CONTROL_BREAK_REQ:
                ESP_LOGI(TAG, "BREAK state requested: state = %d)", tn_break);
                break;
            case CONTROL_BREAK_ON:
                tn_break = 1;
                ESP_LOGI(TAG, "BREAK ON: set TX to LOW");
                break;
            case CONTROL_BREAK_OFF:
                if (tn_break == 1) {
                    // GPIO_OUTPUT_SET(1, 1);
                    // PIN_FUNC_SELECT(PERIPHS_IO_MUX_U0TXD_U, FUNC_U0TXD);
                    tn_break = 0;
                    ESP_LOGI(TAG, "BREAK OFF: set TX to HIGH");
                }
                break;
            case CONTROL_NONE:
                ESP_LOGI(TAG, "CONTROL_NONE");
                break;
            default:
                ESP_LOGW(TAG, "unsupported control [%d], drop it", c);
                break;
            }

            if (CONTROL_NONE != c) {
                bool rst = rts;
                bool boot = dtr;
                gpio_set_level(GPIO_BOOT, boot);
                gpio_set_level(GPIO_RST, rst);
                ESP_LOGI(TAG, "RESET:%s, BOOT:%s", boot ? "H" : "L", rst ? "H" : "L");
            }

            uint8_t respBuf[7] = {IAC, SB, COM_PORT_OPTION, SERVER_SET_CONTROL, c, IAC, SE};
            send_data(telnet->client_socket, respBuf, 7);
            state = TN_STATE_END;
        } break;
        case TN_STATE_SETDATASIZE: {
            if (c != DATASIZE_GET) {
                ESP_LOGI(TAG, "%d bits/char", c);
            } else {
                c = DATASIZE_8BIT;
            }
            uint8_t respBuf[7] = {IAC, SB, COM_PORT_OPTION, SERVER_SET_DATASIZE, c, IAC, SE  };
            send_data(telnet->client_socket, respBuf, 7);
            state = TN_STATE_END;
        } break;
        case TN_STATE_SETSTOPSIZE: {
            if (c != STOPSIZE_GET) {
                ESP_LOGI(TAG, "stopsize %d ", c);
            } else {
                c = STOPSIZE_1BIT;
            }
            uint8_t respBuf[7] = {IAC, SB, COM_PORT_OPTION, SERVER_SET_STOPSIZE, c, IAC, SE  };
            send_data(telnet->client_socket, respBuf, 7);
            state = TN_STATE_END;
        } break;
        case TN_STATE_SETBAUD: {
            tn_baud |= ((uint32_t)c) << (24 - 8 * tn_baudCnt);
            tn_baudCnt++;
            uint32_t b = 115200;
            if (tn_baudCnt == 4) {// we got all four baud rate bytes (big endian)
                if (tn_baud == 0) {// baud rate of zero means we need to send the baud rate
                    uart_get_baudrate(UART_NUM_1, &b);
                    ESP_LOGI(TAG, "get baud %d", b);
                } else {
                    ESP_LOGI(TAG, "%d baud", tn_baud);
                    serial_set_baudrate(tn_baud);
                    b = tn_baud;
                }
                uint8_t respBuf[10] = {IAC, SB, COM_PORT_OPTION, SERVER_SET_BAUDRATE, b >> 24, b >> 16, b >> 8, b, IAC, SE};
                send_data(telnet->client_socket, respBuf, 10);
                state = TN_STATE_END;
            }
        } break;
        case TN_STATE_SETPARITY: {
            if (c != PARITY_GET) {
                ESP_LOGI(TAG, "parity %s", c == PARITY_ODD ? "odd" : c == PARITY_EVEN ? "even" : "none");
            } else {
                c = PARITY_NONE;
            }
            uint8_t respBuf[7] = {IAC, SB, COM_PORT_OPTION, SERVER_SET_PARITY, c, IAC, SE};
            send_data(telnet->client_socket, respBuf, 7);
            state = TN_STATE_END;
        } break;

        default: break;
        }
    }
    telnet->state = state;
}


static void telnet_com_task(void *pvParameters)
{
    char addr_str[128];
    int addr_family = (int)pvParameters;
    int ip_protocol = 0;
    int keepAlive = 1;
    int keepIdle = KEEPALIVE_IDLE;
    int keepInterval = KEEPALIVE_INTERVAL;
    int keepCount = KEEPALIVE_COUNT;
    struct sockaddr_storage dest_addr;

    g_telnet_data = heap_caps_calloc(1, sizeof(telnet_data_t), MALLOC_CAP_8BIT | MALLOC_CAP_INTERNAL);
    if (NULL == g_telnet_data) {
        ESP_LOGE(TAG, "no memory for telnet");
        vTaskDelete(NULL);
    }

    g_telnet_data->state = TN_STATE_NORMAL;

    if (addr_family == AF_INET) {
        struct sockaddr_in *dest_addr_ip4 = (struct sockaddr_in *)&dest_addr;
        dest_addr_ip4->sin_addr.s_addr = htonl(INADDR_ANY);
        dest_addr_ip4->sin_family = AF_INET;
        dest_addr_ip4->sin_port = htons(PORT);
        ip_protocol = IPPROTO_IP;
    }
#ifdef CONFIG_EXAMPLE_IPV6
    else if (addr_family == AF_INET6) {
        struct sockaddr_in6 *dest_addr_ip6 = (struct sockaddr_in6 *)&dest_addr;
        bzero(&dest_addr_ip6->sin6_addr.un, sizeof(dest_addr_ip6->sin6_addr.un));
        dest_addr_ip6->sin6_family = AF_INET6;
        dest_addr_ip6->sin6_port = htons(PORT);
        ip_protocol = IPPROTO_IPV6;
    }
#endif

    g_telnet_data->listen_socket = socket(addr_family, SOCK_STREAM, ip_protocol);
    if (g_telnet_data->listen_socket < 0) {
        ESP_LOGE(TAG, "Unable to create socket: errno %d", errno);
        vTaskDelete(NULL);
        return;
    }
    int opt = 1;
    setsockopt(g_telnet_data->listen_socket, SOL_SOCKET, SO_REUSEADDR, &opt, sizeof(opt));
#if defined(CONFIG_EXAMPLE_IPV4) && defined(CONFIG_EXAMPLE_IPV6)
    // Note that by default IPV6 binds to both protocols, it is must be disabled
    // if both protocols used at the same time (used in CI)
    setsockopt(g_telnet_data->listen_socket, IPPROTO_IPV6, IPV6_V6ONLY, &opt, sizeof(opt));
#endif

    ESP_LOGI(TAG, "Socket created");

    int err = bind(g_telnet_data->listen_socket, (struct sockaddr *)&dest_addr, sizeof(dest_addr));
    if (err != 0) {
        ESP_LOGE(TAG, "Socket unable to bind: errno %d", errno);
        ESP_LOGE(TAG, "IPPROTO: %d", addr_family);
        goto CLEAN_UP;
    }
    ESP_LOGI(TAG, "Socket bound, port %d", PORT);

    err = listen(g_telnet_data->listen_socket, 1);
    if (err != 0) {
        ESP_LOGE(TAG, "Error occurred during listen: errno %d", errno);
        goto CLEAN_UP;
    }

    g_telnet_data->rx_buffer_len = 4096;
    g_telnet_data->rx_buffer = heap_caps_malloc(g_telnet_data->rx_buffer_len, MALLOC_CAP_INTERNAL | MALLOC_CAP_8BIT);

    while (1) {

        ESP_LOGI(TAG, "Socket listening");

        struct sockaddr_storage source_addr; // Large enough for both IPv4 or IPv6
        socklen_t addr_len = sizeof(source_addr);
        g_telnet_data->client_socket = accept(g_telnet_data->listen_socket, (struct sockaddr *)&source_addr, &addr_len);
        if (g_telnet_data->client_socket < 0) {
            ESP_LOGE(TAG, "Unable to accept connection: errno %d", errno);
            break;
        }

        // Set tcp keepalive option
        setsockopt(g_telnet_data->client_socket, SOL_SOCKET, SO_KEEPALIVE, &keepAlive, sizeof(int));
        setsockopt(g_telnet_data->client_socket, IPPROTO_TCP, TCP_KEEPIDLE, &keepIdle, sizeof(int));
        setsockopt(g_telnet_data->client_socket, IPPROTO_TCP, TCP_KEEPINTVL, &keepInterval, sizeof(int));
        setsockopt(g_telnet_data->client_socket, IPPROTO_TCP, TCP_KEEPCNT, &keepCount, sizeof(int));
        // Convert ip address to string
        if (source_addr.ss_family == PF_INET) {
            inet_ntoa_r(((struct sockaddr_in *)&source_addr)->sin_addr, addr_str, sizeof(addr_str) - 1);
        }
#ifdef CONFIG_EXAMPLE_IPV6
        else if (source_addr.ss_family == PF_INET6) {
            inet6_ntoa_r(((struct sockaddr_in6 *)&source_addr)->sin6_addr, addr_str, sizeof(addr_str) - 1);
        }
#endif
        ESP_LOGI(TAG, "Socket accepted ip address: %s", addr_str);
        serial_set_telnet(true);

        while (1) {
            int len;
            len = recv(g_telnet_data->client_socket, g_telnet_data->rx_buffer, g_telnet_data->rx_buffer_len - 1, 0);
            if (len < 0) {
                ESP_LOGE(TAG, "Error occurred during receiving: errno %d(%s)", errno, strerror(errno));
            } else if (len == 0) {
                ESP_LOGW(TAG, "Connection closed");
                break;
            } else {
                // ESP_LOGI(TAG, "------ Received %d bytes", len);
                telnet_parse(g_telnet_data, g_telnet_data->rx_buffer, len);
            }
        }

        serial_set_telnet(false);
        shutdown(g_telnet_data->client_socket, 0);
        close(g_telnet_data->client_socket);
    }

CLEAN_UP:
    close(g_telnet_data->listen_socket);
    vTaskDelete(NULL);
}

void telnet_com_start(void)
{

#ifdef CONFIG_EXAMPLE_IPV4
    xTaskCreate(telnet_com_task, "telnet task", 4096, (void *)AF_INET, 5, NULL);
#endif
#ifdef CONFIG_EXAMPLE_IPV6
    xTaskCreate(telnet_com_task, "telnet task", 4096, (void *)AF_INET6, 5, NULL);
#endif
}
