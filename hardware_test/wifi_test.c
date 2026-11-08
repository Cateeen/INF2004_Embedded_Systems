
#include "pico/stdlib.h"
#include "pico/cyw43_arch.h"
#include <stdio.h>

#define WIFI_SSID "wx"
#define WIFI_PASSWORD "wenxinxin"

#define WIFI_TIMEOUT_MS 30000

int main(void)
{
    // Initialise USB serial
    stdio_init_all();
    while (!stdio_usb_connected()) {
        sleep_ms(50);
    }

    printf("\n=================================\n");
    printf("       PICO W WIFI TEST\n");
    printf("=================================\n");
    fflush(stdout);

    // Step 1: Initialise CYW43 Wi-Fi hardware and lwIP
    printf("[1] Initialising Wi-Fi...\n");
    fflush(stdout);

    int result = cyw43_arch_init();

    if (result != 0) {
        printf("[ERROR] Wi-Fi initialisation failed: %d\n", result);
        fflush(stdout);

        while (true) {
            sleep_ms(1000);
        }
    }

    printf("[SUCCESS] Wi-Fi initialised!\n");
    fflush(stdout);

    // Step 2: Enable Wi-Fi station mode
    cyw43_arch_enable_sta_mode();

    printf("[2] Station mode enabled!\n");
    fflush(stdout);

    // Step 3: Connect to Wi-Fi
    printf("[3] Connecting to Wi-Fi...\n");
    fflush(stdout);

    result = cyw43_arch_wifi_connect_timeout_ms(
        WIFI_SSID,
        WIFI_PASSWORD,
        CYW43_AUTH_WPA2_AES_PSK,
        WIFI_TIMEOUT_MS
    );

    if (result != 0) {
        printf("[ERROR] Wi-Fi connection failed: %d\n", result);
        fflush(stdout);

        cyw43_arch_deinit();

        while (true) {
            sleep_ms(3000);
            printf("[RETRY] Please restart the Pico W to retry.\n");
            fflush(stdout);
        }
    }

    printf("[SUCCESS] Connected to Wi-Fi!\n");
    printf("[INFO] Network: %s\n", WIFI_SSID);
    fflush(stdout);

    // Step 4: Keep servicing the polling architecture
    while (true) {

        // Process CYW43 and lwIP background work
        cyw43_arch_poll();

        // Check Wi-Fi connection status
        int status = cyw43_wifi_link_status(
            &cyw43_state,
            CYW43_ITF_STA
        );

        if (status == CYW43_LINK_UP) {
            printf("[CONNECTED] Wi-Fi connection active.\n");
        } else {
            printf("[WARNING] Wi-Fi connection status: %d\n", status);
        }

        fflush(stdout);

        sleep_ms(3000);
    }

    return 0;
}
