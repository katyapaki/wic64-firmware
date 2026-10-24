#ifndef WIC64_TCP_CLIENT_H
#define WIC64_TCP_CLIENT_H

#include "data.h"

#include "WiFiClient.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"

namespace WiC64 {
    class TcpClient {
        public: static const char* TAG;

        private:
            WiFiClient m_client;
            static const uint16_t MAX_READ_CHUNK_SIZE = 8192;

            // c64-binmon doorbell (WIC64_CMD_TCP_NOTIFY): instead of the C64
            // polling, FLAG2 is pulsed when data waits; again after
            // RING_AGAIN_MS if the C64 has not read by then.
            static const uint32_t RING_AGAIN_MS = 50;
            static const uint32_t QUIET_MS = 5;

            // The doorbell task asks while commands run on the service task.
            SemaphoreHandle_t m_lock;
            volatile bool m_notify = false;
            volatile bool m_rung = false;
            volatile uint32_t m_rungAt = 0;

            void doorbell(void);
            static void doorbellTask(void*);

        public:
            TcpClient();
            bool connected(void);
            int open(const char* host, const uint16_t port);
            int32_t available(void);
            int64_t read(uint8_t* data);
            int32_t write(Data* data);
            int32_t write(uint8_t* data, uint32_t size);
            void close(void);

            // Bit 0: ring the doorbell; bit 1: fast transfers.
            void notify(uint8_t mode);
            void notifyServed(void) { m_rung = false; }
    };
}

#endif // WIC64_TCP_CLIENT_H