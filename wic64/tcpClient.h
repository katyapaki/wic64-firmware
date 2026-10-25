#ifndef WIC64_TCP_CLIENT_H
#define WIC64_TCP_CLIENT_H

#include "data.h"

#include "WiFiClient.h"
#include "Preferences.h"
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

            // Keeping the link (mode bit 2): when the connection is gone the
            // firmware opens it again by itself, every KEEP_RETRY_MS, to the
            // host of the last open, and rings once it is back; the next
            // read answers STATUS_RENEWED. A closed connection does not
            // ring: the C64 stays undisturbed until the peer is back.
            static const uint32_t KEEP_RETRY_MS = 1000;
            static const uint32_t KEEP_TIMEOUT_MS = 1000;
            char m_host[256] = {0};
            uint16_t m_port = 0;
            volatile bool m_keep = false;
            volatile bool m_renewed = false;
            void keep(void);
            static void keepTask(void*);

            // The kept link survives a restart of the WiC64 (its reset
            // button, a flash): it is stored in flash ("binmon": mode,
            // host, port), written only when it changes, and taken up
            // again at the start. Whatever ends keeping clears it.
            Preferences m_stored;
            uint8_t m_storedMode = 0;
            void store(uint8_t mode);
            void restore(void);

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

            // Bit 0: ring the doorbell; bit 1: fast transfers; bit 2: keep
            // the link. Returns the bits this firmware knows.
            static const uint8_t NOTIFY_MODES = 7;
            uint8_t notify(uint8_t mode);
            void notifyServed(void) { m_rung = false; }
            // A read reports a connection the firmware opened again once:
            // true the first time after it.
            bool takeRenewed(void);
            // Another program uses the WiC64 (a command c64-binmon's stub
            // never sends): no more rings, fast transfers or keeping.
            void otherCommand(uint8_t id);
    };
}

#endif // WIC64_TCP_CLIENT_H