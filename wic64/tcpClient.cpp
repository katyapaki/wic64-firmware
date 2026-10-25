#include "wic64.h"
#include "tcpClient.h"
#include "service.h"
#include "userport.h"
#include "commands/commands.h"

namespace WiC64 {
    const char* TcpClient::TAG = "TCPCLIENT";

    extern Service *service;
    extern Userport *userport;
    extern TcpClient *tcpClient;

    TcpClient::TcpClient() {
        m_lock = xSemaphoreCreateMutex();
        xTaskCreatePinnedToCore(doorbellTask, "DOORBELL", 4096, NULL, 5, NULL, 1);
        restore();
        xTaskCreatePinnedToCore(keepTask, "TCPKEEP", 4096, NULL, 4, NULL, 1);
        ESP_LOGI(TAG, "TCP client initialized");
    }

    bool TcpClient::connected(void) {
        xSemaphoreTake(m_lock, portMAX_DELAY);
        bool c = m_client.connected();
        xSemaphoreGive(m_lock);
        return c;
    }

    int TcpClient::open(const char* host, const uint16_t port) {
        bool connected = false;
        m_notify = false;
        m_keep = false;
        m_renewed = false;
        userport->fast = false;
        // Another host is another program's: forget the stored link. The
        // stub opening its bridge again keeps it (no flash write).
        if (m_storedMode && m_stored.getString("host", "") != host) store(0);
        // The host to keep the link to, if the C64 asks for it next.
        strncpy(m_host, host, sizeof m_host - 1);
        m_host[sizeof m_host - 1] = 0;
        m_port = port;
        if (this->connected()) {
            ESP_LOGW(TAG, "Closing previously opened connection");
            close();
        }

        xSemaphoreTake(m_lock, portMAX_DELAY);
        // remoteTimeout: 5 s, or what the request before asked for ($32).
        connected = m_client.connect(host, port, (int32_t) remoteTimeout);
        xSemaphoreGive(m_lock);

        ESP_LOG_LEVEL((connected ? ESP_LOG_INFO : ESP_LOG_ERROR), TAG,
            "%s connection to %s on port %d",
            connected ? "Opened" : "Failed to open",
            host,
            port);

        return connected;
    }

    int32_t TcpClient::available(void) {
        xSemaphoreTake(m_lock, portMAX_DELAY);
        int32_t n = m_client.available();
        xSemaphoreGive(m_lock);
        return n;
    }

    int64_t TcpClient::read(uint8_t* data) {
        int64_t read = -1;

        xSemaphoreTake(m_lock, portMAX_DELAY);
        if (m_client.available()) {
            read = m_client.read(data, MAX_READ_CHUNK_SIZE);
            ESP_LOGI(TAG, "Read %lld bytes", read);
        }
        xSemaphoreGive(m_lock);
        return read;
    }

    int32_t TcpClient::write(Data *data) {
        return write(data->data(), data->size());
    }

    int32_t TcpClient::write(uint8_t *data, uint32_t size) {
        ESP_LOGI(TAG, "Writing %d bytes", size);

        xSemaphoreTake(m_lock, portMAX_DELAY);
        int32_t written = m_client.write(data, size);
        xSemaphoreGive(m_lock);

        if (written < size) {
            if (written <= 0) {
                ESP_LOGE(TAG, "Failed to write any data");
            }
            else {
                ESP_LOGE(TAG, "Wrote only %d of %d bytes", written, size);
            }
        }
        else {
            ESP_LOGI(TAG, "Wrote %d bytes", written);
        }

        return written;
    }

    void TcpClient::close(void) {
        ESP_LOGI(TAG, "Closing connection");
        m_notify = false;
        m_keep = false;
        m_renewed = false;
        userport->fast = false;
        store(0);
        xSemaphoreTake(m_lock, portMAX_DELAY);
        m_client.stop();
        xSemaphoreGive(m_lock);
    }

    uint8_t TcpClient::notify(uint8_t mode) {
        m_rung = false;
        m_notify = mode & 1;
        userport->fast = (mode & 2) != 0;
        m_keep = (mode & 4) != 0 && m_host[0] != 0;
        ESP_LOGI(TAG, "Doorbell %s%s%s", m_notify ? "on" : "off", userport->fast ? ", fast" : "",
            m_keep ? ", keeping the link" : "");
        store(m_keep ? (mode & NOTIFY_MODES) : 0);
        return NOTIFY_MODES;
    }

    void TcpClient::store(uint8_t mode) {
        if (mode == m_storedMode && (mode == 0 || m_stored.getString("host", "") == m_host)) return;
        if (mode) {
            m_stored.putString("host", m_host);
            m_stored.putUShort("port", m_port);
        }
        m_stored.putUChar("mode", mode);
        m_storedMode = mode;
        ESP_LOGI(TAG, mode ? "Keeping the link across restarts" : "Not keeping the link across restarts");
    }

    void TcpClient::restore(void) {
        m_stored.begin("binmon");
        m_storedMode = m_stored.getUChar("mode", 0);
        if (!(m_storedMode & 4)) return;
        String host = m_stored.getString("host", "");
        if (host.isEmpty()) return;
        strncpy(m_host, host.c_str(), sizeof m_host - 1);
        m_host[sizeof m_host - 1] = 0;
        m_port = m_stored.getUShort("port", 0);
        m_notify = m_storedMode & 1;
        userport->fast = (m_storedMode & 2) != 0;
        m_keep = true;
        ESP_LOGI(TAG, "Keeping the link to %s on port %d, as before the restart", m_host, m_port);
    }

    bool TcpClient::takeRenewed(void) {
        bool r = m_renewed;
        m_renewed = false;
        return r;
    }

    void TcpClient::otherCommand(uint8_t id) {
        switch (id) {
            case WIC64_CMD_TCP_OPEN:
            case WIC64_CMD_TCP_READ:
            case WIC64_CMD_TCP_WRITE:
            case WIC64_CMD_TCP_CLOSE:
            case WIC64_CMD_TCP_AVAILABLE:
            case WIC64_CMD_TCP_NOTIFY:
            case WIC64_CMD_SET_REMOTE_TIMEOUT:
                return;
        }
        if (m_notify || m_keep) ESP_LOGI(TAG, "Another program uses the WiC64: doorbell off");
        m_notify = false;
        m_keep = false;
        m_renewed = false;
        userport->fast = false;
        store(0);
    }

    void TcpClient::keep(void) {
        if (!m_keep || m_renewed) return;
        // Open again outside the lock's long hold only when nothing runs:
        // a command in progress has the client.
        if (!service->idle()) return;
        if (xSemaphoreTake(m_lock, 0) != pdTRUE) return;
        bool gone = !m_client.connected();
        if (gone) {
            m_client.stop();
            bool back = m_client.connect(m_host, m_port, (int32_t) KEEP_TIMEOUT_MS);
            if (back && m_keep) {
                m_renewed = true;
                m_rung = false;
                ESP_LOGI(TAG, "Opened the connection to %s on port %d again", m_host, m_port);
            } else if (back) {
                m_client.stop();  // given up meanwhile
            }
        }
        xSemaphoreGive(m_lock);
    }

    void TcpClient::keepTask(void*) {
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(KEEP_RETRY_MS));
            if (tcpClient != NULL) tcpClient->keep();
        }
    }

    void TcpClient::doorbell(void) {
        if (!m_notify || !service->idle()) return;

        uint32_t now = millis();
        if (m_rung && now - m_rungAt < RING_AGAIN_MS) return;

        // Never wait here: a command may hold the client for seconds.
        if (xSemaphoreTake(m_lock, 0) != pdTRUE) return;
        bool connected = m_client.connected();
        // Keeping the link, a closed connection is no news for the C64; a
        // connection opened again is.
        bool waiting = m_keep ? m_renewed || (connected && m_client.available() > 0)
                              : !connected || m_client.available() > 0;
        xSemaphoreGive(m_lock);

        if (waiting && userport->ringDoorbell(QUIET_MS)) {
            m_rung = true;
            m_rungAt = now;
        }
    }

    void TcpClient::doorbellTask(void*) {
        while (true) {
            vTaskDelay(pdMS_TO_TICKS(2));
            if (tcpClient != NULL) tcpClient->doorbell();
        }
    }
}