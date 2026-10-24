#include "wic64.h"
#include "tcpClient.h"
#include "service.h"
#include "userport.h"

namespace WiC64 {
    const char* TcpClient::TAG = "TCPCLIENT";

    extern Service *service;
    extern Userport *userport;
    extern TcpClient *tcpClient;

    TcpClient::TcpClient() {
        m_lock = xSemaphoreCreateMutex();
        xTaskCreatePinnedToCore(doorbellTask, "DOORBELL", 4096, NULL, 5, NULL, 1);
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
        if (this->connected()) {
            ESP_LOGW(TAG, "Closing previously opened connection");
            close();
        }

        xSemaphoreTake(m_lock, portMAX_DELAY);
        connected = m_client.connect(host, port, 5000);
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
        xSemaphoreTake(m_lock, portMAX_DELAY);
        m_client.stop();
        xSemaphoreGive(m_lock);
    }

    void TcpClient::notify(bool on) {
        m_rung = false;
        m_notify = on;
        ESP_LOGI(TAG, "Doorbell %s", on ? "on" : "off");
    }

    void TcpClient::doorbell(void) {
        if (!m_notify || !service->idle()) return;

        uint32_t now = millis();
        if (m_rung && now - m_rungAt < RING_AGAIN_MS) return;

        // Never wait here: a command may hold the client for seconds.
        if (xSemaphoreTake(m_lock, 0) != pdTRUE) return;
        bool waiting = !m_client.connected() || m_client.available() > 0;
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