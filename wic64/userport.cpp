#include "driver/gpio.h"
#include "esp32-hal.h"
#include <cmath>

#include "userport.h"
#include "service.h"
#include "settings.h"
#include "led.h"
#include "utilities.h"

ESP_EVENT_DEFINE_BASE(USERPORT_EVENTS);

namespace WiC64 {
    const char* Userport::TAG = "USERPORT";
    portMUX_TYPE Userport::mutex = portMUX_INITIALIZER_UNLOCKED;

    extern Userport *userport;
    extern Service *service;
    extern Settings *settings;
    extern Led *led;

    Userport::Userport() {
        gpio_install_isr_service(
            ESP_INTR_FLAG_IRAM |
            ESP_INTR_FLAG_LOWMED |
            ESP_INTR_FLAG_LEVEL3);

        esp_event_loop_args_t event_loop_args = {
            .queue_size = 1,
            .task_name = TAG,
            .task_priority = 32,
            .task_stack_size = 8192,
            .task_core_id = 0
        };

        esp_event_loop_create(&event_loop_args, &event_loop_handle);

        esp_event_handler_register_with(
            event_loop_handle,
            USERPORT_EVENTS,
            USERPORT_REQUEST_INITIATED,
            onRequestInitiated,
            NULL);

        esp_event_handler_register_with(
            event_loop_handle,
            USERPORT_EVENTS,
            USERPORT_READY_TO_SEND,
            onReadyToSend,
            NULL);

        esp_event_handler_register_with(
            event_loop_handle,
            USERPORT_EVENTS,
            USERPORT_TRANSFER_COMPLETED,
            onTransferCompleted,
            NULL);
    }

    void Userport::connect() {
        setPortToInput();

        gpio_set_level(HANDSHAKE_LINE_ESP_TO_C64, 1);
        gpio_set_direction(HANDSHAKE_LINE_ESP_TO_C64, GPIO_MODE_OUTPUT);

        gpio_set_direction(HANDSHAKE_LINE_C64_TO_ESP, GPIO_MODE_INPUT);
        gpio_set_pull_mode(HANDSHAKE_LINE_C64_TO_ESP, GPIO_PULLUP_ONLY);
        gpio_pullup_en(HANDSHAKE_LINE_C64_TO_ESP);
        gpio_set_drive_capability(HANDSHAKE_LINE_C64_TO_ESP, GPIO_DRIVE_CAP_MAX);

        gpio_set_intr_type(HANDSHAKE_LINE_C64_TO_ESP, pc2Edge ? GPIO_INTR_NEGEDGE : GPIO_INTR_LOW_LEVEL);
        gpio_intr_enable(HANDSHAKE_LINE_C64_TO_ESP);
        gpio_isr_handler_add(HANDSHAKE_LINE_C64_TO_ESP, (gpio_isr_t) onHandshakeSignalReceived, NULL);

        gpio_set_direction(DATA_DIRECTION_LINE, GPIO_MODE_INPUT);

        connected = true;

        ESP_LOGI(TAG, "Userport connected, accepting requests");

        if (settings->rebooting()) {
            userport->sendHandshakeSignalAfterReboot();
            settings->rebooting(false);
        }
    }

    void Userport::disconnect() {
        setPortToInput();

        gpio_set_direction(HANDSHAKE_LINE_ESP_TO_C64, GPIO_MODE_INPUT);

        gpio_set_direction(HANDSHAKE_LINE_C64_TO_ESP, GPIO_MODE_INPUT);
        gpio_set_intr_type(HANDSHAKE_LINE_C64_TO_ESP, GPIO_INTR_DISABLE);
        gpio_isr_handler_remove(HANDSHAKE_LINE_C64_TO_ESP);

        gpio_set_direction(DATA_DIRECTION_LINE, GPIO_MODE_INPUT);

        connected = false;
        ESP_LOGI(TAG, "Userport disconnected, ignoring requests");
    }

    bool Userport::isConnected() {
        return connected;
    }

    bool Userport::isReadyToReceive(void) {
        return IS_HIGH(DATA_DIRECTION_LINE);
    }

    bool Userport::isReadyToSend(void) {
        return IS_LOW(DATA_DIRECTION_LINE);
    }

    bool Userport::isTransferPending(void) {
        return transferState == TRANSFER_STATE_PENDING;
    }

    void Userport::setTransferRunning() {
        transferState = TRANSFER_STATE_RUNNING;
    }

    bool Userport::isInitiallySending(void) {
        return (transferType == TRANSFER_TYPE_SEND_FULL ||
                transferType == TRANSFER_TYPE_SEND_PARTIAL) &&
                previousTransferType != TRANSFER_TYPE_SEND_PARTIAL;
    }

    bool Userport::isSending(void) {
        return isSending(transferType);
    }

    bool Userport::isSending(TRANSFER_TYPE type) {
        return type == TRANSFER_TYPE_SEND_PARTIAL ||
            type == TRANSFER_TYPE_SEND_FULL;
    }

    void Userport::setPortToInput() {
        port_config.mode = GPIO_MODE_INPUT;
        gpio_config(&port_config);
        led->off();
        ESP_LOGV(TAG, "Port set to input");
    }

    void Userport::setPortToOutput() {
        port_config.mode = GPIO_MODE_OUTPUT;
        gpio_config(&port_config);
        led->on();
        ESP_LOGV(TAG, "Port set to output");
    }

    inline void Userport::pulse() {
        SET_LOW(HANDSHAKE_LINE_ESP_TO_C64);
        esp_rom_delay_us(pulseUs);
        SET_HIGH(HANDSHAKE_LINE_ESP_TO_C64);
    }

    inline void Userport::sendHandshakeSignal() {
        pulse();
        timeOfLastHandshakeSentUs = esp_timer_get_time();
    }

    // A handshake the ESP sends on its own, not in answer to one from the
    // C64: the C64 sees each as a bit in CIA 2's ICR, and two that come
    // before it looks become one. The fixed pauses kept them apart; in fast
    // mode keep at least HANDSHAKE_GAP_US between them (the C64 looks every
    // ~10 us while it waits).
    void Userport::sendSpacedHandshake() {
        if (fast) {
            int64_t wait = timeOfLastHandshakeSentUs + HANDSHAKE_GAP_US - esp_timer_get_time();
            if (wait > 0) esp_rom_delay_us((uint32_t) wait);
        }
        sendHandshakeSignal();
    }

    void Userport::setPc2Edge(bool edge) {
        pc2Edge = edge;
        if (connected) {
            gpio_set_intr_type(HANDSHAKE_LINE_C64_TO_ESP, edge ? GPIO_INTR_NEGEDGE : GPIO_INTR_LOW_LEVEL);
        }
    }

    void Userport::dumpTrace(void) {
        uint32_t end = traceNext;
        uint32_t n = end < TRACE_SIZE ? end : TRACE_SIZE;
        uint32_t last = traceBuf[(end - 1) & (TRACE_SIZE - 1)].us;
        for (uint32_t k = end - n; k != end; k++) {
            const TraceEvent &e = traceBuf[k & (TRACE_SIZE - 1)];
            ESP_LOGW(TAG, "trace %8d us %c %02x %u", (int) (e.us - last), e.what, e.a, e.b);
        }
    }

    inline void Userport::readByte(uint8_t *byte) {
        (*byte) = 0;
        IS_HIGH(PB0) && ((*byte) |= 1);
        IS_HIGH(PB1) && ((*byte) |= 2);
        IS_HIGH(PB2) && ((*byte) |= 4);
        IS_HIGH(PB3) && ((*byte) |= 8);
        IS_HIGH(PB4) && ((*byte) |= 16);
        IS_HIGH(PB5) && ((*byte) |= 32);
        IS_HIGH(PB6) && ((*byte) |= 64);
        IS_HIGH(PB7) && ((*byte) |= 128);
    }

    inline void Userport::readNextByte() {
        readByte(buffer+pos);
        continueTransfer();
    }

    inline void Userport::writeByte(uint8_t *byte) {
        register uint8_t value = (*byte);
        (value & 1)   ? SET_HIGH(PB0) : SET_LOW(PB0);
        (value & 2)   ? SET_HIGH(PB1) : SET_LOW(PB1);
        (value & 4)   ? SET_HIGH(PB2) : SET_LOW(PB2);
        (value & 8)   ? SET_HIGH(PB3) : SET_LOW(PB3);
        (value & 16)  ? SET_HIGH(PB4) : SET_LOW(PB4);
        (value & 32)  ? SET_HIGH(PB5) : SET_LOW(PB5);
        (value & 64)  ? SET_HIGH(PB6) : SET_LOW(PB6);
        (value & 128) ? SET_HIGH(PB7) : SET_LOW(PB7);
    }

    inline void IRAM_ATTR Userport::writeFirstByte() {
        writeNextByte();
    }

    inline void Userport::writeNextByte() {
        writeByte(buffer+pos);
        continueTransfer();
    }

    void Userport::startTransfer(
            TRANSFER_TYPE type,
            uint8_t *data,
            uint32_t size,
            callback_t onSuccess,
            callback_t onFailure) {

        this->transferType = type;
        this->onSuccessCallback = onSuccess;
        this->onFailureCallback = onFailure;

        this->buffer = data;
        this->size = size;
        this->pos = 0;

        transferState = isInitiallySending()
            ? TRANSFER_STATE_PENDING
            : TRANSFER_STATE_RUNNING;

        ESP_LOGV(TAG, "%s %d bytes...", isSending(type) ? "Sending" : "Receiving", size);

        timeTransferStarted = millis();
        createTimeoutTask();

        trace('B', (uint8_t) type, (uint16_t) size, esp_timer_get_time());
        if (isInitiallySending()) {
            ESP_LOGV(TAG, "Sending initial handshake to start pending transfer");
            if (!skips(2)) vTaskDelay(pdMS_TO_TICKS(1));
            trace('I', 0, 0, esp_timer_get_time());
            sendSpacedHandshake(); // first handshake the c64 is waiting for after changing direction
        }

        if (type == TRANSFER_TYPE_RECEIVE_PARTIAL ||
            previousTransferType == TRANSFER_TYPE_RECEIVE_PARTIAL ||
            previousTransferType == TRANSFER_TYPE_SEND_PARTIAL) {

            ESP_LOGV(TAG, "Sending initial handshake signal");
            if (!skips(1)) vTaskDelay(pdMS_TO_TICKS(5));
            trace('K', 0, 0, esp_timer_get_time());
            sendSpacedHandshake();
        }
    }

    inline IRAM_ATTR void Userport::continueTransfer(void) {
        if (++pos < size) {
            // In the byte loop: the time is the interrupt's (isrNow).
            pulse();
            timeOfLastHandshakeSentUs = isrNow;
            trace('p', 0, (uint16_t) pos, isrNow);
        } else {
            userport->previousTransferType = userport->transferType;
            userport->transferType = TRANSFER_TYPE_NONE;
            post(USERPORT_TRANSFER_COMPLETED);
        }
    }

    void Userport::onTransferCompleted(void* arg, esp_event_base_t base, int32_t id, void* data) {
        TRANSFER_TYPE type = userport->previousTransferType;

        if (type == TRANSFER_TYPE_SEND_FULL || type == TRANSFER_TYPE_RECEIVE_FULL) {
            if (type == TRANSFER_TYPE_SEND_FULL) {
                userport->timeOfFinalHandshake = esp_timer_get_time();
                userport->trailingHandshakeDue = true;
            }
            userport->trace('L', (uint8_t) type, 0, esp_timer_get_time());
            userport->sendSpacedHandshake();

            if (type == TRANSFER_TYPE_RECEIVE_FULL && !userport->skips(4)) {
                vTaskDelay(pdMS_TO_TICKS(1));
            }

            ESP_LOGD(TAG, "Sent final handshake signal");
        }

        userport->trace('C', (uint8_t) type, (uint16_t) userport->size, esp_timer_get_time());
        userport->deleteTimeoutTask();
        userport->onFailureCallback = NULL;

        float sec = (millis() - userport->timeTransferStarted) / 1000.0;
        float kbs = userport->size/sec/1024;

        if (kbs != INFINITY) {
            ESP_LOGI(TAG, "%d bytes %s, transfer completed in %.4f sec, approx. %.2fkb/s",
            userport->size, userport->isSending(userport->previousTransferType) ? "sent" : "received", sec, kbs);
        } else {
            ESP_LOGI(TAG, "%d bytes %s, transfer completed",
                userport->size, userport->isSending(userport->previousTransferType) ? "sent" : "received");
        }

        if (!userport->skips(8)) {
            vTaskDelay(pdMS_TO_TICKS(3));
        } else if (type == TRANSFER_TYPE_SEND_FULL) {
            // The port must stay driven until the C64 has read the last byte.
            userport->waitForTrailingHandshake(2000);
        }

        if (userport->previousTransferType != TRANSFER_TYPE_SEND_PARTIAL) {
            userport->setPortToInput();
        }

        if (userport->onSuccessCallback != NULL) {
            userport->onSuccessCallback(userport->buffer, userport->size);
        }
    }

    void Userport::abortTransfer(const char* reason) {
        trace('A', (uint8_t) transferType, (uint16_t) pos, esp_timer_get_time());
        ESP_LOGE(TAG, "Aborting transfer: %s", reason);
        dumpTrace();
        setPortToInput();

        transferType = TRANSFER_TYPE_NONE;
        previousTransferType = TRANSFER_TYPE_NONE;
        transferState = TRANSFER_STATE_NONE;

        if (onFailureCallback != NULL) {
            onFailureCallback(buffer, pos ? pos-1 : 0);
            onFailureCallback = NULL;
        }

        deleteTimeoutTask();
    }

    void Userport::waitForTrailingHandshake(uint32_t maxUs) {
        int64_t until = esp_timer_get_time() + maxUs;
        while (trailingHandshakeDue && esp_timer_get_time() < until) {
            esp_rom_delay_us(5);
        }
    }

    bool Userport::ringDoorbell(uint32_t quietMs) {
        bool rung = false;

        portENTER_CRITICAL(&mutex);
        // A C64 handshake since the last request we completed means it has
        // started something we did not finish (a request still on its way
        // to the service, or one we dropped), and waits for a handshake: a
        // pulse now would pass for it. Only after LONG_QUIET_MS of silence,
        // longer than the C64 waits, is it sure to have given up.
        uint32_t quiet = (uint32_t) ((esp_timer_get_time() - timeOfLastActivityUs) / 1000);
        bool clean = timeOfLastHandshakeUs <= timeOfLastRequestEndUs;
        if (connected &&
            transferType == TRANSFER_TYPE_NONE &&
            ((clean && quiet >= quietMs) || quiet >= LONG_QUIET_MS)) {
            trace('R', 0, 0, esp_timer_get_time());
            sendHandshakeSignal();
            rung = true;
        }
        portEXIT_CRITICAL(&mutex);

        return rung;
    }

    void Userport::requestEnded(void) {
        // As of the answer's final handshake, not now: the pauses after it
        // leave the C64 time to start its next request, which must count
        // as activity after the end.
        timeOfLastRequestEndUs = timeOfFinalHandshake;
    }

    void Userport::createTimeoutTask(void) {
        deleteTimeoutTask();

        xTaskCreatePinnedToCore(timeoutTask, "TIMEOUT", 4096, NULL, 5, &timeoutTaskHandle, 0);

        if (timeoutTaskHandle == NULL) {
            abortTransfer("Could not create timeout supervisor task");
        }
    }

    void Userport::deleteTimeoutTask(void) {
        portENTER_CRITICAL(&mutex);

        if (timeoutTaskHandle != NULL) {
            vTaskDelete(timeoutTaskHandle);
            timeoutTaskHandle = NULL;
        }

        portEXIT_CRITICAL(&mutex);
    }

    void Userport::resetTimeout(void) {
        timeOfLastActivityUs = esp_timer_get_time();
    }

    bool Userport::hasTimedOut(void) {
        return (esp_timer_get_time() - timeOfLastActivityUs) / 1000 > (int64_t) transferTimeout;
    }

    void Userport::onRequestInitiated(void* arg, esp_event_base_t base, int32_t event_id, void* data) {

        if (!userport->isReadyToReceive()) {
            userport->handleLineNoise();
            return;
        }

        uint8_t id;
        userport->readByte(&id);

        if (Protocol::exists(id)) {
            Protocol *protocol = Protocol::get(id);

            ESP_LOGI(TAG, WIC64_SEPARATOR);

            ESP_LOGI(TAG, "Received %s protocol id " WIC64_FORMAT_PROTOCOL " (%c)",
                protocol->name(),
                protocol->id(),
                protocol->id());

            ESP_LOGI(TAG, WIC64_SEPARATOR);

            userport->resetLineNoiseCount();
            service->receiveRequest(protocol);

        } else {
            ESP_LOGW(TAG, "Received unsupported protocol id " WIC64_FORMAT_PROTOCOL, id);
            userport->handleLineNoise();
        }
    }

    void Userport::receivePartial(uint8_t *data, uint32_t size, callback_t onSuccess) {
        startTransfer(TRANSFER_TYPE_RECEIVE_PARTIAL, data, size, onSuccess, NULL);
    }

    void Userport::receivePartial(uint8_t *data, uint32_t size, callback_t onSuccess, callback_t onFailure) {
        startTransfer(TRANSFER_TYPE_RECEIVE_PARTIAL, data, size, onSuccess, onFailure);
    }

    void Userport::receive(uint8_t *data, uint32_t size, callback_t onSuccess) {
        startTransfer(TRANSFER_TYPE_RECEIVE_FULL, data, size, onSuccess, NULL);
    }

    void Userport::receive(uint8_t *data, uint32_t size, callback_t onSuccess, callback_t onFailure) {
        startTransfer(TRANSFER_TYPE_RECEIVE_FULL, data, size, onSuccess, onFailure);
    }

    void IRAM_ATTR Userport::onReadyToSend(void *arg, esp_event_base_t base, int32_t id, void *data) {
        ESP_LOGV(TAG, "Received handshake after change of transfer direction");

        if (!userport->isReadyToSend()) {
            ESP_LOGE(TAG, "Userport not ready to send - PA2 is still HIGH");
            userport->handleLineNoise();
            return;
        }
        userport->resetLineNoiseCount();
        userport->setPortToOutput();
        userport->setTransferRunning();

        ESP_LOGV(TAG, "Initiating send by writing first byte");
        userport->isrNow = esp_timer_get_time();
        userport->writeFirstByte();
    }

    void Userport::sendPartial(uint8_t *data, uint32_t size, callback_t onSuccess) {
        startTransfer(TRANSFER_TYPE_SEND_PARTIAL, data, size, onSuccess, NULL);
    }

    void Userport::sendPartial(uint8_t *data, uint32_t size, callback_t onSuccess, callback_t onFailure) {
        startTransfer(TRANSFER_TYPE_SEND_PARTIAL, data, size, onSuccess, onFailure);
    }

    void Userport::send(uint8_t *data, uint32_t size, callback_t onSuccess) {
        startTransfer(TRANSFER_TYPE_SEND_FULL, data, size, onSuccess, NULL);
    }

    void Userport::send(uint8_t *data, uint32_t size, callback_t onSuccess, callback_t onFailure) {
        startTransfer(TRANSFER_TYPE_SEND_FULL, data, size, onSuccess, onFailure);
    }

    void Userport::onHandshakeSignalReceived(void) {
        // The one timer read per byte: the C64's handshake, activity for the
        // timeout, and the time the byte loop's own handshake goes out.
        int64_t now = esp_timer_get_time();
        userport->trace('H', (uint8_t) ((userport->transferType << 4) | userport->transferState),
                        (uint16_t) userport->pos, now);
        if (userport->trailingHandshakeDue && userport->transferType == TRANSFER_TYPE_NONE) {
            userport->trailingHandshakeDue = false;
            userport->trace('T', 0, 0, now);
            // The read of the last byte, soon after the final handshake; a
            // C64 that gave up halfway sends none, and its next request must
            // not be taken for it.
            if (now - userport->timeOfFinalHandshake < 20000) return;
        }
        userport->isrNow = now;
        userport->timeOfLastHandshakeUs = now;
        userport->timeOfLastActivityUs = now;

        if (userport->transferState == TRANSFER_STATE_PENDING) {
            userport->post(USERPORT_READY_TO_SEND);
        }

        else if (userport->transferType == TRANSFER_TYPE_NONE) {
            userport->post(USERPORT_REQUEST_INITIATED);
        }

        else if (userport->transferType == TRANSFER_TYPE_RECEIVE_FULL ||
            userport->transferType == TRANSFER_TYPE_RECEIVE_PARTIAL) {
            userport->readNextByte();
        }
        else if (userport->transferType == TRANSFER_TYPE_SEND_FULL ||
            userport->transferType == TRANSFER_TYPE_SEND_PARTIAL) {
            userport->writeNextByte();
        }
    }

    void Userport::resetLineNoiseCount(void) {
        lineNoiseCount = 0;
    }

    void Userport::handleLineNoise(void) {
        if (++lineNoiseCount >= 8) {
            if (transferType != TRANSFER_TYPE_NONE) {
                abortTransfer("Line noise detected");
            }

            ESP_LOGW(TAG, "Line noise detected => disconnecting userport for 500ms");

            userport->disconnect();
            vTaskDelay(pdMS_TO_TICKS(500));
            userport->connect();

            resetLineNoiseCount();
        }
    }

    void Userport::sendHandshakeSignalBeforeReboot() {
        sendHandshakeSignal();
    }

    void Userport::sendHandshakeSignalAfterReboot(void) {
        ESP_LOGW(TAG, "Confirming reboot in 2000ms...");
        xTaskCreatePinnedToCore(sendHandshakeAfterRebootTask, "HANDSHAKE", 4096, NULL, 5, NULL, 0);
    }

    void Userport::sendHandshakeAfterRebootTask(void *) {
        vTaskDelay(pdMS_TO_TICKS(2000));
        ESP_LOGW(TAG, "Confirming reboot by sending single handshake signal");
        userport->sendHandshakeSignal();
        vTaskDelete(NULL);
    }

    inline void IRAM_ATTR Userport::post(userport_event_t event) {
        BaseType_t task_unblocked = pdFALSE;

        esp_err_t err = esp_event_isr_post_to(
            userport->event_loop_handle,
            USERPORT_EVENTS, event,
            NULL, 0,
            &task_unblocked);
        userport->trace(err == ESP_OK ? 'P' : 'X', (uint8_t) event, 0, esp_timer_get_time());

        if (task_unblocked) {
            portYIELD_FROM_ISR();
        }
    }

    void Userport::timeoutTask(void* unused) {
        userport->resetTimeout();

        while(true) {
            vTaskDelay(pdMS_TO_TICKS(250));

            if (userport->hasTimedOut()) {
                userport->abortTransfer("Timed out");
            }
        }
    }
}
