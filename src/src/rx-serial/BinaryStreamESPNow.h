#pragma once

#if defined(PLATFORM_ESP8266)

#include "FIFO.h"
#include <Arduino.h>

class BinaryStreamESPNow {
public:
    static constexpr uint16_t statsId = 5765U;

    BinaryStreamESPNow() = default;
    ~BinaryStreamESPNow();

    void begin();
    void end();
    void handle();
    bool isActive() const { return initialized; }
    bool hasPeer() const { return peerConfigured; }
    bool setPeer(const uint8_t *mac, uint8_t channel);

    uint32_t getQueuedBytes() const { return totalQueuedBytes; }
    uint32_t getDroppedBytes() const { return totalDroppedBytes; }
    uint32_t getSentBytes() const { return totalSentBytes; }
    uint16_t getPeakFifoBytes() const { return peakFifoBytes; }
    uint32_t getSendCalls() const { return totalSendCalls; }
    uint16_t getMaxQueuedChunk() const { return maxQueuedChunk; }
    uint32_t getSendCallbacks() const { return totalSendCallbacks; }
    uint32_t getSendFailures() const { return totalSendFailures; }
    uint32_t getRecvPackets() const { return totalRecvPackets; }
    uint32_t getRecvBytes() const { return totalRecvBytes; }
    uint8_t getPeerChannel() const { return peerChannel; }
    const uint8_t *getPeerMac() const { return peerMac; }

    void queueBytes(const uint8_t *data, uint16_t len);

private:
    static constexpr uint16_t streamBufferSize = 4096U;
    static constexpr uint16_t streamPayloadSize = 200U;

    struct StreamHeader {
        uint8_t magic[2];
        uint16_t seq;
        uint8_t len;
    } __attribute__((packed));

    FIFO<streamBufferSize> *streamFifo = nullptr;
    bool initialized = false;
    bool peerConfigured = false;
    uint8_t peerMac[6] = {0};
    uint8_t peerChannel = 0;

    uint32_t totalQueuedBytes = 0;
    uint32_t totalDroppedBytes = 0;
    uint32_t totalSentBytes = 0;
    uint16_t peakFifoBytes = 0;
    uint32_t totalSendCalls = 0;
    uint16_t maxQueuedChunk = 0;
    uint32_t totalSendCallbacks = 0;
    uint32_t totalSendFailures = 0;
    uint32_t totalRecvPackets = 0;
    uint32_t totalRecvBytes = 0;

    uint8_t txBuffer[sizeof(StreamHeader) + streamPayloadSize] = {};
    uint16_t pendingPayloadLen = 0;
    uint16_t nextSeq = 0;
    bool pendingReady = false;
    bool sendInFlight = false;

    bool ensureStreamFifo();
    bool configurePeer(const uint8_t *mac, uint8_t channel);
    bool buildPendingPacket();
    void trySend();
    void onSendComplete(uint8_t status);
    void onReceive(uint8_t *mac, uint8_t *data, uint8_t len);

    static void sendCallback(uint8_t *mac, uint8_t status);
    static void recvCallback(uint8_t *mac, uint8_t *data, uint8_t len);
};

void binaryStreamEspNowStart();
void binaryStreamEspNowStop();
void binaryStreamEspNowHandle();
bool binaryStreamEspNowIsActive();
bool binaryStreamEspNowHasPeer();
bool binaryStreamEspNowSetPeer(const uint8_t *mac, uint8_t channel);
void binaryStreamEspNowQueueBytes(const uint8_t *data, uint16_t len);
uint32_t binaryStreamEspNowGetQueuedBytes();
uint32_t binaryStreamEspNowGetDroppedBytes();
uint32_t binaryStreamEspNowGetSentBytes();
uint16_t binaryStreamEspNowGetPeakFifoBytes();
uint32_t binaryStreamEspNowGetSendCalls();
uint16_t binaryStreamEspNowGetMaxQueuedChunk();
uint32_t binaryStreamEspNowGetSendCallbacks();
uint32_t binaryStreamEspNowGetSendFailures();
uint32_t binaryStreamEspNowGetRecvPackets();
uint32_t binaryStreamEspNowGetRecvBytes();
uint8_t binaryStreamEspNowGetPeerChannel();
const uint8_t *binaryStreamEspNowGetPeerMac();

#else

inline void binaryStreamEspNowStart() {}
inline void binaryStreamEspNowStop() {}
inline void binaryStreamEspNowHandle() {}
inline bool binaryStreamEspNowIsActive() { return false; }
inline bool binaryStreamEspNowHasPeer() { return false; }
inline bool binaryStreamEspNowSetPeer(const uint8_t *, uint8_t) { return false; }
inline void binaryStreamEspNowQueueBytes(const uint8_t *, uint16_t) {}
inline uint32_t binaryStreamEspNowGetQueuedBytes() { return 0; }
inline uint32_t binaryStreamEspNowGetDroppedBytes() { return 0; }
inline uint32_t binaryStreamEspNowGetSentBytes() { return 0; }
inline uint16_t binaryStreamEspNowGetPeakFifoBytes() { return 0; }
inline uint32_t binaryStreamEspNowGetSendCalls() { return 0; }
inline uint16_t binaryStreamEspNowGetMaxQueuedChunk() { return 0; }
inline uint32_t binaryStreamEspNowGetSendCallbacks() { return 0; }
inline uint32_t binaryStreamEspNowGetSendFailures() { return 0; }
inline uint32_t binaryStreamEspNowGetRecvPackets() { return 0; }
inline uint32_t binaryStreamEspNowGetRecvBytes() { return 0; }
inline uint8_t binaryStreamEspNowGetPeerChannel() { return 0; }
inline const uint8_t *binaryStreamEspNowGetPeerMac()
{
    static uint8_t empty[6] = {0};
    return empty;
}

#endif
