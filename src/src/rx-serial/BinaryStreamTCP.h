#pragma once

#include "FIFO.h"

#if defined(PLATFORM_ESP32) || defined(PLATFORM_ESP8266)

#if defined(PLATFORM_ESP8266)
#include "ESPAsyncTCP.h"
#else
#include "AsyncTCP.h"
#endif

#if defined(PLATFORM_ESP8266)
    #ifndef ELRS_BINARY_STREAM_BUFFER_SIZE
        #define ELRS_BINARY_STREAM_BUFFER_SIZE 10240U
    #endif
    #ifndef ELRS_BINARY_STREAM_FLUSH_THRESHOLD
        #define ELRS_BINARY_STREAM_FLUSH_THRESHOLD 384U
    #endif
    #ifndef ELRS_BINARY_STREAM_WRITE_CHUNK_SIZE
        #if defined(TCP_SND_BUF)
            #define ELRS_BINARY_STREAM_WRITE_CHUNK_SIZE TCP_SND_BUF
        #else
            #define ELRS_BINARY_STREAM_WRITE_CHUNK_SIZE 2304U
        #endif
    #endif
    #ifndef ELRS_BINARY_STREAM_START_SEND_THRESHOLD
        #define ELRS_BINARY_STREAM_START_SEND_THRESHOLD (ELRS_BINARY_STREAM_WRITE_CHUNK_SIZE / 2U)
    #endif
    #ifndef ELRS_BINARY_STREAM_ACK_KICK_THRESHOLD
        #if defined(TCP_MSS)
            #define ELRS_BINARY_STREAM_ACK_KICK_THRESHOLD TCP_MSS
        #else
            #define ELRS_BINARY_STREAM_ACK_KICK_THRESHOLD 1024U
        #endif
    #endif
    #ifndef ELRS_BINARY_STREAM_TAIL_POLL_IDLE_US
        #define ELRS_BINARY_STREAM_TAIL_POLL_IDLE_US 500000U
    #endif
#else
    #ifndef ELRS_BINARY_STREAM_BUFFER_SIZE
        #define ELRS_BINARY_STREAM_BUFFER_SIZE 16384U
    #endif
    #ifndef ELRS_BINARY_STREAM_FLUSH_THRESHOLD
        #define ELRS_BINARY_STREAM_FLUSH_THRESHOLD 2048U
    #endif
    #ifndef ELRS_BINARY_STREAM_WRITE_CHUNK_SIZE
        #define ELRS_BINARY_STREAM_WRITE_CHUNK_SIZE 2048U
    #endif
    #ifndef ELRS_BINARY_STREAM_START_SEND_THRESHOLD
        #define ELRS_BINARY_STREAM_START_SEND_THRESHOLD 1024U
    #endif
    #ifndef ELRS_BINARY_STREAM_ACK_KICK_THRESHOLD
        #define ELRS_BINARY_STREAM_ACK_KICK_THRESHOLD 1024U
    #endif
#endif

class BinaryStreamTCP {
public:
    static constexpr uint16_t tcpPort = 5763U;

    BinaryStreamTCP() = default;
    ~BinaryStreamTCP();

    void begin(uint16_t port = tcpPort);
    void end();
    void handle();
    bool isActive() const { return tcpInitialized; }
    bool hasClient() const;
    uint32_t getQueuedBytes() const { return totalQueuedBytes; }
    uint32_t getDroppedBytes() const { return totalDroppedBytes; }
    uint32_t getAddedBytes() const { return totalAddedBytes; }
    uint16_t getPeakFifoBytes() const { return peakFifoBytes; }
    uint32_t getSendCalls() const { return totalSendCalls; }
    uint32_t getAckCallbacks() const { return totalAckCallbacks; }
    uint32_t getTimeoutCallbacks() const { return totalTimeoutCallbacks; }
    uint16_t getMaxClientSpace() const { return maxClientSpace; }
    uint16_t getMaxQueuedChunk() const { return maxQueuedChunk; }
    uint16_t getMaxAckLen() const { return maxAckLen; }
    uint32_t getTotalAckBytes() const { return totalAckBytes; }

    void queueBytes(const uint8_t *data, uint16_t len);

private:
    static constexpr uint16_t streamBufferSize = ELRS_BINARY_STREAM_BUFFER_SIZE;
    static constexpr uint16_t streamFlushThreshold = ELRS_BINARY_STREAM_FLUSH_THRESHOLD;
    static constexpr uint16_t streamWriteChunkSize = ELRS_BINARY_STREAM_WRITE_CHUNK_SIZE;
    static constexpr uint16_t streamStartSendThreshold = ELRS_BINARY_STREAM_START_SEND_THRESHOLD;
    static constexpr uint16_t streamAckKickThreshold = ELRS_BINARY_STREAM_ACK_KICK_THRESHOLD;
    static constexpr uint32_t streamTailPollIdleUs = ELRS_BINARY_STREAM_TAIL_POLL_IDLE_US;

    AsyncServer *tcpServer = nullptr;
    AsyncClient *tcpClient = nullptr;
    FIFO<streamBufferSize> *streamFifo = nullptr;
    bool tcpInitialized = false;
    bool streamFlushInProgress = false;
    bool streamSendPending = false;
    uint32_t totalQueuedBytes = 0;
    uint32_t totalDroppedBytes = 0;
    uint32_t totalAddedBytes = 0;
    uint16_t peakFifoBytes = 0;
    uint32_t totalSendCalls = 0;
    uint32_t totalAckCallbacks = 0;
    uint32_t totalTimeoutCallbacks = 0;
    uint16_t maxClientSpace = 0;
    uint16_t maxQueuedChunk = 0;
    uint16_t maxAckLen = 0;
    uint32_t totalAckBytes = 0;
    uint32_t lastQueueAtUs = 0;
#if defined(PLATFORM_ESP32)
    portMUX_TYPE streamMux = portMUX_INITIALIZER_UNLOCKED;
#endif

    void resetStreamState();
    bool ensureStreamFifo();
    void flushToClient();
    void lock();
    void unlock();

    void clientConnect(AsyncClient *client);
    void clientDisconnect(AsyncClient *client);
    static void handleNewClient(void *arg, AsyncClient *client);
    static void handleDisconnect(void *arg, AsyncClient *client);
    static void handleTimeOut(void *arg, AsyncClient *client, uint32_t time);
    static void handleError(void *arg, AsyncClient *client, err_t error);
    static void handleAck(void *arg, AsyncClient *client, size_t len, uint32_t time);
    static void handlePoll(void *arg, AsyncClient *client);
};

void binaryStreamTcpStart(uint16_t port = BinaryStreamTCP::tcpPort);
void binaryStreamTcpStop();
void binaryStreamTcpHandle();
bool binaryStreamTcpIsActive();
bool binaryStreamTcpHasClient();
void binaryStreamQueueBytes(const uint8_t *data, uint16_t len);
uint32_t binaryStreamTcpGetQueuedBytes();
uint32_t binaryStreamTcpGetDroppedBytes();
uint32_t binaryStreamTcpGetAddedBytes();
uint16_t binaryStreamTcpGetPeakFifoBytes();
uint32_t binaryStreamTcpGetSendCalls();
uint32_t binaryStreamTcpGetAckCallbacks();
uint32_t binaryStreamTcpGetTimeoutCallbacks();
uint16_t binaryStreamTcpGetMaxClientSpace();
uint16_t binaryStreamTcpGetMaxQueuedChunk();
uint16_t binaryStreamTcpGetMaxAckLen();
uint32_t binaryStreamTcpGetTotalAckBytes();

#endif
