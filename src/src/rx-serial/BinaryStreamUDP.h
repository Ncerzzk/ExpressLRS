#pragma once

#include "FIFO.h"

#if defined(PLATFORM_ESP32) || defined(PLATFORM_ESP8266)

#include <WiFiUdp.h>

#if defined(PLATFORM_ESP8266)
  #ifndef ELRS_BINARY_STREAM_UDP_BUFFER_SIZE
    #define ELRS_BINARY_STREAM_UDP_BUFFER_SIZE 10240U
  #endif
  #ifndef ELRS_BINARY_STREAM_UDP_WRITE_CHUNK_SIZE
    #define ELRS_BINARY_STREAM_UDP_WRITE_CHUNK_SIZE 1200U
  #endif
  #ifndef ELRS_BINARY_STREAM_UDP_FLUSH_THRESHOLD
    #define ELRS_BINARY_STREAM_UDP_FLUSH_THRESHOLD 1024U
  #endif
  #ifndef ELRS_BINARY_STREAM_UDP_TAIL_FLUSH_US
    #define ELRS_BINARY_STREAM_UDP_TAIL_FLUSH_US 300U
  #endif
#else
  #ifndef ELRS_BINARY_STREAM_UDP_BUFFER_SIZE
    #define ELRS_BINARY_STREAM_UDP_BUFFER_SIZE 16384U
  #endif
  #ifndef ELRS_BINARY_STREAM_UDP_WRITE_CHUNK_SIZE
    #define ELRS_BINARY_STREAM_UDP_WRITE_CHUNK_SIZE 1200U
  #endif
  #ifndef ELRS_BINARY_STREAM_UDP_FLUSH_THRESHOLD
    #define ELRS_BINARY_STREAM_UDP_FLUSH_THRESHOLD 1024U
  #endif
  #ifndef ELRS_BINARY_STREAM_UDP_TAIL_FLUSH_US
    #define ELRS_BINARY_STREAM_UDP_TAIL_FLUSH_US 300U
  #endif
#endif

class BinaryStreamUDP {
public:
    static constexpr uint16_t udpPort = 5764U;

    BinaryStreamUDP() = default;
    ~BinaryStreamUDP();

    void begin(uint16_t port = udpPort);
    void end();
    void handle();
    bool isActive() const { return udpInitialized; }
    bool hasPeer() const { return peerPort != 0; }
    void setPeer(const IPAddress &ip, uint16_t port);

    uint32_t getQueuedBytes() const { return totalQueuedBytes; }
    uint32_t getDroppedBytes() const { return totalDroppedBytes; }
    uint32_t getSentBytes() const { return totalSentBytes; }
    uint16_t getPeakFifoBytes() const { return peakFifoBytes; }
    uint32_t getSendCalls() const { return totalSendCalls; }
    uint16_t getMaxQueuedChunk() const { return maxQueuedChunk; }
    uint16_t getPeerPort() const { return peerPort; }

    void queueBytes(const uint8_t *data, uint16_t len);

private:
    static constexpr uint16_t streamBufferSize = ELRS_BINARY_STREAM_UDP_BUFFER_SIZE;
    static constexpr uint16_t streamWriteChunkSize = ELRS_BINARY_STREAM_UDP_WRITE_CHUNK_SIZE;
    static constexpr uint16_t streamFlushThreshold = ELRS_BINARY_STREAM_UDP_FLUSH_THRESHOLD;
    static constexpr uint32_t streamTailFlushUs = ELRS_BINARY_STREAM_UDP_TAIL_FLUSH_US;

    WiFiUDP udp;
    FIFO<streamBufferSize> *streamFifo = nullptr;
    IPAddress peerIp;
    uint16_t peerPort = 0;
    bool udpInitialized = false;
    bool streamFlushInProgress = false;
    uint32_t totalQueuedBytes = 0;
    uint32_t totalDroppedBytes = 0;
    uint32_t totalSentBytes = 0;
    uint16_t peakFifoBytes = 0;
    uint32_t totalSendCalls = 0;
    uint16_t maxQueuedChunk = 0;
    uint32_t lastQueueAtUs = 0;
#if defined(PLATFORM_ESP32)
    portMUX_TYPE streamMux = portMUX_INITIALIZER_UNLOCKED;
#endif

    void resetStreamState();
    bool ensureStreamFifo();
    void flushToPeer();
    void discoverPeer();
    void lock();
    void unlock();
};

void binaryStreamUdpStart(uint16_t port = BinaryStreamUDP::udpPort);
void binaryStreamUdpStop();
void binaryStreamUdpHandle();
bool binaryStreamUdpIsActive();
bool binaryStreamUdpHasPeer();
void binaryStreamUdpQueueBytes(const uint8_t *data, uint16_t len);
uint32_t binaryStreamUdpGetQueuedBytes();
uint32_t binaryStreamUdpGetDroppedBytes();
uint32_t binaryStreamUdpGetSentBytes();
uint16_t binaryStreamUdpGetPeakFifoBytes();
uint32_t binaryStreamUdpGetSendCalls();
uint16_t binaryStreamUdpGetMaxQueuedChunk();
uint16_t binaryStreamUdpGetPeerPort();
void binaryStreamUdpSetPeer(const IPAddress &ip, uint16_t port);

#endif
