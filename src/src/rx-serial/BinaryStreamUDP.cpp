#include "BinaryStreamUDP.h"

#if defined(PLATFORM_ESP32) || defined(PLATFORM_ESP8266)

namespace {
BinaryStreamUDP *binaryStreamUdpOwner = nullptr;
constexpr char kUdpReadyReply[] = "ELRS-UDP-READY";
}

BinaryStreamUDP::~BinaryStreamUDP()
{
    end();
}

void BinaryStreamUDP::resetStreamState()
{
    streamFlushInProgress = false;
    peerIp = IPAddress();
    peerPort = 0;
    lastQueueAtUs = 0;
    if (streamFifo != nullptr)
    {
        streamFifo->flush();
    }
}

bool BinaryStreamUDP::ensureStreamFifo()
{
    if (streamFifo != nullptr)
    {
        return true;
    }

    streamFifo = new FIFO<streamBufferSize>();
    return streamFifo != nullptr;
}

void BinaryStreamUDP::lock()
{
#if defined(PLATFORM_ESP32)
    portENTER_CRITICAL(&streamMux);
#endif
}

void BinaryStreamUDP::unlock()
{
#if defined(PLATFORM_ESP32)
    portEXIT_CRITICAL(&streamMux);
#endif
}

void BinaryStreamUDP::discoverPeer()
{
    int packetSize = 0;
    while ((packetSize = udp.parsePacket()) > 0)
    {
        peerIp = udp.remoteIP();
        peerPort = udp.remotePort();

        uint8_t discard[64];
        while (packetSize > 0)
        {
            const int toRead = min<int>(packetSize, static_cast<int>(sizeof(discard)));
            const int bytesRead = udp.read(discard, toRead);
            if (bytesRead <= 0)
            {
                break;
            }
            packetSize -= bytesRead;
        }

        udp.beginPacket(peerIp, peerPort);
        udp.write(reinterpret_cast<const uint8_t *>(kUdpReadyReply), sizeof(kUdpReadyReply) - 1);
        udp.endPacket();
    }
}

void BinaryStreamUDP::setPeer(const IPAddress &ip, uint16_t port)
{
    if (port == 0)
    {
        return;
    }

    peerIp = ip;
    peerPort = port;
}

void BinaryStreamUDP::flushToPeer()
{
    if (streamFlushInProgress || peerPort == 0 || streamFifo == nullptr)
    {
        return;
    }

    streamFlushInProgress = true;
    while (peerPort != 0)
    {
        const uint8_t *segmentPtr = nullptr;
        size_t segmentLen = 0;

        lock();
        if (streamFifo->size() > 0)
        {
            segmentPtr = streamFifo->headPtr();
            segmentLen = min<size_t>(streamFifo->contiguousSize(), streamWriteChunkSize);
        }
        unlock();

        if (segmentPtr == nullptr || segmentLen == 0)
        {
            break;
        }

        if (udp.beginPacket(peerIp, peerPort) == 0)
        {
            break;
        }

        const size_t written = udp.write(segmentPtr, segmentLen);
        if (udp.endPacket() == 0 || written == 0)
        {
            break;
        }

        lock();
        streamFifo->skip(written);
        unlock();

        totalSentBytes += written;
        totalSendCalls++;
        maxQueuedChunk = max<uint16_t>(maxQueuedChunk, static_cast<uint16_t>(min<size_t>(written, UINT16_MAX)));

        if (written < segmentLen)
        {
            break;
        }
    }

    streamFlushInProgress = false;
}

void BinaryStreamUDP::begin(uint16_t port)
{
    end();
    resetStreamState();
    udp.begin(port);
    udpInitialized = true;
}

void BinaryStreamUDP::end()
{
    udpInitialized = false;
    resetStreamState();
    udp.stop();
    if (streamFifo != nullptr)
    {
        delete streamFifo;
        streamFifo = nullptr;
    }
}

void BinaryStreamUDP::handle()
{
    if (!udpInitialized)
    {
        return;
    }

    discoverPeer();
    if (peerPort != 0)
    {
        const uint32_t nowUs = micros();
        bool shouldFlush = false;

        lock();
        const uint16_t fifoSize = streamFifo != nullptr ? streamFifo->size() : 0;
        shouldFlush = fifoSize >= streamFlushThreshold;
        if (!shouldFlush && fifoSize > 0 && lastQueueAtUs != 0 && static_cast<uint32_t>(nowUs - lastQueueAtUs) >= streamTailFlushUs)
        {
            shouldFlush = true;
        }
        unlock();

        if (shouldFlush)
        {
            flushToPeer();
        }
    }
}

void BinaryStreamUDP::queueBytes(const uint8_t *data, uint16_t len)
{
    if (data == nullptr || len == 0 || peerPort == 0)
    {
        return;
    }

    if (!ensureStreamFifo())
    {
        return;
    }

    lock();
    if (streamFifo->free() < len)
    {
        const uint16_t toDrop = min<uint16_t>(len - streamFifo->free(), streamFifo->size());
        streamFifo->skip(toDrop);
        totalDroppedBytes += toDrop;
    }
    streamFifo->pushBytes(data, len);
    const size_t fifoSize = streamFifo->size();
    totalQueuedBytes += len;
    peakFifoBytes = max<uint16_t>(peakFifoBytes, static_cast<uint16_t>(fifoSize));
    lastQueueAtUs = micros();
    unlock();

    if (fifoSize >= streamFlushThreshold)
    {
        flushToPeer();
    }
}

void binaryStreamUdpStart(uint16_t port)
{
    binaryStreamUdpStop();
    binaryStreamUdpOwner = new BinaryStreamUDP();
    binaryStreamUdpOwner->begin(port);
}

void binaryStreamUdpStop()
{
    if (binaryStreamUdpOwner == nullptr)
    {
        return;
    }

    binaryStreamUdpOwner->end();
    delete binaryStreamUdpOwner;
    binaryStreamUdpOwner = nullptr;
}

void binaryStreamUdpHandle()
{
    if (binaryStreamUdpOwner != nullptr)
    {
        binaryStreamUdpOwner->handle();
    }
}

bool binaryStreamUdpIsActive()
{
    return binaryStreamUdpOwner != nullptr && binaryStreamUdpOwner->isActive();
}

bool binaryStreamUdpHasPeer()
{
    return binaryStreamUdpOwner != nullptr && binaryStreamUdpOwner->hasPeer();
}

void binaryStreamUdpQueueBytes(const uint8_t *data, uint16_t len)
{
    if (binaryStreamUdpOwner != nullptr)
    {
        binaryStreamUdpOwner->queueBytes(data, len);
    }
}

uint32_t binaryStreamUdpGetQueuedBytes()
{
    return binaryStreamUdpOwner != nullptr ? binaryStreamUdpOwner->getQueuedBytes() : 0;
}

uint32_t binaryStreamUdpGetDroppedBytes()
{
    return binaryStreamUdpOwner != nullptr ? binaryStreamUdpOwner->getDroppedBytes() : 0;
}

uint32_t binaryStreamUdpGetSentBytes()
{
    return binaryStreamUdpOwner != nullptr ? binaryStreamUdpOwner->getSentBytes() : 0;
}

uint16_t binaryStreamUdpGetPeakFifoBytes()
{
    return binaryStreamUdpOwner != nullptr ? binaryStreamUdpOwner->getPeakFifoBytes() : 0;
}

uint32_t binaryStreamUdpGetSendCalls()
{
    return binaryStreamUdpOwner != nullptr ? binaryStreamUdpOwner->getSendCalls() : 0;
}

uint16_t binaryStreamUdpGetMaxQueuedChunk()
{
    return binaryStreamUdpOwner != nullptr ? binaryStreamUdpOwner->getMaxQueuedChunk() : 0;
}

uint16_t binaryStreamUdpGetPeerPort()
{
    return binaryStreamUdpOwner != nullptr ? binaryStreamUdpOwner->getPeerPort() : 0;
}

void binaryStreamUdpSetPeer(const IPAddress &ip, uint16_t port)
{
    if (binaryStreamUdpOwner != nullptr)
    {
        binaryStreamUdpOwner->setPeer(ip, port);
    }
}

#endif
