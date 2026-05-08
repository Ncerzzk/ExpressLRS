#include "BinaryStreamESPNow.h"

#if defined(PLATFORM_ESP8266)

extern "C" {
#include <espnow.h>
#include <user_interface.h>
}

namespace {
BinaryStreamESPNow *binaryStreamEspNowOwner = nullptr;
constexpr uint8_t kMagic[2] = {'E', 'N'};
}

BinaryStreamESPNow::~BinaryStreamESPNow()
{
    end();
}

bool BinaryStreamESPNow::ensureStreamFifo()
{
    if (streamFifo != nullptr)
    {
        return true;
    }

    streamFifo = new FIFO<streamBufferSize>();
    return streamFifo != nullptr;
}

bool BinaryStreamESPNow::configurePeer(const uint8_t *mac, uint8_t channel)
{
    if (mac == nullptr)
    {
        return false;
    }

    if (peerConfigured)
    {
        esp_now_del_peer(peerMac);
        peerConfigured = false;
    }

    memcpy(peerMac, mac, sizeof(peerMac));
    peerChannel = channel != 0 ? channel : wifi_get_channel();
    if (peerChannel == 0)
    {
        peerChannel = 1;
    }

    if (esp_now_add_peer(peerMac, ESP_NOW_ROLE_CONTROLLER, peerChannel, nullptr, 0) != 0)
    {
        memset(peerMac, 0, sizeof(peerMac));
        peerChannel = 0;
        return false;
    }

    peerConfigured = true;
    pendingReady = false;
    sendInFlight = false;
    pendingPayloadLen = 0;
    return true;
}

void BinaryStreamESPNow::begin()
{
    end();
    if (esp_now_init() != 0)
    {
        return;
    }

    esp_now_register_send_cb(sendCallback);
    esp_now_register_recv_cb(recvCallback);
    if (esp_now_set_self_role(ESP_NOW_ROLE_CONTROLLER) != 0)
    {
        esp_now_unregister_recv_cb();
        esp_now_unregister_send_cb();
        esp_now_deinit();
        return;
    }

    initialized = true;
}

void BinaryStreamESPNow::end()
{
    if (peerConfigured)
    {
        esp_now_del_peer(peerMac);
        peerConfigured = false;
    }

    if (initialized)
    {
        esp_now_unregister_recv_cb();
        esp_now_unregister_send_cb();
        esp_now_deinit();
    }

    initialized = false;
    sendInFlight = false;
    pendingReady = false;
    pendingPayloadLen = 0;
    peerChannel = 0;
    memset(peerMac, 0, sizeof(peerMac));

    if (streamFifo != nullptr)
    {
        streamFifo->flush();
        delete streamFifo;
        streamFifo = nullptr;
    }
}

bool BinaryStreamESPNow::setPeer(const uint8_t *mac, uint8_t channel)
{
    if (!initialized)
    {
        begin();
    }

    if (!initialized)
    {
        return false;
    }

    return configurePeer(mac, channel);
}

bool BinaryStreamESPNow::buildPendingPacket()
{
    if (!peerConfigured || !ensureStreamFifo() || streamFifo->size() == 0)
    {
        return false;
    }

    StreamHeader header{};
    header.magic[0] = kMagic[0];
    header.magic[1] = kMagic[1];
    header.seq = nextSeq;
    header.len = static_cast<uint8_t>(min<size_t>(streamFifo->size(), streamPayloadSize));
    pendingPayloadLen = header.len;
    memcpy(txBuffer, &header, sizeof(header));

    uint16_t copied = 0;
    while (copied < pendingPayloadLen)
    {
        const uint8_t *segmentPtr = streamFifo->ptrAt(copied);
        if (segmentPtr == nullptr)
        {
            return false;
        }
        const uint16_t segmentLen = min<uint16_t>(streamFifo->contiguousSizeFrom(copied), pendingPayloadLen - copied);
        memcpy(txBuffer + sizeof(header) + copied, segmentPtr, segmentLen);
        copied += segmentLen;
    }

    pendingReady = true;
    return true;
}

void BinaryStreamESPNow::trySend()
{
    if (!initialized || !peerConfigured || sendInFlight)
    {
        return;
    }

    if (!pendingReady && !buildPendingPacket())
    {
        return;
    }

    sendInFlight = true;
    totalSendCalls++;
    maxQueuedChunk = max<uint16_t>(maxQueuedChunk, pendingPayloadLen);
    const int result = esp_now_send(peerMac, txBuffer, sizeof(StreamHeader) + pendingPayloadLen);
    if (result != 0)
    {
        sendInFlight = false;
        totalSendFailures++;
    }
}

void BinaryStreamESPNow::onSendComplete(uint8_t status)
{
    totalSendCallbacks++;
    sendInFlight = false;

    if (!pendingReady)
    {
        return;
    }

    if (status == 0)
    {
        streamFifo->skip(pendingPayloadLen);
        totalSentBytes += pendingPayloadLen;
        nextSeq++;
        pendingReady = false;
        pendingPayloadLen = 0;
    }
    else
    {
        totalSendFailures++;
    }
}

void BinaryStreamESPNow::sendCallback(uint8_t *mac, uint8_t status)
{
    (void)mac;
    if (binaryStreamEspNowOwner != nullptr)
    {
        binaryStreamEspNowOwner->onSendComplete(status);
    }
}

void BinaryStreamESPNow::onReceive(uint8_t *mac, uint8_t *data, uint8_t len)
{
    (void)mac;
    if (data == nullptr || len < sizeof(StreamHeader))
    {
        return;
    }

    const StreamHeader *header = reinterpret_cast<const StreamHeader *>(data);
    if (header->magic[0] != kMagic[0] || header->magic[1] != kMagic[1])
    {
        return;
    }

    const uint8_t payloadLen = header->len;
    if (static_cast<uint16_t>(sizeof(StreamHeader)) + payloadLen > len)
    {
        return;
    }

    totalRecvPackets++;
    totalRecvBytes += payloadLen;
}

void BinaryStreamESPNow::recvCallback(uint8_t *mac, uint8_t *data, uint8_t len)
{
    if (binaryStreamEspNowOwner != nullptr)
    {
        binaryStreamEspNowOwner->onReceive(mac, data, len);
    }
}

void BinaryStreamESPNow::handle()
{
    if (!initialized || !peerConfigured)
    {
        return;
    }

    trySend();
}

void BinaryStreamESPNow::queueBytes(const uint8_t *data, uint16_t len)
{
    if (data == nullptr || len == 0 || !peerConfigured)
    {
        return;
    }

    if (!ensureStreamFifo())
    {
        return;
    }

    if (streamFifo->free() < len)
    {
        const uint16_t toDrop = min<uint16_t>(len - streamFifo->free(), streamFifo->size());
        streamFifo->skip(toDrop);
        totalDroppedBytes += toDrop;
        if (pendingReady && pendingPayloadLen > 0)
        {
            pendingReady = false;
            pendingPayloadLen = 0;
        }
    }
    streamFifo->pushBytes(data, len);
    totalQueuedBytes += len;
    peakFifoBytes = max<uint16_t>(peakFifoBytes, static_cast<uint16_t>(streamFifo->size()));

    trySend();
}

void binaryStreamEspNowStart()
{
    binaryStreamEspNowStop();
    binaryStreamEspNowOwner = new BinaryStreamESPNow();
    binaryStreamEspNowOwner->begin();
}

void binaryStreamEspNowStop()
{
    if (binaryStreamEspNowOwner == nullptr)
    {
        return;
    }

    binaryStreamEspNowOwner->end();
    delete binaryStreamEspNowOwner;
    binaryStreamEspNowOwner = nullptr;
}

void binaryStreamEspNowHandle()
{
    if (binaryStreamEspNowOwner != nullptr)
    {
        binaryStreamEspNowOwner->handle();
    }
}

bool binaryStreamEspNowIsActive()
{
    return binaryStreamEspNowOwner != nullptr && binaryStreamEspNowOwner->isActive();
}

bool binaryStreamEspNowHasPeer()
{
    return binaryStreamEspNowOwner != nullptr && binaryStreamEspNowOwner->hasPeer();
}

bool binaryStreamEspNowSetPeer(const uint8_t *mac, uint8_t channel)
{
    if (binaryStreamEspNowOwner == nullptr)
    {
        binaryStreamEspNowStart();
    }

    return binaryStreamEspNowOwner != nullptr && binaryStreamEspNowOwner->setPeer(mac, channel);
}

void binaryStreamEspNowQueueBytes(const uint8_t *data, uint16_t len)
{
    if (binaryStreamEspNowOwner != nullptr)
    {
        binaryStreamEspNowOwner->queueBytes(data, len);
    }
}

uint32_t binaryStreamEspNowGetQueuedBytes()
{
    return binaryStreamEspNowOwner != nullptr ? binaryStreamEspNowOwner->getQueuedBytes() : 0;
}

uint32_t binaryStreamEspNowGetDroppedBytes()
{
    return binaryStreamEspNowOwner != nullptr ? binaryStreamEspNowOwner->getDroppedBytes() : 0;
}

uint32_t binaryStreamEspNowGetSentBytes()
{
    return binaryStreamEspNowOwner != nullptr ? binaryStreamEspNowOwner->getSentBytes() : 0;
}

uint16_t binaryStreamEspNowGetPeakFifoBytes()
{
    return binaryStreamEspNowOwner != nullptr ? binaryStreamEspNowOwner->getPeakFifoBytes() : 0;
}

uint32_t binaryStreamEspNowGetSendCalls()
{
    return binaryStreamEspNowOwner != nullptr ? binaryStreamEspNowOwner->getSendCalls() : 0;
}

uint16_t binaryStreamEspNowGetMaxQueuedChunk()
{
    return binaryStreamEspNowOwner != nullptr ? binaryStreamEspNowOwner->getMaxQueuedChunk() : 0;
}

uint32_t binaryStreamEspNowGetSendCallbacks()
{
    return binaryStreamEspNowOwner != nullptr ? binaryStreamEspNowOwner->getSendCallbacks() : 0;
}

uint32_t binaryStreamEspNowGetSendFailures()
{
    return binaryStreamEspNowOwner != nullptr ? binaryStreamEspNowOwner->getSendFailures() : 0;
}

uint32_t binaryStreamEspNowGetRecvPackets()
{
    return binaryStreamEspNowOwner != nullptr ? binaryStreamEspNowOwner->getRecvPackets() : 0;
}

uint32_t binaryStreamEspNowGetRecvBytes()
{
    return binaryStreamEspNowOwner != nullptr ? binaryStreamEspNowOwner->getRecvBytes() : 0;
}

uint8_t binaryStreamEspNowGetPeerChannel()
{
    return binaryStreamEspNowOwner != nullptr ? binaryStreamEspNowOwner->getPeerChannel() : 0;
}

const uint8_t *binaryStreamEspNowGetPeerMac()
{
    static uint8_t empty[6] = {0};
    return binaryStreamEspNowOwner != nullptr ? binaryStreamEspNowOwner->getPeerMac() : empty;
}

#endif
