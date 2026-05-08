#include "BinaryStreamTCP.h"
#include "logging.h"

#if defined(PLATFORM_ESP32) || defined(PLATFORM_ESP8266)

extern void wifiBridgeSuspendHttpService();
extern void wifiBridgeResumeHttpService();

namespace {
BinaryStreamTCP *binaryStreamOwner = nullptr;
}

BinaryStreamTCP::~BinaryStreamTCP()
{
    end();
}

void BinaryStreamTCP::resetStreamState()
{
    streamFlushInProgress = false;
    streamSendPending = false;
    lastQueueAtUs = 0;
    if (streamFifo != nullptr)
    {
        streamFifo->flush();
    }
}

bool BinaryStreamTCP::ensureStreamFifo()
{
    if (streamFifo != nullptr)
    {
        return true;
    }

    streamFifo = new FIFO<streamBufferSize>();
    return streamFifo != nullptr;
}

void BinaryStreamTCP::lock()
{
#if defined(PLATFORM_ESP32)
    portENTER_CRITICAL(&streamMux);
#elif defined(PLATFORM_STM32)
    noInterrupts();
#endif
}

void BinaryStreamTCP::unlock()
{
#if defined(PLATFORM_ESP32)
    portEXIT_CRITICAL(&streamMux);
#elif defined(PLATFORM_STM32)
    interrupts();
#endif
}

void BinaryStreamTCP::flushToClient()
{
    if (streamFlushInProgress || tcpClient == nullptr || !tcpClient->connected())
    {
        return;
    }

    streamFlushInProgress = true;
    if (!tcpClient->canSend())
    {
        streamFlushInProgress = false;
        return;
    }
    size_t remainingBudget = min<size_t>(streamWriteChunkSize, tcpClient->space());
    if (remainingBudget == 0)
    {
        streamFlushInProgress = false;
        return;
    }
    maxClientSpace = max<uint16_t>(maxClientSpace, static_cast<uint16_t>(min<size_t>(tcpClient->space(), UINT16_MAX)));

    size_t queuedTotal = 0;
    while (remainingBudget > 0 && tcpClient->connected())
    {
        const uint8_t *segmentPtr = nullptr;
        size_t segmentLen = 0;

        lock();
        if (streamFifo != nullptr && streamFifo->size() > 0)
        {
            segmentPtr = streamFifo->headPtr();
            segmentLen = min<size_t>(streamFifo->contiguousSize(), remainingBudget);
        }
        unlock();

        if (segmentPtr == nullptr || segmentLen == 0)
        {
            break;
        }

        const size_t queued = tcpClient->add(reinterpret_cast<const char *>(segmentPtr), segmentLen, ASYNC_WRITE_FLAG_COPY);
        if (queued == 0)
        {
            break;
        }

        lock();
        streamFifo->skip(queued);
        streamSendPending = (streamFifo->size() > 0);
        unlock();

        queuedTotal += queued;
        totalAddedBytes += queued;
        maxQueuedChunk = max<uint16_t>(maxQueuedChunk, static_cast<uint16_t>(min<size_t>(queued, UINT16_MAX)));
        remainingBudget -= queued;

        if (queued < segmentLen || tcpClient->space() == 0)
        {
            break;
        }
    }

    if (queuedTotal > 0)
    {
        totalSendCalls++;
        (void)tcpClient->send();
    }

    streamFlushInProgress = false;
}

void BinaryStreamTCP::begin(uint16_t port)
{
    end();

    resetStreamState();
    tcpServer = new AsyncServer(port);
    tcpServer->setNoDelay(true);
    tcpServer->onClient(handleNewClient, this);
    tcpServer->begin();
    tcpInitialized = true;
}

void BinaryStreamTCP::end()
{
    tcpInitialized = false;
    resetStreamState();

    if (tcpClient != nullptr)
    {
        tcpClient->close(true);
        tcpClient = nullptr;
    }

    if (tcpServer != nullptr)
    {
        delete tcpServer;
        tcpServer = nullptr;
    }

    if (streamFifo != nullptr)
    {
        delete streamFifo;
        streamFifo = nullptr;
    }
}

bool BinaryStreamTCP::hasClient() const
{
    return tcpClient != nullptr && tcpClient->connected();
}

void BinaryStreamTCP::handle()
{
    // Keep the sender ACK-driven to preserve larger batches.
}

void BinaryStreamTCP::queueBytes(const uint8_t *data, uint16_t len)
{
    if (data == nullptr || len == 0 || tcpClient == nullptr || !tcpClient->connected())
    {
        return;
    }

    if (!ensureStreamFifo())
    {
        return;
    }

    bool shouldKickSend = false;
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
    streamSendPending = (fifoSize > 0);
    lastQueueAtUs = micros();
    shouldKickSend = streamSendPending &&
        tcpClient->canSend() &&
        (fifoSize >= streamStartSendThreshold);
    unlock();

    if (shouldKickSend)
    {
        flushToClient();
    }
}

void BinaryStreamTCP::clientConnect(AsyncClient *client)
{
    if (tcpClient != nullptr && tcpClient != client)
    {
        tcpClient->close(true);
    }

    resetStreamState();
    totalQueuedBytes = 0;
    totalDroppedBytes = 0;
    totalAddedBytes = 0;
    peakFifoBytes = 0;
    totalSendCalls = 0;
    totalAckCallbacks = 0;
    totalTimeoutCallbacks = 0;
    maxClientSpace = 0;
    maxQueuedChunk = 0;
    maxAckLen = 0;
    totalAckBytes = 0;
    tcpClient = client;
    wifiBridgeSuspendHttpService();
    client->setNoDelay(true);
    client->onDisconnect(handleDisconnect, this);
    client->onError(handleError, this);
    client->onTimeout(handleTimeOut, this);
    client->onAck(handleAck, this);
    client->onPoll(handlePoll, this);

    if (!ensureStreamFifo())
    {
        client->close(true);
    }
}

void BinaryStreamTCP::clientDisconnect(AsyncClient *client)
{
    if (client != tcpClient)
    {
        return;
    }

    tcpClient = nullptr;
    resetStreamState();
    wifiBridgeResumeHttpService();
}

void BinaryStreamTCP::handleNewClient(void *arg, AsyncClient *client)
{
    DBGLN("BinaryStreamTCP(%x) connected ip %s", client, client->remoteIP().toString().c_str());
    reinterpret_cast<BinaryStreamTCP *>(arg)->clientConnect(client);
}

void BinaryStreamTCP::handleDisconnect(void *arg, AsyncClient *client)
{
    reinterpret_cast<BinaryStreamTCP *>(arg)->clientDisconnect(client);
}

void BinaryStreamTCP::handleTimeOut(void *arg, AsyncClient *client, uint32_t time)
{
    (void)time;
    auto *self = reinterpret_cast<BinaryStreamTCP *>(arg);
    if (client == nullptr || client != self->tcpClient || !client->connected())
    {
        return;
    }
    self->totalTimeoutCallbacks++;
    self->flushToClient();
}

void BinaryStreamTCP::handleError(void *arg, AsyncClient *client, err_t error)
{
    (void)error;
    reinterpret_cast<BinaryStreamTCP *>(arg)->clientDisconnect(client);
}

void BinaryStreamTCP::handleAck(void *arg, AsyncClient *client, size_t len, uint32_t time)
{
    (void)client;
    (void)time;
    auto *self = reinterpret_cast<BinaryStreamTCP *>(arg);
    self->totalAckCallbacks++;
    self->totalAckBytes += len;
    self->maxAckLen = max<uint16_t>(self->maxAckLen, static_cast<uint16_t>(min<size_t>(len, UINT16_MAX)));
    self->flushToClient();
}

void BinaryStreamTCP::handlePoll(void *arg, AsyncClient *client)
{
    auto *self = reinterpret_cast<BinaryStreamTCP *>(arg);
    if (client == nullptr || client != self->tcpClient || !client->connected() || !client->canSend() || self->streamFifo == nullptr)
    {
        return;
    }

    size_t fifoSize = 0;
    self->lock();
    fifoSize = self->streamFifo->size();
    self->unlock();

    if (fifoSize == 0 || fifoSize > self->streamAckKickThreshold)
    {
        return;
    }

    const uint32_t nowUs = micros();
    if (self->lastQueueAtUs != 0 && static_cast<uint32_t>(nowUs - self->lastQueueAtUs) >= self->streamTailPollIdleUs)
    {
        self->flushToClient();
    }
}

void binaryStreamTcpStart(uint16_t port)
{
    binaryStreamTcpStop();

    binaryStreamOwner = new BinaryStreamTCP();
    binaryStreamOwner->begin(port);
}

void binaryStreamTcpStop()
{
    if (binaryStreamOwner == nullptr)
    {
        return;
    }

    binaryStreamOwner->end();
    delete binaryStreamOwner;
    binaryStreamOwner = nullptr;
}

void binaryStreamTcpHandle()
{
    if (binaryStreamOwner == nullptr)
    {
        return;
    }

    binaryStreamOwner->handle();
}

bool binaryStreamTcpIsActive()
{
    return binaryStreamOwner != nullptr && binaryStreamOwner->isActive();
}

bool binaryStreamTcpHasClient()
{
    return binaryStreamOwner != nullptr && binaryStreamOwner->hasClient();
}

void binaryStreamQueueBytes(const uint8_t *data, uint16_t len)
{
    if (binaryStreamOwner == nullptr)
    {
        return;
    }

    binaryStreamOwner->queueBytes(data, len);
}

uint32_t binaryStreamTcpGetQueuedBytes()
{
    return (binaryStreamOwner != nullptr) ? binaryStreamOwner->getQueuedBytes() : 0;
}

uint32_t binaryStreamTcpGetDroppedBytes()
{
    return (binaryStreamOwner != nullptr) ? binaryStreamOwner->getDroppedBytes() : 0;
}

uint32_t binaryStreamTcpGetAddedBytes()
{
    return (binaryStreamOwner != nullptr) ? binaryStreamOwner->getAddedBytes() : 0;
}

uint16_t binaryStreamTcpGetPeakFifoBytes()
{
    return (binaryStreamOwner != nullptr) ? binaryStreamOwner->getPeakFifoBytes() : 0;
}

uint32_t binaryStreamTcpGetSendCalls()
{
    return (binaryStreamOwner != nullptr) ? binaryStreamOwner->getSendCalls() : 0;
}

uint32_t binaryStreamTcpGetAckCallbacks()
{
    return (binaryStreamOwner != nullptr) ? binaryStreamOwner->getAckCallbacks() : 0;
}

uint32_t binaryStreamTcpGetTimeoutCallbacks()
{
    return (binaryStreamOwner != nullptr) ? binaryStreamOwner->getTimeoutCallbacks() : 0;
}

uint16_t binaryStreamTcpGetMaxClientSpace()
{
    return (binaryStreamOwner != nullptr) ? binaryStreamOwner->getMaxClientSpace() : 0;
}

uint16_t binaryStreamTcpGetMaxQueuedChunk()
{
    return (binaryStreamOwner != nullptr) ? binaryStreamOwner->getMaxQueuedChunk() : 0;
}

uint16_t binaryStreamTcpGetMaxAckLen()
{
    return (binaryStreamOwner != nullptr) ? binaryStreamOwner->getMaxAckLen() : 0;
}

uint32_t binaryStreamTcpGetTotalAckBytes()
{
    return (binaryStreamOwner != nullptr) ? binaryStreamOwner->getTotalAckBytes() : 0;
}

#endif
