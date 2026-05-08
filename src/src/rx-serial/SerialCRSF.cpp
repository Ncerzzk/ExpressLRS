#include "SerialCRSF.h"
#include "common.h"
#include "OTA.h"
#include "device.h"
#include "telemetry.h"
#include "BinaryStreamTCP.h"
#include "BinaryStreamUDP.h"
#include "BinaryStreamESPNow.h"
#if defined(TARGET_RX) && defined(PLATFORM_ESP8266)
  #define ELRS_MINIMAL_RX_WIFI_BRIDGE
#endif

#if defined(USE_MSP_WIFI) && !defined(ELRS_MINIMAL_RX_WIFI_BRIDGE)
#include "tcpsocket.h"
extern TCPSOCKET wifi2tcp;
#endif
#if defined(PLATFORM_ESP8266) && !defined(ELRS_BINARY_STREAM_MAX_BYTES_PER_CALL)
    #define ELRS_BINARY_STREAM_MAX_BYTES_PER_CALL 1920U
#endif

int SerialCRSF::getMaxSerialReadSize()
{
    #if defined(PLATFORM_ESP8266)
    return 384;
    #else
    return 1024;
    #endif
}

void SerialCRSF::processSerialInput()
{
    #if defined(PLATFORM_ESP8266)
    if (_inputPort == nullptr)
    {
        return;
    }

    uint16_t remainingBudget = ELRS_BINARY_STREAM_MAX_BYTES_PER_CALL;
    while (remainingBudget > 0)
    {
        if (_inputPort->hasPeekBufferAPI())
        {
            const size_t peekAvailable = _inputPort->peekAvailable();
            if (peekAvailable == 0)
            {
                break;
            }

            const uint16_t bytesToConsume = min<uint16_t>(min<size_t>(peekAvailable, getMaxSerialReadSize()), remainingBudget);
            auto *buffer = reinterpret_cast<uint8_t *>(const_cast<char *>(_inputPort->peekBuffer()));
            if (buffer == nullptr || bytesToConsume == 0)
            {
                break;
            }

            processBytes(buffer, bytesToConsume);
            _inputPort->peekConsume(bytesToConsume);
            remainingBudget -= bytesToConsume;
            continue;
        }

        uint8_t buffer[getMaxSerialReadSize()];
        const size_t available = _inputPort->available();
        if (available == 0)
        {
            break;
        }

        const uint16_t bytesToRead = min<uint16_t>(min<size_t>(available, getMaxSerialReadSize()), remainingBudget);
        const size_t bytesRead = _inputPort->readBytes(buffer, bytesToRead);
        if (bytesRead == 0)
        {
            break;
        }

        processBytes(buffer, bytesRead);
        remainingBudget -= bytesRead;
    }
    #else
    SerialIO::processSerialInput();
    #endif
}

extern Telemetry telemetry;
extern void reset_into_bootloader();
extern void UpdateModelMatch(uint8_t model);

void SerialCRSF::sendQueuedData(uint32_t maxBytesToSend)
{
    uint32_t bytesWritten = 0;
    #if defined(USE_MSP_WIFI) && !defined(ELRS_MINIMAL_RX_WIFI_BRIDGE)
    uint8_t OutPktLen;
    while ((OutPktLen = wifi2tcp.crsfCrsfOutAvailable(maxBytesToSend - bytesWritten)))
    {
        uint8_t OutData[OutPktLen];
        wifi2tcp.crsfCrsfOutPop(OutData);
        this->_outputPort->write(OutData, OutPktLen); // write the packet out
        bytesWritten += OutPktLen;
    }
    #endif
    SerialIO::sendQueuedData(maxBytesToSend - bytesWritten);
}

void SerialCRSF::queueLinkStatisticsPacket()
{
    // Note size of crsfLinkStatistics_t used, not full elrsLinkStatistics_t
    constexpr uint8_t payloadLen = sizeof(crsfLinkStatistics_t);

    constexpr uint8_t outBuffer[] = {
        payloadLen + 4,
        CRSF_ADDRESS_FLIGHT_CONTROLLER,
        CRSF_FRAME_SIZE(payloadLen),
        CRSF_FRAMETYPE_LINK_STATISTICS
    };

    uint8_t crc = crsf_crc.calc(outBuffer[3]);
    crc = crsf_crc.calc((byte *)&CRSF::LinkStatistics, payloadLen, crc);

    _fifo.lock();
    if (_fifo.ensure(outBuffer[0] + 1))
    {
        _fifo.pushBytes(outBuffer, sizeof(outBuffer));
        _fifo.pushBytes((byte *)&CRSF::LinkStatistics, payloadLen);
        _fifo.push(crc);
    }
    _fifo.unlock();
}

uint32_t SerialCRSF::sendRCFrame(bool frameAvailable, bool frameMissed, uint32_t *channelData)
{
    if (!frameAvailable)
        return DURATION_IMMEDIATELY;

    crsf_channels_s PackedRCdataOut;
    PackedRCdataOut.ch0 = channelData[0];
    PackedRCdataOut.ch1 = channelData[1];
    PackedRCdataOut.ch2 = channelData[2];
    PackedRCdataOut.ch3 = channelData[3];
    PackedRCdataOut.ch4 = channelData[4];
    PackedRCdataOut.ch5 = channelData[5];
    PackedRCdataOut.ch6 = channelData[6];
    PackedRCdataOut.ch7 = channelData[7];
    PackedRCdataOut.ch8 = channelData[8];
    PackedRCdataOut.ch9 = channelData[9];
    PackedRCdataOut.ch10 = channelData[10];
    PackedRCdataOut.ch11 = channelData[11];
    PackedRCdataOut.ch12 = channelData[12];
    PackedRCdataOut.ch13 = channelData[13];

    // In 16ch mode, do not output RSSI/LQ on channels
    if (OtaIsFullRes && OtaSwitchModeCurrent == smHybridOr16ch)
    {
        PackedRCdataOut.ch14 = channelData[14];
        PackedRCdataOut.ch15 = channelData[15];
    }
    else
    {
        // Not in 16-channel mode, send LQ and RSSI dBm
        int32_t rssiDBM = CRSF::LinkStatistics.active_antenna == 0 ? -CRSF::LinkStatistics.uplink_RSSI_1 : -CRSF::LinkStatistics.uplink_RSSI_2;

        PackedRCdataOut.ch14 = UINT10_to_CRSF(fmap(CRSF::LinkStatistics.uplink_Link_quality, 0, 100, 0, 1023));
        PackedRCdataOut.ch15 = UINT10_to_CRSF(map(constrain(rssiDBM, ExpressLRS_currAirRate_RFperfParams->RXsensitivity, -50),
                                                   ExpressLRS_currAirRate_RFperfParams->RXsensitivity, -50, 0, 1023));
    }

    constexpr uint8_t outBuffer[] = {
        // No need for length prefix as we aren't using the FIFO
        CRSF_ADDRESS_FLIGHT_CONTROLLER,
        CRSF_FRAME_SIZE(sizeof(PackedRCdataOut)),
        CRSF_FRAMETYPE_RC_CHANNELS_PACKED
    };

    uint8_t crc = crsf_crc.calc(outBuffer[2]);
    crc = crsf_crc.calc((byte *)&PackedRCdataOut, sizeof(PackedRCdataOut), crc);

    _outputPort->write(outBuffer, sizeof(outBuffer));
    _outputPort->write((byte *)&PackedRCdataOut, sizeof(PackedRCdataOut));
    _outputPort->write(crc);
    return DURATION_IMMEDIATELY;
}

void SerialCRSF::queueMSPFrameTransmission(uint8_t* data)
{
    const uint8_t totalBufferLen = CRSF_FRAME_SIZE(data[1]);
    if (totalBufferLen <= CRSF_FRAME_SIZE_MAX)
    {
        data[0] = CRSF_ADDRESS_FLIGHT_CONTROLLER;
        _fifo.lock();
        _fifo.push(totalBufferLen);
        _fifo.pushBytes(data, totalBufferLen);
        _fifo.unlock();
    }
}

void SerialCRSF::processBytes(uint8_t *bytes, uint16_t size)
{
    if (size == 0)
    {
        return;
    }

    const bool hasTcpClient = binaryStreamTcpHasClient();
    const bool hasUdpPeer = binaryStreamUdpHasPeer();
    const bool hasEspNowPeer = binaryStreamEspNowHasPeer();
    if (!hasTcpClient && !hasUdpPeer && !hasEspNowPeer)
    {
        return;
    }

    if (hasTcpClient)
    {
        binaryStreamQueueBytes(bytes, size);
    }
    if (hasUdpPeer)
    {
        binaryStreamUdpQueueBytes(bytes, size);
    }
    if (hasEspNowPeer)
    {
        binaryStreamEspNowQueueBytes(bytes, size);
    }
}
