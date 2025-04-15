#if defined(GPIO_PIN_PWM_OUTPUTS)

#include "devServoOutput.h"
#include "PWM.h"
#include "CRSF.h"
#include "config.h"
#include "logging.h"
#include "rxtx_intf.h"

#if defined(PLATFORM_ESP8266)
#include <FS.h>
#else
#include <SPIFFS.h>
#endif
#include <ArduinoJson.h>

static int8_t servoPins[PWM_MAX_CHANNELS];
static pwm_channel_t pwmChannels[PWM_MAX_CHANNELS];
static uint16_t pwmChannelValues[PWM_MAX_CHANNELS];

#if (defined(PLATFORM_ESP32))
static DShotRMT *dshotInstances[PWM_MAX_CHANNELS] = {nullptr};
const uint8_t RMT_MAX_CHANNELS = 8;
#endif

// true when the RX has a new channels packet
static bool newChannelsAvailable;
// Absolute max failsafe time if no update is received, regardless of LQ
static constexpr uint32_t FAILSAFE_ABS_TIMEOUT_MS = 1000U;

typedef struct {
    uint8_t channel_idx;
    float k;
    int16_t offset;
}scaler_t;

typedef struct {
    uint8_t channel_idx;
    scaler_t *scalers;
    uint8_t scaler_cnt;
    int16_t min;
    int16_t max;
}mixer_channel_t;

mixer_channel_t **mixer_channels = nullptr;

static mixer_channel_t **init_mixer_cfg()
{
    File file = SPIFFS.open("mixer.json", "r");
    if(!file){
        return nullptr;
    }
    DynamicJsonDocument doc(2048);
    deserializeJson(doc, file);
    if(!doc["mixer_enable"].as<bool>()){
        return nullptr;
    }

    int mixer_num = doc["mixer_cfg"].as<JsonArray>().size();
    mixer_channel_t **mixer_channels;
    mixer_channels = new mixer_channel_t *[GPIO_PIN_PWM_OUTPUTS_COUNT];
    for(int i = 0; i< GPIO_PIN_PWM_OUTPUTS_COUNT; ++i){
        mixer_channels[i] = nullptr;;
    }

    for (int i = 0; i < mixer_num; ++i)
    {
        JsonObject obj = doc["mixer_cfg"].as<JsonArray>()[i].as<JsonObject>();

        JsonArray scalers_json = obj["scalers"].as<JsonArray>();
        mixer_channel_t *mixer_channel = new mixer_channel_t;
        mixer_channel->scaler_cnt = scalers_json.size();
        mixer_channel->scalers = new scaler_t[scalers_json.size()];
        mixer_channel->min = obj["min"];
        mixer_channel->max = obj["max"];

        uint8_t scaler_cnt = 0;
        for (JsonVariant scaler_ : scalers_json)
        {
            JsonObject scaler_obj = scaler_.as<JsonObject>();

            mixer_channel->scalers[scaler_cnt].channel_idx = constrain((int8_t)scaler_obj["input_chn"], 0, 15);
            mixer_channel->scalers[scaler_cnt].k = scaler_obj["k"].as<float>();
            mixer_channel->scalers[scaler_cnt].offset = scaler_obj["offset"];
            scaler_cnt++;
        }

        mixer_channels[i] = mixer_channel;
    }

    return mixer_channels;
}

#include "ICM42670P.h"
#include "SensorFusion.h"

typedef struct{
    bool enable;
    uint8_t chn;
    float kp;
    float kd;
    float ki;
    int16_t last_err;
    int32_t i_sum; 
}PID_S;

static ICM42670 IMU(Wire,0);
static struct{
    bool gyro_enable;
    PID_S pid[3];
}Gyro_Cfg={0};
static inv_imu_sensor_event_t imu_event;

static void init_gyro()
{
    File file = SPIFFS.open("gyro.json", "r");
    if(!file ){
        return ;
    }
    DynamicJsonDocument doc(2048);
    deserializeJson(doc, file);
    file.close();

    
    if(! doc["gyro_enable"].as<bool>() || GPIO_PIN_SDA == UNDEF_PIN || GPIO_PIN_SCL == UNDEF_PIN){
        Gyro_Cfg.gyro_enable = false;
        return ;
    }

    Gyro_Cfg.gyro_enable = true;
    for(int i = 0; i < 3; ++i){
        Gyro_Cfg.pid[i].enable = doc["gyro_cfg"].as<JsonArray>()[i]["enable"];
        Gyro_Cfg.pid[i].chn = doc["gyro_cfg"].as<JsonArray>()[i]["chn"];
        Gyro_Cfg.pid[i].kp = doc["gyro_cfg"].as<JsonArray>()[i]["kp"];
        Gyro_Cfg.pid[i].kd = doc["gyro_cfg"].as<JsonArray>()[i]["kd"];
        Gyro_Cfg.pid[i].ki = doc["gyro_cfg"].as<JsonArray>()[i]["ki"];
    }

    bool ret = IMU.begin();
    IMU.startAccel(800,16);
    // Gyro ODR = 100 Hz and Full Scale Range = 2000 dps
    IMU.startGyro(800,2000);
    // Wait IMU to start
    delay(100);

    DBGLN("Gyro Init:%d\n",ret);
}

static void gyro_pid_control(uint32_t *crsf_data){
    if(!Gyro_Cfg.gyro_enable){
        return ;
    }


    // Get last event
    IMU.getDataFromRegisters(imu_event);

    for(int i=0; i < 3; ++i){
        if(Gyro_Cfg.pid[i].enable){
            uint8_t chn = Gyro_Cfg.pid[i].chn;
            int16_t err = 0 - imu_event.gyro[i];
            Gyro_Cfg.pid[i].i_sum += err;
            crsf_data[chn] += err * Gyro_Cfg.pid[i].kp
                + (err - Gyro_Cfg.pid[i].last_err) * Gyro_Cfg.pid[i].kd 
                + Gyro_Cfg.pid[i].i_sum * Gyro_Cfg.pid[i].ki;
            Gyro_Cfg.pid[i].last_err = err;
        }
    }
}

void imu_update(int16_t *gyro, int16_t *acc){
    IMU.getDataFromRegisters(imu_event);
    if(gyro){
        memcpy(gyro,imu_event.gyro,sizeof(int16_t) * 3);
    }
    if(acc){
        memcpy(acc,imu_event.accel,sizeof(int16_t) * 3);
    }
}

void ICACHE_RAM_ATTR servoNewChannelsAvailable()
{
    newChannelsAvailable = true;
}

uint16_t servoOutputModeToFrequency(eServoOutputMode mode)
{
    switch (mode)
    {
    case som50Hz:
        return 50U;
    case som60Hz:
        return 60U;
    case som100Hz:
        return 100U;
    case som160Hz:
        return 160U;
    case som333Hz:
        return 333U;
    case som400Hz:
        return 400U;
    case som10KHzDuty:
        return 10000U;
    default:
        return 0;
    }
}

void servoWrite(uint8_t ch, uint16_t us)
{
    const rx_config_pwm_t *chConfig = config.GetPwmChannel(ch);
#if defined(PLATFORM_ESP32)
    if ((eServoOutputMode)chConfig->val.mode == somDShot)
    {
        // DBGLN("Writing DShot output: us: %u, ch: %d", us, ch);
        if (dshotInstances[ch])
        {
            dshotInstances[ch]->send_dshot_value(((us - 1000) * 2) + 47); // Convert PWM signal in us to DShot value
        }
    }
    else
#endif
    if (servoPins[ch] != UNDEF_PIN && pwmChannelValues[ch] != us)
    {
        pwmChannelValues[ch] = us;
        if ((eServoOutputMode)chConfig->val.mode == somOnOff)
        {
            digitalWrite(servoPins[ch], us > 1500);
        }
        else if ((eServoOutputMode)chConfig->val.mode == som10KHzDuty)
        {
            PWM.setDuty(pwmChannels[ch], constrain(us, 1000, 2000) - 1000);
        }
        else
        {
            PWM.setMicroseconds(pwmChannels[ch], us / (chConfig->val.narrow + 1));
        }
    }
}

static void servosFailsafe()
{
    constexpr unsigned SERVO_FAILSAFE_MIN = 988U;
    for (int ch = 0 ; ch < GPIO_PIN_PWM_OUTPUTS_COUNT ; ++ch)
    {
        const rx_config_pwm_t *chConfig = config.GetPwmChannel(ch);
        if (chConfig->val.failsafeMode == PWMFAILSAFE_SET_POSITION) {
            // Note: Failsafe values do not respect the inverted flag, failsafe values are absolute
            uint16_t us = chConfig->val.failsafe + SERVO_FAILSAFE_MIN;
            // Always write the failsafe position even if the servo has never been started,
            // so all the servos go to their expected position
            servoWrite(ch, us);
        }
        else if (chConfig->val.failsafeMode == PWMFAILSAFE_NO_PULSES) {
            servoWrite(ch, 0);
        }
        else if (chConfig->val.failsafeMode == PWMFAILSAFE_LAST_POSITION) {
            // do nothing
        }
    }
}

static uint16_t scaler_cal(uint32_t *crsf_data, uint8_t chn)
{
    if (!crsf_data)
    {
        return 0;
    }

    if (!mixer_channels || !mixer_channels[chn])
    {
        return CRSF_to_US(crsf_data[chn]);
    }

    mixer_channel_t *mixer_channel_cfg = mixer_channels[chn];
    int16_t ret = 0;
    for (int i = 0; i < mixer_channel_cfg->scaler_cnt; ++i)
    {
        scaler_t &scaler = mixer_channel_cfg->scalers[i];
        int16_t maped = CRSF_to_N(crsf_data[scaler.channel_idx], 1000);
        ret += (maped + scaler.offset) * scaler.k;
    }
    ret = constrain(ret, 0, 1000);

    return fmap(ret, 0, 1000, mixer_channel_cfg->min, mixer_channel_cfg->max);
}

static void servosUpdate(unsigned long now)
{
    static uint32_t lastUpdate;
    if (newChannelsAvailable)
    {
        newChannelsAvailable = false;
        lastUpdate = now;
        gyro_pid_control(ChannelData);
        for (int ch = 0 ; ch < GPIO_PIN_PWM_OUTPUTS_COUNT ; ++ch)
        {
            const rx_config_pwm_t *chConfig = config.GetPwmChannel(ch);
            const unsigned crsfVal = ChannelData[chConfig->val.inputChannel];
            // crsfVal might 0 if this is a switch channel, and it has not been
            // received yet. Delay initializing the servo until the channel is valid
            if (crsfVal == 0)
            {
                continue;
            }
            uint16_t us = mixer_channels ? scaler_cal(ChannelData, ch) : CRSF_to_US(crsfVal);
            // Flip the output around the mid value if inverted
            // (1500 - usOutput) + 1500
            if (chConfig->val.inverted)
            {
                us = 3000U - us;
            }
            servoWrite(ch, us);
        } /* for each servo */
    }     /* if newChannelsAvailable */

    // LQ goes to 0 (100 packets missed in a row)
    // OR last update older than FAILSAFE_ABS_TIMEOUT_MS
    // go to failsafe
    else if (lastUpdate && ((getLq() == 0) || (now - lastUpdate > FAILSAFE_ABS_TIMEOUT_MS)))
    {
        servosFailsafe();
        lastUpdate = 0;
    }
}

static void initialize()
{
    if (!OPT_HAS_SERVO_OUTPUT)
    {
        return;
    }

#if defined(PLATFORM_ESP32)
    uint8_t rmtCH = 0;
#endif
    for (int ch = 0; ch < GPIO_PIN_PWM_OUTPUTS_COUNT; ++ch)
    {
        pwmChannelValues[ch] = UINT16_MAX;
        pwmChannels[ch] = -1;
        int8_t pin = GPIO_PIN_PWM_OUTPUTS[ch];
#if (defined(DEBUG_LOG) || defined(DEBUG_RCVR_LINKSTATS)) && (defined(PLATFORM_ESP8266) || defined(PLATFORM_ESP32))
        // Disconnect the debug UART pins if DEBUG_LOG
        if (pin == U0RXD_GPIO_NUM || pin == U0TXD_GPIO_NUM)
        {
            pin = UNDEF_PIN;
        }
#endif
        // Mark servo pins that are being used for serial (or other purposes) as disconnected
        auto mode = (eServoOutputMode)config.GetPwmChannel(ch)->val.mode;
        if (mode >= somSerial)
        {
            pin = UNDEF_PIN;
        }
#if defined(PLATFORM_ESP32)
        else if (mode == somDShot)
        {
            if (rmtCH < RMT_MAX_CHANNELS)
            {
                auto gpio = (gpio_num_t)pin;
                auto rmtChannel = (rmt_channel_t)rmtCH;
                DBGLN("Initializing DShot: gpio: %u, ch: %d, rmtChannel: %u", gpio, ch, rmtChannel);
                pinMode(pin, OUTPUT);
                dshotInstances[ch] = new DShotRMT(gpio, rmtChannel); // Initialize the DShotRMT instance
                rmtCH++;
            }
            pin = UNDEF_PIN;
        }
#endif
        servoPins[ch] = pin;
        // Initialize all servos to low ASAP
        if (pin != UNDEF_PIN)
        {
            if (mode == somOnOff)
            {
                DBGLN("Initializing digital output: ch: %d, pin: %d", ch, pin);
            }
            else
            {
                DBGLN("Initializing PWM output: ch: %d, pin: %d", ch, pin);
            }

            pinMode(pin, OUTPUT);
            digitalWrite(pin, LOW);
        }
    }
    mixer_channels = init_mixer_cfg();
    if(mixer_channels){
        DBGLN("Mixer Enabler!\n");
    }else{
        DBGLN("Mixer Disable!\n");
    }
    init_gyro();
}

static int start()
{
    for (int ch = 0; ch < GPIO_PIN_PWM_OUTPUTS_COUNT; ++ch)
    {
        const rx_config_pwm_t *chConfig = config.GetPwmChannel(ch);
        auto frequency = servoOutputModeToFrequency((eServoOutputMode)chConfig->val.mode);
        if (frequency && servoPins[ch] != UNDEF_PIN)
        {
            pwmChannels[ch] = PWM.allocate(servoPins[ch], frequency);
        }
#if defined(PLATFORM_ESP32)
        else if (((eServoOutputMode)chConfig->val.mode) == somDShot)
        {
            dshotInstances[ch]->begin(DSHOT300, false); // Set DShot protocol and bidirectional dshot bool
            dshotInstances[ch]->send_dshot_value(0);         // Set throttle low so the ESC can continue initialsation
        }
#endif
    }
    return DURATION_NEVER;
}

static int event()
{
    if (!OPT_HAS_SERVO_OUTPUT || connectionState == disconnected)
    {
        // Disconnected should come after failsafe on the RX,
        // so it is safe to shut down when disconnected
        return DURATION_NEVER;
    }
    else if (connectionState == wifiUpdate)
    {
#ifndef SIMPLE_FC
        for (int ch = 0; ch < GPIO_PIN_PWM_OUTPUTS_COUNT; ++ch)
        {
            if (pwmChannels[ch] != -1)
            {
                PWM.release(pwmChannels[ch]);
                pwmChannels[ch] = -1;
            }
#if defined(PLATFORM_ESP32)
            if (dshotInstances[ch] != nullptr)
            {
                delete dshotInstances[ch];
                dshotInstances[ch] = nullptr;
            }
#endif
            servoPins[ch] = UNDEF_PIN;
        }
#endif
        return DURATION_NEVER;
    }
    return DURATION_IMMEDIATELY;
}

static int timeout()
{
    servosUpdate(millis());
    return DURATION_IMMEDIATELY;
}

device_t ServoOut_device = {
    .initialize = initialize,
    .start = start,
    .event = event,
    .timeout = timeout,
};

#endif
