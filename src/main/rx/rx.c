/*
 * This file is part of Cleanflight.
 *
 * Cleanflight is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * Cleanflight is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stdlib.h>

#include <string.h>

#include "platform.h"

#include "build/build_config.h"
#include "build/debug.h"

#include "common/maths.h"
#include "common/utils.h"

#include "config/feature.h"
#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"


#include "drivers/adc.h"
#include "drivers/rx_pwm.h"
#include "drivers/rx_spi.h"
#include "drivers/serial.h"
#include "drivers/time.h"

#include "fc/config.h"
#include "fc/rc_controls.h"
#include "fc/rc_modes.h"

#include "flight/failsafe.h"

#include "io/serial.h"

#include "rx/rx.h"
#include "rx/pwm.h"
#include "rx/sbus.h"
#include "rx/spektrum.h"
#include "rx/sumd.h"
#include "rx/sumh.h"
#include "rx/msp.h"
#include "rx/xbus.h"
#include "rx/ibus.h"
#include "rx/jetiexbus.h"
#include "rx/rx_spi.h"
#include "rx/crsf.h"
#include "rx/eleres.h"
#include "rx/uib_rx.h"


//#define DEBUG_RX_SIGNAL_LOSS

const char rcChannelLetters[] = "AERT5678";

uint16_t rssi = 0;                  // range: [0;1023]
uint16_t link_quality = 0;          // range: [0;1023], like RSSI.

static bool rxSignalReceived = false;
static bool rxFlightChannelsValid = false;

static timeUs_t rxLastUpdateTimeUs = 0;
static timeUs_t rxLastValidFrameTimeUs = 0;

int16_t rcRaw[MAX_SUPPORTED_RC_CHANNEL_COUNT];     // interval [1000;2000]
int16_t rcData[MAX_SUPPORTED_RC_CHANNEL_COUNT];     // interval [1000;2000]
uint32_t rcInvalidPulsPeriod[MAX_SUPPORTED_RC_CHANNEL_COUNT];

#define MAX_INVALID_PULS_TIME    300

#define SKIP_RC_ON_SUSPEND_PERIOD 1500000           // 1.5 second period in usec (call frequency independent)
#define SKIP_RC_SAMPLES_ON_RESUME  2                // flush 2 samples to drop wrong measurements (timing independent)

rxRuntimeConfig_t rxRuntimeConfig;
static uint8_t rcSampleIndex = 0;

PG_REGISTER_WITH_RESET_TEMPLATE(rxConfig_t, rxConfig, PG_RX_CONFIG, 3);

PG_REGISTER_WITH_RESET_TEMPLATE(rssiConfig_t, rssiConfig, PG_RSSI_CONFIG, 3);

#ifndef RX_SPI_DEFAULT_PROTOCOL
#define RX_SPI_DEFAULT_PROTOCOL 0
#endif
#ifndef SERIALRX_PROVIDER
#define SERIALRX_PROVIDER 0
#endif

#ifndef DEFAULT_RX_TYPE
#define DEFAULT_RX_TYPE   RX_TYPE_NONE
#endif

#ifndef DEFAULT_RSSI_TYPE
#define DEFAULT_RSSI_TYPE   RSSI_TYPE_NONE
#endif

#ifndef DEFAULT_LINK_QUALITY_TYPE
#define DEFAULT_LINK_QUALITY_TYPE   LINK_QUALITY_TYPE_NONE
#endif

#define RX_MIDRC 1500
#define RX_MIN_USEX 885
PG_RESET_TEMPLATE(rxConfig_t, rxConfig,
    .receiverType = DEFAULT_RX_TYPE,
    .rcmap = {0, 1, 3, 2, 4, 5, 6, 7},      // Default to AETR5678 map
    .halfDuplex = 0,
    .serialrx_provider = SERIALRX_PROVIDER,
    .rx_spi_protocol = RX_SPI_DEFAULT_PROTOCOL,
    .spektrum_sat_bind = 0,
    .sbus_inversion = 1,
    .midrc = RX_MIDRC,
    .mincheck = 1100,
    .maxcheck = 1900,
    .rx_min_usec = RX_MIN_USEX,          // any of first 4 channels below this value will trigger rx loss detection
    .rx_max_usec = 2115,                 // any of first 4 channels above this value will trigger rx loss detection
    .rcSmoothing = 1,
);

PG_RESET_TEMPLATE(rssiConfig_t, rssiConfig,
    .rssiType = DEFAULT_RSSI_TYPE,
    .rssiChannel = 0,
    .rssiChannelLow = PWM_RANGE_MIN,
    .rssiChannelHigh = PWM_RANGE_MAX,
    .rssiAdcLow = RSSI_ADC_CENTIVOLT_MIN,
    .rssiAdcHigh = RSSI_ADC_CENTIVOLT_MAX
);

PG_RESET_TEMPLATE(linkQualityConfig_t, linkQualityConfig,
    .linkQualityType = DEFAULT_LINK_QUALITY_TYPE,
    .linkQualityChannel = 0,
    .linkQualityChannelLow = PWM_RANGE_MIN,
    .linkQualityChannelHigh = PWM_RANGE_MAX
);


void resetAllRxChannelRangeConfigurations(void)
{
    // set default calibration to full range and 1:1 mapping
    for (int i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
        rxChannelRangeConfigsMutable(i)->min = PWM_RANGE_MIN;
        rxChannelRangeConfigsMutable(i)->max = PWM_RANGE_MAX;
    }
}

PG_REGISTER_ARRAY_WITH_RESET_FN(rxChannelRangeConfig_t, NON_AUX_CHANNEL_COUNT, rxChannelRangeConfigs, PG_RX_CHANNEL_RANGE_CONFIG, 0);

void pgResetFn_rxChannelRangeConfigs(rxChannelRangeConfig_t *rxChannelRangeConfigs)
{
    // set default calibration to full range and 1:1 mapping
    for (int i = 0; i < NON_AUX_CHANNEL_COUNT; i++) {
        rxChannelRangeConfigs[i].min = PWM_RANGE_MIN;
        rxChannelRangeConfigs[i].max = PWM_RANGE_MAX;
    }
}

static uint16_t nullReadRawRC(const rxRuntimeConfig_t *rxRuntimeConfig, uint8_t channel)
{
    UNUSED(rxRuntimeConfig);
    UNUSED(channel);

    return PPM_RCVR_TIMEOUT;
}

static uint8_t nullFrameStatus(void)
{
    return RX_FRAME_PENDING;
}

static bool isPulseValid(uint16_t pulseDuration)
{
    return  pulseDuration >= rxConfig()->rx_min_usec &&
            pulseDuration <= rxConfig()->rx_max_usec;
}

#ifdef USE_SERIAL_RX
bool serialRxInit(const rxConfig_t *rxConfig, rxRuntimeConfig_t *rxRuntimeConfig)
{
    bool enabled = false;
    switch (rxConfig->serialrx_provider) {
#ifdef USE_SERIALRX_SPEKTRUM
    case SERIALRX_SPEKTRUM1024:
    case SERIALRX_SPEKTRUM2048:
        enabled = spektrumInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_SBUS
    case SERIALRX_SBUS:
        enabled = sbusInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_SUMD
    case SERIALRX_SUMD:
        enabled = sumdInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_SUMH
    case SERIALRX_SUMH:
        enabled = sumhInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_XBUS
    case SERIALRX_XBUS_MODE_B:
    case SERIALRX_XBUS_MODE_B_RJ01:
        enabled = xBusInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_IBUS
    case SERIALRX_IBUS:
        enabled = ibusInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_JETIEXBUS
    case SERIALRX_JETIEXBUS:
        enabled = jetiExBusInit(rxConfig, rxRuntimeConfig);
        break;
#endif
#ifdef USE_SERIALRX_CRSF
    case SERIALRX_CRSF:
        enabled = crsfRxInit(rxConfig, rxRuntimeConfig);
        break;
#endif
    default:
        enabled = false;
        break;
    }
    return enabled;
}
#endif

void rxInit(void)
{
    rxRuntimeConfig.rcReadRawFn = nullReadRawRC;
    rxRuntimeConfig.rcFrameStatusFn = nullFrameStatus;
    rxRuntimeConfig.rxSignalTimeout = DELAY_10_HZ;
    rxRuntimeConfig.requireFiltering = false;
    rcSampleIndex = 0;

    for (int i = 0; i < MAX_SUPPORTED_RC_CHANNEL_COUNT; i++) {
        rcData[i] = rxConfig()->midrc;
        rcInvalidPulsPeriod[i] = millis() + MAX_INVALID_PULS_TIME;
    }

    rcData[THROTTLE] = (feature(FEATURE_3D)) ? rxConfig()->midrc : rxConfig()->rx_min_usec;

    // Initialize ARM switch to OFF position when arming via switch is defined
    for (int i = 0; i < MAX_MODE_ACTIVATION_CONDITION_COUNT; i++) {
        if (modeActivationConditions(i)->modeId == BOXARM && IS_RANGE_USABLE(&modeActivationConditions(i)->range)) {
            // ARM switch is defined, determine an OFF value
            uint16_t value;
            if (modeActivationConditions(i)->range.startStep > 0) {
                value = MODE_STEP_TO_CHANNEL_VALUE((modeActivationConditions(i)->range.startStep - 1));
            } else {
                value = MODE_STEP_TO_CHANNEL_VALUE((modeActivationConditions(i)->range.endStep + 1));
            }
            // Initialize ARM AUX channel to OFF value
            rcData[modeActivationConditions(i)->auxChannelIndex + NON_AUX_CHANNEL_COUNT] = value;
        }
    }

    switch (rxConfig()->receiverType) {
#if defined(USE_RX_PWM) || defined(USE_RX_PPM)
        case RX_TYPE_PWM:
        case RX_TYPE_PPM:
            rxPwmInit(rxConfig(), &rxRuntimeConfig);
            break;
#endif

#ifdef USE_SERIAL_RX
        case RX_TYPE_SERIAL:
            if (!serialRxInit(rxConfig(), &rxRuntimeConfig)) {
                rxConfigMutable()->receiverType = RX_TYPE_NONE;
                rxRuntimeConfig.rcReadRawFn = nullReadRawRC;
                rxRuntimeConfig.rcFrameStatusFn = nullFrameStatus;
            }
            break;
#endif

#ifdef USE_RX_MSP
        case RX_TYPE_MSP:
            rxMspInit(rxConfig(), &rxRuntimeConfig);
            break;
#endif

#ifdef USE_RX_UIB
        case RX_TYPE_UIB:
            rxUIBInit(rxConfig(), &rxRuntimeConfig);
            break;
#endif

#ifdef USE_RX_SPI
        case RX_TYPE_SPI:
            if (!rxSpiInit(rxConfig(), &rxRuntimeConfig)) {
                rxConfigMutable()->receiverType = RX_TYPE_NONE;
                rxRuntimeConfig.rcReadRawFn = nullReadRawRC;
                rxRuntimeConfig.rcFrameStatusFn = nullFrameStatus;
            }
            break;
#endif

        case RX_TYPE_NONE:
        default:
            // Already configured for NONE
            break;
    }
}

static uint8_t calculateChannelRemapping(const uint8_t *channelMap, uint8_t channelMapEntryCount, uint8_t channelToRemap)
{
    if (channelToRemap < channelMapEntryCount) {
        return channelMap[channelToRemap];
    }
    return channelToRemap;
}

bool rxIsReceivingSignal(void)
{
    return rxSignalReceived;
}

bool rxAreFlightChannelsValid(void)
{
    return rxFlightChannelsValid;
}

void suspendRxSignal(void)
{
    failsafeOnRxSuspend();
}

void resumeRxSignal(void)
{
    failsafeOnRxResume();
}

bool rxUpdateCheck(timeUs_t currentTimeUs, timeDelta_t currentDeltaTime)
{
    UNUSED(currentDeltaTime);

    bool rxDataReceived = false;
    const uint8_t frameStatus = rxRuntimeConfig.rcFrameStatusFn();
    if (frameStatus & RX_FRAME_COMPLETE) {
        rxDataReceived = true;
        rxSignalReceived = ((frameStatus & RX_FRAME_FAILSAFE) == 0);
        rxLastValidFrameTimeUs = currentTimeUs;
    }
    else {  // RX_FRAME_PENDING
        // Check for valid signal timeout - if we are RX_FRAME_PENDING for too long assume signall loss
        if ((currentTimeUs - rxLastValidFrameTimeUs) >= rxRuntimeConfig.rxSignalTimeout) {
            rxSignalReceived = false;
        }
    }

    return rxDataReceived || ((int32_t)(currentTimeUs - rxLastUpdateTimeUs) >= 0); // data driven or 50Hz
}

#define FILTERING_SAMPLE_COUNT  5
static uint16_t applyChannelFiltering(uint8_t chan, uint16_t sample)
{
    static int16_t rcSamples[MAX_SUPPORTED_RC_CHANNEL_COUNT][FILTERING_SAMPLE_COUNT];
    static bool rxSamplesCollected = false;

    // Update the recent samples
    rcSamples[chan][rcSampleIndex % FILTERING_SAMPLE_COUNT] = sample;

    // Until we have enough data - return unfiltered samples
    if (!rxSamplesCollected) {
        if (rcSampleIndex < FILTERING_SAMPLE_COUNT) {
            return sample;
        }
        rxSamplesCollected = true;
    }

    // Assuming a step transition from 1000 -> 2000 different filters will yield the following output:
    //  No filter:              1000, 2000, 2000, 2000, 2000        - 0 samples delay
    //  3-point moving average: 1000, 1333, 1667, 2000, 2000        - 2 samples delay
    //  3-point median:         1000, 1000, 2000, 2000, 2000        - 1 sample delay
    //  5-point median:         1000, 1000, 1000, 2000, 2000        - 2 sample delay

    // For the same filters - noise rejection capabilities (2 out of 5 outliers
    //  No filter:              1000, 2000, 1000, 2000, 1000, 1000, 1000
    //  3-point MA:             1000, 1333, 1333, 1667, 1333, 1333, 1000    - noise has reduced magnitude, but spread over more samples
    //  3-point median:         1000, 1000, 1000, 2000, 1000, 1000, 1000    - high density noise is not removed
    //  5-point median:         1000, 1000, 1000, 1000, 1000, 1000, 1000    - only 3 out of 5 outlier noise will get through

    // Apply 5-point median filtering. This filter has the same delay as 3-point moving average, but better noise rejection
    return quickMedianFilter5_16(rcSamples[chan]);
}

void calculateRxChannelsAndUpdateFailsafe(timeUs_t currentTimeUs)
{
    const timeMs_t currentTimeMs = millis();
    rxLastUpdateTimeUs = currentTimeUs + DELAY_50_HZ;

    rxFlightChannelsValid = true;
    
    // Read and process channel data
    for (int channel = 0; channel < rxRuntimeConfig.channelCount; channel++) {
        const uint8_t rawChannel = calculateChannelRemapping(rxConfig()->rcmap, REMAPPABLE_CHANNEL_COUNT, channel);

        // sample the channel
        uint16_t sample = (*rxRuntimeConfig.rcReadRawFn)(&rxRuntimeConfig, rawChannel);

        // apply the rx calibration to flight channel
        if (channel < NON_AUX_CHANNEL_COUNT && sample != PPM_RCVR_TIMEOUT) {
            sample = scaleRange(sample, rxChannelRangeConfigs(channel)->min, rxChannelRangeConfigs(channel)->max, PWM_RANGE_MIN, PWM_RANGE_MAX);
            sample = MIN(MAX(PWM_PULSE_MIN, sample), PWM_PULSE_MAX);
        }

        // Store as rxRaw
        rcRaw[channel] = sample;

        // Apply invalid pulse value logic
        if (!isPulseValid(sample)) {
            sample = rcData[channel];   // hold channel, replace with old value
            if ((currentTimeMs > rcInvalidPulsPeriod[channel]) && (channel < NON_AUX_CHANNEL_COUNT)) {
                rxFlightChannelsValid = false;
            }
        } else {
            rcInvalidPulsPeriod[channel] = currentTimeMs + MAX_INVALID_PULS_TIME;
        }

        // Update rcData channel value
        if (rxRuntimeConfig.requireFiltering) {
            rcData[channel] = sample;
        } else {
            rcData[channel] = applyChannelFiltering(channel, sample);
        }
    }

    // Update failsafe
    if (rxFlightChannelsValid && rxSignalReceived) {
        failsafeOnValidDataReceived();
    } else {
        failsafeOnValidDataFailed();
    }

    rcSampleIndex++;
}

void parseRcChannels(const char *input)
{
    for (const char *c = input; *c; c++) {
        const char *s = strchr(rcChannelLetters, *c);
        if (s && (s < rcChannelLetters + MAX_MAPPABLE_RX_INPUTS))
            rxConfigMutable()->rcmap[s - rcChannelLetters] = c - input;
    }
}

#ifndef SWAP
#define SWAP(type,a,b) { type temp=(a);(a)=(b);(b)=temp; }
#endif

// Scale and constrain an rssi value to 0 to 1024 range
uint16_t scale_and_constrain_for_rssi_or_link_quality(uint32_t current_value, uint32_t low_range_value, uint32_t high_range_value)
{    
    uint32_t value_range = abs(high_range_value - low_range_value);
    if (value_range == 0) {
        // User range isn't meaningful, return 0 for RSSI (and avoid divide by zero)
        return 0;
    }
    // Automatically detect and accomodate inverted ranges
    bool range_is_inverted = (high_range_value < low_range_value);
    // Constrain to the possible range - values outside are clipped to ends 
    current_value = constrain(current_value,
                              range_is_inverted ? high_range_value : low_range_value,
                              range_is_inverted ? low_range_value : high_range_value);

    if (range_is_inverted) {
        // Swap values so we can treat them as low->high uniformly in the code that follows
        current_value = high_range_value + abs(current_value - low_range_value);
        SWAP(uint32_t, low_range_value, high_range_value);
    }

    // Scale the value down to a 0 - 1024 range
    uint32_t value_scaled = (((float)current_value - (float)low_range_value) / (float)value_range) * 1024;
    // Make absolutely sure the value is clipped to the 0 - 1024 range. This should handle things if the
    // value retrieved falls outside the user-supplied range.
    return constrain(value_scaled, 0, 1024);
}

static void updateRSSIPWM(void)
{
    int16_t pwmRssi = 0;
    int8_t rssiChannel = rssiConfig()->rssiChannel;
    // Read value of AUX channel as rssi
    pwmRssi = rcData[rssiChannel - 1];

    // Range of rawPwmRssi is [1000;2000]. rssi should be in [0;1023];
    rssi = scale_and_constrain_for_rssi_or_link_quality(pwmRssi, rssiConfig()->rssiChannelLow, rssiConfig()->rssiChannelHigh);
}

#define RSSI_ADC_SAMPLE_COUNT 16
#define RSSI_CENTIVOLT_RATIO 0.0805664063

static void updateRSSIADC(timeUs_t currentTimeUs)
{
#ifndef USE_ADC
    UNUSED(currentTimeUs);
    rssi = 0;
#else
    static uint16_t adcRssiSamples[RSSI_ADC_SAMPLE_COUNT];
    static uint16_t adcRssiSampleIndex = 0;
    static timeUs_t rssiUpdateAtUs = 0;

    if ((int32_t)(currentTimeUs - rssiUpdateAtUs) < 0) {
        return;
    }
    rssiUpdateAtUs = currentTimeUs + DELAY_50_HZ;

    adcRssiSampleIndex = (adcRssiSampleIndex + 1) % RSSI_ADC_SAMPLE_COUNT;
    adcRssiSamples[adcRssiSampleIndex] = adcGetChannel(ADC_RSSI);

    int32_t adcRssiMean = 0;
    for (int sampleIndex = 0; sampleIndex < RSSI_ADC_SAMPLE_COUNT; sampleIndex++) {
        adcRssiMean += adcRssiSamples[sampleIndex];
    }

    // Range = 0 to 330 (3.3 volts)
    int16_t rssiInCentivolts = (adcRssiMean / RSSI_ADC_SAMPLE_COUNT) * RSSI_CENTIVOLT_RATIO;
    // Constrain and convert to 0..1023
    rssi = scale_and_constrain_for_rssi_or_link_quality(rssiInCentivolts, rssiConfig()->rssiAdcLow, rssiConfig()->rssiAdcHigh);
#endif
}


static void updateRSSIeleres(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
#ifdef USE_RX_ELERES
    rssi = eleresRssi();
#else
    rssi = 0;
#endif // USE_RX_ELERES
}


static void updateRSSIMavlinkRadio()
{
    // TODO - some kind of check that we are actually using Mavlink radio?
#ifdef USE_RX_ELERES
    rssi = mavlinkRadioRssi();
#else
    rssi = 0;
#endif // USE_RX_ELERES
}





// Original updateRSSI routine

/*
void updateRSSI(timeUs_t currentTimeUs)
{
    // Read RSSI
    if (rxConfig()->rssi_channel > 0) {
        rssiUpdated = updateRSSIPWM();
    } else if (feature(FEATURE_RSSI_ADC)) {
        rssiUpdated = updateRSSIADC(currentTimeUs);
#ifdef USE_RX_ELERES
    } else if (rxConfig()->receiverType == RX_TYPE_SPI && rxConfig()->rx_spi_protocol == RFM22_ELERES) {
        rssiUpdated = updateRSSIeleres(currentTimeUs);
#endif
    }

    if (rssiUpdated) {
        // Apply RSSI inversion
        if (rxConfig()->rssiInvert) {
            rssi = 1023 - rssi;
        }

    // Apply scaling
    rssi = constrain((uint32_t)rssi * rxConfig()->rssi_scale / 100, 0, 1023);
}
*/



void updateRSSI(timeUs_t currentTimeUs)
{
    switch (rssiConfig()->rssiType) {
        case RSSI_TYPE_ADC:
            updateRSSIADC(currentTimeUs);
            break;

        case RSSI_TYPE_RC_CHANNEL:
            updateRSSIPWM();
            break;

        case RSSI_TYPE_ELERES:
            updateRSSIeleres(currentTimeUs);
            break;

        case RSSI_TYPE_MAVLINK_RADIO:
            updateRSSIMavlinkRadio();
            break;

        case RSSI_TYPE_NONE:
        default:
            // Clear both value to 0
            rssi = 0;
            break;
    }
}


// Link Quality
// ============



static void updateLinkQualitySpektrumSatFade(timeUs_t currentTimeUs)
{
    UNUSED(currentTimeUs);
#ifdef USE_SERIALRX_SPEKTRUM
    uint16_t currentSpektrumFadeCount = spektrumGetFadeCount();
    // TODO-- Constrain with values; just raw here as placeholder to start
    link_quality = currentSpektrumFadeCount;
#else
    link_quality = 0;
#endif
}

static void updateLinkQualityPWM(void)
{
    int16_t pwmLinkQuality = 0;
    int8_t linkQualityChannel = linkQualityConfig()->linkQualityChannel;
    // Read value of AUX channel as rssi
    pwmLinkQuality = rcData[linkQualityChannel - 1];

    // Range of rawPwmRssi is [1000;2000]. rssi should be in [0;1023];
    link_quality = scale_and_constrain_for_rssi_or_link_quality(pwmLinkQuality, linkQualityConfig()->linkQualityChannelLow, linkQualityConfig()->linkQualityChannelHigh);
}


void updateLinkQuality(timeUs_t currentTimeUs)
{
    // TO DO 
    currentTimeUs = currentTimeUs + 0;

    switch (linkQualityConfig()->linkQualityType) {

        case LINK_QUALITY_TYPE_RC_CHANNEL:
            updateLinkQualityPWM();
            break;

        case LINK_QUALITY_TYPE_SPEKTRUM_SAT_FADE:
            updateLinkQualitySpektrumSatFade(currentTimeUs);
            break;

        case LINK_QUALITY_TYPE_SBUS_PACKET_DROP:
            // TODO
            break;

        case LINK_QUALITY_TYPE_ELERES:
            // TODO
            break;

        case LINK_QUALITY_TYPE_NONE:
        default:
            // Clear value to 0
            link_quality = 0;
            break;
    }
}



uint16_t rxGetRefreshRate(void)
{
    return rxRuntimeConfig.rxRefreshRate;
}
