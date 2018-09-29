//#ifdef USE_OSD

//asdfasfasfd

//#if defined(USE_GPS)

#define USE_MAX7456 // HACK - Why am I having to do this??

#include "common/maths.h"
#include "common/utils.h"

#include "drivers/display.h"
#include "drivers/max7456_symbols.h"
#include "drivers/time.h"

#include "fc/runtime_config.h"

#include "io/osd.h"
#include "io/osd_map.h"

#include "navigation/navigation.h"
#include "navigation/navigation_private.h"

#include "rx/rx.h"

// TODO: How to avoid duplication with osd.c? Can we put this in an .h file successfully?
static inline int32_t osdGetAltitude(void)
{
#if defined(USE_NAV)
    return getEstimatedActualPosition(Z);
#elif defined(USE_BARO)
    return baro.alt;
#else
    return 0;
#endif
}


// relativeHeadingInCentidegrees will only be relevant for some displayTypes. Provide if available/relevant.
static uint8_t GetMapSymbolForOsdDisplayType(osd_map_element_display_type_e osdMapElementDisplayType, int32_t relativeHeadingInCentidegrees, bool displayAsStale)
{
    switch (osdMapElementDisplayType) {
        case OSD_MAP_ELEMENT_DISPLAY_TYPE_HOME_ICON:
            return SYM_HOME;
            break;
		case OSD_MAP_ELEMENT_DISPLAY_TYPE_SELF_CRAFT:
            // this is a little up triangle
            return SYM_AIRCRAFT;
            break;
        case OSD_MAP_ELEMENT_DISPLAY_TYPE_OTHER_CRAFT:
            // If this craft is stale, draw as craft icon alternating with * to indicate
            // position is uncertain. Unfortunately charcter ? is not currently part of character set.
            // TODO: Add ? to character set and use for this purpose.
            if (displayAsStale && OSD_ALTERNATING_CHOICES(500, 2) == 0) {
                return '*';
            } else {
                // Drawing aircraft icon, rotate appropriately
                return osdGetRotatedArrowCharacter(CENTIDEGREES_TO_DEGREES(relativeHeadingInCentidegrees));
            }
			break;
        default:
            // Show something, but this indicates a problem (an unhandled type)
            return '!';
            break;
    }
}

static uint8_t GetMapSymbolForOsdMapElement(osdMapElement_t * pOsdMapElement)
{
	return GetMapSymbolForOsdDisplayType(pOsdMapElement->osdMapElementDisplayType, pOsdMapElement->relativeHeadingInCentidegrees, pOsdMapElement->displayAsStale);
}

const int OSDCharWidth = 12;
const int OSDCharHeight = 18;

// Calculates Pos X & Y for a point of interest using the given scale value.
// Puts X,Y values into poiX and poiY.
// Returns true if fits on-screen, false if not.
static bool CalculatePosXYForSingleElementUsingScale(osdScreenSetup_t * pOsdScreenSetup, osdMapElement_t * pOsdMapElement, uint32_t scale, int * pPoiX, int * pPoiY)
{
    // Calculate location of the aircraft in map
    int points = pOsdMapElement->poiDistanceInCentimeters / (float)(scale / OSDCharHeight);

    float pointsX = points * pOsdMapElement->poiSin;
    /*int*/ (*pPoiX) = pOsdScreenSetup->midX + roundf(pointsX / OSDCharWidth);
    if ((*pPoiX) < pOsdScreenSetup->minX || (*pPoiX) > pOsdScreenSetup->maxX) {
        return false;
    }

    float pointsY = points * pOsdMapElement->poiCos;
    /*int*/ (*pPoiY) = pOsdScreenSetup->midY - roundf(pointsY / OSDCharHeight);
    if ((*pPoiY) < pOsdScreenSetup->minY || (*pPoiY) > pOsdScreenSetup->maxY) {
        return false;
    }

    if ((*pPoiX) == pOsdScreenSetup->midX && (*pPoiY) == pOsdScreenSetup->midY) {
        // We're over the map center symbol, so we would be drawing
        // over it even if we increased the scale. Although during
        // actual display we'll alternate between drawing the center
        // symbol or drawing the MapElement, here we treat it as
        // if we'll always be drawing the MapElement.
    } else {
        uint8_t c;
        if (displayReadCharWithAttr(osdDisplayPort, *pPoiX, *pPoiY, &c, NULL) && c != SYM_BLANK) {
            // Something else written here. If the display doesn't support reading
			// back characters, we assume there's nothing.
            return false;
        }
    }

    // The scale fits this MapElement
    return true;
}

// Simple scale fitter
static uint32_t CalculateFittingScaleForSingleElement_NEW(osdScreenSetup_t * pOsdScreenSetup, osdMapElement_t * pOsdMapElement)
{
	int fittingXScale = (pOsdMapElement->poiDistanceInCentimeters / (pOsdScreenSetup->maxX - pOsdScreenSetup->midX)) * 2;
	int fittingYScale = (pOsdMapElement->poiDistanceInCentimeters / (pOsdScreenSetup->maxY - pOsdScreenSetup->midY)) * 2;

	return MIN(fittingXScale, fittingYScale);

	// It's possible a little padding will help when crafts are in motion to decrease scale jitter issues.
	// But I don't see any problems with this right now in dry static testing so I'm going to hold off.
}

// Clip scale so it fits between user-defined map min & max zoom scales
uint32_t clipScaleToMinMax(uint32_t scaleToClip)
{
	scaleToClip = MAX(scaleToClip, osdConfig()->map_min_zoom_scale_in_centimeters);
	scaleToClip = MIN(scaleToClip, osdConfig()->map_max_zoom_scale_in_centimeters);
	return scaleToClip;
}

// Calculate an appropriate scale to use when drawing the map
static uint32_t CalculateFittingScaleForMap(osdScreenSetup_t * pOsdScreenSetup,
                                     // Array of OSD elements other than the center element. All positions
                                     // are relative to the center.
                                     osdMapElement_t osdMapElements[],
                                     // How many OSD Map elements are actually in array
                                     int8_t osdMapElementCount,
									 // TODO: Use configurable minimum map dimension, not hardwired table?
                                     uint32_t minimumStartingMapScale)
{
    uint32_t maxScale = minimumStartingMapScale;
    for (int i = 0; i < osdMapElementCount; i++) {
        uint32_t fittingScaleForCurrentElement = CalculateFittingScaleForSingleElement_NEW(pOsdScreenSetup, &(osdMapElements[i]));
        maxScale = MAX(fittingScaleForCurrentElement, maxScale);
    }
    // The largest scale should encompass everything
    return clipScaleToMinMax(maxScale);
}

void eraseSingleMapElementFromDisplay(osdMapElement_t * pOsdMapElement)
{
    if (OSD_VISIBLE(pOsdMapElement->drawn)) {
        uint8_t osdX = OSD_X(pOsdMapElement->drawn);
        uint8_t osdY = OSD_Y(pOsdMapElement->drawn);
        // Erase mail poiSymbol
        displayWriteChar(osdDisplayPort, osdX, osdY, SYM_BLANK);
        // Erase any additional string being shown (relative altitude display for now)
        if (pOsdMapElement->additionalStringLength > 0) {
            osdX += 1;
            for (int i = 0; i < pOsdMapElement->additionalStringLength; i++) {
                displayWriteChar(osdDisplayPort, osdX + i, osdY, SYM_BLANK);
            }
        }
        pOsdMapElement->drawn = 0;
    }
}

void eraseAllMapElementsFromDisplay(osdMapElement_t * pOsdMapElements, int osdMapElementCount)
{
     // Erase all the extant drawings on the map of OSDMapElements
    for (int i = 0; i < osdMapElementCount; i++) {
        eraseSingleMapElementFromDisplay(&(pOsdMapElements[i]));
    }   
}

uint32_t getMapScaleForAutoRelativeModes(uint32_t currentActualAutoScale, 
                                         uint32_t currentAutoScaleToUseInAutoRelativeModes, 
                                         uint16_t rcChannelValue)
{
	uint32_t adjustedScale = 0;

    uint32_t minMapZoomScaleInCm = osdConfig()->map_min_zoom_scale_in_centimeters;
    uint32_t maxMapZoomScaleInCm = osdConfig()->map_max_zoom_scale_in_centimeters;

    uint16_t minMapScaleAdjustmentAutoRangePwm = osdConfig()->map_scale_adjustment_auto_range_pwm_min;
    uint16_t maxMapScaleAdjustmentAutoRangePwm = osdConfig()->map_scale_adjustment_auto_range_pwm_max;

    if (rcChannelValue < minMapScaleAdjustmentAutoRangePwm) {
        // If we are beneath the Auto range, scale the channel value to be between the minZoomScaleInCentimeters and the current (possibly hold) Auto value.
		adjustedScale = minMapZoomScaleInCm + (((currentAutoScaleToUseInAutoRelativeModes - minMapZoomScaleInCm) * (rcChannelValue - PWM_RANGE_MIN)) / (minMapScaleAdjustmentAutoRangePwm  - PWM_RANGE_MIN));
    } else if (rcChannelValue > maxMapScaleAdjustmentAutoRangePwm) {
        // If we are above the Auto range, scale the channel value to be between the current (possibly hold) Auto value and maxZoomScaleInCentimeters
		adjustedScale = currentAutoScaleToUseInAutoRelativeModes + (((maxMapZoomScaleInCm - currentAutoScaleToUseInAutoRelativeModes) * (rcChannelValue - maxMapScaleAdjustmentAutoRangePwm)) / (PWM_RANGE_MAX - maxMapScaleAdjustmentAutoRangePwm));
    } else {
		//  Just use the real auto-scale value.
		adjustedScale = currentActualAutoScale;
	}
	adjustedScale = clipScaleToMinMax(adjustedScale);
	return adjustedScale;
}

uint32_t getMapScaleForAbsoluteZoomMode(uint32_t currentActualAutoScale,
                                        uint16_t rcChannelValue)
{
    uint16_t minMapScaleAdjustmentAutoRangePwm = osdConfig()->map_scale_adjustment_auto_range_pwm_min;
    uint16_t maxMapScaleAdjustmentAutoRangePwm = osdConfig()->map_scale_adjustment_auto_range_pwm_max;

    // The size of the range in PWM that the auto-range removes from the total pwm range of the channel
    int pwmAutoRangeSize = (maxMapScaleAdjustmentAutoRangePwm - minMapScaleAdjustmentAutoRangePwm);
    // How many PWM values are devoted to user-controlled zooming?
    int totalAvailablePwmRange = (PWM_RANGE_MAX - PWM_RANGE_MIN) - pwmAutoRangeSize;
    // When calculating the ratio, we have to compensate for the auto-range, and the easiest way to do
    // this is remove it from the math. So we adjust things so that the auto-range does not have to be
    // considered in the calcuation.
    int currentPwmToUseInRatio = 0;
    if (rcChannelValue < minMapScaleAdjustmentAutoRangePwm) {
        // No need to remove the auto range if below the notch of the auto range
        currentPwmToUseInRatio = rcChannelValue - PWM_RANGE_MIN;
    } else if (rcChannelValue > maxMapScaleAdjustmentAutoRangePwm) {
        // Remove the auto range from consideration if above it
        currentPwmToUseInRatio = rcChannelValue - pwmAutoRangeSize - PWM_RANGE_MIN;
    } else {
        // Misconfigured?
        return currentActualAutoScale;
    }

	uint32_t maxZoomScaleCm = osdConfig()->map_max_zoom_scale_in_centimeters;
	uint32_t minZoomScaleCm = osdConfig()->map_min_zoom_scale_in_centimeters;

	uint32_t tmpResult = minZoomScaleCm + (((maxZoomScaleCm - minZoomScaleCm) * currentPwmToUseInRatio) / totalAvailablePwmRange);
    return tmpResult;
}

// Clip to min/max RC values. Keeps the range between 1000 and 2000 for
// simplicity.
uint16_t clipRcChannelValueToMinMax(uint16_t currentRcValue)
{
	currentRcValue = MAX(currentRcValue, PWM_RANGE_MIN);
	currentRcValue = MIN(currentRcValue, PWM_RANGE_MAX);
	return currentRcValue;
}

// Apply any relvant scale adjustment applicable from the map scale RC channel
uint32_t mapScaleAdjustmentFromZoomChannel(uint32_t currentAutoScale)
{
    uint16_t minMapScaleAdjustmentAutoRangePwm = osdConfig()->map_scale_adjustment_auto_range_pwm_min;
    uint16_t maxMapScaleAdjustmentAutoRangePwm = osdConfig()->map_scale_adjustment_auto_range_pwm_max;

    // This routine assumes it can readily be misconfigured, and rather than
    // try to anticipate all the weird edge cases, reverts to the default
    // Auto zoom scale if it can't make sense of things.

    // If there's a scale adjustment channel configured
    if (osdConfig()->map_scale_adjustment_rc_channel != 0) {

		// Used in MAP_SCALE_ZOOM_MODE_AUTO_RELATIVE_WITH_HOLD; see below
        static uint32_t storedAutoScaleForHold = 0;
        static bool hasAutoScaleBeenStoredForHold = false;

        uint16_t rcChannelValue = rcData[osdConfig()->map_scale_adjustment_rc_channel - 1];
		// Clip to min/max
		rcChannelValue = clipRcChannelValueToMinMax(rcChannelValue);

		// If valid values haven't been set up for the auto range, we'll 
		// behave as if there is no auto-range configured.

		bool validAutoRangeConfigured = minMapScaleAdjustmentAutoRangePwm >= PWM_RANGE_MIN && minMapScaleAdjustmentAutoRangePwm <= PWM_RANGE_MAX &&
										maxMapScaleAdjustmentAutoRangePwm >= PWM_RANGE_MIN && maxMapScaleAdjustmentAutoRangePwm <= PWM_RANGE_MAX &&
										minMapScaleAdjustmentAutoRangePwm < maxMapScaleAdjustmentAutoRangePwm;
        // Are we in the auto-range?
        bool channelPwmValueCurrentlyInAuto = (rcChannelValue >= minMapScaleAdjustmentAutoRangePwm && rcChannelValue <= maxMapScaleAdjustmentAutoRangePwm);
		// Use Auto value if we auto mode is configured properly and we are in the auto range in the channel.
        if (validAutoRangeConfigured && channelPwmValueCurrentlyInAuto) {
			// Clear stored hold value (relevant for hold mode)
			storedAutoScaleForHold = 0;
			hasAutoScaleBeenStoredForHold = false;
            // Use the auto value
            return currentAutoScale;
        }

        // TODO: Make sure that channelValue is really only 1000-2000, right?

        switch (osdConfig()->map_scale_zoom_mode) {
            // In the next two modes, the zoom channel works relative to the current Auto scale. The idea is that Auto scale is often doing *almost* the right
            // thing, and just needs a little correction, or that it at least provides a good starting point.
            case OSD_MAP_SCALE_ZOOM_MODE_AUTO_RELATIVE:
                // In this AUTO_RELATIVE mode, the auto scale is always reset to be whatever the current auto scale would be to encompass the
                // current map elements. This means that the zoom channel works like an offset or relative adjustment, and will zoom in and out
                // according to what the currentAutoScale does, but at a lesser or greater zoom value.
                return getMapScaleForAutoRelativeModes(currentAutoScale, currentAutoScale, rcChannelValue);
                break;
            case OSD_MAP_SCALE_ZOOM_MODE_AUTO_RELATIVE_WITH_HOLD:
            {
                // In this AUTO_RELATIVE mode, the auto scale used is the value extant at the moment the RC channel 
                // exits the auto mode. This way the user can pick a particular zoom setting and stay there as long
				// as they like, but can always return to auto to bring everything available on to the map.               
                if (hasAutoScaleBeenStoredForHold == false)
                {
                    storedAutoScaleForHold = currentAutoScale;
					hasAutoScaleBeenStoredForHold = true;
                }                    

                return getMapScaleForAutoRelativeModes(currentAutoScale, storedAutoScaleForHold, rcChannelValue);
                break;
            }
            // In this mode, the zoom channel will track linearly between minZoomScaleInCentimeters and maxZoomScaleInCentimeters, 
            // with the omission of the auto-range.
            case OSD_MAP_SCALE_ZOOM_MODE_ABSOLUTE:
            {
                return getMapScaleForAbsoluteZoomMode(currentAutoScale, rcChannelValue);
                break;
            }
            // If not configured/misconfigured, we just return current auto-scale value.
            default:
                return currentAutoScale;
        }
    }
    // Again, we just return the current Auto scale if we can't figure out what else to do.
    return currentAutoScale;
}

int getOsdMapElementMaxPosX(osdMapElementXYInfo_t * pOsdMapElement)
{
    int maxPosX = pOsdMapElement->poiX;
    if (pOsdMapElement->hasAdditionalString) {
        maxPosX += strlen(pOsdMapElement->additionalString);
    }
    return maxPosX;
}

bool doOsdMapElementsOverlap(osdMapElementXYInfo_t * pOsdMapElementOne, osdMapElementXYInfo_t * pOsdMapElementTwo)
{
    int minPosXOne = pOsdMapElementOne->poiX;
    int maxPosXOne = getOsdMapElementMaxPosX(pOsdMapElementOne);

    int minPosXTwo = pOsdMapElementTwo->poiX;
    int maxPosXTwo = getOsdMapElementMaxPosX(pOsdMapElementTwo);

    bool xRangesOverlap =  MAX(minPosXOne, minPosXTwo) <= MIN(maxPosXOne, maxPosXTwo);
    bool yOverlaps = pOsdMapElementOne->poiY == pOsdMapElementTwo->poiY;
    return xRangesOverlap && yOverlaps;
}

int getCountOfXYNeighbors(osdMapElementXYInfo_t * pOsdMapElementXYInfos, uint16_t osdMapElementXYCount, osdMapElementXYInfo_t * pOsdMapElementToCheckForNeighborsOf)
{
    int neighborCount = 0;

    // Go through all the OSD Map Elements looking for overlap of coordinates
    for (int i = 0; i < osdMapElementXYCount; i++) {        
        if (doOsdMapElementsOverlap(pOsdMapElementToCheckForNeighborsOf, &(pOsdMapElementXYInfos[i]) )) {
            neighborCount++;
        }
    }
    return neighborCount;
}

void markXYElementsAsNoLongerEligibleToDraw(osdMapElementXYInfo_t * pOsdMapElementXYInfos, uint16_t osdMapElementXYCount, osdMapElementXYInfo_t * pOsdMapElementToCheckForNeighborsOf)
{
    // Go through all the OSD Map Elements looking for given X,Y
    for (int i = 0; i < osdMapElementXYCount; i++) {
        if (doOsdMapElementsOverlap(pOsdMapElementToCheckForNeighborsOf, &(pOsdMapElementXYInfos[i]) )) {
			pOsdMapElementXYInfos[i].eligibleToBeDrawn = false;
        }
	}
}

int getNeighborIndexForMultipleXYInSameLocation(osdMapElementXYInfo_t * pOsdMapElementXYInfos, uint16_t osdMapElementXYCount, osdMapElementXYInfo_t * pOsdMapElementToCheckForNeighborsOf, int neighborCount)
{
	// We vary the interval depending on how many total elements are at a given location.
	// 2 elements = 500 ms each (blink every half second)
	// 3 elements = 300 ms each (blink every 1/3 second)
	// 4+ elements = 250 ms each (blink every 1/4 second)
	// Feedback welcome on these intervals, they are just guesses to start.
	int blinkIntervalInMilliseconds = 500;
	if (neighborCount == 2) {
		blinkIntervalInMilliseconds = 500;
	} else if (neighborCount == 3) {
		blinkIntervalInMilliseconds = 300;
	} else if (neighborCount >= 4) {
		blinkIntervalInMilliseconds = 250;
	}

	// The relative index of the neighbor that gets to be visible this time through.
	// If there are 3 possible map elements, this would be 0, 1 or 2, no matter where they actually
	// fall in the pOsdMapElementXYInfos array.
	int relativeNeighborIndex = OSD_ALTERNATING_CHOICES(blinkIntervalInMilliseconds, neighborCount);

	// Go through all the OSD MapElement XYs, find the index we want
	int neighborXYInfoIndexToDraw = 0;
	int currentNeighborIndex = 0;
    for (int i = 0; i < osdMapElementXYCount; i++) {
        if (doOsdMapElementsOverlap(pOsdMapElementToCheckForNeighborsOf, &(pOsdMapElementXYInfos[i]) )) {
			if (currentNeighborIndex == relativeNeighborIndex) {
				// This is the neighborindex to draw
				neighborXYInfoIndexToDraw = i;				
			}
            // Not at right neighbor number yet, keep looking 
			currentNeighborIndex++;			
		}
	}

	return neighborXYInfoIndexToDraw;
}

static void osdDrawMapImpl(int32_t referenceHeadingInCentidegrees, uint8_t referenceSym,  // Top right corner legend element
                            // Symbol for center (home, craft, etc.)
                            osd_map_element_display_type_e centerSymbolDisplayType,
                            // Array of OSD elements other than the center element. All positions
                            // are relative to the center.
                            osdMapElement_t * pOsdMapElements, // MAX_OSD_ELEMENTS
                            // How many OSD Map elements are actually in array
                            int8_t osdMapElementCount,
                            // Scale being used to draw map at present by the map
                            uint32_t *pMapUsedScale)

{
    // TODO: These need to be tested with several setups. We might
    // need to make them configurable. SLG - Yes, DEFINITELY needs
	// to be configurable.
    const int hMargin = 3;
    const int vMargin = 3;

    char buf[16];

    osdScreenSetup_t osdScreenSetup;

    osdScreenSetup.minX = hMargin;
    osdScreenSetup.maxX = osdDisplayPort->cols - 1 - hMargin;
    osdScreenSetup.minY = vMargin;
    osdScreenSetup.maxY = osdDisplayPort->rows - 1 - vMargin;
    osdScreenSetup.midX = osdDisplayPort->cols / 2;
    osdScreenSetup.midY = osdDisplayPort->rows / 2;

    // Fixed marks
    if (referenceSym) {
        displayWriteChar(osdDisplayPort, osdScreenSetup.maxX, osdScreenSetup.minY, SYM_DIRECTION);
        displayWriteChar(osdDisplayPort, osdScreenSetup.maxX, osdScreenSetup.minY + 1, referenceSym);
    }
    displayWriteChar(osdDisplayPort, osdScreenSetup.minX, osdScreenSetup.maxY, SYM_SCALE);

    // Maximum distance to any of the other map elements
    uint32_t maxPoiDistanceInCentimeters = 0;

    // Go through all the OSD MapElements..
    for (int i = 0; i < osdMapElementCount; i++) {
        // Pre-calculate some trig for later repeated use.
        pOsdMapElements[i].relativePoiDirectionInCentidegrees = DEGREES_TO_CENTIDEGREES(osdNormalizeAngle(CENTIDEGREES_TO_DEGREES(pOsdMapElements[i].poiDirectionInCentidegrees - referenceHeadingInCentidegrees)));

        float relativePoiAngleInRadians = DEGREES_TO_RADIANS(CENTIDEGREES_TO_DEGREES(pOsdMapElements[i].relativePoiDirectionInCentidegrees));
        pOsdMapElements[i].poiSin = sin_approx(relativePoiAngleInRadians);
        pOsdMapElements[i].poiCos = cos_approx(relativePoiAngleInRadians);
        maxPoiDistanceInCentimeters = MAX(pOsdMapElements[i].poiDistanceInCentimeters, maxPoiDistanceInCentimeters);
    }

    uint32_t initialScale;
    float scaleToUnit;
    int scaleUnitDivisor;
    char symUnscaled;
    char symScaled;
    int maxDecimals;

    switch (osdConfig()->units) {
        case OSD_UNIT_IMPERIAL:
			// TODO: Fix potential scale issues for Imperial!!!
            initialScale = 16; // 16m ~= 0.01miles
            //scaleToUnit = 100 / 1609.3440f; // scale to 0.01mi for osdFormatCentiNumber()
			scaleToUnit = 10 / 1609.3440f; // scale to 0.01mi for osdFormatCentiNumber() -- SLG
            scaleUnitDivisor = 0;			
			// Scaling should move to feet I think below a mile? But this is how I found it... -- SLG
            symUnscaled = SYM_MI;
            symScaled = SYM_MI;
            maxDecimals = 2;
            break;
        case OSD_UNIT_UK:
            FALLTHROUGH;
        case OSD_UNIT_METRIC:
            initialScale = 100; // 10m as initial scale
            //scaleToUnit = 100; // scale to cm for osdFormatCentiNumber()
			scaleToUnit = 10;    // Tscale to cm for osdFormatCentiNumber() -- SLG
            scaleUnitDivisor = 1000; // Convert to km when scale gets bigger than 999m
            symUnscaled = SYM_M;
            symScaled = SYM_KM;
            maxDecimals = 0;
            break;
    }

    uint32_t scale = initialScale;

    if (STATE(GPS_FIX)) {
		
        // Calculate scale that works for all OSD MapElements
        scale = CalculateFittingScaleForMap(&osdScreenSetup, pOsdMapElements, osdMapElementCount, initialScale);

        // Apply any adjustments to scale from the map zoom RC Channel
        scale = mapScaleAdjustmentFromZoomChannel(scale);

		// X,Y positions and desired map symbols to be drawn there.
		// (+1 to leave room for center element; see below)
		osdMapElementXYInfo_t osdMapElementXYInfos[MAX_OSD_ELEMENTS+1];

        // Go through all the OSD Map Elements, erasing prior drawings and
		// collecting info about position and what to draw next.		
        for (int i = 0; i < osdMapElementCount; i++) {
			// Erase the prior position (if drawn)            
			eraseSingleMapElementFromDisplay(&(pOsdMapElements[i]));
            // Get specifics for what to draw for this OSD Map Element (X,Y pos, symbol character, etc.)
            osdMapElementXYInfos[i].foundFittingScale = CalculatePosXYForSingleElementUsingScale(&osdScreenSetup, &(pOsdMapElements[i]), scale, &osdMapElementXYInfos[i].poiX, &osdMapElementXYInfos[i].poiY);
			// Automatically not eligible to be drawn if it doesn't fit on screen
			osdMapElementXYInfos[i].eligibleToBeDrawn = osdMapElementXYInfos[i].foundFittingScale;
			osdMapElementXYInfos[i].poiSymbol = GetMapSymbolForOsdMapElement(&(pOsdMapElements[i]));
            // Compose Relative altitude display string
            osdMapElementXYInfos[i].hasAdditionalString = pOsdMapElements[i].osdMapElementDisplayType == OSD_MAP_ELEMENT_DISPLAY_TYPE_OTHER_CRAFT &&
                                                          osdMapElementXYInfos[i].eligibleToBeDrawn;
            if (osdMapElementXYInfos[i].hasAdditionalString) {
                // Get difference in altitude between other craft and this craft
                int32_t thisCraftAltitudeInCm = osdGetAltitude();
                int32_t otherCraftAltitudeInCm = pOsdMapElements[i].altitudeInCentimeters;
                int32_t altitudeDifferenceInCm = otherCraftAltitudeInCm - thisCraftAltitudeInCm;
                osdFormatAltitudeSymbol(&(osdMapElementXYInfos[i].additionalString[0]), altitudeDifferenceInCm, false, true);
            }
            else
            {
                // Clear the string
                osdMapElementXYInfos[i].additionalString[0] = '\0';
            }
		}
		// For clarity
		int osdMapElementXYCount = osdMapElementCount;

		// Add center screen element (home icon/craft icon, depending on which type of map we are drawing)
		// We add it here since we don't need the full treatment of being an OSD MapElement, as we always
		// know exactly where it goes. We want it as part of this group so it can participate in overlap blinking.
		osdMapElementXYInfos[osdMapElementXYCount].poiX = osdScreenSetup.midX;
		osdMapElementXYInfos[osdMapElementXYCount].poiY = osdScreenSetup.midY;
        osdMapElementXYInfos[osdMapElementXYCount].foundFittingScale = true;
		osdMapElementXYInfos[osdMapElementXYCount].eligibleToBeDrawn = true;
		osdMapElementXYInfos[osdMapElementXYCount].poiSymbol = GetMapSymbolForOsdDisplayType(centerSymbolDisplayType, 0, false);
        osdMapElementXYInfos[osdMapElementXYCount].hasAdditionalString = false;
        osdMapElementXYInfos[osdMapElementXYCount].additionalString[0] = '\0';
		osdMapElementXYCount++;	

        // Go through all the OSD Map Element XYs,  
		// this time actually drawing symbols on screen. 
        for (int i = 0; i < osdMapElementXYCount; i++) {
			// Is this element able to be drawn, potentially?
			if (osdMapElementXYInfos[i].eligibleToBeDrawn) {
				// At first, assume we'll draw this map element
				int osdMapElementXYIndexToDraw = i;
				// Do any other map elements share overlapping positions on the map?				
				int neighborCount = getCountOfXYNeighbors(osdMapElementXYInfos, osdMapElementXYCount, &(osdMapElementXYInfos[i]));
				// If there is more than one symbol at a given X,Y location, we display each possible symbol alternately, with a delay,
				// so that all available symbols can be seen in turn, in a brief interval.
				if (neighborCount > 1) {
					// Here we choose which of the overlapping symbols at a single X,Y location should actually be drawn
                    // HACK - Confirm that overlap is really really the issue????
					//osdMapElementXYIndexToDraw = getNeighborIndexForMultipleXYInSameLocation(osdMapElementXYInfos, osdMapElementXYCount, &(osdMapElementXYInfos[i]), neighborCount);
                    
                    // Wait, this CRASHES ultimately??? Well, ok, I'm just pulling some random index, might be offscreen...
                    //osdMapElementXYIndexToDraw = 1;
				}
                int poiX = osdMapElementXYInfos[osdMapElementXYIndexToDraw].poiX;
                int poiY = osdMapElementXYInfos[osdMapElementXYIndexToDraw].poiY;
                uint8_t poiSymbol = osdMapElementXYInfos[osdMapElementXYIndexToDraw].poiSymbol;

                // Write character to screen
                displayWriteChar(osdDisplayPort, poiX, poiY, poiSymbol);

                // Assume 0 length until actually set
                pOsdMapElements[osdMapElementXYIndexToDraw].additionalStringLength = 0;

                // Display additional string (if applicable)
                if (osdMapElementXYInfos[osdMapElementXYIndexToDraw].hasAdditionalString) {
                    int additionalStringLength = (int)strlen(osdMapElementXYInfos[osdMapElementXYIndexToDraw].additionalString);
                    displayWriteWithAttr(osdDisplayPort, poiX + 1, poiY, osdMapElementXYInfos[osdMapElementXYIndexToDraw].additionalString, TEXT_ATTRIBUTES_NONE);
                    pOsdMapElements[osdMapElementXYIndexToDraw].additionalStringLength = additionalStringLength;                   
                }
                // Update saved location
                pOsdMapElements[osdMapElementXYIndexToDraw].drawn = OSD_POS(poiX, poiY) | OSD_VISIBLE_FLAG;
				// Mark all map elements that share this X,Y as no longer eligible to draw
				markXYElementsAsNoLongerEligibleToDraw(osdMapElementXYInfos, osdMapElementXYCount, &(osdMapElementXYInfos[osdMapElementXYIndexToDraw]) );
			}
            else
            {
                // Not eligible to be drawn
                pOsdMapElements[i].drawn = 0;
            }
		}
    }

    // Draw the used scale
    bool scaled = osdFormatCentiNumber(buf, scale * scaleToUnit, scaleUnitDivisor, maxDecimals, 2, 3);
    buf[3] = scaled ? symScaled : symUnscaled;
    buf[4] = '\0';
    displayWrite(osdDisplayPort, osdScreenSetup.minX + 1, osdScreenSetup.maxY, buf);
    *pMapUsedScale = scale;
}

// There are no guarantees that other crafts will be expired properly in all circumstances. 
// Instead it is likely we'll just lose contact with them completely. To deal with this, 
// first we show them as stale after an (configurable) interval. Then after another,
// (configurable) interval we expire them completely, and remove them from our records.
static void markCraftsAsStaleAndExpireIfNeeded()
{
	timeMs_t currentTimeInMilliseconds = millis();
	
	if (otherCraftCount > 0) {
		// Work backwards to allow removeOtherCraft to work w/o corrupting the array & index consistency
		for (int i = otherCraftCount - 1; i >= 0; i--) {
			// If fully expired, remove completely from OtherCraft array
			if (otherCraftConfig()->expire_interval_in_milliseconds > 0 &&
                currentTimeInMilliseconds - otherCraftsToTrack[i].timeInMillsecondsLastUpdated > otherCraftConfig()->expire_interval_in_milliseconds) {			
				removeOtherCraft(i);			
			// If merely stale, add stale marker
			} else if (otherCraftConfig()->stale_interval_in_milliseconds > 0 && 
                       currentTimeInMilliseconds - otherCraftsToTrack[i].timeInMillsecondsLastUpdated > otherCraftConfig()->stale_interval_in_milliseconds) {
				otherCraftsToTrack[i].IsStale = true;
			}
		}
	}
}


void setPositionAndSymbolForMapElement(osdMapElement_t * pOsdMapElement, 
									   const gpsOrigin_s * pGpsOrigin, 
								       // UNSURE What this really represents
									   const fpVector3_t * pCenterScreenFpVector,
									   const gpsLocation_t * pMapElementGpsLocation, 
									   osd_map_element_display_type_e mapElementDisplayType)
{
    // Is altitude already relative? Unsure! Subtle problem if not every pilot starts from same home altitude. TODO.
    fpVector3_t otherCraftPos;
    geoConvertGeodeticToLocal(pGpsOrigin, pMapElementGpsLocation, &otherCraftPos, GEO_ALT_RELATIVE);

    // Show with given symbol
    pOsdMapElement->osdMapElementDisplayType = mapElementDisplayType;

    // Calculate distance & bearing to other craft
    pOsdMapElement->poiDistanceInCentimeters = calculateDistanceBetweenPoints(pCenterScreenFpVector, &otherCraftPos);
    pOsdMapElement->poiDirectionInCentidegrees = calculateBearingBetweenPoints(pCenterScreenFpVector, &otherCraftPos);
}


static void osdDrawMap(uint32_t * pUsedScale, 
						const bool hasValidGpsFix, 
						// Heading for the center element - what direction is up.
						// For radar, this will be the heading of the craft. 
						// For map mode, center is home and map up is north.									 
						const int32_t referenceHeadingInCentidegrees, 		
						// referenceSym (if non-zero) is drawn at the upper right corner below a small
						// arrow to indicate the map reference to the user.
						const uint8_t referenceSym,
						//const gpsOrigin_s * pGpsOrigin, 
						// UNSURE What this really represents
						const fpVector3_t * pCenterScreenFpVector,									 
						// Type of display for the center screen icon (home, self-craft)
						osd_map_element_display_type_e centerSymbolDisplayType,
						// Should we try to display the 'special' symbol
						bool shouldShowSpecialSymbol,
						// GPS coordinates & location for the 'special' symbol.
						const gpsLocation_t * pSpecialSymbolGpsLocation,
						// Heading for the 'special' symbol
						const int32_t specialSymbolAbsoluteHeadingInCentidegrees,
						// Display type for the 'special' symbol (home, self-craft)
						const osd_map_element_display_type_e specialSymbolDisplayType)
{
    // Map Elements array
	// These are the elements we will draw on the map (Other craft, home position, waypoints, etc.)
    static int osdMapElementCount = 0;
    static osdMapElement_t osdMapElements[MAX_OSD_ELEMENTS];

	// Where the FC originally acquired GPS fix
	const gpsOrigin_s * pGpsOrigin = &posControl.gpsOrigin;

	// We erase immediately before doing stale/expiration, so that we can get the screen cleared of any
	// craft that are about to disappear.    
	eraseAllMapElementsFromDisplay(&osdMapElements[0], osdMapElementCount);

	// Mark craft as stale, or remove them from the array entirely if they have been dormant for a while.
	markCraftsAsStaleAndExpireIfNeeded();
	
	// Here we are about to rebuild the OSD Map Elements, so we reset their count.
	osdMapElementCount = 0;
	
	// First, add special symbol (home or self symbol, depending on mode), if applicable
	if (shouldShowSpecialSymbol) {
		setPositionAndSymbolForMapElement(&(osdMapElements[osdMapElementCount]), 
										  pGpsOrigin, 								       
										  pCenterScreenFpVector,
										  pSpecialSymbolGpsLocation, 
										  specialSymbolDisplayType);
		osdMapElements[osdMapElementCount].absoluteHeadingInCentidegrees = specialSymbolAbsoluteHeadingInCentidegrees;
		osdMapElements[osdMapElementCount].relativeHeadingInCentidegrees = wrap_36000(osdMapElements[osdMapElementCount].absoluteHeadingInCentidegrees - referenceHeadingInCentidegrees);
		osdMapElementCount++;
	}

    // Add in all the other crafts we know about
    for (int otherCraftIndex = 0; otherCraftIndex < otherCraftCount; otherCraftIndex++) {
        // Both sides must have a valid position before we can add the other craft to the display
        if (hasValidGpsFix && (otherCraftsToTrack[otherCraftIndex].FixType == GPS_FIX_2D || otherCraftsToTrack[otherCraftIndex].FixType == GPS_FIX_3D))
        {
			setPositionAndSymbolForMapElement(&(osdMapElements[osdMapElementCount]), 
											  pGpsOrigin, 								       
											  pCenterScreenFpVector,
											  &otherCraftsToTrack[otherCraftIndex].LLH, 
											  OSD_MAP_ELEMENT_DISPLAY_TYPE_OTHER_CRAFT);
			
            // Ground course becomes heading of the map element if this is another craft
			osdMapElements[osdMapElementCount].absoluteHeadingInCentidegrees = wrap_36000(DECIDEGREES_TO_CENTIDEGREES(otherCraftsToTrack[otherCraftIndex].GroundCourseInDecidegrees)); 		
			osdMapElements[osdMapElementCount].relativeHeadingInCentidegrees = wrap_36000(osdMapElements[osdMapElementCount].absoluteHeadingInCentidegrees - referenceHeadingInCentidegrees);
            // Keep track of the altitude for relative altitude display 
            osdMapElements[osdMapElementCount].altitudeInCentimeters = otherCraftsToTrack[otherCraftIndex].LLH.alt;
			// Display as stale if needed
			osdMapElements[osdMapElementCount].displayAsStale = otherCraftsToTrack[otherCraftIndex].IsStale;
            osdMapElementCount++;
        }
    }

	// TODO: Waypoints can go here, if we add them.
    //---------------------------------------------

    osdDrawMapImpl(referenceHeadingInCentidegrees, referenceSym, centerSymbolDisplayType, &osdMapElements[0], osdMapElementCount, pUsedScale);
}

// Radar Map
// ---------
// 
// Craft is in center of screen, objects drawn in relation to craft and its
// current direction of travel.
void osdDrawRadar(uint32_t * pUsedScale)
{
	// Reference heading in Centidegrees. 
	// This is the current heading of the craft displaying the radar.
	// Assume north until we get a valid heading available.
	uint32_t referenceHeadingInCentidegrees = 0;
	if (osdIsHeadingValid()) {
		referenceHeadingInCentidegrees = DEGREES_TO_CENTIDEGREES(osdNormalizeAngle(DECIDEGREES_TO_DEGREES(osdGetHeading())));
	}

	uint8_t referenceSym = 0;

	// Do we have a valid GPS fix?
	bool hasValidGpsFix = posControl.gpsOrigin.valid;

	// Center screen position as FpVector. Again, for radar, this is the craft's position.
	const fpVector3_t *pCenterScreenFpVector = &posEstimator.gps.pos;

	osdDrawMap(pUsedScale, 
				hasValidGpsFix, 
				referenceHeadingInCentidegrees, 
				referenceSym,
				pCenterScreenFpVector, 
				OSD_MAP_ELEMENT_DISPLAY_TYPE_SELF_CRAFT, 
				STATE(GPS_FIX_HOME),
				&GPS_home,
				0,
				OSD_MAP_ELEMENT_DISPLAY_TYPE_HOME_ICON);
}

// Home Map
// --------
// 
// Home is in center of screen, objects drawn in relation to home.
void osdDrawHomeMap(// Heading for the center element - what direction is up.
 						   // For radar, this will be the heading of the craft. 
						   // For map mode, center is home and map up is north.									 
						   int32_t referenceHeadingInCentidegrees, 		
						   // referenceSym (if non-zero) is drawn at the upper right corner below a small
						   // arrow to indicate the map reference to the user.
						   uint8_t referenceSym,
						   uint32_t * pUsedScale)
{
	// Do we have a valid GPS fix?
	bool hasValidGpsFix = posControl.gpsOrigin.valid;
	
	// Where fix was originally acquired
	const gpsOrigin_s * pGpsOrigin = &posControl.gpsOrigin;

	// Center screen position as FpVector. Again, for home map, this is the home location.
	fpVector3_t homeLocationFpVector;
	geoConvertGeodeticToLocal(pGpsOrigin, &GPS_home, &homeLocationFpVector, GEO_ALT_RELATIVE); 

	// Special symbol is our own craft, and here is its rotation
	int32_t craftAbsoluteHeadingInCentidegrees = DEGREES_TO_CENTIDEGREES(osdNormalizeAngle(DECIDEGREES_TO_DEGREES(osdGetHeading())));

	osdDrawMap(pUsedScale, 
   			   hasValidGpsFix, 
			   referenceHeadingInCentidegrees, 
			   referenceSym,
			   // Center screen 
			   &homeLocationFpVector, 
			   OSD_MAP_ELEMENT_DISPLAY_TYPE_HOME_ICON, 
			   // 'Special' symbol
			   STATE(GPS_FIX),
			   &gpsSol.llh,
			   craftAbsoluteHeadingInCentidegrees,
			   OSD_MAP_ELEMENT_DISPLAY_TYPE_SELF_CRAFT);
}

// For reference -- remove once sure


/* Draws a map with the given symbol in the center and given point of interest
 * defined by its distance in meters and direction in degrees.
 * referenceHeading indicates the up direction in the map, in degrees, while
 * referenceSym (if non-zero) is drawn at the upper right corner below a small
 * arrow to indicate the map reference to the user. The drawn argument is an
 * in-out used to store the last position where the craft was drawn to avoid
 * erasing all screen on each redraw.
 */

/*
static void osdDrawMap(int referenceHeading, uint8_t referenceSym, uint8_t centerSym,
                       uint32_t poiDistance, int16_t poiDirection, uint8_t poiSymbol,
                       uint16_t *drawn, uint32_t *usedScale)
{
    // TODO: These need to be tested with several setups. We might
    // need to make them configurable.
    const int hMargin = 1;
    const int vMargin = 1;

    // TODO: Get this from the display driver?
    const int charWidth = 12;
    const int charHeight = 18;

    char buf[16];

    uint8_t minX = hMargin;
    uint8_t maxX = osdDisplayPort->cols - 1 - hMargin;
    uint8_t minY = vMargin;
    uint8_t maxY = osdDisplayPort->rows - 1 - vMargin;
    uint8_t midX = osdDisplayPort->cols / 2;
    uint8_t midY = osdDisplayPort->rows / 2;

    // Fixed marks
    if (referenceSym) {
        displayWriteChar(osdDisplayPort, maxX, minY, SYM_DIRECTION);
        displayWriteChar(osdDisplayPort, maxX, minY + 1, referenceSym);
    }
    displayWriteChar(osdDisplayPort, minX, maxY, SYM_SCALE);
    displayWriteChar(osdDisplayPort, midX, midY, centerSym);

    // First, erase the previous drawing.
    if (OSD_VISIBLE(*drawn)) {
        displayWriteChar(osdDisplayPort, OSD_X(*drawn), OSD_Y(*drawn), SYM_BLANK);
        *drawn = 0;
    }

    uint32_t initialScale;
    float scaleToUnit;
    int scaleUnitDivisor;
    char symUnscaled;
    char symScaled;
    int maxDecimals;
    const unsigned scaleMultiplier = 2;
    // We try to reduce the scale when the POI will be around half the distance
    // between the center and the closers map edge, to avoid too much jumping
    const int scaleReductionMultiplier = MIN(midX - hMargin, midY - vMargin) / 2;

    switch (osdConfig()->units) {
        case OSD_UNIT_IMPERIAL:
            initialScale = 16; // 16m ~= 0.01miles
            scaleToUnit = 100 / 1609.3440f; // scale to 0.01mi for osdFormatCentiNumber()
            scaleUnitDivisor = 0;
            symUnscaled = SYM_MI;
            symScaled = SYM_MI;
            maxDecimals = 2;
            break;
        case OSD_UNIT_UK:
            FALLTHROUGH;
        case OSD_UNIT_METRIC:
            initialScale = 10; // 10m as initial scale
            scaleToUnit = 100; // scale to cm for osdFormatCentiNumber()
            scaleUnitDivisor = 1000; // Convert to km when scale gets bigger than 999m
            symUnscaled = SYM_M;
            symScaled = SYM_KM;
            maxDecimals = 0;
            break;
    }

    // Try to keep the same scale when getting closer until we draw over the center point
    uint32_t scale = initialScale;
    if (*usedScale) {
        scale = *usedScale;
        if (scale > initialScale && poiDistance < *usedScale * scaleReductionMultiplier) {
            scale /= scaleMultiplier;
        }
    }

    if (STATE(GPS_FIX)) {

        int directionToPoi = osdGetHeadingAngle(poiDirection - referenceHeading);
        float poiAngle = DEGREES_TO_RADIANS(directionToPoi);
        float poiSin = sin_approx(poiAngle);
        float poiCos = cos_approx(poiAngle);

        // Now start looking for a valid scale that lets us draw everything
        int ii;
        for (ii = 0; ii < 50; ii++) {
            // Calculate location of the aircraft in map
            int points = poiDistance / ((float)scale / charHeight);

            float pointsX = points * poiSin;
            int poiX = midX - roundf(pointsX / charWidth);
            if (poiX < minX || poiX > maxX) {
                scale *= scaleMultiplier;
                continue;
            }

            float pointsY = points * poiCos;
            int poiY = midY + roundf(pointsY / charHeight);
            if (poiY < minY || poiY > maxY) {
                scale *= scaleMultiplier;
                continue;
            }

            if (poiX == midX && poiY == midY) {
                // We're over the map center symbol, so we would be drawing
                // over it even if we increased the scale. Alternate between
                // drawing the center symbol or drawing the POI.
                if (centerSym != SYM_BLANK && OSD_ALTERNATING_CHOICES(1000, 2) == 0) {
                    break;
                }
            } else {

                uint8_t c;
                if (displayReadCharWithAttr(osdDisplayPort, poiX, poiY, &c, NULL) && c != SYM_BLANK) {
                    // Something else written here, increase scale. If the display doesn't support reading
                    // back characters, we assume there's nothing.
                    //
                    // If we're close to the center, decrease scale. Otherwise increase it.
                    uint8_t centerDeltaX = (maxX - minX) / (scaleMultiplier * 2);
                    uint8_t centerDeltaY = (maxY - minY) / (scaleMultiplier * 2);
                    if (poiX >= midX - centerDeltaX && poiX <= midX + centerDeltaX &&
                        poiY >= midY - centerDeltaY && poiY <= midY + centerDeltaY &&
                        scale > scaleMultiplier) {

                        scale /= scaleMultiplier;
                    } else {
                        scale *= scaleMultiplier;
                    }
                    continue;
                }
            }

            // Draw the point on the map
            if (poiSymbol == SYM_ARROW_UP) {
                // Drawing aircraft, rotate
                int mapHeading = osdGetHeadingAngle(DECIDEGREES_TO_DEGREES(osdGetHeading()) - referenceHeading);
                poiSymbol += mapHeading * 2 / 45;
            }
            displayWriteChar(osdDisplayPort, poiX, poiY, poiSymbol);

            // Update saved location
            *drawn = OSD_POS(poiX, poiY) | OSD_VISIBLE_FLAG;
            break;
        }
    }

    // Draw the used scale
    bool scaled = osdFormatCentiNumber(buf, scale * scaleToUnit, scaleUnitDivisor, maxDecimals, 2, 3);
    buf[3] = scaled ? symScaled : symUnscaled;
    buf[4] = '\0';
    displayWrite(osdDisplayPort, minX + 1, maxY, buf);
    *usedScale = scale;
}
*/


/* Draws a map with the home in the center and the craft moving around.
 * See osdDrawMap() for reference.
 */

/*
static void osdDrawHomeMap(int referenceHeading, uint8_t referenceSym, uint16_t *drawn, uint32_t *usedScale)
{
    osdDrawMap(referenceHeading, referenceSym, SYM_HOME, GPS_distanceToHome, GPS_directionToHome, SYM_ARROW_UP, drawn, usedScale);
}
*/

/* Draws a map with the aircraft in the center and the home moving around.
 * See osdDrawMap() for reference.
 */
/*
static void osdDrawRadar(uint16_t *drawn, uint32_t *usedScale)
{
    int16_t reference = DECIDEGREES_TO_DEGREES(osdGetHeading());
    int16_t poiDirection = osdGetHeadingAngle(GPS_directionToHome + 180);
    osdDrawMap(reference, 0, SYM_ARROW_UP, GPS_distanceToHome, poiDirection, SYM_HOME, drawn, usedScale);
}
*/

//#endif  // USE_OSD
//#endif  // USE_GPS