
/*
 * This file is part of iNav.
 *
 * iNav is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * iNav is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with iNav.  If not, see <http://www.gnu.org/licenses/>.
 */

#pragma once

#include <stdint.h>

extern uint8_t otherCraftCount;
extern otherCraftPosition_t otherCraftsToTrack[MAX_OTHER_CRAFTS_TO_TRACK];

 extern navigationPosEstimator_t posEstimator;

// The maximum number of OSD elements we can display at one time.
// (Maximum nubmer of crafts + 1 for special symbol [home/other craft])
#define MAX_OSD_ELEMENTS (MAX_OTHER_CRAFTS_TO_TRACK + 1)

typedef enum {
    // Draws a home icon
    OSD_MAP_ELEMENT_DISPLAY_TYPE_HOME_ICON,
    // Draws symbol representing self-craft.
    OSD_MAP_ELEMENT_DISPLAY_TYPE_SELF_CRAFT,
    // Draws a rotated arrow corresponding to Craft heading.
    OSD_MAP_ELEMENT_DISPLAY_TYPE_OTHER_CRAFT
    // Other types are conceivable - numbered waypoints, for example?
} osd_map_element_display_type_e;

#define MAX_ADDITIONAL_POI_TEXT_LENGTH 8

typedef struct osdMapElementXYInfo {
    // Does this X,Y position fit on the actual screen?
    bool foundFittingScale;    
    // Is this Map Element eligible to be drawn?
    bool eligibleToBeDrawn;        
    // X,Y position of MapElement
    int poiX;
    int poiY;
    // Symbol character to be drawn there
    uint8_t poiSymbol;    
    // Do we have an additional string to draw?
    bool hasAdditionalString;
    // Additional characters to be drawn to right of 
    // poiSymbol (e.g. relative altitude for other craft)
    char additionalString[MAX_ADDITIONAL_POI_TEXT_LENGTH];
    // Is this element overlapping one or more elements?
    bool inOverlapSet;
    // If inOverlapSet is true, this is the index of the leftmost
    // elment in the set, which becomes the identifier for the overlap set.
    int overlapSetIndex;
} osdMapElementXYInfo_t;

typedef struct osdMapElement_s {
    osd_map_element_display_type_e osdMapElementDisplayType;
    uint32_t poiDistanceInCentimeters;
    // Absolute direction to the map element; 0 = north
    int32_t poiDirectionInCentidegrees;    
    // relative direction to the map element, from the center of the map's perspective      
    int32_t relativePoiDirectionInCentidegrees; 
    // Absolute heading of the map element; 0 = north. This is the direction the craft is facing, if relevant, otherwise 0. 
    // TODO - Remove? Not actually needed? Here in the short term for debugging at least.
    int32_t absoluteHeadingInCentidegrees;      
    // Relative heading of the map element, from the center of the map's perspective. If not relevant to the map element this is 0.
    int32_t relativeHeadingInCentidegrees;
    // Altitude in centimeters (meters * 100)
    int32_t altitudeInCentimeters; 
    float poiSin;
    float poiCos;
    // The value of the symbol/character for the on-screen OSD byte
    uint16_t drawn;
    // Length of additional string (0 if no additional string)
    uint8_t additionalStringLength;
    // Set to true when this OSD element is "stale", and should be shown to be a uncertain position in the display.
    bool displayAsStale;
} osdMapElement_t;

void eraseSingleMapElementFromDisplay(osdMapElement_t * pOsdMapElement);
void eraseAllMapElementsFromDisplay(osdMapElement_t * pOsdMapElements, int osdMapElementCount);

void osdDrawRadar(uint32_t * pUsedScale);
void osdDrawHomeMap(// Heading for the center element - what direction is up.
                           // For radar, this will be the heading of the craft. 
                           // For map mode, center is home and map up is north.                                  
                           int32_t referenceHeadingInCentidegrees,      
                           // referenceSym (if non-zero) is drawn at the upper right corner below a small
                           // arrow to indicate the map reference to the user.
                           uint8_t referenceSym,
                           uint32_t * pUsedScale);