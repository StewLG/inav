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

#include <stdbool.h>
#include <stdint.h>

#include "config/parameter_group.h"
#include "fc/config.h"
#include "io/gps.h"

// To start, just store top 5 craft. (I'm unsure about dynamic allocations in such a cramped
// memory environment. Guidance from other devs would be welcome. -- SLG)
#define MAX_OTHER_CRAFTS_TO_TRACK 5

typedef struct otherCraftPosition_s {
    // UID for the other Flight Controller
    uint32_t UID_0;
    uint32_t UID_1;
    uint32_t UID_2;
    // From RawGPS
    uint8_t FixType;
    uint8_t NumSat;
    // Lat, Long, Height (altitude)
    gpsLocation_t LLH;
    uint16_t Speed;
    uint16_t GroundCourseInDecidegrees;
    // Name of the other Craft
    uint8_t CraftName[MAX_NAME_LENGTH];
    // Timestamp (system time in milliseconds) this CraftPosition was last updated
    timeMs_t timeInMillsecondsLastUpdated;
	// This CraftPosition is stale (and should be marked accordingly as uncertain when displayed)
	bool IsStale;
} otherCraftPosition_t;


void updateCraftPosition(otherCraftPosition_t * pOtherCraftPosition);
otherCraftPosition_t * getCraftPositionByUid(uint32_t UID_0, uint32_t UID_1, uint32_t UID_2);
otherCraftPosition_t * findOldestCraftPosition();
void removeOtherCraft(int craftIndex);

typedef struct otherCraftConfig_s {
    // If true, MSP connections are welcome to send other Craft position information, if known.    
    bool msp_should_send_craft_positions; 
    // Stale interval (craft marked as stale on OSD map)
    uint32_t stale_interval_in_milliseconds;
    // Expire interval (craft removed entirely from OSD map)
    uint32_t expire_interval_in_milliseconds;
} otherCraftConfig_t;

PG_DECLARE(otherCraftConfig_t, otherCraftConfig);
