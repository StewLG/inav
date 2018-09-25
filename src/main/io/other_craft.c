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
 * along with Cleanflight.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <stdbool.h>
#include <stdint.h>
#include <stddef.h>

#include "config/parameter_group.h"
#include "config/parameter_group_ids.h"

#include "io/other_craft.h"
#include "io/osd.h"

PG_REGISTER_WITH_RESET_TEMPLATE(otherCraftConfig_t, otherCraftConfig, PG_OTHER_CRAFT_CONFIG, 0);

PG_RESET_TEMPLATE(otherCraftConfig_t, otherCraftConfig,
    // Don't send craft positions unless we are explictly told to over MSP
    .msp_should_send_craft_positions = false,
    // Default stale interval (craft marked as stale on OSD map)
    .stale_interval_in_milliseconds = 4000,
    // Default expire interval (craft removed entirely from OSD map)
    .expire_interval_in_milliseconds = 60000,
);

// How many Other Crafts are actually being tracked?
uint8_t otherCraftCount = 0;

// Array of information about other Crafts we are actively tracking
otherCraftPosition_t otherCraftsToTrack[MAX_OTHER_CRAFTS_TO_TRACK];

// Update a Craft position
void updateCraftPosition(otherCraftPosition_t * pUpdatedCraftPosition)
{
    // See if this Craft is already being tracked.
    otherCraftPosition_t * pExistingCraftPosition = getCraftPositionByUid(pUpdatedCraftPosition->UID_0, pUpdatedCraftPosition->UID_1, pUpdatedCraftPosition->UID_2);
    if (pExistingCraftPosition != NULL) {
        // If it is already being tracked, update it.
        *pExistingCraftPosition = *pUpdatedCraftPosition;
    }
    // We aren't already storing this Craft Position
    else {
        // We still have at least one spare slot for a CraftPosition available
        if (otherCraftCount < MAX_OTHER_CRAFTS_TO_TRACK) {
            // Store it in the next available spare slot
            otherCraftsToTrack[otherCraftCount] = *pUpdatedCraftPosition;
            otherCraftCount++;
        }
        // All the CraftPosition slots are filled
        else {
            // Find the oldest (least current) CraftPosition and replace with this one,
            // which is presumed to be newly arrived. This privledges new Crafts over
            // old Crafts, but if we actually have > MAX_OTHER_CRAFTS_TO_TRACK Crafts that are actively in flight,
            // this could cause aggravating displays as Crafts fight for space in the
            // array as their competing position messages arrived. Something to think about more.
            otherCraftPosition_t * pOldestCraftPosition = findOldestCraftPosition();
            *pOldestCraftPosition = *pUpdatedCraftPosition;
        }
    }
}

// Get the stored CraftPosition with the given UID, or NULL if not found.
otherCraftPosition_t * getCraftPositionByUid(uint32_t UID_0, uint32_t UID_1, uint32_t UID_2)
{
    for (int i = 0; i < otherCraftCount; i++) {
        if (otherCraftsToTrack[i].UID_0 ==  UID_0 &&
            otherCraftsToTrack[i].UID_1 ==  UID_1 &&
            otherCraftsToTrack[i].UID_2 ==  UID_2)
        {
            return &otherCraftsToTrack[i];
        }
    }
    return NULL;
}

// Get the stored CraftPosition with the oldest timestamp, or NULL if no stored CraftPositions.
otherCraftPosition_t * findOldestCraftPosition()
{
    // Earliest Craft Position found so far
    otherCraftPosition_t * pOldestCraftPosition = NULL;

    for (int i = 0; i < otherCraftCount; i++) {
        if (pOldestCraftPosition == NULL || otherCraftsToTrack[i].timeInMillsecondsLastUpdated < pOldestCraftPosition->timeInMillsecondsLastUpdated) {
            pOldestCraftPosition = &otherCraftsToTrack[i];
        }
    }
    return pOldestCraftPosition;
}

// Remove the otherCraftPosition stored at the current index, pack the array
void removeOtherCraft(int craftIndex)
{
	// Only remove the otherCraftPosition if there's actually something stored there
	if (craftIndex < otherCraftCount) {
		// First, clear display this craft has (if any)
		//eraseMapElementFromDisplay(craftIndex);
		// Move subsequent entries down 
		for (int i = craftIndex; i < otherCraftCount; i++) {
			// (but only if there is a subsequent entry to move)
			if (i != (otherCraftCount-1)) {
				otherCraftsToTrack[i] = otherCraftsToTrack[i+1];	
			}
		}
		// Decrease the CraftCount to reflect the fact that we've removed one
		otherCraftCount--;	
	}

}


