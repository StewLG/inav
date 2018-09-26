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

#pragma once

#include "common/time.h"
#include "common/vector.h"

#include "config/parameter_group.h"

#include "drivers/vcd.h"

#include "io/other_craft.h"

#include "navigation/navigation_pos_estimator_private.h"

#ifndef OSD_ALTERNATE_LAYOUT_COUNT
#define OSD_ALTERNATE_LAYOUT_COUNT 3
#endif
#define OSD_LAYOUT_COUNT (OSD_ALTERNATE_LAYOUT_COUNT + 1)

#define OSD_VISIBLE_FLAG    0x0800
#define OSD_VISIBLE(x)      ((x) & OSD_VISIBLE_FLAG)
#define OSD_POS(x,y)        ((x) | ((y) << 5))
#define OSD_X(x)            ((x) & 0x001F)
#define OSD_Y(x)            (((x) >> 5) & 0x001F)
#define OSD_POS_MAX         0x3FF
#define OSD_POS_MAX_CLI     (OSD_POS_MAX | OSD_VISIBLE_FLAG)

typedef enum {
    OSD_RSSI_VALUE,
    OSD_MAIN_BATT_VOLTAGE,
    OSD_CROSSHAIRS,
    OSD_ARTIFICIAL_HORIZON,
    OSD_HORIZON_SIDEBARS,
    OSD_ONTIME,
    OSD_FLYTIME,
    OSD_FLYMODE,
    OSD_CRAFT_NAME,
    OSD_THROTTLE_POS,
    OSD_VTX_CHANNEL,
    OSD_CURRENT_DRAW,
    OSD_MAH_DRAWN,
    OSD_GPS_SPEED,
    OSD_GPS_SATS,
    OSD_ALTITUDE,
    OSD_ROLL_PIDS,
    OSD_PITCH_PIDS,
    OSD_YAW_PIDS,
    OSD_POWER,
    OSD_GPS_LON,
    OSD_GPS_LAT,
    OSD_HOME_DIR,
    OSD_HOME_DIST,
    OSD_HEADING,
    OSD_VARIO,
    OSD_VARIO_NUM,
    OSD_AIR_SPEED,
    OSD_ONTIME_FLYTIME,
    OSD_RTC_TIME,
    OSD_MESSAGES,
    OSD_GPS_HDOP,
    OSD_MAIN_BATT_CELL_VOLTAGE,
    OSD_THROTTLE_POS_AUTO_THR,
    OSD_HEADING_GRAPH,
    OSD_EFFICIENCY_MAH_PER_KM,
    OSD_WH_DRAWN,
    OSD_BATTERY_REMAINING_CAPACITY,
    OSD_BATTERY_REMAINING_PERCENT,
    OSD_EFFICIENCY_WH_PER_KM,
    OSD_TRIP_DIST,
    OSD_ATTITUDE_PITCH,
    OSD_ATTITUDE_ROLL,
    OSD_MAP_NORTH,
    OSD_MAP_TAKEOFF,
    OSD_RADAR,
    OSD_WIND_SPEED_HORIZONTAL,
    OSD_WIND_SPEED_VERTICAL,
    OSD_REMAINING_FLIGHT_TIME_BEFORE_RTH,
    OSD_REMAINING_DISTANCE_BEFORE_RTH,
    OSD_HOME_HEADING_ERROR,
    OSD_CRUISE_HEADING_ERROR,
    OSD_CRUISE_HEADING_ADJUSTMENT,
    OSD_SAG_COMPENSATED_MAIN_BATT_VOLTAGE,
    OSD_MAIN_BATT_SAG_COMPENSATED_CELL_VOLTAGE,
    OSD_POWER_SUPPLY_IMPEDANCE,
    OSD_LEVEL_PIDS,
    OSD_POS_XY_PIDS,
    OSD_POS_Z_PIDS,
    OSD_VEL_XY_PIDS,
    OSD_VEL_Z_PIDS,
    OSD_HEADING_P,
    OSD_BOARD_ALIGN_ROLL,
    OSD_BOARD_ALIGN_PITCH,
    OSD_RC_EXPO,
    OSD_RC_YAW_EXPO,
    OSD_THROTTLE_EXPO,
    OSD_PITCH_RATE,
    OSD_ROLL_RATE,
    OSD_YAW_RATE,
    OSD_MANUAL_RC_EXPO,
    OSD_MANUAL_RC_YAW_EXPO,
    OSD_MANUAL_PITCH_RATE,
    OSD_MANUAL_ROLL_RATE,
    OSD_MANUAL_YAW_RATE,
    OSD_NAV_FW_CRUISE_THR,
    OSD_NAV_FW_PITCH2THR,
    OSD_FW_MIN_THROTTLE_DOWN_PITCH_ANGLE,
    OSD_DEBUG, // Intentionally absent from configurator and CMS. Set it from CLI.
    OSD_FW_ALT_PID_OUTPUTS,
    OSD_FW_POS_PID_OUTPUTS,
    OSD_MC_VEL_X_PID_OUTPUTS,
    OSD_MC_VEL_Y_PID_OUTPUTS,
    OSD_MC_VEL_Z_PID_OUTPUTS,
    OSD_MC_POS_XYZ_P_OUTPUTS,
    OSD_ITEM_COUNT // MUST BE LAST
} osd_items_e;

typedef enum {
    OSD_UNIT_IMPERIAL,
    OSD_UNIT_METRIC,
    OSD_UNIT_UK, // Show speed in mp/h, other values in metric
} osd_unit_e;

typedef enum {
    OSD_STATS_ENERGY_UNIT_MAH,
    OSD_STATS_ENERGY_UNIT_WH,
} osd_stats_energy_unit_e;

typedef enum {
    OSD_CROSSHAIRS_STYLE_DEFAULT,
    OSD_CROSSHAIRS_STYLE_AIRCRAFT,
} osd_crosshairs_style_e;

typedef enum {
    OSD_SIDEBAR_SCROLL_NONE,
    OSD_SIDEBAR_SCROLL_ALTITUDE,
    OSD_SIDEBAR_SCROLL_GROUND_SPEED,
    OSD_SIDEBAR_SCROLL_HOME_DISTANCE,
} osd_sidebar_scroll_e;

typedef enum {
    // Map scale zooming is absolute within the defined min/max scale osd_map_min_zoom_scale_in_centimeters - osd_map_max_zoom_scale_in_centimeters
    OSD_MAP_SCALE_ZOOM_MODE_ABSOLUTE,
    // Map scale zooming is relative to the current value of auto    
    OSD_MAP_SCALE_ZOOM_MODE_AUTO_RELATIVE,
    // Map scale zooming is relative to the current value of auto, but auto value only updated when actually in auto range
    OSD_MAP_SCALE_ZOOM_MODE_AUTO_RELATIVE_WITH_HOLD,
} osd_map_scale_zoom_mode_e;

typedef struct osdConfig_s {
    // Layouts
    uint16_t item_pos[OSD_LAYOUT_COUNT][OSD_ITEM_COUNT];

    // Alarms
    uint8_t rssi_alarm; // rssi %
    uint16_t time_alarm; // fly minutes
    uint16_t alt_alarm; // positive altitude in m
    uint16_t dist_alarm; // home distance in m
    uint16_t neg_alt_alarm; // abs(negative altitude) in m

    videoSystem_e video_system;
    uint8_t row_shiftdown;

    // Preferences
    uint8_t main_voltage_decimals;
    uint8_t ahi_reverse_roll;
    uint8_t crosshairs_style; // from osd_crosshairs_style_e
    uint8_t left_sidebar_scroll; // from osd_sidebar_scroll_e
    uint8_t right_sidebar_scroll; // from osd_sidebar_scroll_e
    uint8_t sidebar_scroll_arrows;

    uint8_t units; // from osd_unit_e
    uint8_t stats_energy_unit; // from osd_stats_energy_unit_e

    bool    estimations_wind_compensation; // use wind compensation for estimated remaining flight/distance
    uint8_t coordinate_digits;

    // Map configuration settings
    uint32_t map_min_zoom_scale_in_centimeters;
    uint32_t map_max_zoom_scale_in_centimeters;
    uint8_t map_scale_adjustment_rc_channel;
    uint16_t map_scale_adjustment_auto_range_pwm_min;
    uint16_t map_scale_adjustment_auto_range_pwm_max;
    uint8_t map_scale_zoom_mode;  // from osd_map_scale_zoom_mode_e
    uint8_t map_h_margin;
    uint8_t map_v_margin;
} osdConfig_t;

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

typedef struct osdScreenSetup_s {
    uint8_t minX;
    uint8_t maxX;
    uint8_t minY;
    uint8_t maxY;
    uint8_t midX;
    uint8_t midY;
} osdScreenSetup_t;

PG_DECLARE(osdConfig_t, osdConfig);

struct displayPort_s;
void osdInit(struct displayPort_s *osdDisplayPort);
void osdUpdate(timeUs_t currentTimeUs);
void osdStartFullRedraw(void);
// Sets a fixed OSD layout ignoring the RC input. Set it
// to -1 to disable the override.
void osdOverrideLayout(int layout);
bool osdItemIsFixed(osd_items_e item);
void eraseSingleMapElementFromDisplay(osdMapElement_t * pOsdMapElement);
void eraseAllMapElementsFromDisplay(osdMapElement_t * pOsdMapElements, int osdMapElementCount);

uint8_t osdGetRotatedArrowCharacter(int rotationInDegrees);