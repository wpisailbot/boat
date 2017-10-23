/*

Analyzes NMEA 2000 PGNs.

(C) 2009-2015, Kees Verruijt, Harlingen, The Netherlands.

This file is part of CANboat.

CANboat is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

CANboat is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with CANboat.  If not, see <http://www.gnu.org/licenses/>.

*/

#pragma once
#include <stdint.h>
#include <stdbool.h>

#define UINT16_OUT_OF_RANGE (MAX_UINT16 - 1)
#define UINT16_UNKNOWN (MAX_UINT16)

#ifndef ARRAY_SIZE
# define ARRAY_SIZE(x) (sizeof(x) / sizeof(x[0]))
#endif

/*
 * Notes on the NMEA 2000 packet structure
 * ---------------------------------------
 *
 * http://www.nmea.org/Assets/pgn059392.pdf tells us that:
 * - All messages shall set the reserved bit in the CAN ID field to zero on transmit.
 * - Data field reserve bits or reserve bytes shall be filled with ones. i.e. a reserve
 *   byte will be set to a hex value of FF, a single reservie bit would be set to a value of 1.
 * - Data field extra bytes shall be illed with a hex value of FF.
 * - If the PGN in a Command or Request is not recognized by the destination it shall
 *   reply with the PGN 059392 ACK or NACK message using a destination specific address.
 *
 */

/*
 * NMEA 2000 uses the 8 'data' bytes as follows:
 * data[0] is an 'order' that increments, or not (depending a bit on implementation).
 * If the size of the packet <= 7 then the data follows in data[1..7]
 * If the size of the packet > 7 then the next byte data[1] is the size of the payload
 * and data[0] is divided into 5 bits index into the fast packet, and 3 bits 'order
 * that increases.
 * This means that for 'fast packets' the first bucket (sub-packet) contains 6 payload
 * bytes and 7 for remaining. Since the max index is 31, the maximal payload is
 * 6 + 31 * 7 = 223 bytes
 */

/*
 * Some packets include a "SID", explained by Maretron as follows:
 * SID: The sequence identifier field is used to tie related PGNs together. For example,
 * the DST100 will transmit identical SIDs for Speed (PGN 128259) and Water depth
 * (128267) to indicate that the readings are linked together (i.e., the data from each
 * PGN was taken at the same time although reported at slightly different times).
 */

#define FASTPACKET_INDEX (0)
#define FASTPACKET_SIZE (1)
#define FASTPACKET_BUCKET_0_SIZE (6)
#define FASTPACKET_BUCKET_N_SIZE (7)
#define FASTPACKET_BUCKET_0_OFFSET (2)
#define FASTPACKET_BUCKET_N_OFFSET (1)
#define FASTPACKET_MAX_INDEX (0x1f)
#define FASTPACKET_MAX_SIZE (FASTPACKET_BUCKET_0_SIZE + FASTPACKET_BUCKET_N_SIZE * (FASTPACKET_MAX_INDEX - 1))

#define Pi (3.141592654)
#define RadianToDegree (360.0 / 2 / Pi)
#define BYTES(x) ((x)*(8))

#define RES_LAT_LONG_PRECISION (10000000) /* 1e7 */
#define RES_LAT_LONG (1.0e-7)
#define RES_LAT_LONG_64 (1.0e-16)

typedef struct
{
  const char * name;
  uint32_t size;     /* Size in bits. All fields are contiguous in message; use 'reserved' fields to fill in empty bits. */
# define LEN_VARIABLE (0)
  double resolution; /* Either a positive real value or one of the following RES_ special values */
# define RES_NOTUSED (0)
# define RES_RADIANS (1e-4)
# define RES_ROTATION (1e-3/32.0)
# define RES_HIRES_ROTATION ((1e-3/32.0) * 0.0001)
# define RES_ASCII (-1.0)
# define RES_LATITUDE (-2.0)
# define RES_LONGITUDE (-3.0)
# define RES_DATE (-4.0)
# define RES_TIME (-5.0)
# define RES_TEMPERATURE (-6.0)
# define RES_6BITASCII (-7.0)            /* Actually not used in N2K, only in N183 AIS */
# define RES_INTEGER (-8.0)
# define RES_LOOKUP (-9.0)
# define RES_BINARY (-10.0)
# define RES_MANUFACTURER (-11.0)
# define RES_STRING (-12.0)
# define RES_FLOAT (-13.0)
# define RES_PRESSURE (-14.0)
# define RES_STRINGLZ (-15.0)            /* ASCII string starting with length byte and terminated by zero byte */
# define RES_STRINGLAU (-16.0)           /* ASCII or UNICODE string starting with length byte and ASCII/Unicode byte */
# define RES_DECIMAL (-17.0)
# define RES_BITFIELD (-18.0)
# define RES_TEMPERATURE_HIGH (-19.0)
# define RES_TEMPERATURE_HIRES (-20.0)
# define RES_PRESSURE_HIRES (-21.0)
# define MAX_RESOLUTION_LOOKUP 21

  bool hasSign; /* Is the value signed, e.g. has both positive and negative values? */
  const char * units; /* String containing the 'Dimension' (e.g. s, h, m/s, etc.) unless it starts with , in which
                 * case it contains a set of lookup values.
                 */
  const char * description;
  int32_t offset; /* Only used for SAE J1939 values with sign; these are in Offset/Excess-K notation instead
                   * of two's complement as used by NMEA 2000.
                   * See http://en.wikipedia.org/wiki/Offset_binary
                   */
  const char * camelName; /* Filled by C, no need to set in initializers. */
} Field;

typedef struct
{
  const char * name;
  const char * resolution;
} Resolution;

static const Resolution types[MAX_RESOLUTION_LOOKUP] =
{ { "ASCII text", 0 }
, { "Latitude", 0 }
, { "Longitude", 0 }
, { "Date", "1" }
, { "Time", "0.0001" }
, { "Temperature", "0.01" }
, { "6 Bit ASCII text", 0 }
, { "Integer", "1" }
, { "Lookup table", 0 }
, { "Binary data", 0 }
, { "Manufacturer code", 0 }
, { "String with start/stop byte", 0 }
, { "IEEE Float", 0 }
, { "Pressure", 0 }
, { "ASCII string starting with length byte", 0 }
, { "ASCII or UNICODE string starting with length and control byte", 0 }
, { "Decimal encoded number", 0 }
, { "Bitfield", 0 }
, { "Temperature", "0.1" }
, { "Temperature (hires)", "0.001" }
, { "Pressure (hires)", "0.1" }
};


#define LOOKUP_INDUSTRY_CODE (",0=Global,1=Highway,2=Agriculture,3=Construction,4=Marine,5=Industrial")

#define LOOKUP_SHIP_TYPE ( \
          ",0=unavailable" \
          ",20=Wing In Ground,29=Wing In Ground (no other information)" \
          ",30=Fishing,31=Towing,32=Towing exceeds 200m or wider than 25m,33=Engaged in dredging or underwater operations,34=Engaged in diving operations" \
          ",35=Engaged in military operations,36=Sailing,37=Pleasure" \
          ",40=High speed craft,71=High speed craft carrying dangerous goods,72=High speed craft hazard cat B,73=High speed craft hazard cat C,74=High speed craft hazard cat D,79=High speed craft (no additional information)" \
          ",50=Pilot vessel,51=SAR,52=Tug,53=Port tender,54=Anti-pollution,55=Law enforcement,56=Spare,57=Spare #2,58=Medical,59=RR Resolution No.18" \
          ",60=Passenger ship,69=Passenger ship (no additional information)" \
          ",70=Cargo ship,71=Cargo ship carrying dangerous goods,72=Cargo ship hazard cat B,73=Cargo ship hazard cat C,74=Cargo ship hazard cat D,79=Cargo ship (no additional information)" \
          ",80=Tanker,81=Tanker carrying dangerous goods,82=Tanker hazard cat B,83=Tanker hazard cat C,84=Tanker hazard cat D,89=Tanker (no additional information)" \
          ",90=Other,91=Other carrying dangerous goods,92=Other hazard cat B,93=Other hazard cat C,94=Other hazard cat D,99=Other (no additional information)" \
          )

/* http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf */
#define LOOKUP_DEVICE_CLASS ( \
          ",0=Reserved for 2000 Use" \
          ",10=System tools" \
          ",20=Safety systems" \
          ",25=Internetwork device" \
          ",30=Electrical Distribution" \
          ",35=Electrical Generation" \
          ",40=Steering and Control surfaces" \
          ",50=Propulsion" \
          ",60=Navigation" \
          ",70=Communication" \
          ",75=Sensor Communication Interface" \
          ",80=Instrumentation/general systems" \
          ",85=External Environment" \
          ",90=Internal Environment" \
          ",100=Deck + cargo + fishing equipment systems" \
          ",120=Display" \
          ",125=Entertainment" \
          )

#define LOOKUP_REPEAT_INDICATOR ( \
          ",0=Initial,1=First retransmission,2=Second retransmission,3=Final retransmission" \
          )

#define LOOKUP_AIS_TRANSCEIVER ( \
          ",0=Channel A VDL reception" \
          ",1=Channel B VDL reception" \
          ",2=Channel A VDL transmission" \
          ",3=Channel B VDL transmission" \
          ",4=Own information not broadcast" \
          ",5=Reserved" \
          )

#define LOOKUP_ENGINE_INSTANCE ( ",0=Single Engine or Dual Engine Port,1=Dual Engine Starboard" )

// http://www.osukl.com/wp-content/uploads/2015/04/3155-UM.pdf
#define LOOKUP_ENGINE_STATUS_1 ( ",0=Check Engine,1=Over Temperature,2=Low Oil Pressure,3=Low Oil Level,4=Low Fuel Pressure,5=Low System Voltage,6=Low Coolant Level,7=Water Flow,8=Water In Fuel,9=Charge Indicator,10=Preheat Indicator,11=High Boost Pressure,12=Rev Limit Exceeded,13=EGR System,14=Throttle Position Sensor,15=Emergency Stop" )
#define LOOKUP_ENGINE_STATUS_2 ( ",0=Warning Level 1,1=Warning Level 2,2=Power Reduction,3=Maintenance Needed,4=Engine Comm Error,5=Sub or Secondary Throttle,6=Neutral Start Protect,7=Engine Shutting Down" )

#define LOOKUP_GEAR_STATUS ( ",0=Forward,1=Neutral,2=Reverse" )

#define LOOKUP_POSITION_ACCURACY ( ",0=Low,1=High" )

#define LOOKUP_RAIM_FLAG ( ",0=not in use,1=in use" )

#define LOOKUP_TIME_STAMP ( ",60=Not available,61=Manual input mode,62=Dead reckoning mode,63=Positioning system is inoperative" )

#define LOOKUP_GNS_AIS ( ",0=undefined,1=GPS,2=GLONASS,3=GPS+GLONASS,4=Loran-C,5=Chayka,6=integrated,7=surveyed,8=Galileo" )
#define LOOKUP_GNS ( ",0=GPS,1=GLONASS,2=GPS+GLONASS,3=GPS+SBAS/WAAS,4=GPS+SBAS/WAAS+GLONASS,5=Chayka,6=integrated,7=surveyed,8=Galileo" )

#define LOOKUP_GNS_METHOD ( ",0=no GNSS,1=GNSS fix,2=DGNSS fix,3=Precise GNSS,4=RTK Fixed Integer,5=RTK float,6=Estimated (DR) mode,7=Manual Input,8=Simulate mode" )

#define LOOKUP_GNS_INTEGRITY ( ",0=No integrity checking,1=Safe,2=Caution" )

#define LOOKUP_SYSTEM_TIME ( ",0=GPS,1=GLONASS,2=Radio Station,3=Local Cesium clock,4=Local Rubidium clock,5=Local Crystal clock" )

#define LOOKUP_MAGNETIC_VARIATION ( \
            ",0=Manual" \
            ",1=Automatic Chart"\
            ",2=Automatic Table"\
            ",3=Automatic Calculation"\
            ",4=WMM 2000"\
            ",5=WMM 2005"\
            ",6=WMM 2010"\
            ",7=WMM 2015"\
            ",8=WMM 2020"\
            )

#define LOOKUP_RESIDUAL_MODE ( ",0=Autonomous,1=Differential enhanced,2=Estimated,3=Simulator,4=Manual" )

#define LOOKUP_WIND_REFERENCE ( ",0=True (ground referenced to North),1=Magnetic (ground referenced to Magnetic North),2=Apparent,3=True (boat referenced),4=True (water referenced)" )

#define LOOKUP_WATER_REFERENCE ( ",0=Paddle wheel,1=Pitot tube,2=Doppler,3=Correlation (ultra sound),4=Electro Magnetic" )

#define LOOKUP_YES_NO ( ",0=No,1=Yes" )
#define LOOKUP_OK_WARNING ( ",0=OK,1=Warning" )

#define LOOKUP_DIRECTION_REFERENCE ( ",0=True,1=Magnetic" )

#define LOOKUP_NAV_STATUS ( \
            ",0=Under way using engine" \
            ",1=At anchor" \
            ",2=Not under command" \
            ",3=Restricted manoeuverability" \
            ",4=Constrained by her draught" \
            ",5=Moored" \
            ",6=Aground" \
            ",7=Engaged in Fishing" \
            ",8=Under way sailing" \
            ",9=Hazardous material, High Speed" \
            ",10=Hazardous material, Wing in Ground" \
            ",14=AIS-SART" )

#define LOOKUP_POWER_FACTOR ( ",0=Leading,1=Lagging,2=Error" )

#define LOOKUP_TEMPERATURE_SOURCE ( \
    ",0=Sea Temperature" \
    ",1=Outside Temperature" \
    ",2=Inside Temperature" \
    ",3=Engine Room Temperature" \
    ",4=Main Cabin Temperature" \
    ",5=Live Well Temperature" \
    ",6=Bait Well Temperature" \
    ",7=Refridgeration Temperature" \
    ",8=Heating System Temperature" \
    ",9=Dew Point Temperature" \
    ",10=Apparent Wind Chill Temperature" \
    ",11=Theoretical Wind Chill Temperature" \
    ",12=Heat Index Temperature" \
    ",13=Freezer Temperature" \
    ",14=Exhaust Gas Temperature" )

#define LOOKUP_HUMIDITY_SOURCE ( \
    ",0=Inside" \
    ",1=Outside" )

#define LOOKUP_PRESSURE_SOURCE ( \
    ",0=Atmospheric" \
    ",1=Water" \
    ",2=Steam" \
    ",3=Compressed Air" \
    ",4=Hydraulic" )

#define LOOKUP_DSC_FORMAT ( \
    ",102=Geographical area" \
    ",112=Distress" \
    ",114=Common interest" \
    ",116=All ships" \
    ",120=Individual stations" \
    ",121=Non-calling purpose" \
    ",123=Individual station automatic" )

#define LOOKUP_DSC_CATEGORY ( \
    ",100=Routine" \
    ",108=Safety" \
    ",110=Urgency" \
    ",112=Distress" )

#define LOOKUP_DSC_NATURE ( \
    ",100=Fire" \
    ",101=Flooding" \
    ",102=Collision" \
    ",103=Grounding" \
    ",104=Listing" \
    ",105=Sinking" \
    ",106=Disabled and adrift" \
    ",107=Undesignated" \
    ",108=Abandoning ship" \
    ",109=Piracy" \
    ",110=Man overboard" \
    ",112=EPIRB emission" )

#define LOOKUP_DSC_FIRST_TELECOMMAND ( \
    ",100=F3E/G3E All modes TP" \
    ",101=F3E/G3E duplex TP" \
    ",103=Polling" \
    ",104=Unable to comply" \
    ",105=End of call" \
    ",106=Data" \
    ",109=J3E TP" \
    ",110=Distress acknowledgement" \
    ",112=Distress relay" \
    ",113=F1B/J2B TTY-FEC" \
    ",115=F1B/J2B TTY-ARQ" \
    ",118=Test" \
    ",121=Ship position or location registration updating" \
    ",126=No information" )

#define LOOKUP_DSC_SECOND_TELECOMMAND ( \
    ",100=No reason given" \
    ",101=Congestion at MSC" \
    ",102=Busy" \
    ",103=Queue indication" \
    ",104=Station barred" \
    ",105=No operator available" \
    ",106=Operator temporarily unavailable" \
    ",107=Equipment disabled" \
    ",108=Unable to use proposed channel" \
    ",109=Unable to use proposed mode" \
    ",110=Ships and aircraft of States not parties to an armed conflict" \
    ",111=Medical transports" \
    ",112=Pay phone/public call office" \
    ",113=Fax/data" \
    ",126=No information" )

#define LOOKUP_DSC_EXPANSION_DATA ( \
    ",100=Enhanced position" \
    ",101=Source and datum of position" \
    ",102=SOG" \
    ",103=COG" \
    ",104=Additional station identification" \
    ",105=Enhanced geographic area" \
    ",106=Number of persons on board" )

#define ACTISENSE_BEM 0x40000 /* Actisense specific fake PGNs */

typedef struct
{
  const char     * description;
  uint32_t   pgn;
  bool       known;             /* Are we pretty sure we've got all fields with correct definitions? */
  uint32_t   size;              /* (Minimal) size of this PGN. Helps to determine fast/single frame and initial malloc */
  uint32_t   repeatingFields;   /* How many fields at the end repeat until the PGN is exhausted? */
  Field      fieldList[28];     /* Note fixed # of fields; increase if needed. RepeatingFields support means this is enough for now. */
  uint32_t   fieldCount;        /* Filled by C, no need to set in initializers. */
  char     * camelDescription;  /* Filled by C, no need to set in initializers. */
  bool       unknownPgn;        /* true = this is a catch-all for unknown PGNs */
} Pgn;

namespace {
//XXX(james): This is so wrong...
Pgn pgnList[] =
{
  /* http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf */
{ "System Time", 126992, true, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Source", 4, RES_LOOKUP, false, LOOKUP_SYSTEM_TIME, "" }
  , { "Reserved", 4, RES_BINARY, false, 0, "Reserved" }
  , { "Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970" }
  , { "Time", BYTES(4), RES_TIME, false, "s", "Seconds since midnight" }
  , { 0 }
  }
}

,
{ "Configuration Information", 126998, false, 0x2a, 0,
  { { "Station ID", BYTES(2), 1, false, 0, "" }
  , { "Station Name", BYTES(2), 1, false, 0, "" }
  , { "A", BYTES(2), 1, false, 0, "" }
  , { "Manufacturer", BYTES(36), RES_ASCII, false, 0, "" }
  , { "Installation Description #1", BYTES(2), 1, false, 0, "" }
  , { "Installation Description #2", BYTES(2), 1, false, 0, "" }
  , { 0 }
  }
}

  /************ PERIODIC DATA PGNs **************/
  /* http://www.nmea.org/Assets/july%202010%20nmea2000_v1-301_app_b_pgn_field_list.pdf */
  /* http://www.maretron.com/support/manuals/USB100UM_1.2.pdf */
  /* http://www8.garmin.com/manuals/GPSMAP4008_NMEA2000NetworkFundamentals.pdf */

  /* http://www.nmea.org/Assets/20130906%20nmea%202000%20%20man%20overboard%20notification%20%28mob%29%20pgn%20127233%20amendment.pdf */

  /* NMEA + Simrad AT10 */
  /* http://www.maretron.com/support/manuals/SSC200UM_1.7.pdf */
  /* molly_rose_E80start.kees */
,
{ "Vessel Heading", 127250, true, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Heading", BYTES(2), RES_RADIANS, false, "rad", "" }
  , { "Deviation", BYTES(2), RES_RADIANS, true, "rad", "" }
  , { "Variation", BYTES(2), RES_RADIANS, true, "rad", "" }
  , { "Reference", 2, RES_LOOKUP, false, LOOKUP_DIRECTION_REFERENCE, "" }
  , { 0 }
  }
}

  /* http://www.maretron.com/support/manuals/SSC200UM_1.7.pdf */
  /* Lengths observed from Simrad RC42 */
,
{ "Rate of Turn", 127251, true, 5, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Rate", BYTES(4), RES_HIRES_ROTATION, true, "rad/s", "" }
  , { 0 }
  }
}

,
{ "Analog Read", 0xFF01, true, 1, 0,
  { { "Val", BYTES(1), 4, false, 0, "" }
  , { "Current", BYTES(1), 1, false, 0, "" }
  , { 0 }
  }
}

,
{ "PWM Write", 0xFF02, true, 1, 0,
  { { "Winch", BYTES(1), 1, false, 0, "" } // Range 0-180
  , { "Rudder", BYTES(1), 1, false, 0, "" } // Range 0-180
  , { "Ballast", BYTES(1), 1, false, 0, "" } // Range 0-180
  , { 0 }
  }
}

// A message to give the SCAMP 8 bytes with which to do
// testing and random junk with; 2 messages for sending
// and receiving
,
{ "Debug SCAMP1", 0xFF03, true, 8, 0,
  { { "D1", BYTES(4), 1, false, 0, "" } // Raw data
  , { "D2", BYTES(4), 1, false, 0, "" } // Raw data
  , { 0 }
  }
}
,
{ "Debug SCAMP2", 0xFF04, true, 8, 0,
  { { "D1", BYTES(4), 1, false, 0, "" } // Raw data
  , { "D2", BYTES(4), 1, false, 0, "" } // Raw data
  , { 0 }
  }
}

,
{ "Attitude", 127257, true, 7, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Yaw", BYTES(2), RES_RADIANS, true, "rad", "" }
  , { "Pitch", BYTES(2), RES_RADIANS, true, "rad", "" }
  , { "Roll", BYTES(2), RES_RADIANS, true, "rad", "" }
  , { 0 }
  }
}

  /* NMEA + Simrad AT10 */
  /* http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf */
,
{ "Magnetic Variation", 127258, true, 6, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Source", 4, RES_LOOKUP, false, LOOKUP_MAGNETIC_VARIATION, "" }
  , { "Reserved", 4, RES_BINARY, false, 0, "Reserved" }
  , { "Age of service", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970" }
  , { "Variation", BYTES(2), RES_RADIANS, true, "rad", "" }
  , { 0 }
  }
}
  /* Engine group PGNs all derived PGN Numbers from */
  /* http://www.maretron.com/products/pdf/J2K100-Data_Sheet.pdf */
  /* http://www.floscan.com/html/blue/NMEA2000.php */



,
{ "Position, Rapid Update", 129025, true, 8, 0,
  { { "Latitude", BYTES(4), RES_LATITUDE, true, "deg", "" }
  , { "Longitude", BYTES(4), RES_LONGITUDE, true, "deg", "" }
  , { 0 }
  }
}

,
{ "COG & SOG, Rapid Update", 129026, true, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "COG Reference", 2, RES_LOOKUP, false, LOOKUP_DIRECTION_REFERENCE, "" }
  , { "Reserved", 6, RES_BINARY, false, 0, "Reserved" }
  , { "COG", BYTES(2), RES_RADIANS, false, "rad", "" }
  , { "SOG", BYTES(2), 0.01, false, "m/s", "" }
  , { "Reserved", BYTES(2), RES_BINARY, false, 0, "Reserved" }
  , { 0 }
  }
}

,
{ "Time & Date", 129033, true, 8, 0,
  { { "Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970" }
  , { "Time", BYTES(4), RES_TIME, false, "s", "Seconds since midnight" }
  , { "Local Offset", BYTES(2), RES_INTEGER, true, "minutes", "Minutes" }
  , { 0 }
  }
}

,
{ "Datum", 129044, true, 24, 0,
  { { "Local Datum", BYTES(4), RES_ASCII, false, 0, "defined in IHO Publication S-60, Appendices B and C."
              " First three chars are datum ID as per IHO tables."
              " Fourth char is local datum subdivision code." }
  , { "Delta Latitude", BYTES(4), RES_LATITUDE, true, "deg", "" }
  , { "Delta Longitude", BYTES(4), RES_LONGITUDE, true, "deg", "" }
  , { "Delta Altitude", BYTES(4), 1e-6, true, "m", "" }
  , { "Reference Datum", BYTES(4), RES_ASCII, false, 0, "defined in IHO Publication S-60, Appendices B and C."
              " First three chars are datum ID as per IHO tables."
              " Fourth char is local datum subdivision code." }
  , { 0 }
  }
}

  /* http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf */
  /* Haven't seen this yet (no way to send PGN 059904 yet) so lengths unknown */
,
{ "GNSS Control Status", 129538, false, 0, 0,
  { { "SV Elevation Mask", BYTES(2), 1, false, 0, "Will not use SV below this elevation" }
  , { "PDOP Mask", BYTES(2), 0.01, false, 0, "Will not report position above this PDOP" }
  , { "PDOP Switch", BYTES(2), 0.01, false, 0, "Will report 2D position above this PDOP" }
  , { "SNR Mask", BYTES(2), 0.01, false, 0, "Will not use SV below this SNR" }
  , { "GNSS Mode (desired)", 3, RES_LOOKUP, false, ",0=1D,1=2D,2=3D,3=Auto,4=Reserved,5=Reserved,6=Error", "" }
  , { "DGNSS Mode (desired)", 3, RES_LOOKUP, false, ",0=no SBAS,1=SBAS,3=SBAS", "" }
  , { "Position/Velocity Filter", 2, 1, false, 0, "" }
  , { "Max Correction Age", BYTES(2), 1, false, 0, "" }
  , { "Antenna Altitude for 2D Mode", BYTES(2), 0.01, false, "m", "" }
  , { "Use Antenna Altitude for 2D Mode", 2, RES_LOOKUP, false, ",0=use last 3D height,1=Use antenna altitude", "" }
  , { 0 }
  }
}

  /* http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf */
,
{ "GNSS DOPs", 129539, true, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Desired Mode", 3, RES_LOOKUP, false, ",0=1D,1=2D,2=3D,3=Auto,4=Reserved,5=Reserved,6=Error", "" }
  , { "Actual Mode", 3, RES_LOOKUP, false, ",0=1D,1=2D,2=3D,3=Auto,4=Reserved,5=Reserved,6=Error", "" }
  , { "Reserved", 2, RES_BINARY, false, 0, "Reserved" }
  , { "HDOP", BYTES(2), 0.01, true, 0, "Horizontal dilution of precision" }
  , { "VDOP", BYTES(2), 0.01, true, 0, "Vertical dilution of precision" }
  , { "TDOP", BYTES(2), 0.01, false, 0, "Time dilution of precision" }
  , { 0 }
  }
}

,
{ "GNSS Sats in View", 129540, true, 0xff, 7,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Mode", 2, RES_LOOKUP, false, ",3=Range residuals used to calculate position", "" }
  , { "Reserved", 6, RES_BINARY, false, 0, "Reserved" }
  , { "Sats in View", BYTES(1), 1, false, 0, "" }
  , { "PRN", BYTES(1), 1, false, 0, "" }
  , { "Elevation", BYTES(2), RES_RADIANS, false, "rad", "" }
  , { "Azimuth", BYTES(2), RES_RADIANS, false, "rad", "" }
  , { "SNR", BYTES(2), 0.01, false, "dB", "" }
  , { "Range residuals", BYTES(4), 1, true, 0, "" }
  , { "Status", 4, RES_LOOKUP, false, ",0=Not tracked,1=Tracked,2=Used,3=Not tracked+Diff,4=Tracked+Diff,5=Used+Diff", "" }
  , { "Reserved", 4, RES_BINARY, false, 0, "Reserved" }
  , { 0 }
  }
}


  /* http://askjackrabbit.typepad.com/ask_jack_rabbit/page/7/ */
,
{ "Wind Data", 130306, true, 6, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Wind Speed", BYTES(2), 0.01, false, "m/s", "" }
  , { "Wind Angle", BYTES(2), RES_RADIANS, false, "rad", "" }
  , { "Reference", 3, RES_LOOKUP, false, LOOKUP_WIND_REFERENCE, "" }
  , { 0 }
  }
}

  /* Water temperature, Transducer Measurement */
,
{ "Environmental Parameters", 130310, true, 7, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Water Temperature", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { "Outside Ambient Air Temperature", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { "Atmospheric Pressure", BYTES(2), RES_PRESSURE, false, "hPa", "" }
  , { 0 }
  }
}

,
{ "Environmental Parameters", 130311, true, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Temperature Source", 6, RES_LOOKUP, false, LOOKUP_TEMPERATURE_SOURCE, "" }
  , { "Humidity Source", 2, RES_LOOKUP, false, LOOKUP_HUMIDITY_SOURCE, "" }
  , { "Temperature", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { "Humidity", BYTES(2), 100.0/25000, true, "%", "" }
  , { "Atmospheric Pressure", BYTES(2), RES_PRESSURE, false, "hPa", "" }
  , { 0 }
  }
}

,
{ "Temperature", 130312, true, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Temperature Instance", BYTES(1), 1, false, 0, "" }
  , { "Temperature Source", BYTES(1), RES_LOOKUP, false, LOOKUP_TEMPERATURE_SOURCE, "" }
  , { "Actual Temperature", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { "Set Temperature", BYTES(2), RES_TEMPERATURE, false, "K", "" }
  , { 0 }
  }
}

,
{ "Humidity", 130313, true, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Humidity Instance", BYTES(1), 1, false, 0, "" }
  , { "Humidity Source", BYTES(1), RES_LOOKUP, false, LOOKUP_HUMIDITY_SOURCE, "" }
  , { "Actual Humidity", BYTES(2), 100.0/25000, true, "%", "" }
  , { "Set Humidity", BYTES(2), 100.0/25000, true, "%", "" }
  , { 0 }
  }
}

,
{ "Actual Pressure", 130314, false, 8, 0,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Pressure Instance", BYTES(1), 1, false, 0, "" }
  , { "Pressure Source", BYTES(1), RES_LOOKUP, false, LOOKUP_PRESSURE_SOURCE, "" }
  , { "Pressure", BYTES(4), RES_PRESSURE_HIRES, false, "dPa", "" }
  , { 0 }
  }
}

  /* http://www.maretron.com/support/manuals/GPS100UM_1.2.pdf */
,
{ "GNSS Position Data", 129029, true, 51, 3,
  { { "SID", BYTES(1), 1, false, 0, "" }
  , { "Date", BYTES(2), RES_DATE, false, "days", "Days since January 1, 1970" }
  , { "Time", BYTES(4), RES_TIME, false, "s", "Seconds since midnight" }
  , { "Latitude", BYTES(8), RES_LATITUDE, true, "deg", "" }
  , { "Longitude", BYTES(8), RES_LONGITUDE, true, "deg", "" }
  , { "Altitude", BYTES(8), 1e-6, true, "m", "Altitude referenced to WGS-84" }
  , { "GNSS type", 4, RES_LOOKUP, false, LOOKUP_GNS, "" }
  , { "Method", 4, RES_LOOKUP, false, LOOKUP_GNS_METHOD, "" }
  , { "Integrity", 2, RES_LOOKUP, false, LOOKUP_GNS_INTEGRITY, "" }
  , { "Reserved", 6, RES_BINARY, false, 0, "Reserved" }
  , { "Number of SVs", BYTES(1), 1, false, 0, "Number of satellites used in solution" }
  , { "HDOP", BYTES(2), 0.01, true, 0, "Horizontal dilution of precision" }
  , { "PDOP", BYTES(2), 0.01, true, 0, "Probable dilution of precision" }
  , { "Geoidal Separation", BYTES(2), 0.01, true, "m", "Geoidal Separation" }
  , { "Reference Stations", BYTES(1), 1, false, 0, "Number of reference stations" }
  , { "Reference Station Type", 4, RES_LOOKUP, false, LOOKUP_GNS, "" }
  , { "Reference Station ID", 12, 1, false, "" }
  , { "Age of DGNSS Corrections", BYTES(2), 0.01, false, "s", "" }
  , { 0 }
  }
}

};

size_t pgnListSize = ARRAY_SIZE(pgnList);
}
