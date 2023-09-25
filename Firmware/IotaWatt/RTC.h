#pragma once

/****************************************************************************************************************************
 * 
 *  RTC - Support for the RTC.
 * 
 *  Due to unavailability of PFC8523 in Sep 2021, an alternate RTC was needed.  The St M41T81 was used as it has the same
 *  SOIC8 footprint and pinout, so was a direct electrical substitute on the existing PCB.
 *  However, while it does essentially the same thing, the details are different.
 *  Rather than add a lot of "this or that" code that accesses the RTC, this class will transparently support a 
 *  subset of methods regardless of the underlying RTC. 
 *  
 *  Previous RTC support was implemented with the Adafruit RTClib, based on the JeeLabs library. This derivative
 *  work is based on that, and copies the DateTime and TimeSpan classes directly. It is therefore perpetuated with 
 *  the same MIT license.
 *  
 * ***************************************************************************************************************************/

#include <Arduino.h>
#include <Wire.h>
class TimeSpan;

/** Registers */
#define PCF8523_ADDRESS 0x68       ///< I2C address for PCF8523
#define PCF8523_CLKOUTCONTROL 0x0F ///< Timer and CLKOUT control register
#define PCF8523_CONTROL_1 0x00     ///< Control and status register 1
#define PCF8523_CONTROL_2 0x01     ///< Control and status register 2
#define PCF8523_CONTROL_3 0x02     ///< Control and status register 3
#define PCF8523_TIMER_B_FRCTL 0x12 ///< Timer B source clock frequency control
#define PCF8523_TIMER_B_VALUE 0x13 ///< Timer B value (number clock periods)
#define PCF8523_OFFSET 0x0E        ///< Offset register
#define PCF8523_STATUSREG 0x03     ///< Status register

#define SECONDS_PER_DAY 86400L ///< 60 * 60 * 24
#define SECONDS_FROM_1970_TO_2000 946684800 ///< Unixtime for 2000-01-01 00:00:00, useful for initialization

class DateTime {
public:
  DateTime(uint32_t t = SECONDS_FROM_1970_TO_2000);
  DateTime(uint16_t year, uint8_t month, uint8_t day, uint8_t hour = 0,
           uint8_t min = 0, uint8_t sec = 0);
  DateTime(const DateTime &copy);
  DateTime(const char *date, const char *time);
  DateTime(const __FlashStringHelper *date, const __FlashStringHelper *time);
  DateTime(const char *iso8601date);
  bool isValid() const;
  char *toString(char *buffer);

  /*!
      @brief  Return the year.
      @return Year (range: 2000--2099).
  */
  uint16_t year() const { return 2000U + yOff; }
  /*!
      @brief  Return the month.
      @return Month number (1--12).
  */
  uint8_t month() const { return m; }
  /*!
      @brief  Return the day of the month.
      @return Day of the month (1--31).
  */
  uint8_t day() const { return d; }
  /*!
      @brief  Return the hour
      @return Hour (0--23).
  */
  uint8_t hour() const { return hh; }

  uint8_t twelveHour() const;
  /*!
      @brief  Return whether the time is PM.
      @return 0 if the time is AM, 1 if it's PM.
  */
  uint8_t isPM() const { return hh >= 12; }
  /*!
      @brief  Return the minute.
      @return Minute (0--59).
  */
  uint8_t minute() const { return mm; }
  /*!
      @brief  Return the second.
      @return Second (0--59).
  */
  uint8_t second() const { return ss; }

  uint8_t dayOfTheWeek() const;

  /* 32-bit times as seconds since 2000-01-01. */
  uint32_t secondstime() const;

  /* 32-bit times as seconds since 1970-01-01. */
  uint32_t unixtime(void) const;

  /*!
      Format of the ISO 8601 timestamp generated by `timestamp()`. Each
      option corresponds to a `toString()` format as follows:
  */
  enum timestampOpt {
    TIMESTAMP_FULL, //!< `YYYY-MM-DDThh:mm:ss`
    TIMESTAMP_TIME, //!< `hh:mm:ss`
    TIMESTAMP_DATE  //!< `YYYY-MM-DD`
  };
  String timestamp(timestampOpt opt = TIMESTAMP_FULL);

  DateTime operator+(const TimeSpan &span);
  DateTime operator-(const TimeSpan &span);
  TimeSpan operator-(const DateTime &right);
  bool operator<(const DateTime &right) const;

  /*!
      @brief  Test if one DateTime is greater (later) than another.
      @warning if one or both DateTime objects are invalid, returned value is
        meaningless
      @see use `isValid()` method to check if DateTime object is valid
      @param right DateTime object to compare
      @return True if the left DateTime is later than the right one,
        false otherwise
  */
  bool operator>(const DateTime &right) const { return right < *this; }

  /*!
      @brief  Test if one DateTime is less (earlier) than or equal to another
      @warning if one or both DateTime objects are invalid, returned value is
        meaningless
      @see use `isValid()` method to check if DateTime object is valid
      @param right DateTime object to compare
      @return True if the left DateTime is earlier than or equal to the
        right one, false otherwise
  */
  bool operator<=(const DateTime &right) const { return !(*this > right); }

  /*!
      @brief  Test if one DateTime is greater (later) than or equal to another
      @warning if one or both DateTime objects are invalid, returned value is
        meaningless
      @see use `isValid()` method to check if DateTime object is valid
      @param right DateTime object to compare
      @return True if the left DateTime is later than or equal to the right
        one, false otherwise
  */
  bool operator>=(const DateTime &right) const { return !(*this < right); }
  bool operator==(const DateTime &right) const;

  /*!
      @brief  Test if two DateTime objects are not equal.
      @warning if one or both DateTime objects are invalid, returned value is
        meaningless
      @see use `isValid()` method to check if DateTime object is valid
      @param right DateTime object to compare
      @return True if the two objects are not equal, false if they are
  */
  bool operator!=(const DateTime &right) const { return !(*this == right); }

protected:
  uint8_t yOff; ///< Year offset from 2000
  uint8_t m;    ///< Month 1-12
  uint8_t d;    ///< Day 1-31
  uint8_t hh;   ///< Hours 0-23
  uint8_t mm;   ///< Minutes 0-59
  uint8_t ss;   ///< Seconds 0-59
};

/**************************************************************************/
/*!
    @brief  Timespan which can represent changes in time with seconds accuracy.
*/
/**************************************************************************/
class TimeSpan {
public:
  TimeSpan(int32_t seconds = 0);
  TimeSpan(int16_t days, int8_t hours, int8_t minutes, int8_t seconds);
  TimeSpan(const TimeSpan &copy);

  /*!
      @brief  Number of days in the TimeSpan
              e.g. 4
      @return int16_t days
  */
  int16_t days() const { return _seconds / 86400L; }
  /*!
      @brief  Number of hours in the TimeSpan
              This is not the total hours, it includes the days
              e.g. 4 days, 3 hours - NOT 99 hours
      @return int8_t hours
  */
  int8_t hours() const { return _seconds / 3600 % 24; }
  /*!
      @brief  Number of minutes in the TimeSpan
              This is not the total minutes, it includes days/hours
              e.g. 4 days, 3 hours, 27 minutes
      @return int8_t minutes
  */
  int8_t minutes() const { return _seconds / 60 % 60; }
  /*!
      @brief  Number of seconds in the TimeSpan
              This is not the total seconds, it includes the days/hours/minutes
              e.g. 4 days, 3 hours, 27 minutes, 7 seconds
      @return int8_t seconds
  */
  int8_t seconds() const { return _seconds % 60; }
  /*!
      @brief  Total number of seconds in the TimeSpan, e.g. 358027
      @return int32_t seconds
  */
  int32_t totalseconds() const { return _seconds; }

  TimeSpan operator+(const TimeSpan &right);
  TimeSpan operator-(const TimeSpan &right);

protected:
  int32_t _seconds; ///< Actual TimeSpan value is stored as seconds
};

enum RTCmodel   // Value is the I2C address
{
    unknown,
    PCF8523,
    M41T81,
    MCP7940
};

#define PCF8523_ADDR 0x68
#define M41T81_ADDR 0xd0
#define MCP7940_ADDR 0x6f

class RTC {

    protected:
        TwoWire *RTCWireBus;
        RTCmodel _model = unknown;
        uint8_t _RTCaddr = MCP7940_ADDR;
        void readBytes(uint8_t memoryAddress, uint8_t len = 1);

    union MCP7940Reg {
      struct {
        // 0x00 RTCSEC
        uint8_t second : 7;
        bool start_osc : 1;
        // 0x01 RTCMIN
        uint8_t minute : 7;
        uint8_t unused_1 : 1;
        // 0x02 RTCHOUR
        uint8_t hour : 6;
        uint8_t hour_12_24 : 1;
        uint8_t unused_2 : 1;
        // 0x03 RTCWKDAY
        uint8_t weekday : 3;
        uint8_t unused_3 : 2;
        uint8_t osc_run : 1;
        uint8_t unused_3_1 : 2;
        // 0x04 RTCDATE
        uint8_t day : 6;
        uint8_t unused_4 : 2;
        // 0x05 RTCMTH
        uint8_t month : 5;
        uint8_t leap_year : 1;
        uint8_t unused_5 : 2;
        // 0x06 RTCYEAR
        uint8_t year;
        // 0x07 CONTROL
        uint8_t square_wave_freq : 2;
        bool course_trim_enable : 1;
        bool ext_osc_en : 1;
        bool alarm_0_en : 1;
        bool alarm_1_en : 1;
        bool squarewave_output_en : 1;
        bool out : 1;
        // 0x08 OSCTRIM
        int8_t trim_value; //Shifted right by 1 (so 1 == 2 clock cycles)
      } reg;
      mutable uint8_t raw[sizeof(reg)];
    } mcp7940_;

    public:
        boolean begin(TwoWire *wireInstance = &Wire);
        boolean isRunning();
        boolean lostPower();
        void resetLostPower();
        void adjust(const DateTime &dt);
        DateTime now();
        boolean lowBattery();
        void stop();
        String model();
        boolean isPCF8525();
        boolean isM41T81();
        boolean isMPC7940();

        void dumpRegs(TwoWire *wireInstance = &Wire);
};