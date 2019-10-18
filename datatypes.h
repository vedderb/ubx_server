/*
    Copyright 2019 Benjamin Vedder	benjamin@vedder.se

    This program is free software: you can redistribute it and/or modify
    it under the terms of the GNU General Public License as published by
    the Free Software Foundation, either version 3 of the License, or
    (at your option) any later version.

    This program is distributed in the hope that it will be useful,
    but WITHOUT ANY WARRANTY; without even the implied warranty of
    MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
    GNU General Public License for more details.

    You should have received a copy of the GNU General Public License
    along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#ifndef DATATYPES_H_
#define DATATYPES_H_

#include <stdint.h>
#include <stdbool.h>

#ifdef __cplusplus
extern "C++" {
#include <QObject>
}
#endif

// Orientation data
typedef struct {
    float q0;
    float q1;
    float q2;
    float q3;
    float integralFBx;
    float integralFBy;
    float integralFBz;
    float accMagP;
    int initialUpdateDone;
} ATTITUDE_INFO;

// ============== RTCM Datatypes ================== //

typedef struct {
    double t_tow;       // Time of week (GPS)
    double t_tod;       // Time of day (GLONASS)
    double t_wn;        // Week number
    int staid;          // ref station id
    bool sync;          // True if more messages are coming
    int type;           // RTCM Type
} rtcm_obs_header_t;

typedef struct {
    double P[2];        // Pseudorange observation
    double L[2];        // Carrier phase observation
    uint8_t cn0[2];     // Carrier-to-Noise density [dB Hz]
    uint8_t lock[2];    // Lock. Set to 0 when the lock has changed, 127 otherwise. TODO: is this correct?
    uint8_t prn;        // Sattelite
    uint8_t freq;       // Frequency slot (GLONASS)
    uint8_t code[2];    // Code indicator
} rtcm_obs_t;

typedef struct {
    int staid;
    double lat;
    double lon;
    double height;
    double ant_height;
} rtcm_ref_sta_pos_t;

typedef struct {
    double tgd;           // Group delay differential between L1 and L2 [s]
    double c_rs;          // Amplitude of the sine harmonic correction term to the orbit radius [m]
    double c_rc;          // Amplitude of the cosine harmonic correction term to the orbit radius [m]
    double c_uc;          // Amplitude of the cosine harmonic correction term to the argument of latitude [rad]
    double c_us;          // Amplitude of the sine harmonic correction term to the argument of latitude [rad]
    double c_ic;          // Amplitude of the cosine harmonic correction term to the angle of inclination [rad]
    double c_is;          // Amplitude of the sine harmonic correction term to the angle of inclination [rad]
    double dn;            // Mean motion difference [rad/s]
    double m0;            // Mean anomaly at reference time [radians]
    double ecc;           // Eccentricity of satellite orbit
    double sqrta;         // Square root of the semi-major axis of orbit [m^(1/2)]
    double omega0;        // Longitude of ascending node of orbit plane at weekly epoch [rad]
    double omegadot;      // Rate of right ascension [rad/s]
    double w;             // Argument of perigee [rad]
    double inc;           // Inclination [rad]
    double inc_dot;       // Inclination first derivative [rad/s]
    double af0;           // Polynomial clock correction coefficient (clock bias) [s]
    double af1;           // Polynomial clock correction coefficient (clock drift) [s/s]
    double af2;           // Polynomial clock correction coefficient (rate of clock drift) [s/s^2]
    double toe_tow;       // Time of week [s]
    uint16_t toe_wn;      // Week number [week]
    double toc_tow;       // Clock reference time of week [s]
    int sva;              // SV accuracy (URA index)
    int svh;              // SV health (0:ok)
    int code;             // GPS/QZS: code on L2, GAL/CMP: data sources
    int flag;             // GPS/QZS: L2 P data flag, CMP: nav type
    double fit;           // fit interval (h)
    uint8_t prn;          // Sattelite
    uint8_t iode;         // Issue of ephemeris data
    uint16_t iodc;        // Issue of clock data
} rtcm_ephemeris_t;

typedef struct {
    bool decode_all;
    int buffer_ptr;
    int len;
    uint8_t buffer[1100];
    rtcm_obs_header_t header;
    rtcm_obs_t obs[64];
    rtcm_ref_sta_pos_t pos;
    rtcm_ephemeris_t eph;
    void(*rx_rtcm_obs)(rtcm_obs_header_t *header, rtcm_obs_t *obs, int obs_num);
    void(*rx_rtcm_1005_1006)(rtcm_ref_sta_pos_t *pos);
    void(*rx_rtcm_1019)(rtcm_ephemeris_t *eph);
    void(*rx_rtcm)(uint8_t *data, int len, int type);
} rtcm3_state;

typedef struct {
    int id;
    float px;
    float py;
    float height;
    float dist_last;
} UWB_ANCHOR;

// ============== UBLOX Datatypes ================== //

typedef struct {
    uint16_t ref_station_id;
    uint32_t i_tow; // GPS time of week of the navigation epoch
    float pos_n; // Position north in meters
    float pos_e; // Position east in meters
    float pos_d; // Position down in meters
    float acc_n; // Accuracy north in meters
    float acc_e; // Accuracy east in meters
    float acc_d; // Accuracy down in meters
    bool fix_ok; // A valid fix
    bool diff_soln; // Differential corrections are applied
    bool rel_pos_valid; // Relative position components and accuracies valid
    int carr_soln; // fix_type 0: no fix, 1: float, 2: fix
} ubx_nav_relposned;

typedef struct {
    uint32_t i_tow; // GPS time of week of the navigation epoch
    uint32_t dur; // Passed survey-in observation time (s)
    double meanX; // Current survey-in mean position ECEF X coordinate
    double meanY; // Current survey-in mean position ECEF Y coordinate
    double meanZ; // Current survey-in mean position ECEF Z coordinate
    float meanAcc; // Current survey-in mean position accuracy
    uint32_t obs; // Number of position observations used during survey-in
    bool valid; // Survey-in position validity flag, 1 = valid, otherwise 0
    bool active; // Survey-in in progress flag, 1 = in-progress, otherwise 0
} ubx_nav_svin;

typedef struct {
    uint32_t i_tow; // GPS time of week of the navigation epoch

    /*
     * Fractional part of i_tow. (range +/-500000)
     * The precise GPS time of week in seconds is:
     * (i_tow * 1e-3) + (f_tow * 1e-9)
     */
    int32_t f_tow;

    int16_t weel; // GPS week number of the navigation epoch

    /*
     * GPSfix Type, range 0..5
     * 0x00 = No Fix
     * 0x01 = Dead Reckoning only
     * 0x02 = 2D-Fix
     * 0x03 = 3D-Fix
     * 0x04 = GPS + dead reckoning combined
     * 0x05 = Time only fix
     * 0x06..0xff: reserved
     */
    uint8_t gps_fix;

    bool gpsfixok; // Fix within limits (e.g. DOP & accuracy)
    bool diffsoln; // DGPS used
    bool wknset; // Valid GPS week number
    bool towset; // Valid GPS time of week
    double ecef_x; // ECEF X coordinate
    double ecef_y; // ECEF Y coordinate
    double ecef_z; // ECEF Z coordinate
    float p_acc; // 3D Position Accuracy Estimate
    float ecef_vx; // ECEF X velocity
    float ecef_vy; // ECEF Y velocity
    float ecef_vz; // ECEF Z velocity
    float s_acc; // Speed Accuracy Estimate
    float p_dop; // Position DOP
    uint8_t num_sv; // Number of SVs used in Nav Solution
} ubx_nav_sol;

typedef struct {
    double pr_mes;
    double cp_mes;
    float do_mes;
    uint8_t gnss_id;
    uint8_t sv_id;
    uint8_t freq_id;
    uint16_t locktime;
    uint8_t cno;
    uint8_t pr_stdev;
    uint8_t cp_stdev;
    uint8_t do_stdev;
    bool pr_valid;
    bool cp_valid;
    bool half_cyc_valid;
    bool half_cyc_sub;
} ubx_rxm_rawx_obs;

typedef struct {
    double rcv_tow;
    uint16_t week;
    int8_t leaps;
    uint8_t num_meas;
    bool leap_sec;
    bool clk_reset;
    ubx_rxm_rawx_obs obs[64];
} ubx_rxm_rawx;

typedef struct {
    uint32_t baudrate;
    bool in_rtcm3;
    bool in_rtcm2;
    bool in_nmea;
    bool in_ubx;
    bool out_rtcm3;
    bool out_nmea;
    bool out_ubx;
} ubx_cfg_prt_uart;

typedef struct {
    bool in_rtcm3;
    bool in_rtcm2;
    bool in_nmea;
    bool in_ubx;
    bool out_rtcm3;
    bool out_nmea;
    bool out_ubx;
} ubx_cfg_prt_usb;

typedef struct {
    bool lla; // Use lla instead of ecef
    int mode; // Mode. 0 = Disabled, 1 = Survey in, 2 = Fixed
    double ecefx_lat;
    double ecefy_lon;
    double ecefz_alt;
    float fixed_pos_acc; // Fixed position accuracy
    uint32_t svin_min_dur; // SVIN minimum duration (s)
    float svin_acc_limit; // SVIN accuracy limit
} ubx_cfg_tmode3;

typedef struct {
    bool clear_io_port;
    bool clear_msg_conf;
    bool clear_inf_msg;
    bool clear_nav_conf;
    bool clear_rxm_conf;
    bool clear_sen_conf;
    bool clear_rinv_conf;
    bool clear_ant_conf;
    bool clear_log_conf;
    bool clear_fts_conf;

    bool save_io_port;
    bool save_msg_conf;
    bool save_inf_msg;
    bool save_nav_conf;
    bool save_rxm_conf;
    bool save_sen_conf;
    bool save_rinv_conf;
    bool save_ant_conf;
    bool save_log_conf;
    bool save_fts_conf;

    bool load_io_port;
    bool load_msg_conf;
    bool load_inf_msg;
    bool load_nav_conf;
    bool load_rxm_conf;
    bool load_sen_conf;
    bool load_rinv_conf;
    bool load_ant_conf;
    bool load_log_conf;
    bool load_fts_conf;

    bool dev_bbr; // Battery backed RAM
    bool dev_flash; // Flash
    bool dev_eeprom; // EEPROM
    bool dev_spi_flash; // SPI flash
} ubx_cfg_cfg;

typedef struct {
    bool apply_dyn; // Apply dynamic model settings
    bool apply_min_el; // Apply minimum elevation settings
    bool apply_pos_fix_mode; // Apply fix mode settings
    bool apply_pos_mask; // Apply position mask settings
    bool apply_time_mask; // Apply time mask settings
    bool apply_static_hold_mask; // Apply static hold settings
    bool apply_dgps; // Apply DGPS settings.
    bool apply_cno; // Apply CNO threshold settings (cnoThresh, cnoThreshNumSVs).
    bool apply_utc; // Apply UTC settings

    /*
     * Dynamic platform model:
     * 0: portable
     * 2: stationary
     * 3: pedestrian
     * 4: automotive
     * 5: sea
     * 6: airborne with <1g acceleration
     * 7: airborne with <2g acceleration
     * 8: airborne with <4g acceleration
     * 9: wrist worn watch
     */
    uint8_t dyn_model;

    /*
     * Position Fixing Mode:
     * 1: 2D only
     * 2: 3D only
     * 3: auto 2D/3D
     */
    uint8_t fix_mode;

    double fixed_alt; // Fixed altitude (mean sea level) for 2D fix mode. (m)
    double fixed_alt_var; // Fixed altitude variance for 2D mode. (m^2)
    int8_t min_elev; // Minimum Elevation for a GNSS satellite to be used in NAV (deg)
    float p_dop; // Position DOP Mask to use
    float t_dop; // Time DOP Mask to use
    uint16_t p_acc; // Position Accuracy Mask (m)
    uint16_t t_acc; // Time Accuracy Mask (m)
    uint8_t static_hold_thres; // Static hold threshold (cm/s)
    uint8_t dgnss_timeout; // DGNSS (RTK) timeout (s)
    uint8_t cno_tres_num_sat; // Number of satellites required to have C/N0 above cnoThresh for a fix to be attempted
    uint8_t cno_tres; // C/N0 threshold for deciding whether to attempt a fix (dBHz)
    uint16_t static_hold_max_dist; // Static hold distance threshold (before quitting static hold) (m)

    /*
     * UTC standard to be used:
     * 0: Automatic; receiver selects based on GNSS configuration (see GNSS time bases).
     * 3: UTC as operated by the U.S. Naval Observatory (USNO); derived from GPS time
     * 6: UTC as operated by the former Soviet Union; derived from GLONASS time
     * 7: UTC as operated by the National Time Service Center, China; derived from BeiDou time
     */
    uint8_t utc_standard;
} ubx_cfg_nav5;

typedef struct {
    uint8_t tp_idx; // Timepulse selection. 0=TP1, 1=TP2
    int16_t ant_cable_delay; // Antenna cable delay in ns
    int16_t rf_group_delay; // RF group delay in ns
    uint32_t freq_period; // Frequency or time period, Hz or us
    uint32_t freq_period_lock; // Frequency or time period when locked to GNSS time, Hz or us
    uint32_t pulse_len_ratio; // Pulse length or duty cycle, us or 2^-32
    uint32_t pulse_len_ratio_lock; // Pulse length or duty cycle when locked to GNSS time, us or 2^-32
    int32_t user_config_delay; // User configurable time pulse delay, ns

    /*
     * If set enable time pulse; if pin assigned to another function, other function takes
     * precedence. Must be set for FTS variant.
     */
    bool active;

    /*
     * If set synchronize time pulse to GNSS as soon as GNSS time is valid. If not
     * set, or before GNSS time is valid use local clock. This flag is ignored by
     * the FTS product variant; in this case the receiver always locks to the best
     * available time/frequency reference (which is not necessarily GNSS).
     */
    bool lockGnssFreq;

    /*
     * If set the receiver switches between the timepulse settings given by 'freqPeriodLocked' &
     * 'pulseLenLocked' and those given by 'freqPeriod' & 'pulseLen'. The 'Locked' settings are
     * used where the receiver has an accurate sense of time. For non-FTS products, this occurs
     * when GNSS solution with a reliable time is available, but for FTS products the setting syncMode
     * field governs behavior. In all cases, the receiver only uses 'freqPeriod' & 'pulseLen' when
     * the flag is unset.
     */
    bool lockedOtherSet;

    /*
     * If set 'freqPeriodLock' and 'freqPeriod' are interpreted as frequency,
     * otherwise interpreted as period.
     */
    bool isFreq;

    /*
     * If set 'pulseLenRatioLock' and 'pulseLenRatio' interpreted as pulse
     * length, otherwise interpreted as duty cycle.
     */
    bool isLength;

    /*
     * Align pulse to top of second (period time must be integer fraction of 1s).
     * Also set 'lockGnssFreq' to use this feature.
     * This flag is ignored by the FTS product variant; it is assumed to be always set
     * (as is lockGnssFreq). Set maxSlewRate and maxPhaseCorrRate fields of CFG-SMGR to
     * 0 to disable alignment.
     */
    bool alignToTow;

    /*
     * Pulse polarity:
     * 0: falling edge at top of second
     * 1: rising edge at top of second
     */
    bool polarity;

    /*
     * Timegrid to use:
     * 0: UTC
     * 1: GPS
     * 2: GLONASS
     * 3: BeiDou
     * 4: Galileo (not supported in protocol versions less than 18)
     * This flag is only relevant if 'lockGnssFreq' and 'alignToTow' are set.
     * Note that configured GNSS time is estimated by the receiver if locked to
     * any GNSS system. If the receiver has a valid GNSS fix it will attempt to
     * steer the TP to the specified time grid even if the specified time is not
     * based on information from the constellation's satellites. To ensure timing
     * based purely on a given GNSS, restrict the supported constellations in CFG-GNSS.
     */
    uint8_t gridUtcGnss;

    /*
     * Sync Manager lock mode to use:
     *
     * 0: switch to 'freqPeriodLock' and 'pulseLenRatioLock' as soon as Sync
     * Manager has an accurate time, never switch back to 'freqPeriod' and 'pulseLenRatio'
     *
     * 1: switch to 'freqPeriodLock' and 'pulseLenRatioLock' as soon as Sync Manager has
     * an accurate time, and switch back to 'freqPeriod' and 'pulseLenRatio' as soon as
     * time gets inaccurate.
     *
     * This field is only relevant for the FTS product variant.
     * This field is only relevant if the flag 'lockedOtherSet' is set.
     */
    uint8_t syncMode;
} ubx_cfg_tp5;

#endif /* DATATYPES_H_ */
