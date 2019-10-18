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

#include "basestation.h"
#include <QDebug>
#include <utility.h>
#include <cmath>

BaseStation::BaseStation(QObject *parent) : QObject(parent)
{
    mUblox = new Ublox(this);
    mTcpServer = new TcpBroadcast(this);
    mTimer = new QTimer(this);
    mTimer->start(40);

    mLat = 0.0;
    mLon = 0.0;
    mHeight = 0.0;

    mBasePosSet = false;
    mLat = 57.71433340;
    mLon = 12.89086553;
    mHeight = 217.783;
    mIsFixedBase = false;

    mVerbose = false;
    mAccThres = 4.0;
    mMoveThres = 2.0;

    connect(mTimer, SIGNAL(timeout()),
            this, SLOT(timerSlot()));
    connect(mUblox, SIGNAL(rxRawx(ubx_rxm_rawx)),
            this, SLOT(rxRawx(ubx_rxm_rawx)));
    connect(mUblox, SIGNAL(rxNavSol(ubx_nav_sol)), this, SLOT(rxNavSol(ubx_nav_sol)));
    connect(mUblox, SIGNAL(rtcmRx(QByteArray,int)), this, SLOT(rtcmRx(QByteArray,int)));
}

bool BaseStation::startBase(QString ublox, int tcpPort, int rateMs, bool encodeRaw,
                            bool surveyIn, double fixedLat, double fixedLon, double fixedHeight)
{
    bool res = mUblox->connectSerial(ublox, 115200);

    if (!res) {
        qWarning() << "Could not connect to ublox";
        return false;
    }

    res = mTcpServer->startTcpServer(tcpPort);

    if (!res) {
        qWarning() << "Could not start tcp server";
    }

    if (res) {
        // Serial port baud rate
        // if it is too low the buffer will overfill and it won't work properly.
        ubx_cfg_prt_uart uart;
        uart.baudrate = 115200;
        uart.in_ubx = true;
        uart.in_nmea = true;
        uart.in_rtcm2 = false;
        uart.in_rtcm3 = true;
        uart.out_ubx = true;
        uart.out_nmea = true;
        uart.out_rtcm3 = true;
        mUblox->ubxCfgPrtUart(&uart);

        mUblox->ubxCfgRate(rateMs, 1, 0);

        // Raw data output
        mUblox->ubxCfgMsg(UBX_CLASS_RXM, UBX_RXM_RAWX, encodeRaw ? 1 : 0); // Every second
        mUblox->ubxCfgMsg(UBX_CLASS_RXM, UBX_RXM_SFRBX, encodeRaw ? 1 : 0); // Every second
        mUblox->ubxCfgMsg(UBX_CLASS_NAV, UBX_NAV_SOL, encodeRaw ? 1 : 0); // Every second

        // RTCM Message output
        mUblox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1005, encodeRaw ? 0 : 1);
        mUblox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1074, encodeRaw ? 0 : 1);
        mUblox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1077, encodeRaw ? 0 : 1);
        mUblox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1084, encodeRaw ? 0 : 1);
        mUblox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1087, encodeRaw ? 0 : 1);
        mUblox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1094, encodeRaw ? 0 : 1);
        mUblox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1097, encodeRaw ? 0 : 1);
        mUblox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1124, encodeRaw ? 0 : 1);
        mUblox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1127, encodeRaw ? 0 : 1);
        mUblox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_1230, encodeRaw ? 0 : 1);
        mUblox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_4072_0, encodeRaw ? 0 : 1);
        mUblox->ubxCfgMsg(UBX_CLASS_RTCM3, UBX_RTCM3_4072_1, encodeRaw ? 0 : 1);

        // Stationary dynamic model
        ubx_cfg_nav5 nav5;
        memset(&nav5, 0, sizeof(ubx_cfg_nav5));
        nav5.apply_dyn = true;
        nav5.dyn_model = 2;
        mUblox->ubxCfgNav5(&nav5);

        // Base station survey in mode
        ubx_cfg_tmode3 cfg_mode;
        memset(&cfg_mode, 0, sizeof(cfg_mode));
        cfg_mode.mode = 1; // Survey in
        cfg_mode.fixed_pos_acc = 5.0;
        cfg_mode.svin_min_dur = 20;
        cfg_mode.svin_acc_limit = 10.0;

        if (!surveyIn) {
            mIsFixedBase = true;
            mBasePosSet = true;
            mLat = fixedLat;
            mLon = fixedLon;
            mHeight = fixedHeight;

            cfg_mode.mode = 2;
            cfg_mode.lla = true;
            cfg_mode.ecefx_lat = fixedLat;
            cfg_mode.ecefy_lon = fixedLon;
            cfg_mode.ecefz_alt = fixedHeight;
        }

        if (!encodeRaw) {
            mUblox->ubxCfgTmode3(&cfg_mode);
        }

        // Save everything
        ubx_cfg_cfg cfg;
        memset(&cfg, 0, sizeof(ubx_cfg_cfg));
        cfg.save_io_port = true;
        cfg.save_msg_conf = true;
        cfg.save_inf_msg = true;
        cfg.save_nav_conf = true;
        cfg.save_rxm_conf = true;
        cfg.save_sen_conf = true;
        cfg.save_rinv_conf = true;
        cfg.save_ant_conf = true;
        cfg.save_log_conf = true;
        cfg.save_fts_conf = true;
        cfg.dev_bbr = true;
        cfg.dev_flash = true;
        mUblox->ubloxCfgCfg(&cfg);
    }

    return res;
}

void BaseStation::setVerbose(bool verbose)
{
    mVerbose = verbose;
}

void BaseStation::setAccThres(double thres)
{
    mAccThres = thres;
}

void BaseStation::setMoveThres(double thres)
{
    mMoveThres = thres;
}

void BaseStation::timerSlot()
{
    // TODO: check data age and print message if no data is coming.
}

void BaseStation::rxRawx(ubx_rxm_rawx rawx)
{
    uint8_t data_gps[1024];
    uint8_t data_glo[1024];
    uint8_t data_ref[512];

    int gps_len = 0;
    int glo_len = 0;
    int ref_len = 0;

    rtcm_obs_header_t header;
    rtcm_obs_t obs[rawx.num_meas];

    header.staid = 0;
    header.t_wn = rawx.week;
    header.t_tow = rawx.rcv_tow;
    header.t_tod = fmod(rawx.rcv_tow - (double)rawx.leaps + 10800.0, 86400.0);

    bool has_gps = false;
    bool has_glo = false;
    bool has_ref = false;

    for (int i = 0;i < rawx.num_meas;i++) {
        ubx_rxm_rawx_obs *raw_obs = &rawx.obs[i];

        if (raw_obs->gnss_id == 0) {
            has_gps = true;
        } else if (raw_obs->gnss_id == 6) {
            has_glo = true;
        }
    }

    // GPS
    if (has_gps) {
        int obs_ind = 0;
        for (int i = 0;i < rawx.num_meas;i++) {
            ubx_rxm_rawx_obs *raw_obs = &rawx.obs[i];

            if (raw_obs->gnss_id == 0) {
                obs[obs_ind].P[0] = raw_obs->pr_mes;
                obs[obs_ind].L[0] = raw_obs->cp_mes;
                obs[obs_ind].cn0[0] = raw_obs->cno;
                obs[obs_ind].lock[0] = raw_obs->locktime > 2000 ? 127 : 0;
                obs[obs_ind].code[0] = CODE_L1C;
                obs[obs_ind].prn = raw_obs->sv_id;
                obs_ind++;
            }
        }
        header.sync = has_glo;
        rtcm3_encode_1002(&header, obs, obs_ind, data_gps, &gps_len);
    }

    // GLONASS
    if (has_glo) {
        int obs_ind = 0;
        for (int i = 0;i < rawx.num_meas;i++) {
            ubx_rxm_rawx_obs *raw_obs = &rawx.obs[i];

            if (raw_obs->gnss_id == 6) {
                obs[obs_ind].P[0] = raw_obs->pr_mes;
                obs[obs_ind].L[0] = raw_obs->cp_mes;
                obs[obs_ind].cn0[0] = raw_obs->cno;
                obs[obs_ind].lock[0] = raw_obs->locktime > 2000 ? 127 : 0;
                obs[obs_ind].code[0] = CODE_L1C;
                obs[obs_ind].prn = raw_obs->sv_id;
                obs[obs_ind].freq = raw_obs->freq_id;
                obs_ind++;
            }
        }
        header.sync = 0;
        rtcm3_encode_1010(&header, obs, obs_ind, data_glo, &glo_len);
    }

    // Base station position
    if (mBasePosSet) {
        rtcm_ref_sta_pos_t pos;
        pos.ant_height = 0.0;
        pos.height = mHeight;
        pos.lat = mLat;
        pos.lon = mLon;
        pos.staid = 0;
        rtcm3_encode_1006(pos, data_ref, &ref_len);
        has_ref = true;
    }

    QByteArray message;
    QString msgs;
    if (has_gps) {
        message.append((char*)data_gps, gps_len);
        msgs += "1002 ";
    }

    if (has_glo) {
        message.append((char*)data_glo, glo_len);
        msgs += "1010 ";
    }

    if (has_ref) {
        message.append((char*)data_ref, ref_len);
        msgs += "1006 ";
    }

    mTcpServer->broadcastData(message);

    if (mVerbose) {
        qDebug() << "Broadcasting msgs:" << msgs;
    }
}

void BaseStation::rxNavSol(ubx_nav_sol sol)
{
    if (mIsFixedBase) {
        return;
    }

    double llh[3];
    utility::xyzToLlh(sol.ecef_x, sol.ecef_y, sol.ecef_z, &llh[0], &llh[1], &llh[2]);

    if (sol.p_acc < mAccThres) {
        if (mBasePosSet) {
            double xyz[3];
            utility::llhToXyz(mLat, mLon, mHeight, &xyz[0], &xyz[1], &xyz[2]);

            double diff = sqrt(pow(sol.ecef_x - xyz[0], 2.0) + pow(sol.ecef_y - xyz[1], 2.0) + pow(sol.ecef_z - xyz[2], 2.0));

            if (mVerbose) {
                qDebug() << "Position rx, difference:" << diff << "m";
            }

            if (diff > mMoveThres) {
                qDebug() << "Base diff:" << diff << "m, updating base station position.";
                mLat = llh[0];
                mLon = llh[1];
                mHeight = llh[2];
            }
        } else {
            qDebug() << "Initial base station position set.";
            mLat = llh[0];
            mLon = llh[1];
            mHeight = llh[2];
            mBasePosSet = true;
        }
    } else {
        qDebug() << "Nmea rx, acc too low (" << sol.p_acc << "m ).";
    }
}

void BaseStation::rtcmRx(QByteArray data, int type)
{
    mTcpServer->broadcastData(data);

    if (mVerbose) {
        qDebug() << "RTCM RX:" << type;
    }
}
