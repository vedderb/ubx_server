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

#include "rover.h"
#include "utility.h"
#include <QDebug>
#include <string>
#include <locale>
#include <cmath>

Rover::Rover(QObject *parent) : QObject(parent)
{
    mTcpSocket = new QTcpSocket(this);
    mTcpConnected = false;
    mUblox = new Ublox(this);
    mTcpServer = new TcpBroadcast(this);
    memset(&mRtcmState, 0, sizeof(mRtcmState));
    mRtcmState.decode_all = true;
    mDynModel = 6;
    mVerbose = false;
    mNmeaServer = new NmeaServer(this);

    mTimer = new QTimer(this);
    mTimer->start(40);

    connect(mTimer, SIGNAL(timeout()),
            this, SLOT(timerSlot()));
    connect(mUblox, SIGNAL(rxNavSol(ubx_nav_sol)), this, SLOT(rxNavSol(ubx_nav_sol)));
    connect(mUblox, SIGNAL(rxRelPosNed(ubx_nav_relposned)),
            this, SLOT(rxRelPosNed(ubx_nav_relposned)));
    connect(mUblox, SIGNAL(rxGga(int,NmeaServer::nmea_gga_info_t)),
            this, SLOT(rxGga(int,NmeaServer::nmea_gga_info_t)));

    connect(mTcpSocket, SIGNAL(readyRead()), this, SLOT(tcpInputDataAvailable()));
    connect(mTcpSocket, SIGNAL(connected()), this, SLOT(tcpInputConnected()));
    connect(mTcpSocket, SIGNAL(disconnected()),
            this, SLOT(tcpInputDisconnected()));
    connect(mTcpSocket, SIGNAL(error(QAbstractSocket::SocketError)),
            this, SLOT(tcpInputError(QAbstractSocket::SocketError)));
}

bool Rover::startRover(QString ublox, QString tcpServer, int tcpPort, int rateMs)
{
    bool res = mUblox->connectSerial(ublox, 115200);

    if (!res) {
        qWarning() << "Could not connect to ublox";
        return false;
    }

    mTcpSocket->abort();

    if (!tcpServer.isEmpty()) {
        mTcpSocket->connectToHost(tcpServer, tcpPort);
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

        // USB port configuration
        ubx_cfg_prt_usb usb;
        usb.in_ubx = true;
        usb.in_nmea = true;
        usb.in_rtcm2 = false;
        usb.in_rtcm3 = true;
        usb.out_ubx = true;
        usb.out_nmea = true;
        usb.out_rtcm3 = true;
        mUblox->ubxCfgPrtUsb(&usb);

        mUblox->ubxCfgRate(rateMs, 1, 0);
        mUblox->ubxCfgMsg(UBX_CLASS_NAV, UBX_NAV_SOL, 1);
        mUblox->ubxCfgMsg(UBX_CLASS_NAV, UBX_NAV_RELPOSNED, 1);
        mUblox->ubxCfgMsg(UBX_CLASS_NMEA, UBX_NMEA_GGA, 1);

        // Dynamic model
        ubx_cfg_nav5 nav5;
        memset(&nav5, 0, sizeof(ubx_cfg_nav5));
        nav5.apply_dyn = true;
        nav5.dyn_model = mDynModel;
        mUblox->ubxCfgNav5(&nav5);

        // No Servey in
        ubx_cfg_tmode3 cfg_mode;
        memset(&cfg_mode, 0, sizeof(cfg_mode));
        cfg_mode.mode = 0;
        cfg_mode.fixed_pos_acc = 5.0;
        cfg_mode.svin_min_dur = 20;
        cfg_mode.svin_acc_limit = 10.0;
        mUblox->ubxCfgTmode3(&cfg_mode);

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

void Rover::stopRover()
{
    mUblox->disconnectSerial();
    mTcpSocket->abort();
}

bool Rover::startPositionServer(int tcpPort)
{
    return mTcpServer->startTcpServer(tcpPort);
}

bool Rover::startNmeaServer(int tcpPort)
{
    return mNmeaServer->startTcpServer(tcpPort);
}

bool Rover::setDynModel(int model)
{
    if (model >= 0 && model <= 9) {
        mDynModel = model;
        return true;
    }

    qWarning() << "Invalid dynamic model:" << model;
    return false;
}

void Rover::setVerbose(bool verbose)
{
    mVerbose = verbose;
}

void Rover::timerSlot()
{

}

void Rover::rxNavSol(ubx_nav_sol sol)
{
    setlocale(LC_NUMERIC, "C");

    double llh[3];
    utility::xyzToLlh(sol.ecef_x, sol.ecef_y, sol.ecef_z,
                      &llh[0], &llh[1], &llh[2]);

    double enuMat[9];
    double enuVel[3];
    utility::createEnuMatrix(llh[0], llh[1], enuMat);
    enuVel[0] = enuMat[0] * sol.ecef_vx + enuMat[1] * sol.ecef_vy + enuMat[2] * sol.ecef_vz;
    enuVel[1] = enuMat[3] * sol.ecef_vx + enuMat[4] * sol.ecef_vy + enuMat[5] * sol.ecef_vz;
    enuVel[2] = enuMat[6] * sol.ecef_vx + enuMat[7] * sol.ecef_vy + enuMat[8] * sol.ecef_vz;

    double heading = atan2(enuVel[1], enuVel[0]);
    double hSpeed = sqrt(pow(enuVel[0], 2) + pow(enuVel[1], 2));

    QString str;
    str.sprintf("llh,%d,%d,%.3f,%.3f,%.8f,%.8f,%.3f,%.3f,%.3f,%.3f,%.3f,%.3f",
                sol.diffsoln, sol.num_sv, sol.p_acc, sol.s_acc,
                llh[0], llh[1], llh[2],
            heading, hSpeed,
            enuVel[0], enuVel[1], enuVel[2]);

    if (mVerbose) {
        qDebug() << str;
    }

    str += "\n";
    mTcpServer->broadcastData(str.toLocal8Bit());
}

void Rover::rxRelPosNed(ubx_nav_relposned pos)
{
    setlocale(LC_NUMERIC, "C");

    QString str;
    str.sprintf("enu,%d,%.3f,%.3f,%.3f,"
                "%.3f,%.3f,%.3f",
                pos.carr_soln,
                pos.pos_e, pos.pos_n, -pos.pos_d,
                pos.acc_e, pos.acc_n, pos.acc_d);

    if (mVerbose) {
        qDebug() << str;
    }

    str += "\n";
    mTcpServer->broadcastData(str.toLocal8Bit());
}

void Rover::rxGga(int fields, NmeaServer::nmea_gga_info_t gga)
{
    (void)fields;

    if (mVerbose) {
        QString type = "Unknown";

        switch (gga.fix_type) {
        case 0: type = "Invalid"; break;
        case 1: type = "SPP"; break;
        case 2: type = "DGPS"; break;
        case 4: type = "RTK Fix"; break;
        case 5: type = "RTK Float"; break;
        default: break;
        }

        qDebug() << "LLH:" << gga.lat << gga.lon << gga.height <<
                    "Fix type:" << gga.fix_type << "(" << type << "," << gga.diff_age << "s )";
    }

    mNmeaServer->sendNmeaGga(gga);
}

void Rover::tcpInputConnected()
{
    mTcpConnected = true;
    qDebug() << "Connected to base station TCP server.";
}

void Rover::tcpInputDisconnected()
{
    qDebug() << "Disconnected from base station TCP server.";
    mTcpConnected = false;
}

void Rover::tcpInputDataAvailable()
{
    for (char c: mTcpSocket->readAll()) {
        int res = rtcm3_input_data((unsigned char)c, &mRtcmState);

        if (res > 1000) {
            QByteArray msg((char*)mRtcmState.buffer, mRtcmState.len + 3);
            mUblox->writeRaw(msg);
        }

        if (res == 1005 || res == 1006) {
            setlocale(LC_NUMERIC, "C");

            QString str;
            str.sprintf("base,%.8f,%.8f,%.3f",
                        mRtcmState.pos.lat, mRtcmState.pos.lon, mRtcmState.pos.height);

            if (mVerbose) {
                qDebug() << str;
            }

            str += "\n";
            mTcpServer->broadcastData(str.toLocal8Bit());
        }
    }
}

void Rover::tcpInputError(QAbstractSocket::SocketError socketError)
{
    (void)socketError;

    QString errorStr = mTcpSocket->errorString();
    qWarning() << "TcpError:" << errorStr;

    mTcpSocket->close();
    mTcpConnected = false;
}
