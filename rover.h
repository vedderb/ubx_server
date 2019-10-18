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

#ifndef ROVER_H
#define ROVER_H

#include <QObject>
#include <QTimer>
#include <QTcpSocket>
#include "ublox.h"
#include "tcpbroadcast.h"
#include "nmeaserver.h"

class Rover : public QObject
{
    Q_OBJECT
public:
    explicit Rover(QObject *parent = nullptr);
    bool startRover(QString ublox, QString tcpServer, int tcpPort, int rateMs = 1000);
    void stopRover();
    bool startPositionServer(int tcpPort);
    bool startNmeaServer(int tcpPort);
    bool setDynModel(int model);
    void setVerbose(bool verbose);

signals:

public slots:

private slots:
    void timerSlot();
    void rxNavSol(ubx_nav_sol sol);
    void rxRelPosNed(ubx_nav_relposned pos);
    void rxGga(int fields, NmeaServer::nmea_gga_info_t gga);
    void tcpInputConnected();
    void tcpInputDisconnected();
    void tcpInputDataAvailable();
    void tcpInputError(QAbstractSocket::SocketError socketError);

private:
    QTimer *mTimer;
    Ublox *mUblox;
    TcpBroadcast *mTcpServer;
    NmeaServer *mNmeaServer;
    QTcpSocket *mTcpSocket;
    bool mTcpConnected;
    rtcm3_state mRtcmState;
    int mDynModel;
    bool mVerbose;

};

#endif // ROVER_H
