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

#ifndef BASESTATION_H
#define BASESTATION_H

#include <QObject>
#include <QTimer>
#include "ublox.h"
#include "tcpbroadcast.h"

class BaseStation : public QObject
{
    Q_OBJECT
public:
    explicit BaseStation(QObject *parent = nullptr);
    bool startBase(QString ublox, int tcpPort, int rateMs = 1000, bool encodeRaw = false,
                   bool surveyIn = true, double fixedLat = 0.0, double fixedLon = 0.0, double fixedHeight = 0.0);
    void setVerbose(bool verbose);
    void setAccThres(double thres);
    void setMoveThres(double thres);

signals:

public slots:

private slots:
    void timerSlot();
    void rxRawx(ubx_rxm_rawx rawx);
    void rxNavSol(ubx_nav_sol sol);
    void rtcmRx(QByteArray data, int type);

private:
    QTimer *mTimer;
    Ublox *mUblox;
    TcpBroadcast *mTcpServer;
    double mLat;
    double mLon;
    double mHeight;
    double mBasePosSet;
    bool mIsFixedBase;
    bool mVerbose;
    double mAccThres;
    double mMoveThres;

};

#endif // BASESTATION_H
