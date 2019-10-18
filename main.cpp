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

#include <QCoreApplication>
#include <signal.h>
#include "basestation.h"
#include "rover.h"

void showHelp()
{
    qDebug() << "Arguments";
    qDebug() << "--help : Show help text";
    qDebug() << "--ublox [tty] : Ublox port, e.g. /dev/ttyACM0";
    qDebug() << "--tcpport [port] : RTCM server port, e.g. 65100";
    qDebug() << "--baseaccthres [th] : Accuracy threshold for base position, e.g. 4.0 (m)";
    qDebug() << "--basemovethres [th] : Movement threshold for base position, e.g. 4.0 (m)";
    qDebug() << "--basepos [lat]:[lon]:[height] : Fixed base pos, e.g. 57.714:12.890:200 ";
    qDebug() << "--baseencoderaw : Encode raw data to RTCM. Works with e.g. ublox M8T as base.";
    qDebug() << "--isrover : Act as a rover";
    qDebug() << "--roverrtcmserver [ip] : Rover RTCM server, e.g. 192.168.1.3";
    qDebug() << "--roverrtcmport [port] : Rover RTCM port, e.g. 65100";
    qDebug() << "--roverdatatcpport [tcpPort] : Start data tcp server on tcpPort";
    qDebug() << "--rovernmeaserver [tcpPort] : Start NMEA TCP server on tcpPort";
    qDebug() << "--roverdynmodel [model] : Dynamic model, e.g. 2 (stat), 4 (car), 5 (sea), 6 (air 1G), 7 (air 2G), 8 (air 4G)";
    qDebug() << "--ratems [ms] : Navigation rate in milliseconds.";
    qDebug() << "--verbose : Print more debug information";
}

static void m_cleanup(int sig)
{
    (void)sig;
    qApp->quit();
}

int main(int argc, char *argv[])
{
    QCoreApplication a(argc, argv);

    signal(SIGINT, m_cleanup);
    signal(SIGTERM, m_cleanup);

    QStringList args = QCoreApplication::arguments();
    QString ubxPort = "/dev/ttyACM0";
    int tcpPort = 65100;
    bool isRover = false;
    QString roverRtcmServer;
    int roverRtcmPort = 65100;
    int roverDataTcpPort = -1;
    int roverNmeaTcpPort = -1;
    bool verbose = false;
    double baseAccThres = 4.0;
    double baseMoveThres = 2.0;
    int roverDynModel = 6;
    int rateMs = 1000;
    double fixedLat = 57.71433340;
    double fixedLon = 12.89086553;
    double fixedHeight = 217.783;
    bool isFixedBase = false;
    bool baseUseRaw = false;

    for (int i = 0;i < args.size();i++) {
        // Skip the program argument
        if (i == 0) {
            continue;
        }

        QString str = args.at(i).toLower();

        // Skip path argument
        if (i >= args.size() && args.size() >= 3) {
            break;
        }

        bool dash = str.startsWith("-") && !str.startsWith("--");
        bool found = false;

        if ((dash && str.contains('h')) || str == "--help") {
            showHelp();
            found = true;
            return 0;
        }

        if (str == "--ublox") {
            if ((i - 1) < args.size()) {
                i++;
                ubxPort = args.at(i);
                found = true;
            }
        }

        if (str == "--tcpport") {
            if ((i - 1) < args.size()) {
                i++;
                bool ok;
                tcpPort = args.at(i).toInt(&ok);
                found = ok;
            }
        }

        if (str == "--baseaccthres") {
            if ((i - 1) < args.size()) {
                i++;
                bool ok;
                baseAccThres = args.at(i).toDouble(&ok);
                found = ok;
            }
        }

        if (str == "--basemovethres") {
            if ((i - 1) < args.size()) {
                i++;
                bool ok;
                baseMoveThres = args.at(i).toDouble(&ok);
                found = ok;
            }
        }

        if (str == "--basepos") {
            if ((i + 1) < args.size()) {
                i++;
                QString tmp = args.at(i);
                QStringList baseData = tmp.split(":");

                if (baseData.size() == 3) {
                    found = true;
                    isFixedBase = true;
                    fixedLat = baseData.at(0).toDouble();
                    fixedLon = baseData.at(1).toDouble();
                    fixedHeight = baseData.at(2).toDouble();
                }
            }
        }

        if (str == "--baseencoderaw") {
            baseUseRaw = true;
            found = true;
        }

        if (str == "--isrover") {
            isRover = true;
            found = true;
        }

        if (str == "--roverrtcmserver") {
            if ((i - 1) < args.size()) {
                i++;
                roverRtcmServer = args.at(i);
                found = true;
            }
        }

        if (str == "--roverrtcmport") {
            if ((i - 1) < args.size()) {
                i++;
                bool ok;
                roverRtcmPort = args.at(i).toInt(&ok);
                found = ok;
            }
        }

        if (str == "--roverdatatcpport") {
            if ((i - 1) < args.size()) {
                i++;
                bool ok;
                roverDataTcpPort = args.at(i).toInt(&ok);
                found = ok;
            }
        }

        if (str == "--rovernmeaserver") {
            if ((i - 1) < args.size()) {
                i++;
                bool ok;
                roverNmeaTcpPort = args.at(i).toInt(&ok);
                found = ok;
            }
        }

        if (str == "--roverdynmodel") {
            if ((i - 1) < args.size()) {
                i++;
                bool ok;
                roverDynModel = args.at(i).toInt(&ok);
                found = ok;
            }
        }

        if (str == "--ratems") {
            if ((i - 1) < args.size()) {
                i++;
                bool ok;
                rateMs = args.at(i).toInt(&ok);
                found = ok;
            }
        }

        if (str == "--verbose") {
            verbose = true;
            found = true;
        }

        if (!found) {
            if (dash) {
                qCritical() << "At least one of the flags is invalid:" << str;
            } else {
                qCritical() << "Invalid option:" << str;
            }

            showHelp();
            return 1;
        }
    }

    BaseStation base;
    Rover rover;

    if (isRover) {
        rover.setDynModel(roverDynModel);
        if (!rover.startRover(ubxPort, roverRtcmServer, roverRtcmPort, rateMs)) {
            return -1;
        }

        if (roverDataTcpPort >= 0) {
            if (!rover.startPositionServer(roverDataTcpPort)) {
                return -1;
            }
        }

        if (roverNmeaTcpPort >= 0) {
            if (!rover.startNmeaServer(roverNmeaTcpPort)) {
                return -1;
            }
        }

        rover.setVerbose(verbose);
    } else {
        if (!base.startBase(ubxPort, tcpPort, rateMs,
                            baseUseRaw, !isFixedBase,
                            fixedLat, fixedLon, fixedHeight)) {
            return -1;
        }

        base.setVerbose(verbose);
        base.setAccThres(baseAccThres);
        base.setMoveThres(baseMoveThres);
    }

    return a.exec();
}
