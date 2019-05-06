#pragma once

#include <QtWidgets>
#include <boost/asio.hpp>
#include <map>
#include <thread>
#include <mutex>
#include "udp_data/CommData.h"
#include "ui/state_monitor.hpp"
#include <unordered_map>

class TeamMonitor: public QMainWindow
{
    Q_OBJECT
public:
    TeamMonitor();
    ~TeamMonitor();
protected:
    void closeEvent(QCloseEvent *event);
    void paintEvent(QPaintEvent *event);
    void keyPressEvent(QKeyEvent *event);
private:
    void receive();
    filed_info field_;
    std::thread td_;
    mutable std::mutex p_mutex_;
    std::map<int, player_info> players_;
    std::map<int, int> states_;
    comm_packet pkt_;
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint point_;
    std::unordered_map<int, StateMonitor*> state_monitors_;
};
