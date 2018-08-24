#ifndef SEU_UNIROBOT_DEBUGER_TEAM_MONITOR_HPP
#define SEU_UNIROBOT_DEBUGER_TEAM_MONITOR_HPP

#include <QtWidgets>
#include <boost/asio.hpp>
#include <map>
#include <thread>
#include <mutex>
#include "model.hpp"

class team_monitor: public QMainWindow
{
    Q_OBJECT
public:
    team_monitor();
protected:
    void closeEvent(QCloseEvent *event);
    void paintEvent(QPaintEvent *event);
private:
    void receive();
    filed_info field_;
    std::thread td_;
    mutable std::mutex p_mutex_;
    std::map<int, player_info> players_;
    boost::asio::ip::udp::socket socket_;
    boost::asio::ip::udp::endpoint point_;
};

#endif
