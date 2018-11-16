#include "imu.hpp"
#include <boost/asio.hpp>
#include "configuration.hpp"

using namespace std;
using namespace boost::asio;

boost::asio::io_service imu_service;
unsigned char imu_header[] = {0x55, 0x51, 0x52, 0x53, 0x55};
const float g_ = 9.8;
struct SAcc
{
    short a[3];
    short T;
};
struct SGyro
{
    short w[3];
    short T;
};
struct SAngle
{
    short Angle[3];
    short T;
};
struct SDStatus
{
    short sDStatus[4];
};
SAcc        stcAcc;
SGyro       stcGyro;
SAngle      stcAngle;
SDStatus    stcDStatus;

#define DIN 0x01
#define DOH 0x02
#define DOL 0x03
unsigned char IO[] = {0x0e, 0x0f, 0x10, 0x11};
unsigned char cmd[] = {0xff, 0xaa, 0x00, 0x00, 0x00};

imu::imu(): sensor("imu"), timer(1000), serial_(imu_service)
{
    led_ = 0;
    l_state_ = LED_NORMAL;
    reset_ = false;
    count_ = 0;
    lost_ = false;
    connected_ = false;
}

bool imu::open()
{
    try
    {
        serial_.open(CONF->get_config_value<string>("hardware.imu.dev_name"));
        serial_.set_option(boost::asio::serial_port::baud_rate(CONF->get_config_value<unsigned int>("hardware.imu.baudrate")));
        serial_.set_option(boost::asio::serial_port::flow_control(boost::asio::serial_port::flow_control::none));
        serial_.set_option(boost::asio::serial_port::parity(boost::asio::serial_port::parity::none));
        serial_.set_option(boost::asio::serial_port::stop_bits(boost::asio::serial_port::stop_bits::one));
        serial_.set_option(boost::asio::serial_port::character_size(8));
        return true;
    }
    catch (exception &e)
    {
        std::cout << "\033[33mimu: " << e.what() << "\033[0m\n";
        return false;
    }
}

void imu::run()
{
    if (timer::is_alive_)
    {
        if(connected_)
        {
            if(count_<10)
            {
                lost_ = true;
                notify(SENSOR_IMU);
            }
            else
            {
                lost_ = false;
            }
            count_ = 0;
        }
        led_mtx_.lock();
        led_state state = l_state_;
        bool rst = reset_;
        led_mtx_.unlock();
        auto self(shared_from_this());

        if (state == LED_NORMAL)
        {
            usleep(50000);
            cmd[3] = led_ ? DOH : DOL;
            cmd[2] = IO[3];
            boost::asio::async_write(serial_, boost::asio::buffer(cmd, 5),
                [this, self](boost::system::error_code ec, std::size_t length) {});
            led_ = 1 - led_;
        }
        else if (state == LED_WARNING)
        {
            usleep(50000);
            cmd[2] = IO[3];
            cmd[3] = DOL;
            boost::asio::async_write(serial_, boost::asio::buffer(cmd, 5),
                [this, self](boost::system::error_code ec, std::size_t length) {});
            usleep(450000);
            cmd[3] = DOH;
            boost::asio::async_write(serial_, boost::asio::buffer(cmd, 5),
                [this, self](boost::system::error_code ec, std::size_t length) {});
        }
        else if (state == LED_ERROR)
        {
            usleep(50000);
            cmd[2] = IO[3];
            cmd[3] = DOL;
            boost::asio::async_write(serial_, boost::asio::buffer(cmd, 5),
                    [this, self](boost::system::error_code ec, std::size_t length) {});
            usleep(200000);
            cmd[3] = DOH;
            boost::asio::async_write(serial_, boost::asio::buffer(cmd, 5),
                                     [this, self](boost::system::error_code ec, std::size_t length) {});
            usleep(250000);
            cmd[3] = DOL;
            boost::asio::async_write(serial_, boost::asio::buffer(cmd, 5),
                                     [this, self](boost::system::error_code ec, std::size_t length) {});
            usleep(250000);
            cmd[3] = DOH;
            boost::asio::async_write(serial_, boost::asio::buffer(cmd, 5),
                                     [this, self](boost::system::error_code ec, std::size_t length) {});
        }

        usleep(100000);

        if (rst)
        {
            cmd[2] = 0x01;
            cmd[3] = 0x04;
            boost::asio::async_write(serial_, boost::asio::buffer(cmd, 5),
                [this, self](boost::system::error_code ec, std::size_t length) {});
            reset_ = false;
        }
    }
}

void imu::read_head0()
{
    auto self(shared_from_this());
    boost::asio::async_read(serial_, boost::asio::buffer(buff_, 1),
                            [this, self](boost::system::error_code ec, std::size_t length)
    {
        if (!ec)
        {
            if (buff_[0] == imu_header[0])
            {
                read_head1();
            }
            else
            {
                read_head0();
            }
        }
    });
}

void imu::read_head1()
{
    auto self(shared_from_this());
    boost::asio::async_read(serial_, boost::asio::buffer(buff_ + 1, 1),
                            [this, self](boost::system::error_code ec, std::size_t length)
    {
        if (!ec)
        {
            if (buff_[1] == imu_header[1] || buff_[1] == imu_header[2] || buff_[1] == imu_header[3] || buff_[1] == imu_header[4])
            {
                read_data();
            }
            else
            {
                read_head0();
            }
        }
    });
}

void imu::read_data()
{
    auto self(shared_from_this());
    boost::asio::async_read(serial_, boost::asio::buffer(buff_ + 2, imu_len - 2),
                            [this, self](boost::system::error_code ec, std::size_t length)
    {
        if (!ec)
        {
            unsigned char sum = 0;
            for (int i = 0; i < imu_len - 1; i++)
            {
                sum += buff_[i];
            }

            if (sum == buff_[imu_len - 1])
            {
                connected_ = true;
                switch (buff_[1])
                {
                    case 0x51:
                        memcpy(&stcAcc, &buff_[2], 8);
                        imu_data_.ax = stcAcc.a[0] / 32768.0f * 16.0f * g_;
                        imu_data_.ay = stcAcc.a[1] / 32768.0f * 16.0f * g_;
                        imu_data_.az = stcAcc.a[2] / 32768.0f * 16.0f * g_;
                        break;

                    case 0x52:
                        memcpy(&stcGyro, &buff_[2], 8);
                        imu_data_.wx = stcGyro.w[0] / 32768.0f * 2000.0f;
                        imu_data_.wy = stcGyro.w[1] / 32768.0f * 2000.0f;
                        imu_data_.wz = stcGyro.w[2] / 32768.0f * 2000.0f;
                        break;

                    case 0x53:
                        memcpy(&stcAngle, &buff_[2], 8);
                        imu_data_.roll = stcAngle.Angle[0] / 32768.0f * 180.0f;
                        imu_data_.pitch = stcAngle.Angle[1] / 32768.0f * 180.0f;
                        imu_data_.yaw = stcAngle.Angle[2] / 32768.0f * 180.0f;
                        break;

                    case 0x55:
                        memcpy(&stcDStatus, &buff_[2], 8);
                        sw_data_.sw1 = static_cast<bool>(stcDStatus.sDStatus[1]);
                        sw_data_.sw2 = static_cast<bool>(stcDStatus.sDStatus[0]);

                    default:
                        break;
                }
                count_++;
                notify(SENSOR_IMU);
            }

            read_head0();
        }
    });
}

bool imu::start()
{
    if (!this->open())
    {
        return false;
    }

    is_open_ = true;
    sensor::is_alive_ = true;
    timer::is_alive_ = true;
    td_ = std::move(thread([this]()
    {
        this->read_head0();
        imu_service.run();
    }));
    start_timer();
    return true;
}

void imu::stop()
{
    serial_.close();
    imu_service.stop();
    timer::is_alive_ = false;
    sensor::is_alive_ = false;
    is_open_ = false;
    delete_timer();
}

imu::~imu()
{
    if (td_.joinable())
    {
        td_.join();
    }
}
