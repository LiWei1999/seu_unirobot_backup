#include "imu.hpp"
#include <boost/asio.hpp>
#include "configuration.hpp"
#include "math/math.hpp"
#include "core/worldmodel.hpp"
#include "logger.hpp"

using namespace std;
using namespace boost::asio;
using namespace robot_math;
using namespace Eigen;

boost::asio::io_service imu_service;

const char RST_CMD[] = "AT+RST";

const float g_ = 9.8;

imu::imu(): sensor("imu"), serial_(imu_service)
{
    record_ = true;
    std::vector<float> range = CONF->get_config_vector<float>("not_fall_range.pitch");
    pitch_range_.x() = range[0];
    pitch_range_.y() = range[1];
    range = CONF->get_config_vector<float>("not_fall_range.roll");
    roll_range_.x() = range[0];
    roll_range_.y() = range[1];
    fall_direction_ = FALL_NONE;
    init_dir_ = 0.0;
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
        LOG(LOG_WARN) << "imu: " << e.what() << endll;
        return false;
    }
}

void crc16_update(uint16_t *currectCrc, const uint8_t *src, uint32_t lengthInBytes)
{
    uint32_t crc = *currectCrc;
    uint32_t j;
    for (j=0; j < lengthInBytes; ++j)
    {
        uint32_t i;
        uint32_t byte = src[j];
        crc ^= byte << 8;
        for (i = 0; i < 8; ++i)
        {
            uint32_t temp = crc << 1;
            if (crc & 0x8000)
            {
                temp ^= 0x1021;
            }
            crc = temp;
        }
    } 
    *currectCrc = crc;
}

enum 
{
    kStatus_Idle,
    kStatus_Cmd,
    kStatus_LenLow,
    kStatus_LenHigh,
    kStatus_CRCLow,
    kStatus_CRCHigh,
    kStatus_Data,
};
/**
 * @brief  接收IMU数据
 * @note   在串口接收中断中调用此函数
 * @param  c 串口数据
 * @retval CH_OK
 */
bool imu::Packet_Decode(uint8_t c)
{
    static uint16_t CRCReceived = 0;            /* CRC value received from a frame */
    static uint16_t CRCCalculated = 0;          /* CRC value caluated from a frame */
    static uint8_t status = kStatus_Idle;       /* state machine */
    static uint8_t crc_header[4] = {0x5A, 0xA5, 0x00, 0x00};

    switch(status)
    {
        case kStatus_Idle:
            if(c == 0x5A)
                status = kStatus_Cmd;
            break;
        case kStatus_Cmd:
            rx_pkt_.type = c;
            switch(rx_pkt_.type)
            {
                case 0xA5:  /* Data */
                    status = kStatus_LenLow;
                    break;
                case 0xA6:  /* Ping */
                    OnDataReceived(rx_pkt_);
                    status = kStatus_Idle;
                    break;
                case 0xA7:  /* Ping Respond */
                    rx_pkt_.ofs = 0;
                    status = kStatus_Data;
                    break;
            }
            break;
        case kStatus_LenLow:
            rx_pkt_.payload_len = c;
            crc_header[2] = c;
            status = kStatus_LenHigh;
            break;
        case kStatus_LenHigh:
            rx_pkt_.payload_len |= (c<<8);
            crc_header[3] = c;
            status = kStatus_CRCLow;
            break;
        case kStatus_CRCLow:
            CRCReceived = c;
            status = kStatus_CRCHigh;
            break;
        case kStatus_CRCHigh:
            CRCReceived |= (c<<8);
            rx_pkt_.ofs = 0;
            CRCCalculated = 0;
            status = kStatus_Data;
            break;
        case kStatus_Data:
            rx_pkt_.buf[rx_pkt_.ofs++] = c;
            if(rx_pkt_.type == 0xA7 && rx_pkt_.ofs >= 8)
            {
                rx_pkt_.payload_len = 8;
                OnDataReceived(rx_pkt_);
                status = kStatus_Idle;
            }
            if(rx_pkt_.ofs >= MAX_PACKET_LEN)
            {
                status = kStatus_Idle;
                return false;   
            }

            if(rx_pkt_.ofs >= rx_pkt_.payload_len && rx_pkt_.type == 0xA5)
            {
                /* calculate CRC */
                crc16_update(&CRCCalculated, crc_header, 4);
                crc16_update(&CRCCalculated, rx_pkt_.buf, rx_pkt_.ofs);
                
                /* CRC match */
                if(CRCCalculated == CRCReceived)
                {
                    OnDataReceived(rx_pkt_);
                }
                status = kStatus_Idle;
            }
            break;
        default:
            status = kStatus_Idle;
            break;
    }
    return true;
}

void imu::OnDataReceived(Packet_t &pkt)
{

	if(pkt.type != 0xA5)
    {
        return;
    }
    int offset = 0;
    uint8_t *p = pkt.buf;
    while(offset < pkt.payload_len)
    {
        switch(p[offset])
        {
            case kItemID:
                id = p[1];
                offset += 2;
                break;
            case kItemAccRaw:
            case kItemAccCalibrated:
            case kItemAccFiltered:
            case kItemAccLinear:
                memcpy(acc, p + offset + 1, sizeof(acc));
                imu_data_.ax = acc[0]*0.01;
                imu_data_.ay = acc[1]*0.01;
                imu_data_.az = acc[2]*0.01;
                offset += 7;
                break;
            case kItemGyoRaw:
            case kItemGyoCalibrated:
            case kItemGyoFiltered:
                memcpy(gyo, p + offset + 1, sizeof(gyo));
                offset += 7;
                break;
            case kItemMagRaw:
            case kItemMagCalibrated:
            case kItemMagFiltered:
                memcpy(mag, p + offset + 1, sizeof(mag));
                offset += 7;
                break;
            case kItemRotationEular:
                eular[0] = ((float)(int16_t)(p[offset+1] + (p[offset+2]<<8)))/100;
                eular[1] = ((float)(int16_t)(p[offset+3] + (p[offset+4]<<8)))/100;
                eular[2] = ((float)(int16_t)(p[offset+5] + (p[offset+6]<<8)))/10;
                imu_data_.pitch = eular[0];
                imu_data_.roll = eular[1];
                imu_data_.yaw = eular[2];
                
                if(imu_data_.pitch<pitch_range_.x()) fall_direction_ = FALL_BACKWARD;
                else if(imu_data_.pitch>pitch_range_.y()) fall_direction_ = FALL_FORWARD;
                else if(imu_data_.roll<roll_range_.x()) fall_direction_ = FALL_RIGHT;
                else if(imu_data_.roll>roll_range_.y()) fall_direction_ = FALL_LEFT;
                else fall_direction_ = FALL_NONE;

                offset += 7;
                break;
            case kItemRotationEular2:
                memcpy(eular, p + offset + 1, sizeof(eular));
                offset += 13;
                break;
            case kItemRotationQuat:
                memcpy(quat, p + offset + 1, sizeof(quat));
                offset += 17;
                break;
            case kItemPressure:
                offset += 5;
                break;
            case kItemTemperature:
                offset += 5;
                break;
            default:
				return;
                break;
        }
    }
    if(record_)
    {
        init_dir_=imu_data_.yaw;
        record_ = false;
    }
    imu_data_.yaw = normalize_deg(imu_data_.yaw-init_dir_);
    notify(SENSOR_IMU);
}

void imu::read_data()
{
    auto self(shared_from_this());
    boost::asio::async_read(serial_, boost::asio::buffer(buff_, 1),
                            [this, self](boost::system::error_code ec, std::size_t length)
    {
        if (!ec)
        {
            Packet_Decode(buff_[0]);
        }
        read_data();
    });
}

bool imu::start()
{
    if (!this->open())
    {
        return false;
    }

    is_open_ = true;
    is_alive_ = true;
    td_ = std::move(thread([this]()
    {
        this->read_data();
        imu_service.run();
    }));
    return true;
}

void imu::stop()
{
    serial_.close();
    imu_service.stop();
    is_alive_ = false;
    is_open_ = false;
}

imu::~imu()
{
    if (td_.joinable())
    {
        td_.join();
    }
}
