#ifndef ROBOMASTER_ROBOT_H
#define ROBOMASTER_ROBOT_H

#include <iostream>
#include <cstdint>
#include <thread>
#include "serial_device.h"
#include "protocol_new.hpp"
#include "crc.h"
#include <memory> 
class SerialMain {
public:
	SerialMain(std::string device_path = "/dev/robomaster");
	
	~SerialMain() = default;
	
	bool SenderMain(const io::RobotCtrlData &data);             // 发送数据
	
	bool CommInit();
	
	bool ReceiverMain();                                        // 读取数据
	
	bool ReceiveDataSolve(uint8_t *frame, uint16_t available_length);
	
	uint16_t SenderPackSolve(const uint8_t *data, uint16_t data_length,
							 uint16_t cmd_id, uint8_t *send_buf);
	io::VisionData vision_msg_;

private:
	
	//! Device Information and Buffer Allocation
	std::string device_path_;
	std::shared_ptr<SerialDevice> device_ptr_;
	std::unique_ptr<uint8_t[]> recv_buff_;
	std::unique_ptr<uint8_t[]> send_buff_;
	const unsigned int BUFF_LENGTH = 512;
	bool initialized_ = false;
	
	//! Frame Information
	io::FrameHeader frame_receive_header_;
	io::FrameHeader frame_send_header_;

	bool ReadExact(uint8_t *buffer, uint16_t size);
};
//}

#endif // ROBOMASTER_ROBOT_H
