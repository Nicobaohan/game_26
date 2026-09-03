#include "serial_main.h"

SerialMain::SerialMain(std::string device_path) : device_path_(device_path)
{
	initialized_ = CommInit();
	if (!initialized_)
	{
		std::cout<<"serial init error!!!!!!!!!!"<<std::endl;
	};
}

bool SerialMain::SenderMain(const io::RobotCtrlData &data)
{
	if (!initialized_ || !device_ptr_ || !send_buff_) return false;

	const uint16_t send_length = SenderPackSolve(
		reinterpret_cast<const uint8_t *>(&data), sizeof(data),
		io::CHASSIS_CTRL_CMD_ID, send_buff_.get());
	return device_ptr_->Write(send_buff_.get(), send_length) == send_length;
}

bool SerialMain::CommInit()
{
	
	device_ptr_ = std::make_shared<SerialDevice>(device_path_, 115200); // 比特率115200
	
	if (!device_ptr_->Init())
	{
		return false;
	}
	
	recv_buff_ = std::unique_ptr<uint8_t[]>(new uint8_t[BUFF_LENGTH]);
	send_buff_ = std::unique_ptr<uint8_t[]>(new uint8_t[BUFF_LENGTH]);
	
	frame_receive_header_ = io::FrameHeader{};
	frame_send_header_ = io::FrameHeader{};
	
	return true;
}

bool SerialMain::ReceiverMain()
{
	if (!initialized_ || !device_ptr_ || !recv_buff_) return false;

	uint8_t sof = 0;
	while (initialized_) {
		if (!ReadExact(&sof, sizeof(sof))) return false;
		if (sof == io::HEADER_SOF) break;
	}

	recv_buff_[0] = sof;
	if (!ReadExact(recv_buff_.get() + 1, sizeof(io::FrameHeader) - 1)) return false;

	io::FrameHeader header;
	memcpy(&header, recv_buff_.get(), sizeof(header));
	if (!Verify_CRC8_Check_Sum(recv_buff_.get(), sizeof(header))) return false;

	constexpr uint16_t fixed_size =
		sizeof(io::FrameHeader) + sizeof(uint16_t) + sizeof(uint16_t) + sizeof(io::MsgEndInfo);
	if (header.data_length > BUFF_LENGTH - fixed_size) return false;

	const uint16_t body_size =
		sizeof(uint16_t) + header.data_length + sizeof(uint16_t) + sizeof(io::MsgEndInfo);
	if (!ReadExact(recv_buff_.get() + sizeof(io::FrameHeader), body_size)) return false;

	const uint16_t full_size = sizeof(io::FrameHeader) + body_size;
	io::MsgEndInfo end;
	memcpy(&end, recv_buff_.get() + full_size - sizeof(end), sizeof(end));
	if (end.end1 != io::END1_SOF || end.end2 != io::END2_SOF) return false;

	return ReceiveDataSolve(recv_buff_.get(), full_size - sizeof(io::MsgEndInfo));
}

bool SerialMain::ReadExact(uint8_t *buffer, uint16_t size)
{
	uint16_t offset = 0;
	while (offset < size) {
		const int count = device_ptr_->Read(buffer + offset, size - offset);
		if (count <= 0) return false;
		offset += static_cast<uint16_t>(count);
	}
	return true;
}

bool SerialMain::ReceiveDataSolve(uint8_t *frame, uint16_t available_length)
{
	uint8_t index = 0;
	uint16_t cmd_id = 0;
	constexpr uint16_t minimum_frame_size =
		sizeof(io::FrameHeader) + sizeof(uint16_t) + sizeof(uint16_t);
	
	if (frame == nullptr || available_length < minimum_frame_size || *frame != io::HEADER_SOF)
	{
		return false;
	}
	
	memcpy(&frame_receive_header_, frame, sizeof(io::FrameHeader));
	index += sizeof(io::FrameHeader);
	
	const uint16_t frame_size_without_end =
		frame_receive_header_.data_length + sizeof(io::FrameHeader) + sizeof(uint16_t) + sizeof(uint16_t);
	if (frame_size_without_end != available_length) return false;
	if ((!Verify_CRC8_Check_Sum(frame, sizeof(io::FrameHeader))) ||
		(!Verify_CRC16_Check_Sum(frame, frame_size_without_end)))
	{
		std::cout<<"CRC error!!"<<std::endl;
		return false;
	}
	else
	{
		memcpy(&cmd_id, frame + index, sizeof(uint16_t));
		index += sizeof(uint16_t);
		// printf("id:%x\n", cmd_id);
		switch (cmd_id)
		{
			case io::VISION_ID:
			{
				if (frame_receive_header_.data_length != sizeof(io::VisionData)) return false;
				memcpy(&vision_msg_, frame + index, sizeof(io::VisionData));
                //---------------------serial_main  data------------
//                std::cout<<"-----serial_main  data------"<<std::endl;
//				std::cout<<"mode:"<<vision_msg_.mode<<std::endl;
//				std::cout<<"yaw:"<<vision_msg_.yaw<<std::endl;
//				std::cout<<"pitch:"<<vision_msg_.pitch<<std::endl;
//				std::cout<<"quat0:"<<vision_msg_.quaternion[0]<<std::endl;
//				std::cout<<"quat1:"<<vision_msg_.quaternion[1]<<std::endl;
//				std::cout<<"quat2:"<<vision_msg_.quaternion[2]<<std::endl;
//				std::cout<<"quat3:"<<vision_msg_.quaternion[3]<<std::endl;
			}
				return true;
			default:
				return false;
		}
	}
}

uint16_t SerialMain::SenderPackSolve(const uint8_t *data, uint16_t data_length,
									 uint16_t cmd_id, uint8_t *send_buf)
{
	
	uint8_t index = 0;
	frame_send_header_.sof = io::HEADER_SOF;
	frame_send_header_.data_length = data_length;
	frame_send_header_.seq++;
	
	Append_CRC8_Check_Sum(
		reinterpret_cast<uint8_t *>(&frame_send_header_), sizeof(io::FrameHeader));
	
	memcpy(send_buf, &frame_send_header_, sizeof(io::FrameHeader));//assign frame header
	
	index += sizeof(io::FrameHeader);
	
	memcpy(send_buf + index, &cmd_id, sizeof(uint16_t));//assign cmd
	
	index += sizeof(uint16_t);
	
	memcpy(send_buf + index, data, data_length);//assign data
	
	const uint16_t frame_size_without_end =
		data_length + sizeof(io::FrameHeader) + sizeof(uint16_t) + sizeof(uint16_t);
	Append_CRC16_Check_Sum(send_buf, frame_size_without_end);
	
	const io::MsgEndInfo end;
	memcpy(send_buf + frame_size_without_end, &end, sizeof(end));
	return frame_size_without_end + sizeof(end);
}
