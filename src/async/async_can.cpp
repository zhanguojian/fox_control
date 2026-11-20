/*
 * async_can.cpp
 *
 * Created on: nov 19, 2020 13:23
 * Description:
 *
 * Copyright (c) 
 */

#include "async_port/asybc_can.hpp"

#include <net/if.h>
#include <poll.h>
#include <string.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/can.h>

#include <iostream>

namespace fox {
AsyncCAN::AsyncCAN(std::string can_port)
    : port_(can_port), socketcan_stream_(io_context_) {}

AsyncCAN::~AsyncCAN() { Close(); }

bool AsyncCAN::Open() {
  try {
    // 计算接口名称占用的字节数，并检查是否超过最大长度
    const size_t iface_name_size = strlen(port_.c_str()) + 1;

    //如果名称连同结束符超过容量，就返回失败，避免后面复制时越界
    if (iface_name_size > IFNAMSIZ) return false;

    // 创建CAN套接字,PF_CAN	使用 CAN 协议族,SOCK_RAW	使用原始 CAN socket,SOCK_NONBLOCK	将 socket 设置为非阻塞,使用 CAN RAW 协议
    can_fd_ = socket(PF_CAN, SOCK_RAW | SOCK_NONBLOCK, CAN_RAW);

    //成功时，得到一个非负整数文件描述符。
    if (can_fd_ < 0) return false;

    //创建网络接口查询用的结构体
    struct ifreq ifr;

    //把结构体占用的内存全部清零,清零的地址，填充的字节值，填充的大小
    memset(&ifr, 0, sizeof(ifr));

    //将接口名称复制到结构体中
    memcpy(ifr.ifr_name, port_.c_str(), iface_name_size);

    // 查询接口索引，根据 ifr 中的接口名称查询接口索引。SIOCGIFINDEX指定本次请求是“查询网络接口索引，can_fd_是文件描述符
    const int ioctl_result = ioctl(can_fd_, SIOCGIFINDEX, &ifr);

    //失败时，返回-1
    if (ioctl_result < 0) {
      Close();
      return false;
    }

    //sockaddr_can结构体,创建 CAN 地址结构体
    struct sockaddr_can addr;
    //把结构体占用的内存全部清零,清零的地址，填充的字节值，填充的大小
    memset(&addr, 0, sizeof(addr));
    //填充结构体
    //AF_CAN表示CAN协议族
    addr.can_family = AF_CAN;
    //ifr_ifindex是接口索引
    addr.can_ifindex = ifr.ifr_ifindex;

    //绑定文件描述符和地址,将之前创建的 socket 绑定到指定 CAN 接口。
    const int bind_result =
        bind(can_fd_, (struct sockaddr *)&addr, sizeof(addr));

    //失败时，返回-1
    if (bind_result < 0) {
      Close();
      return false;
    }

    port_opened_ = true;
    std::cout << "Start listening to port: " << port_ << std::endl;
  } catch (std::system_error &e) {
    port_opened_ = false;
    std::cout << e.what() << std::endl;
    return false;
  }

  // give some work to io_service to start async io chain
  socketcan_stream_.assign(can_fd_);

#if ASIO_VERSION < 101200L
  io_context_.post(std::bind(&AsyncCAN::ReadFromPort, this,
                             std::ref(rcv_frame_),
                             std::ref(socketcan_stream_)));
#else
  asio::post(io_context_,
             std::bind(&AsyncCAN::ReadFromPort, this, std::ref(rcv_frame_),
                       std::ref(socketcan_stream_)));
#endif

  // start io thread
  io_thread_ = std::thread([this]() { io_context_.run(); });

  return true;
}

void AsyncCAN::Close() {
  if (!port_opened_.exchange(false) && can_fd_ < 0) {
    return;
  }
  io_context_.stop();
  if (io_thread_.joinable() &&
      io_thread_.get_id() != std::this_thread::get_id()) {
    io_thread_.join();
  }
  io_context_.reset();

  if (can_fd_ >= 0) {
    ::close(can_fd_);
    can_fd_ = -1;
  }
}

bool AsyncCAN::IsOpened() const { return port_opened_; }

void AsyncCAN::DefaultReceiveCallback(can_frame *rx_frame) {
  //std::hex是16进制输出，std::dec是10进制输出
  std::cout << std::hex << rx_frame->can_id << "  ";
  for (int i = 0; i < rx_frame->can_dlc; i++)
    std::cout << std::hex << int(rx_frame->data[i]) << " ";
  std::cout << std::dec << std::endl;
}

void AsyncCAN::ReadFromPort(struct can_frame &rec_frame,
                            asio::posix::basic_stream_descriptor<> &stream) {
  auto sthis = shared_from_this();
  //提交一次异步读取.参数1接收缓冲区,参数2读取完成后执行的函数。
  stream.async_read_some(
      asio::buffer(&rec_frame, sizeof(rec_frame)),
      [sthis](boost::system::error_code error, size_t) {
        if (error) {
          sthis->port_opened_ = false;
          return;
        }

        if (sthis->rcv_cb_ != nullptr)
          sthis->rcv_cb_(&sthis->rcv_frame_);
        else
          //没有自定义回调，就打印原始帧
          sthis->DefaultReceiveCallback(&sthis->rcv_frame_);

        //再次登记读取，形成持续接收
        sthis->ReadFromPort(std::ref(sthis->rcv_frame_),
                            std::ref(sthis->socketcan_stream_));
      });
}

void AsyncCAN::SendFrame(const struct can_frame &frame) {
  // 异步发送期间必须持有帧副本，避免调用方栈变量提前失效。
  auto pending_frame = std::make_shared<struct can_frame>(frame);
  asio::async_write(
      socketcan_stream_, asio::buffer(pending_frame.get(), sizeof(frame)),
      [pending_frame](boost::system::error_code error, size_t) {
        if (error) {
          std::cerr << "Failed to send CAN frame" << std::endl;
        }
      });
}

}  // namespace fox