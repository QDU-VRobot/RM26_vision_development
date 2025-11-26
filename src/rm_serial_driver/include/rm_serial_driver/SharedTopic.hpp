#pragma once

// clang-format off
/* === MODULE MANIFEST V2 ===
module_description: SharedTopic 是一个基于 UART 的多 Topic 数据共享与解析服务端模块 / SharedTopic is a UART-based multi-topic data sharing and parsing server module
constructor_args:
  - uart_name: "usart1"
  - task_stack_depth: 2048
  - buffer_size: 256
  - topic_configs:
    - "topic1"
    - ["topic2", "libxr_def_domain"]
template_args: []
required_hardware: uart_name, ramfs
depends: []
=== END MANIFEST === */
// clang-format on

#include <cstdint>
#include <cstring>

#include "app_framework.hpp"
#include "libxr_def.hpp"
#include "libxr_rw.hpp"
#include "libxr_type.hpp"
#include "logger.hpp"
#include "message.hpp"
#include "semaphore.hpp"
#include "thread.hpp"
#include "uart.hpp"

class SharedTopic : public LibXR::Application
{
 public:
  struct TopicConfig
  {
    const char* name;
    const char* domain = "libxr_def_domain";

    TopicConfig(const char* name) : name(name) {}

    TopicConfig(const char* name, const char* domain) : name(name), domain(domain) {}
  };

  SharedTopic(LibXR::HardwareContainer& hw, LibXR::ApplicationManager& app,
              const char* uart_name, uint32_t task_stack_depth, uint32_t buffer_size,
              std::initializer_list<TopicConfig> topic_configs)
      : uart_(hw.template Find<LibXR::UART>(uart_name)),
        server_(buffer_size),
        rx_buffer_(new uint8_t[buffer_size], buffer_size),
        cmd_name_(new char[sizeof("shared_topic:") + strlen(uart_name)]),
        cmd_file_((strcpy(cmd_name_, "shared_topic:"),
                   strcpy(cmd_name_ + strlen("shared_topic:"), uart_name),
                   LibXR::RamFS::CreateFile(cmd_name_, CommandFunc, this)))
  {
    for (auto config : topic_configs)
    {
      auto domain = LibXR::Topic::Domain(config.domain);
      auto topic = LibXR::Topic::Find(config.name, &domain);
      if (topic == nullptr)
      {
        XR_LOG_ERROR("Topic not found: %s/%s", config.domain, config.name);
        ASSERT(false);
      }
      server_.Register(topic);
    }

    hw.template FindOrExit<LibXR::RamFS>({"ramfs"})->Add(cmd_file_);

    rx_thread_.Create(this, RxThreadFun, "SharedTopic::RxThread", task_stack_depth,
                      LibXR::Thread::Priority::REALTIME);

    app.Register(*this);
  }

  static void RxThreadFun(SharedTopic* self)
  {
    LibXR::Semaphore sem;
    LibXR::ReadOperation op(sem);
    while (true)
    {
      self->uart_->Read({nullptr, 0}, op);
      auto size =
          LibXR::max(sizeof(LibXR::Topic::PackedDataHeader),
                     LibXR::min(self->uart_->read_port_->Size(), self->rx_buffer_.size_));
      auto ans = self->uart_->Read(LibXR::RawData{self->rx_buffer_.addr_, size}, op);

      if (ans == ErrorCode::OK)
      {
        self->server_.ParseData(LibXR::RawData{self->rx_buffer_.addr_, size});
      }

      self->rx_count_ += size;
    }
  }

  void OnMonitor() override {}

  static int CommandFunc(SharedTopic* self, int argc, char** argv)
  {
    if (argc == 1)
    {
      LibXR::STDIO::Printf("Usage:\r\n");
      LibXR::STDIO::Printf("  monitor [time_ms] [interval_ms] - test received speed\r\n");
      return 0;
    }
    else if (argc == 4)
    {
      if (strcmp(argv[1], "monitor") == 0)
      {
        int time = atoi(argv[2]);
        int delay = atoi(argv[3]);
        auto start = self->rx_count_;
        while (time > 0)
        {
          LibXR::Thread::Sleep(delay);
          LibXR::STDIO::Printf(
              "%f Mbps\r\n", static_cast<float>(self->rx_count_ - start) * 8.0 / 1024.0 /
                                 1024.0 / delay * 1000.0);
          time -= delay;
          start = self->rx_count_;
        }
      }
    }
    else
    {
      LibXR::STDIO::Printf("Error: Invalid arguments.\r\n");
      return -1;
    }

    return 0;
  }

 private:
  LibXR::UART* uart_;

  LibXR::Topic::Server server_;

  LibXR::RawData rx_buffer_;

  size_t rx_count_ = 0;

  char* cmd_name_;

  LibXR::RamFS::File cmd_file_;

  LibXR::Thread rx_thread_;
  LibXR::Thread tx_thread_;
};
