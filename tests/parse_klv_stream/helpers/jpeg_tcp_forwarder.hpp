#pragma once
#include <arpa/inet.h>
#include <netinet/in.h>
#include <netinet/tcp.h>
#include <sys/ioctl.h>
#include <sys/socket.h>
#include <sys/time.h>
#include <unistd.h>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <string>

#include "app_logging.h"

class JpegTcpForwarder
{
public:
    JpegTcpForwarder(const std::string &ip = "127.0.0.1", int port = 5006) : ip_(ip), port_(port) {}
    ~JpegTcpForwarder() { closeSock(); }
    JpegTcpForwarder(const JpegTcpForwarder &) = delete;
    JpegTcpForwarder &operator=(const JpegTcpForwarder &) = delete;

    void attach(GstAppSink *sink)
    {
        GstAppSinkCallbacks cb{};
        cb.new_sample = &JpegTcpForwarder::onSample;
        gst_app_sink_set_callbacks(sink, &cb, this, nullptr);
    }

private:
    using Clock = std::chrono::steady_clock;

    static GstFlowReturn onSample(GstAppSink *sink, gpointer user)
    {
        auto *self = static_cast<JpegTcpForwarder *>(user);
        GstSample *sample = gst_app_sink_pull_sample(sink);
        if (!sample)
            return GST_FLOW_OK;
        GstBuffer *buf = gst_sample_get_buffer(sample);
        GstMapInfo map;
        if (buf && gst_buffer_map(buf, &map, GST_MAP_READ))
        {
            self->send(map.data, map.size);
            gst_buffer_unmap(buf, &map);
        }
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    bool connectNow()
    {
        const auto now = Clock::now();
        if (now - last_try_ < std::chrono::seconds(1))
            return false;
        last_try_ = now;
        fd_ = socket(AF_INET, SOCK_STREAM, 0);
        if (fd_ < 0)
            return false;
        sockaddr_in a{};
        a.sin_family = AF_INET;
        a.sin_port = htons(port_);
        inet_pton(AF_INET, ip_.c_str(), &a.sin_addr);
        if (connect(fd_, reinterpret_cast<sockaddr *>(&a), sizeof(a)) != 0)
        {
            closeSock();
            return false;
        }
        int one = 1;
        setsockopt(fd_, IPPROTO_TCP, TCP_NODELAY, &one, sizeof(one));
        timeval tv{0, 200000};         // a stuck viewer blocks us for 200 ms at most
        setsockopt(fd_, SOL_SOCKET, SO_SNDTIMEO, &tv, sizeof(tv));
        LOG_DEBUG("Video preview: connected to the map viewer (%s:%d)", ip_.c_str(), port_);
        return true;
    }

    void closeSock()
    {
        if (fd_ >= 0)
        {
            close(fd_);
            fd_ = -1;
        }
    }

    bool sendAll(const uint8_t *d, size_t n)
    {
        while (n)
        {
            const ssize_t k = ::send(fd_, d, n, MSG_NOSIGNAL);
            if (k <= 0)
                return false;
            d += k;
            n -= static_cast<size_t>(k);
        }
        return true;
    }

    void send(const uint8_t *d, size_t n)
    {
        if (n == 0 || n > kMaxFrame)
            return;
        if (fd_ < 0 && !connectNow())
            return;
        int pending = 0;
        if (ioctl(fd_, TIOCOUTQ, &pending) == 0 && pending > static_cast<int>(3 * n))
            return;
        const uint32_t len = htonl(static_cast<uint32_t>(n));
        if (!sendAll(reinterpret_cast<const uint8_t *>(&len), 4) || !sendAll(d, n))
        {
            LOG_DEBUG("Video preview: viewer disconnected");
            closeSock();
        }
    }

    static constexpr size_t kMaxFrame = 8u * 1024 * 1024;
    std::string ip_;
    int port_;
    int fd_ = -1;
    Clock::time_point last_try_{};
};