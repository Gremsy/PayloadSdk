#pragma once
#include <arpa/inet.h>
#include <sys/socket.h>
#include <unistd.h>

#include <cmath>
#include <iomanip>
#include <sstream>
#include <string>
#include <vector>


class KlvGpsSender
{
    using Clock = std::chrono::steady_clock;

public:
    KlvGpsSender(const std::string &ip = "127.0.0.1", int port = 5005)
    {
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        addr_.sin_family = AF_INET;
        addr_.sin_port = htons(port);
        inet_pton(AF_INET, ip.c_str(), &addr_.sin_addr);
    }
    ~KlvGpsSender() { if (sock_ >= 0) close(sock_); }
    KlvGpsSender(const KlvGpsSender &) = delete;
    KlvGpsSender &operator=(const KlvGpsSender &) = delete;

    bool update(const std::vector<Unpack::TagValuePair> &tags)
    {
        const auto now = Clock::now();
        for (const auto &p : tags)
        {
            const std::string &n = p.tag_name;
            if (n == "Frame Center Latitude")         { clat_ = toDouble(p.value); fp_t_ = now; }
            else if (n == "Frame Center Longitude")   { clon_ = toDouble(p.value); fp_t_ = now; }
            else if (n == "Sensor Latitude")          { slat_ = toDouble(p.value); cam_t_ = now; }
            else if (n == "Sensor Longitude")         { slon_ = toDouble(p.value); cam_t_ = now; }
            else if (n == "Sensor True Altitude")     salt_ = toDouble(p.value);
            else if (n == "Sensor Horizontal FOV")    hfov_ = toDouble(p.value);
            else if (n == "Sensor Vertical FOV")      vfov_ = toDouble(p.value);
            else
                for (int i = 0; i < 4; ++i)
                {
                    const std::string idx = std::to_string(i + 1);
                    if (n == "Offset Corner Latitude Point " + idx)       { dlat_[i] = toDouble(p.value); fp_t_ = now; }
                    else if (n == "Offset Corner Longitude Point " + idx) { dlon_[i] = toDouble(p.value); fp_t_ = now; }
                }
        }

        const bool fp  = fresh(fp_t_, now)  && std::isfinite(clat_) && std::isfinite(clon_);
        const bool cam = fresh(cam_t_, now) && std::isfinite(slat_) && std::isfinite(slon_);
        if (!fp && !cam)
            return false;

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(9);
        bool first = true;
        auto add = [&](int id, double a, double b) {
            if (!first) oss << ";";
            first = false;
            oss << id << "," << a << "," << b;
        };

        if (fp)
        {
            add(1, clat_, clon_);
            bool all = true;
            for (int i = 0; i < 4; ++i)
                all = all && std::isfinite(dlat_[i]) && std::isfinite(dlon_[i]);
            if (all)
                for (int i = 0; i < 4; ++i)             // Point 1..4 -> id 2..5 (TL, TR, BR, BL)
                    add(i + 2, clat_ + dlat_[i], clon_ + dlon_[i]);
        }
        if (cam)
        {
            add(10, slat_, slon_);
            if (std::isfinite(salt_)) add(11, salt_, 0);
            if (std::isfinite(hfov_)) add(12, hfov_, std::isfinite(vfov_) ? vfov_ : 0);
        }

        const std::string s = oss.str();
        sendto(sock_, s.c_str(), s.size(), 0, reinterpret_cast<const sockaddr *>(&addr_), sizeof(addr_));
        return true;
    }

private:
    static bool fresh(Clock::time_point t, Clock::time_point now)
    {
        return std::chrono::duration<double>(now - t).count() < 1.0;
    }

    static double toDouble(const decltype(Unpack::TagValuePair::value) &v)
    {
        switch (v.type)
        {
        case Format::UINT8:  return v.uint8_value;
        case Format::UINT16: return v.uint16_value;
        case Format::UINT32: return v.uint32_value;
        case Format::UINT64: return static_cast<double>(v.uint64_value);
        case Format::INT8:   return v.int8_value;
        case Format::INT16:  return v.int16_value;
        case Format::INT32:  return v.int32_value;
        case Format::INT64:  return static_cast<double>(v.int64_value);
        case Format::FLOAT:  return v.float_value;
        case Format::DOUBLE: return v.double_value;
        default:             return NAN;
        }
    }

    int sock_ = -1;
    sockaddr_in addr_{};
    double clat_ = NAN, clon_ = NAN, slat_ = NAN, slon_ = NAN;
    double salt_ = NAN, hfov_ = NAN, vfov_ = NAN;
    double dlat_[4] = {NAN, NAN, NAN, NAN}, dlon_[4] = {NAN, NAN, NAN, NAN};
    Clock::time_point fp_t_{}, cam_t_{};
};