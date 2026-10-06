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
public:
    KlvGpsSender(const std::string &ip = "127.0.0.1", int port = 5005)
    {
        sock_ = socket(AF_INET, SOCK_DGRAM, 0);
        addr_.sin_family = AF_INET;
        addr_.sin_port = htons(port);
        inet_pton(AF_INET, ip.c_str(), &addr_.sin_addr);
    }

    ~KlvGpsSender()
    {
        if (sock_ >= 0)
            close(sock_);
    }

    KlvGpsSender(const KlvGpsSender &) = delete;
    KlvGpsSender &operator=(const KlvGpsSender &) = delete;

    bool update(const std::vector<Unpack::TagValuePair> &tags)
    {
        double clat = NAN, clon = NAN;
        double dlat[4] = {NAN, NAN, NAN, NAN};
        double dlon[4] = {NAN, NAN, NAN, NAN};

        for (const auto &p : tags)
        {
            const std::string &n = p.tag_name;
            if (n == "Frame Center Latitude")
                clat = toDouble(p.value);
            else if (n == "Frame Center Longitude")
                clon = toDouble(p.value);
            else
            {
                for (int i = 0; i < 4; ++i)
                {
                    const std::string idx = std::to_string(i + 1);
                    if (n == "Offset Corner Latitude Point " + idx)
                        dlat[i] = toDouble(p.value);
                    else if (n == "Offset Corner Longitude Point " + idx)
                        dlon[i] = toDouble(p.value);
                }
            }
        }

        if (!std::isfinite(clat) || !std::isfinite(clon))
            return false;

        bool has_corners = true;
        for (int i = 0; i < 4; ++i)
            has_corners = has_corners && std::isfinite(dlat[i]) && std::isfinite(dlon[i]);

        std::ostringstream oss;
        oss << std::fixed << std::setprecision(9);
        oss << "1," << clat << "," << clon;
        if (has_corners)
            for (int i = 0; i < 4; ++i)
                oss << ";" << (i + 2) << "," << (clat + dlat[i]) << "," << (clon + dlon[i]);

        const std::string s = oss.str();
        sendto(sock_, s.c_str(), s.size(), 0,
               reinterpret_cast<const sockaddr *>(&addr_), sizeof(addr_));
        return true;
    }

private:
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
};