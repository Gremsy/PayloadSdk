#include "klv_decode_interface.hpp"
#include "app_logging.h"

#include <algorithm>
#include <string>

Klv_Decode_Interface::
Klv_Decode_Interface() : unpack(std::make_unique<Unpack>())
{
    cnt_klv_frame=0;
    start_time = std::chrono::steady_clock::now();
}

Klv_Decode_Interface::
~Klv_Decode_Interface()
{
}

void 
Klv_Decode_Interface::
setDemuxCallback(GstElement *demux, CustomData &data)
{
    g_signal_connect(demux, "pad-added", G_CALLBACK(pad_added_handler), &data);
}

void 
Klv_Decode_Interface::
pad_added_handler(GstElement *src, GstPad *new_pad, CustomData *data)
{
    GstElement *sink = nullptr;
    GstPad *sink_pad = nullptr;

    GstCaps *new_pad_caps = gst_pad_get_current_caps(new_pad);
    if (!new_pad_caps)
        return;

    GstStructure *new_pad_struct = gst_caps_get_structure(new_pad_caps, 0);
    if (!new_pad_struct)
    {
        gst_caps_unref(new_pad_caps);
        return;
    }

    const gchar *new_pad_type = gst_structure_get_name(new_pad_struct);
    LOG_DEBUG("Received new pad '%s' from '%s' of type '%s':", GST_PAD_NAME(new_pad), GST_ELEMENT_NAME(src), new_pad_type);

    if (g_str_has_prefix(new_pad_type, "video/x-h264") || g_str_has_prefix(new_pad_type, "video/x-h265"))
    {
        sink_pad = gst_element_get_static_pad(data->videoQueue, "sink");
    }
    else if (g_str_has_prefix(new_pad_type, "meta/x-klv"))
    {
        sink_pad = gst_element_get_static_pad(data->dataQueue, "sink");
    }
    else
    {
        // Unknown stream: route it to a fakesink. Reuse the one created for this
        // pad name on an earlier run, so restarting the pipeline does not keep
        // adding new fakesinks to the bin.
        const std::string fake_name = std::string("fake_") + GST_PAD_NAME(new_pad);
        sink = gst_bin_get_by_name(GST_BIN(data->pipeline), fake_name.c_str());
        if (!sink)
        {
            sink = gst_element_factory_make("fakesink", fake_name.c_str());
            g_object_set(sink, "sync", FALSE, "async", FALSE, nullptr);
            gst_bin_add(GST_BIN(data->pipeline), sink);
            gst_object_ref(sink); // balance the unref below
        }
        gst_element_sync_state_with_parent(sink);
        sink_pad = gst_element_get_static_pad(sink, "sink");
        gst_object_unref(sink);
    }
    GST_DEBUG_BIN_TO_DOT_FILE(GST_BIN(data->pipeline), GST_DEBUG_GRAPH_SHOW_ALL, "pipeline");

    if (!sink_pad || gst_pad_is_linked(sink_pad))
    {
        LOG_DEBUG("We are already linked. Ignoring.");
        gst_caps_unref(new_pad_caps);
        if (sink_pad)
            gst_object_unref(sink_pad);
        return;
    }

    GstPadLinkReturn ret = gst_pad_link(new_pad, sink_pad);
    if (ret != GST_PAD_LINK_OK)
        LOG_ERROR("Type is '%s' but link failed.\n", new_pad_type);
    else
        LOG_DEBUG("Link succeeded (type '%s').\n", new_pad_type);

    gst_caps_unref(new_pad_caps);
    gst_object_unref(sink_pad);
}

void 
Klv_Decode_Interface::
setVideoSinkCallback(GstAppSink *videoSink, CustomData *data)
{
    GstAppSinkCallbacks callbacks{};
    callbacks.new_sample = new_video_sample;
    gst_app_sink_set_callbacks(videoSink, &callbacks, this, nullptr);
}

GstFlowReturn 
Klv_Decode_Interface::
new_video_sample(GstAppSink *videoSink, gpointer user_data)
{
    auto *self = static_cast<Klv_Decode_Interface *>(user_data);

    GstSample *sample = gst_app_sink_pull_sample(videoSink);
    if (!sample)
        return GST_FLOW_OK;

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    if (!buffer) {
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    #if 0
        GstClockTime pts = GST_BUFFER_PTS(buffer);
        LOG_DEBUG("Received PTS: %" G_GUINT64_FORMAT " ns\n", pts);

        if (GST_CLOCK_TIME_IS_VALID(pts)) {
            // Convert PTS to milliseconds
            auto rel_ms = std::chrono::milliseconds(pts / 1000000);
            int total_seconds = rel_ms.count() / 1000;
            int millis = rel_ms.count() % 1000;

            int hours = total_seconds / 3600;
            int minutes = (total_seconds / 60) % 60;
            int seconds = total_seconds % 60;

            std::cout << "[" << self->cnt_frame++ << "] "
                      << "Sender Relative Timestamp: "
                      << std::setfill('0')
                      << std::setw(2) << hours << ":"
                      << std::setw(2) << minutes << ":"
                      << std::setw(2) << seconds << "."
                      << std::setw(3) << millis
                      << std::endl;
        } else {
            std::cout << "[" << self->cnt_frame++ << "] "
                      << "Invalid sender timestamp (PTS is not valid)" << std::endl;
        }
    #endif

    // Optional: process payload
    GstMapInfo map;
    if (gst_buffer_map(buffer, &map, GST_MAP_READ)) {
        // check the frame here

        gst_buffer_unmap(buffer, &map);
    }

    gst_sample_unref(sample);
    return GST_FLOW_OK;
}

void 
Klv_Decode_Interface::
setDataSinkCallback(GstAppSink *dataSink, CustomData *data)
{
    GstAppSinkCallbacks callbacks{};
    callbacks.new_sample = new_data_sample;
    gst_app_sink_set_callbacks(dataSink, &callbacks, this, nullptr);
}

void 
Klv_Decode_Interface::
setPipeline(GstElement *pipeline){
    data.pipeline = pipeline;
}

GstFlowReturn 
Klv_Decode_Interface::
new_data_sample(GstAppSink *dataSink, gpointer user_data)
{
    auto *self = static_cast<Klv_Decode_Interface *>(user_data);

    GstSample *sample = gst_app_sink_pull_sample(dataSink);
    if (!sample)
    {
        // NULL only when appsink is flushing or at EOS, e.g. while the pipeline
        // is being restarted. Returning an error here would post a new error.
        return GST_FLOW_OK;
    }

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    if (!buffer)
    {
        LOG_ERROR("Failed to get buffer from sample.");
        gst_sample_unref(sample);
        return GST_FLOW_OK;
    }

    self->cnt_klv_buffer++;
    self->cnt_klv_frame += self->decodeKlvData(buffer);
    gst_sample_unref(sample);

    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - self->start_time;
    if (elapsed.count() >= 1.0) {
        LOG_DEBUG("Frames in last second: %d (buffers: %d, decode errors: %d)",
                  self->cnt_klv_frame, self->cnt_klv_buffer, self->cnt_klv_error);
        self->cnt_klv_frame = 0;
        self->cnt_klv_buffer = 0;
        self->cnt_klv_error = 0;
        self->start_time = now;
    }

    return GST_FLOW_OK;
}

// 16-byte Universal Key of the MISB ST 0601 UAS Datalink Local Set
static const uint8_t kUasLdsKey[16] = {
    0x06, 0x0E, 0x2B, 0x34, 0x02, 0x0B, 0x01, 0x01,
    0x0E, 0x01, 0x03, 0x01, 0x01, 0x00, 0x00, 0x00};

// Total size (key + BER length + value) of the KLV packet starting at pkt,
// or 0 if the length field is invalid.
static size_t klvPacketSize(const uint8_t *pkt, size_t avail)
{
    size_t idx = sizeof(kUasLdsKey);
    if (idx >= avail)
        return 0;

    size_t value_len = 0;
    const uint8_t first = pkt[idx++];
    if (first < 0x80)
    {
        value_len = first; // BER short form
    }
    else
    {
        const size_t n = first & 0x7F; // BER long form: n length bytes follow
        if (n == 0 || n > 4 || idx + n > avail)
            return 0;
        for (size_t i = 0; i < n; ++i)
            value_len = (value_len << 8) | pkt[idx++];
    }
    return idx + value_len;
}

int 
Klv_Decode_Interface::
decodeKlvData(GstBuffer *buffer)
{
    GstMapInfo map_info;
    if (!gst_buffer_map(buffer, &map_info, GST_MAP_READ))
    {
        LOG_ERROR("Error: Failed to map buffer for reading");
        cnt_klv_error++;
        return 0;
    }

    const uint8_t *buf = map_info.data;
    const size_t size = map_info.size;
    const uint8_t *key_end = kUasLdsKey + sizeof(kUasLdsKey);

    int decoded = 0;
    int errors = 0;
    size_t pos = 0;
    while (pos + sizeof(kUasLdsKey) < size)
    {
        // Find the next packet. Normally it starts right at pos.
        const uint8_t *found = std::search(buf + pos, buf + size, kUasLdsKey, key_end);
        if (found == buf + size)
            break;
        pos = static_cast<size_t>(found - buf);

        const size_t pkt_size = klvPacketSize(buf + pos, size - pos);
        if (pkt_size == 0 || pos + pkt_size > size)
        {
            LOG_WARNING("Truncated or invalid KLV packet at offset %zu (buffer %zu bytes)", pos, size);
            errors++;
            break;
        }

        if (decodeOnePacket(buf + pos, pkt_size))
            decoded++;
        else
            errors++;
        pos += pkt_size;
    }

    if (decoded == 0 && errors == 0)
    {
        LOG_WARNING("No UAS LDS packet found in KLV buffer (%zu bytes)", size);
        errors++;
    }
    cnt_klv_error += errors;

    gst_buffer_unmap(buffer, &map_info);
    return decoded;
}

bool 
Klv_Decode_Interface::
decodeOnePacket(const uint8_t *pkt, size_t size)
{
    tag_values.clear();
    const Unpack::Status st = unpack->unpack_misb(pkt, size, tag_values);
    if (st != Unpack::Status::OK)
    {
        // Same behaviour as before: a packet with a non-OK status is still used
        // if it yielded tags. Only warn, and at most once per second.
        static auto last_warn = std::chrono::steady_clock::time_point{};
        const auto now = std::chrono::steady_clock::now();
        if (now - last_warn >= std::chrono::seconds(1))
        {
            last_warn = now;
            LOG_WARNING("KLV unpack status %d (%zu bytes, %zu tags)",
                        static_cast<int>(st), size, tag_values.size());
        }
        if (tag_values.empty())
            return false;
    }

    gps_sender.update(tag_values);
    printKlvData();
    return true;
}

void Klv_Decode_Interface::printKlvData()
{
    // Print every decoded packet, so the log rate equals the KLV receive rate.
    print_tag_values(tag_values);
    tag_values.clear();
}

void 
Klv_Decode_Interface::
print_tag_values(const std::vector<Unpack::TagValuePair> &tag_values) 
{
    LOG_DEBUG("--Unpacked Tag-Value Pairs:");
    for (const auto &pair : tag_values)
    {
        LOG_DEBUG("-- %s", pair.tag_name.c_str());
        switch (pair.value.type)
        {
        case Format::UINT8:
            LOG_DEBUG("---- %d", static_cast<int>(pair.value.uint8_value));
            break;
        case Format::UINT16:
            LOG_DEBUG("---- %d", pair.value.uint16_value);
            break;
        case Format::UINT32:
            LOG_DEBUG("---- %d", pair.value.uint32_value);
            break;
        case Format::UINT64:
            if (pair.tag_name == "UNIX Time Stamp"){
                LOG_DEBUG("---- [%d] %s", cnt_data, Utils::uint64_to_localtimestamp(pair.value.uint64_value).c_str());
                cnt_data++;
            }
            else
                LOG_DEBUG("---- %d", pair.value.uint64_value);
            break;
        case Format::INT8:
            LOG_DEBUG("---- %d", static_cast<int>(pair.value.int8_value));
            break;
        case Format::INT16:
            LOG_DEBUG("---- %d", pair.value.int16_value);
            break;
        case Format::INT32:
            LOG_DEBUG("---- %d", pair.value.int32_value);
            break;
        case Format::INT64:
            LOG_DEBUG("---- %d", pair.value.int64_value);
            break;
        case Format::FLOAT:
            LOG_DEBUG("---- %.5f", pair.value.float_value);
            break;
        case Format::DOUBLE:
            LOG_DEBUG("---- %.5f", pair.value.double_value);
            break;
        case Format::CHAR_P:
            LOG_DEBUG(pair.value.charp_value ? pair.value.charp_value : "nullptr");
            break;
        default:
            LOG_ERROR("----Unknown Value Type");
            break;
        }
    }
}