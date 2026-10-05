#include "klv_decode_interface.hpp"
#include "app_logging.h"

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

    if (g_str_has_prefix(new_pad_type, "video/x-h264"))
    {
        sink_pad = gst_element_get_static_pad(data->videoQueue, "sink");
    }
    else if (g_str_has_prefix(new_pad_type, "meta/x-klv"))
    {
        sink_pad = gst_element_get_static_pad(data->dataQueue, "sink");
    }
    else
    {
        sink = gst_element_factory_make("fakesink", nullptr);
        gst_bin_add(GST_BIN(data->pipeline), sink);
        gst_element_sync_state_with_parent(sink);
        sink_pad = gst_element_get_static_pad(sink, "sink");
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
        LOG_ERROR("Failed to pull sample.");
        return GST_FLOW_ERROR;
    }

    GstBuffer *buffer = gst_sample_get_buffer(sample);
    if (!buffer)
    {
        LOG_ERROR("Failed to get buffer from sample." );
        gst_sample_unref(sample);
        return GST_FLOW_ERROR;
    }
    if (self->decodeKlvData(buffer)){
        self->cnt_klv_frame++;
    }
    else
        LOG_ERROR("Failed to decode KLV data." );

    gst_sample_unref(sample);


    auto now = std::chrono::steady_clock::now();
    std::chrono::duration<double> elapsed = now - self->start_time;
    if (elapsed.count() >= 1.0) {
        LOG_DEBUG("Frames in last second: %d", self->cnt_klv_frame);
        self->cnt_klv_frame = 0;
        self->start_time = now;
    }


    return GST_FLOW_OK;
}

bool 
Klv_Decode_Interface::
decodeKlvData(GstBuffer *buffer)
{
    GstMapInfo map_info;
    if (!gst_buffer_map(buffer, &map_info, GST_MAP_READ))
    {
        LOG_ERROR("Error: Failed to map buffer for reading");
        return false;
    }

    unsigned char *data = reinterpret_cast<unsigned char *>(map_info.data);
    size_t size = map_info.size;

    unpack->unpack_misb(data, size, tag_values);

    gps_sender.update(tag_values);
    printKlvData();

    gst_buffer_unmap(buffer, &map_info);
    return true;
}

void 
Klv_Decode_Interface::
printKlvData()
{
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