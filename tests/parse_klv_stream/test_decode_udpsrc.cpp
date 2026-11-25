#include "klv_decode_interface.hpp"
#include "app_logging.h"
#include "gps_simulator.hpp"
#include "jpeg_tcp_forwarder.hpp"

#include <atomic>
#include <chrono>
#include <algorithm>
#include <csignal>
#include <cstdlib>
#include <memory>
#include <string>
#include <thread>
#include <vector>

#include <unistd.h>

static std::atomic<bool> g_quit{false};

static void onSigint(int)
{
    if (g_quit.exchange(true))
        _exit(130);
}

// ---------------------------------------------------------------------------
// Stream watchdog
//
// When the sender restarts, its RTP sequence numbers and timestamps and its
// MPEG-TS PTS/PCR all start over. rtpjitterbuffer, tsdemux and the decoder
// still hold the old session's state, so the new stream can be dropped
// forever. The fix is to reset the receiver pipeline (NULL -> PLAYING) when:
//   1. udpsrc gets no data for UDP_TIMEOUT_NS (sender stopped for a while), or
//   2. a new RTP session is detected (sender restarted within UDP_TIMEOUT_NS):
//        a. the SSRC changes, or
//        b. the sequence number jumps (not continuous with the last packet), or
//        c. data resumes after a gap longer than RESUME_GAP_US.
//      (b) and (c) also cover senders that keep the same SSRC after a restart.
//   3. the pipeline posts an error or EOS.
// The pipeline state is only changed from the bus thread, never from a pad
// probe or appsink callback (those run on streaming threads and would deadlock).
// ---------------------------------------------------------------------------
static const guint64 UDP_TIMEOUT_NS = 2 * GST_SECOND;

// A pause in the data longer than this, followed by new data, is treated as a
// sender restart. A sender restart always leaves a gap (process start, encoder
// init). Raise it if the link has normal stalls longer than this.
static const gint64 RESUME_GAP_US = 500 * 1000;

// Sequence number distance (mod 2^16) still treated as the same session:
// forward = packet loss, backward = reordering.
static const guint16 SEQ_MAX_FORWARD = 3000;
static const guint16 SEQ_MAX_BACKWARD = 100;

struct StreamWatch
{
    std::atomic<bool> lost{false};        // no data since the last udpsrc timeout
    std::atomic<bool> restart_req{false}; // set by the probe or bus, handled by busMessageLoop
    std::atomic<bool> have_session{false};

    // Only accessed from the udpsrc streaming thread (the probe), except
    // have_session which restartPipeline() clears while the pipeline is in NULL.
    guint32 ssrc = 0;
    guint16 last_seq = 0;
    gint64 last_arrival_us = 0;
};

// Probe on the udpsrc src pad: each buffer is one RTP packet.
static GstPadProbeReturn onRtpPacket(GstPad *, GstPadProbeInfo *info, gpointer user)
{
    auto *w = static_cast<StreamWatch *>(user);
    GstBuffer *buf = GST_PAD_PROBE_INFO_BUFFER(info);
    if (!buf)
        return GST_PAD_PROBE_OK;

    if (w->lost.exchange(false))
        LOG_INFO("Stream resumed");

    // RTP fixed header: byte 0 bits 7-6 = version (2), bytes 2-3 = sequence
    // number, bytes 8-11 = SSRC (big endian). Read directly so no extra
    // gstreamer-rtp-1.0 dependency is needed.
    guint8 hdr[12];
    if (gst_buffer_extract(buf, 0, hdr, sizeof(hdr)) != sizeof(hdr) || (hdr[0] >> 6) != 2)
        return GST_PAD_PROBE_OK;
    const guint16 seq = guint16((hdr[2] << 8) | hdr[3]);
    const guint32 ssrc = (guint32(hdr[8]) << 24) | (guint32(hdr[9]) << 16) |
                         (guint32(hdr[10]) << 8) | guint32(hdr[11]);
    const gint64 now_us = g_get_monotonic_time();

    if (!w->have_session.load())
    {
        LOG_INFO("RTP session start: SSRC 0x%08x, seq %u", ssrc, seq);
        w->have_session = true;
    }
    else if (!w->restart_req.load()) // a restart is already pending: do not log again
    {
        const gint64 gap_us = now_us - w->last_arrival_us;
        const guint16 expected = guint16(w->last_seq + 1);
        const guint16 fwd = guint16(seq - expected);   // > 0: packets skipped
        const guint16 back = guint16(expected - seq);  // > 0: packet older than expected

        if (ssrc != w->ssrc)
        {
            LOG_WARNING("New RTP session: SSRC 0x%08x -> 0x%08x (seq %u -> %u, gap %lld ms)",
                        w->ssrc, ssrc, w->last_seq, seq, (long long)(gap_us / 1000));
            w->restart_req = true;
        }
        else if (fwd > SEQ_MAX_FORWARD && back > SEQ_MAX_BACKWARD)
        {
            LOG_WARNING("New RTP session: seq jump %u -> %u (gap %lld ms)",
                        w->last_seq, seq, (long long)(gap_us / 1000));
            w->restart_req = true;
        }
        else if (gap_us > RESUME_GAP_US)
        {
            LOG_WARNING("New RTP session: data resumed after %lld ms (seq %u -> %u)",
                        (long long)(gap_us / 1000), w->last_seq, seq);
            w->restart_req = true;
        }
    }

    w->ssrc = ssrc;
    w->last_seq = seq;
    w->last_arrival_us = now_us;
    return GST_PAD_PROBE_OK;
}

static bool restartPipeline(GstElement *pipeline, StreamWatch *w)
{
    LOG_INFO("Restarting receiver pipeline");
    gst_element_set_state(pipeline, GST_STATE_NULL);
    gst_element_get_state(pipeline, nullptr, nullptr, 2 * GST_SECOND);
    w->have_session = false; // the next packet starts a new session
    if (gst_element_set_state(pipeline, GST_STATE_PLAYING) == GST_STATE_CHANGE_FAILURE)
    {
        LOG_ERROR("Restart failed: unable to set the pipeline to PLAYING, will retry");
        return false;
    }
    LOG_INFO("Pipeline restarted, waiting for stream");
    return true;
}

static void busMessageLoop(GstElement *pipeline, StreamWatch *watch)
{
    using Clock = std::chrono::steady_clock;
    const auto min_restart_gap = std::chrono::seconds(1); // avoid restart storms on repeated errors
    auto last_restart = Clock::now() - min_restart_gap;

    GstBus *bus = gst_element_get_bus(pipeline);
    while (!g_quit)
    {
        GstMessage *msg = gst_bus_timed_pop_filtered(bus, 100 * GST_MSECOND,
                                                     (GstMessageType)(GST_MESSAGE_STATE_CHANGED | GST_MESSAGE_ERROR |
                                                                      GST_MESSAGE_EOS | GST_MESSAGE_ELEMENT));

        if (msg != nullptr)
        {
            switch (GST_MESSAGE_TYPE(msg))
            {
            case GST_MESSAGE_ERROR:
            {
                GError *err = nullptr;
                gchar *debug_info = nullptr;
                gst_message_parse_error(msg, &err, &debug_info);
                LOG_ERROR("Error received from element %s: %s", GST_OBJECT_NAME(msg->src), err->message);
                LOG_ERROR("Debugging information: %s", debug_info ? debug_info : "none");
                g_clear_error(&err);
                g_free(debug_info);
                watch->restart_req = true;
                break;
            }
            case GST_MESSAGE_EOS:
                LOG_WARNING("End-Of-Stream reached, restarting.");
                watch->restart_req = true;
                break;
            case GST_MESSAGE_ELEMENT:
                if (gst_message_has_name(msg, "GstUDPSrcTimeout"))
                {
                    // Posted every UDP_TIMEOUT_NS while there is no data. Reset only once per outage.
                    if (!watch->lost.exchange(true))
                    {
                        LOG_WARNING("No data for %.1f s: stream lost", (double)UDP_TIMEOUT_NS / GST_SECOND);
                        watch->restart_req = true;
                    }
                }
                break;
            case GST_MESSAGE_STATE_CHANGED:
                if (GST_MESSAGE_SRC(msg) == GST_OBJECT(pipeline))
                {
                    GstState old_state, new_state, pending_state;
                    gst_message_parse_state_changed(msg, &old_state, &new_state, &pending_state);
                    LOG_DEBUG("Pipeline state changed from %s to %s",
                            gst_element_state_get_name(old_state), gst_element_state_get_name(new_state));
                }
                break;
            default:
                break;
            }
            gst_message_unref(msg);
        }

        if (watch->restart_req && Clock::now() - last_restart >= min_restart_gap)
        {
            watch->restart_req = false;
            last_restart = Clock::now();
            if (!restartPipeline(pipeline, watch))
                watch->restart_req = true;
        }
    }
    gst_object_unref(bus);
}

struct PreviewCtx
{
    GstElement *filter = nullptr;
    int max_width = 1280;
};

static GstPadProbeReturn onPreviewCaps(GstPad *, GstPadProbeInfo *info, gpointer user)
{
    auto *ctx = static_cast<PreviewCtx *>(user);
    GstEvent *ev = GST_PAD_PROBE_INFO_EVENT(info);
    if (GST_EVENT_TYPE(ev) != GST_EVENT_CAPS)
        return GST_PAD_PROBE_OK;
    GstCaps *in = nullptr;
    gst_event_parse_caps(ev, &in);
    const GstStructure *st = in ? gst_caps_get_structure(in, 0) : nullptr;
    int w = 0, h = 0;
    if (st && gst_structure_get_int(st, "width", &w) && gst_structure_get_int(st, "height", &h) && w > 0 && h > 0)
    {
        const int tw = w < ctx->max_width ? (w & ~1) : ctx->max_width;
        const int th = static_cast<int>(static_cast<double>(h) * tw / w + 0.5) & ~1;
        GstCaps *out = gst_caps_new_simple("video/x-raw", "width", G_TYPE_INT, tw,
                                           "height", G_TYPE_INT, th, nullptr);
        g_object_set(G_OBJECT(ctx->filter), "caps", out, nullptr);
        gst_caps_unref(out);
        LOG_DEBUG("Video preview: %dx%d -> %dx%d", w, h, tw, th);
    }
    return GST_PAD_PROBE_OK;
}

int main(int argc, char *argv[])
{
    Klv_Decode_Interface::CustomData data;
    GstAppSink *videoSink = nullptr;
    GstAppSink *dataSink = nullptr;

    std::unique_ptr<Klv_Decode_Interface> decoder_klv(new Klv_Decode_Interface());

    gst_init(&argc, &argv);

    // Options: [--no-gps] [--no-video] and the default GPS as "lat lon [alt]"
    // or "--lat X --lon Y --alt Z".
    GpsFix gps = {10.836414, 106.713832, 30.0};
    bool use_gps = true;
    int qgc_port = 0;
    bool show_stats = false;
    bool use_video = true;
    int vwidth = 1280, vfps = 30, vquality = 75;
    bool use_h265 = false; 
    std::vector<std::string> gps_args;
    for (int i = 1; i < argc; ++i)
    {
        const std::string a = argv[i];
        if (a == "--no-gps")
            use_gps = false;
        else if (a == "--no-video")
            use_video = false;
        else if (a == "--stats")
            show_stats = true;
        else if (a == "--qgc")
            qgc_port = 5600;
        else if (a == "--qgc-port" && i + 1 < argc)
            qgc_port = std::atoi(argv[++i]);
        else if (a == "--video-width" && i + 1 < argc)
            vwidth = std::atoi(argv[++i]);
        else if (a == "--video-fps" && i + 1 < argc)
            vfps = std::atoi(argv[++i]);
        else if (a == "--video-quality" && i + 1 < argc)
            vquality = std::atoi(argv[++i]);
        else if (a == "--h264")
            use_h265 = false;
        else if (a == "--h265")
            use_h265 = true;
        else
            gps_args.push_back(a);
    }
    vwidth = std::min(3840, std::max(160, vwidth)) & ~1;
    vfps = std::min(60, std::max(1, vfps));
    vquality = std::min(100, std::max(10, vquality));
    if (qgc_port < 0 || qgc_port > 65535)
        qgc_port = 0;
    std::string arg_err;
    if (!parseGpsArgs(gps_args, gps, arg_err))
    {
        LOG_ERROR("Invalid GPS option: %s", arg_err.c_str());
        return -1;
    }

    std::string _port = "8554";

    const std::string raw_sink =
        "appsink name=videoSink emit-signals=false async=false sync=false drop=true max-buffers=2 ";
    const int vheight0 = (vwidth * 9 / 16) & ~1;
    const std::string video_tail = use_video
        ? ("tee name=vtee "
           "vtee. ! queue name=rawQueue max-size-buffers=2 max-size-bytes=0 max-size-time=0 leaky=downstream ! " + raw_sink +
           "vtee. ! queue name=jpegQueue max-size-buffers=2 leaky=downstream ! "
           "videorate ! video/x-raw,framerate=" + std::to_string(vfps) + "/1 ! "
           "videoscale ! capsfilter name=jpegScale caps=video/x-raw,width=" + std::to_string(vwidth) +
           ",height=" + std::to_string(vheight0) + " ! "
           "videoconvert ! jpegenc quality=" + std::to_string(vquality) + " ! "
           "appsink name=jpegSink emit-signals=false async=false sync=false drop=true max-buffers=2 ")
        : raw_sink;

    const std::string parser  = use_h265 ? "h265parse"  : "h264parse";
    const std::string decoder = use_h265 ? "avdec_h265" : "avdec_h264";
    const std::string payer   = use_h265 ? "rtph265pay" : "rtph264pay";
    LOG_DEBUG("Video codec: %s", use_h265 ? "H.265" : "H.264");

    const std::string dec_head = qgc_port
        ? "tee name=ht ht. ! queue name=decQueue ! " + decoder + " name=decoder ! "
        : decoder + " name=decoder ! ";
    const std::string qgc_branch = qgc_port
        ? ("ht. ! queue name=qgcQueue max-size-buffers=4 leaky=downstream ! " +
           payer + " config-interval=1 pt=96 ! "
           "udpsink host=127.0.0.1 port=" + std::to_string(qgc_port) + " sync=false async=false ")
        : std::string();

    // reuse=false: only one process may bind this port. With reuse=true (the
    // default) a second receiver on the same port binds silently and the
    // kernel then hands each sender (each source port) to one of the two
    // sockets at random, so some sender restarts never reach this process.
    // With reuse=false a second instance fails with "Could not bind" instead.
    std::string pipeline_desc =
        "udpsrc name=source port=" + _port + " reuse=false timeout=" + std::to_string(UDP_TIMEOUT_NS) + " "
        "caps=application/x-rtp,media=video,clock-rate=90000,encoding-name=MP2T,payload=96 ! rtpjitterbuffer latency=30 ! rtpmp2tdepay "
        "! tsdemux name=demux ! "
        "queue max-size-buffers=0 max-size-bytes=0 max-size-time=0 name=videoQueue ! " +
        parser + " name=parser ! " +
        dec_head +
        video_tail +
        qgc_branch +
        "demux. ! "
        "queue max-size-buffers=60 max-size-bytes=0 max-size-time=0 name=dataQueue ! "
        "appsink name=dataSink emit-signals=false async=false sync=false drop=false max-buffers=60";

    GError *perr = nullptr;
    data.pipeline = gst_parse_launch(pipeline_desc.c_str(), &perr);
    if (!data.pipeline)
    {
        LOG_ERROR("Failed to create pipeline: %s", perr ? perr->message : "unknown");
        g_clear_error(&perr);
        return -1;
    }
    else{
        LOG_INFO("The reciever pipeline: %s", pipeline_desc.c_str());
    }
    g_clear_error(&perr);

    data.source = gst_bin_get_by_name(GST_BIN(data.pipeline), "source");
    data.tsDemux = gst_bin_get_by_name(GST_BIN(data.pipeline), "demux");
    data.videoQueue = gst_bin_get_by_name(GST_BIN(data.pipeline), "videoQueue");
    data.parser = gst_bin_get_by_name(GST_BIN(data.pipeline), "parser");
    data.decoder = gst_bin_get_by_name(GST_BIN(data.pipeline), "decoder");
    data.videoSink = gst_bin_get_by_name(GST_BIN(data.pipeline), "videoSink");
    data.dataQueue = gst_bin_get_by_name(GST_BIN(data.pipeline), "dataQueue");
    data.dataSink = gst_bin_get_by_name(GST_BIN(data.pipeline), "dataSink");

    if (!data.source || !data.tsDemux || !data.videoQueue || !data.parser ||
        !data.decoder || !data.videoSink || !data.dataQueue || !data.dataSink)
    {
        LOG_ERROR("Not all elements could be found in the pipeline.");
        gst_object_unref(data.pipeline);
        return -1;
    }

    videoSink = GST_APP_SINK(data.videoSink);
    dataSink = GST_APP_SINK(data.dataSink);

    decoder_klv->setDemuxCallback(data.tsDemux, data);
    decoder_klv->setDataSinkCallback(dataSink, &data);
    decoder_klv->setVideoSinkCallback(videoSink, &data);
    decoder_klv->setPipeline(data.pipeline);

    JpegTcpForwarder jpeg_forwarder("127.0.0.1", 5006);
    PreviewCtx preview_ctx;
    preview_ctx.max_width = vwidth;
    if (use_video)
    {
        GstElement *jpegSink = gst_bin_get_by_name(GST_BIN(data.pipeline), "jpegSink");
        GstElement *jpegQueue = gst_bin_get_by_name(GST_BIN(data.pipeline), "jpegQueue");
        GstElement *jpegScale = gst_bin_get_by_name(GST_BIN(data.pipeline), "jpegScale");
        if (jpegSink && jpegQueue && jpegScale)
        {
            jpeg_forwarder.attach(GST_APP_SINK(jpegSink));
            preview_ctx.filter = jpegScale;
            GstPad *qpad = gst_element_get_static_pad(jpegQueue, "sink");
            gst_pad_add_probe(qpad, GST_PAD_PROBE_TYPE_EVENT_DOWNSTREAM, onPreviewCaps, &preview_ctx, nullptr);
            gst_object_unref(qpad);
        }
        else
            LOG_ERROR("Preview elements not found, video preview disabled.");
    }

    // Watchdog: watch RTP packets on udpsrc to detect a sender restart.
    StreamWatch watch;
    {
        GstPad *src_pad = gst_element_get_static_pad(data.source, "src");
        gst_pad_add_probe(src_pad, GST_PAD_PROBE_TYPE_BUFFER, onRtpPacket, &watch, nullptr);
        gst_object_unref(src_pad);
    }

    GstStateChangeReturn ret = gst_element_set_state(data.pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE)
    {
        LOG_ERROR("Unable to set the pipeline to the playing state.");
        GstBus *bus = gst_element_get_bus(data.pipeline);
        GstMessage *msg = gst_bus_pop_filtered(bus, GST_MESSAGE_ERROR);
        if (msg)
        {
            GError *err = nullptr;
            gchar *dbg = nullptr;
            gst_message_parse_error(msg, &err, &dbg);
            LOG_ERROR("%s: %s (%s)", GST_OBJECT_NAME(msg->src), err->message, dbg ? dbg : "");
            g_clear_error(&err);
            g_free(dbg);
            gst_message_unref(msg);
        }
        gst_object_unref(bus);
        LOG_ERROR("If udpsrc failed to bind, UDP port %s is already used by another process "
                  "(maybe another Test_Decode_KLV). Check: sudo ss -ulpn 'sport = :%s'",
                  _port.c_str(), _port.c_str());
        gst_element_set_state(data.pipeline, GST_STATE_NULL);
        gst_object_unref(data.pipeline);
        return -1;
    }

    signal(SIGINT, onSigint);

    GpsSimulator *sim = nullptr;
    if (use_gps)
    {
        sim = new GpsSimulator(gps);
        sim->start();
        std::thread(runConsole, sim, &g_quit).detach();
    }

    std::atomic<bool> stats_stop{false};
    std::thread stats_thread;
    if (show_stats)
        stats_thread = std::thread([&]() {
            while (!stats_stop)
            {
                guint v = 0, d = 0;
                g_object_get(data.videoQueue, "current-level-buffers", &v, nullptr);
                g_object_get(data.dataQueue, "current-level-buffers", &d, nullptr);
                LOG_DEBUG("[stats] queued buffers: video=%u klv=%u", v, d);
                for (int i = 0; i < 10 && !stats_stop; ++i)
                    std::this_thread::sleep_for(std::chrono::milliseconds(100));
            }
        });

    busMessageLoop(data.pipeline, &watch);

    stats_stop = true;
    if (stats_thread.joinable())
        stats_thread.join();

    if (sim)
        sim->stop();

    gst_element_set_state(data.pipeline, GST_STATE_NULL);
    gst_object_unref(data.pipeline);

    return 0;
}
