#include "klv_decode_interface.hpp"
#include "app_logging.h"

static void busMessageLoop(GstElement *pipeline)
{
    GstBus *bus = gst_element_get_bus(pipeline);
    gboolean terminate = FALSE;
    while (!terminate)
    {
        GstMessage *msg = gst_bus_timed_pop_filtered(bus, GST_CLOCK_TIME_NONE,
                                                     (GstMessageType)(GST_MESSAGE_STATE_CHANGED | GST_MESSAGE_ERROR | GST_MESSAGE_EOS));

        if (msg != nullptr)
        {
            LOG_DEBUG("Message received: %s", GST_MESSAGE_TYPE_NAME(msg));
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
                terminate = TRUE;
                break;
            }
            case GST_MESSAGE_EOS:
                LOG_DEBUG("End-Of-Stream reached.");
                terminate = TRUE;
                break;
            case GST_MESSAGE_STATE_CHANGED:
                if (GST_MESSAGE_SRC(msg) == GST_OBJECT(pipeline))
                {
                    GstState old_state, new_state, pending_state;
                    gst_message_parse_state_changed(msg, &old_state, &new_state, &pending_state);
                    LOG_DEBUG("Pipeline state changed from %s to %s:\n",
                            gst_element_state_get_name(old_state), gst_element_state_get_name(new_state));
                }
                break;
            default:
                LOG_ERROR("Unexpected message received.");
                break;
            }
            gst_message_unref(msg);
        }
    }
    gst_object_unref(bus);
}

int main(int argc, char *argv[])
{
    Klv_Decode_Interface::CustomData data;
    GstAppSink *videoSink = nullptr;
    GstAppSink *dataSink = nullptr;

    std::unique_ptr<Klv_Decode_Interface> decoder_klv(new Klv_Decode_Interface());

    gst_init(&argc, &argv);

    std::string _port = "8554";

    std::string pipeline_desc =
        "udpsrc name=source port=" + _port + " ! "
        "tsdemux name=demux ! "
        "queue max-size-buffers=0 max-size-bytes=0 max-size-time=0 name=videoQueue ! "
        "h264parse name=h264parse ! "
        "avdec_h264 name=avdec_h264 ! "
        "appsink name=videoSink emit-signals=false async=false sync=false drop=true "
        "demux. ! "
        "queue max-size-buffers=0 max-size-bytes=0 max-size-time=0 name=dataQueue ! "
        "appsink name=dataSink emit-signals=false async=false sync=false drop=true "

        ;

    data.pipeline = gst_parse_launch(pipeline_desc.c_str(), nullptr);
    if (!data.pipeline)
    {
        LOG_ERROR("Failed to create pipeline.");
        return -1;
    }

    data.source = gst_bin_get_by_name(GST_BIN(data.pipeline), "source");
    data.tsDemux = gst_bin_get_by_name(GST_BIN(data.pipeline), "demux");
    data.videoQueue = gst_bin_get_by_name(GST_BIN(data.pipeline), "videoQueue");
    data.h264parse = gst_bin_get_by_name(GST_BIN(data.pipeline), "h264parse");
    data.avdec = gst_bin_get_by_name(GST_BIN(data.pipeline), "avdec_h264");
    data.videoSink = gst_bin_get_by_name(GST_BIN(data.pipeline), "videoSink");
    data.dataQueue = gst_bin_get_by_name(GST_BIN(data.pipeline), "dataQueue");
    data.dataSink = gst_bin_get_by_name(GST_BIN(data.pipeline), "dataSink");

    if (!data.source || !data.tsDemux || !data.videoQueue || !data.h264parse ||
        !data.avdec || !data.videoSink || !data.dataQueue || !data.dataSink)
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

    GstStateChangeReturn ret = gst_element_set_state(data.pipeline, GST_STATE_PLAYING);
    if (ret == GST_STATE_CHANGE_FAILURE)
    {
        LOG_ERROR("Unable to set the pipeline to the playing state.");
        gst_object_unref(data.pipeline);
        return -1;
    }

    busMessageLoop(data.pipeline);

    gst_element_set_state(data.pipeline, GST_STATE_NULL);
    gst_object_unref(data.pipeline);

    return 0;
}