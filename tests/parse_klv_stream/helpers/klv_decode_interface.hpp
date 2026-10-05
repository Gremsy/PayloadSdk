#pragma once
#include <ctime>
#include <chrono>
#include <iomanip>
#include <sstream>

#include <gst/gst.h>
#include <gst/app/gstappsink.h>
#include <chrono>
#include <ctime>
#include <iomanip>
#include <iostream>
#include <sstream>

#include "common_include.hpp"
#include "unpack.hpp"
#include "klv_gps_sender.hpp"

class Klv_Decode_Interface
{
public:
    struct CustomData
    {
        GstElement *pipeline;
        GstElement *source;
        GstElement *tsDemux;
        GstElement *videoQueue;
        GstElement *dataQueue;
        GstElement *h264parse;
        GstElement *avdec;
        GstElement *videoSink;
        GstElement *dataSink;
        GstElement *autoVideoSink;
    };

    struct decode_frame_t{
        cv::Mat decode_frame;

    };

public:
    Klv_Decode_Interface();
    ~Klv_Decode_Interface();

    std::vector<Unpack::TagValuePair> tag_values;

    void setDemuxCallback(GstElement *demux, CustomData &data);
    void setVideoSinkCallback(GstAppSink *videoSink, CustomData *data);
    void setDataSinkCallback(GstAppSink *dataSink, CustomData *data);

    void setPipeline(GstElement *pipeline);

    int cnt_frame = 0, cnt_data=0;

    std::chrono::steady_clock::time_point start_time;
    int cnt_klv_frame;

private:
    std::unique_ptr<Unpack> unpack;
    CustomData data;

    static GstFlowReturn new_video_sample(GstAppSink *videoSink, gpointer user_data);
    static GstFlowReturn new_data_sample(GstAppSink *dataSink, gpointer user_data);
    static void pad_added_handler(GstElement *src, GstPad *pad, CustomData *data);

    bool decodeKlvData(GstBuffer *buffer);
    void printKlvData();
    void print_tag_values(const std::vector<Unpack::TagValuePair> &tag_values);
    KlvGpsSender gps_sender{"127.0.0.1", 5005};


};