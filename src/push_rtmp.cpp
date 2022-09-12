#include <push_rtmp.h>

PushRTMP::PushRTMP(const int width, const int height, const int fps)
{
    m_width = width;
    m_height = height;
    m_fps = fps;

    m_sc = NULL;
    m_frame = NULL;
    m_acc = NULL;
    m_afc = NULL;
    m_codec = NULL;
    m_stream = NULL;

    m_ret = 0;
    m_vpts = 0;
}

bool PushRTMP::initCodec(const string pixel_type)
{
    avcodec_register_all();
    av_register_all();

    try
    {
        // 1.初始化格式转换上下文
        if (pixel_type == "BGRA")
        {
            m_sc = sws_getCachedContext(m_sc, m_width, m_height, AV_PIX_FMT_BGRA,
                                        m_width, m_height, AV_PIX_FMT_YUV420P,
                                        SWS_BICUBIC, 0, 0, 0);
        }
        else if (pixel_type == "Gray")
        {
            m_sc = sws_getCachedContext(m_sc, m_width, m_height, AV_PIX_FMT_GRAY8,
                                        m_width, m_height, AV_PIX_FMT_YUV420P,
                                        SWS_BICUBIC, 0, 0, 0);
        }

        if (!m_sc)
        {
            // throw exception("Failed to initialize format-transfer context!");
            cout << "Failed to initialize format-transfer context!" << endl;
            return false;
        }

        // 2.初始化输出数据格式，未压缩的数据
        m_frame = av_frame_alloc();
        m_frame->format = AV_PIX_FMT_YUV420P;
        m_frame->width = m_width;
        m_frame->height = m_height;
        m_frame->pts = 0;
        //分配m_frame空间
        m_ret = av_frame_get_buffer(m_frame, 32);
        if (m_ret != 0)
        {
            printError(m_ret);
            return false;
        }

        // 3.初始化编码器上下文
        // a.找编码器
        m_codec = avcodec_find_encoder(AV_CODEC_ID_H264);
        if (!m_codec)
        {
            // throw exception("Failed to find encoder!");
            cout << "Failed to find encoder!" << endl;
            return false;
        }

        // b.创建编码器上下文
        m_acc = avcodec_alloc_context3(m_codec);
        if (!m_acc)
        {
            // throw exception("Failed to allocate encoder context!");
            cout << "Failed to allocate encoder context!" << endl;
            return false;
        }

        // c.配置编码器参数
        m_acc->flags |= AV_CODEC_FLAG_GLOBAL_HEADER; //全局参数
        m_acc->codec_id = m_codec->id;
        m_acc->thread_count = 8;

        AVDictionary *param = NULL;
        av_dict_set(&param, "preset", "superfast", 0);
        av_dict_set(&param, "tune", "zerolatency", 0);

        m_acc->bit_rate = 50 * 1024 * 8; // 压缩后每帧30kb
        m_acc->width = m_width;
        m_acc->height = m_height;
        m_acc->time_base = {1, m_fps};
        m_acc->framerate = {m_fps, 1};

        //画面组的大小，多少帧一个关键帧
        m_acc->gop_size = 50;
        m_acc->max_b_frames = 0;
        m_acc->pix_fmt = AV_PIX_FMT_YUV420P;

        // d.打开编码器上下文
        m_ret = avcodec_open2(m_acc, m_codec, &param);
        if (m_ret != 0)
        {
            printError(m_ret);
            return false;
        }

        return true;
    }
    catch (const std::exception &e)
    {
        std::cerr << e.what() << '\n';
        if (m_sc)
        {
            sws_freeContext(m_sc);
            m_sc = NULL;
        }
        if (m_acc)
        {
            avio_closep(&m_afc->pb);
            avcodec_free_context(&m_acc);
        }
        return false;
    }
}

bool PushRTMP::initOut(const string push_addr)
{
    avformat_network_init();

    // 1.创建输出封装器上下文
    m_ret = avformat_alloc_output_context2(&m_afc, 0, "flv", NULL);
    if (m_ret != 0)
    {
        printError(m_ret);
        return false;
    }

    // 2.添加视频流
    m_stream = avformat_new_stream(m_afc, NULL);
    if (!m_stream)
    {
        cout << "Failed to create video stream!" << endl;
        return false;
    }
    //附加标志，这个一定要设置
    m_stream->codecpar->codec_tag = 0;
    //从编码器复制参数
    avcodec_parameters_from_context(m_stream->codecpar, m_acc);
    m_stream->time_base.num = 1;
    av_dump_format(m_afc, 0, push_addr.data(), 1);

    // 3.打开rtmp 的网络输出IO  AVIOContext：输入输出对应的结构体，用于输入输出（读写文件，RTMP协议等）
    m_ret = avio_open(&m_afc->pb, push_addr.data(), AVIO_FLAG_WRITE);
    if (m_ret != 0)
    {
        printError(m_ret);
        return false;
    }

    // 4.写入封装头
    m_ret = avformat_write_header(m_afc, NULL);
    if (m_ret != 0)
    {
        printError(m_ret);
        return false;
    }
    av_init_packet(&m_pack);

    return true;
}

bool PushRTMP::initRTMP(const string push_addr, const string pixel_type)
{
    if (!initCodec(pixel_type))
    {
        cout << "Failed to initialize Codec!" << endl;
        return false;
    }
    if (!initOut(push_addr))
    {
        cout << "Failed to configure out!" << endl;
        return false;
    }
    return true;
}

bool PushRTMP::startPush(const uchar *data, int ele_size)
{
    // 1.rgb to yuv
    //输入的数据结构
    uint8_t *indata[AV_NUM_DATA_POINTERS] = {0};
    // indata[0] bgrbgrbgr
    // plane indata[0] bbbbb indata[1]ggggg indata[2]rrrrr
    indata[0] = (uint8_t *)data;
    int insize[AV_NUM_DATA_POINTERS] = {0};
    //一行（宽）数据的字节数
    insize[0] = m_width * ele_size;
    int h = sws_scale(m_sc, indata, insize, 0, m_height, //源数据
                      m_frame->data, m_frame->linesize);
    if (h <= 0)
    {
        return false;
    }

    // 2.h264编码
    m_frame->pts = m_vpts;
    m_vpts++;
    av_init_packet(&m_pack);
    if (m_acc->codec_type == AVMEDIA_TYPE_VIDEO)
    {
        m_ret = avcodec_send_frame(m_acc, m_frame);
        if (m_ret != 0)
        {
            printError(m_ret);
            return false;
        }
    }
    avcodec_receive_packet(m_acc, &m_pack);
    if (m_pack.size == 0)
    {
        return false;
    }

    // 3.推流
    m_pack.pts = av_rescale_q(m_pack.pts, m_acc->time_base, m_stream->time_base);
    m_pack.dts = m_pack.pts;
    m_pack.duration = av_rescale_q(m_pack.duration, m_acc->time_base, m_stream->time_base);
    m_ret = av_interleaved_write_frame(m_afc, &m_pack);
    if (m_ret != 0)
    {
        printError(m_ret);
        return false;
    }

    return true;
}

void PushRTMP::printError(int err)
{
    char buffer[1024] = {0};
    av_strerror(err, buffer, sizeof(buffer) - 1);
    cout << buffer << endl;
}

PushRTMP::~PushRTMP()
{

    sws_freeContext(m_sc);
    avformat_free_context(m_afc);
    avcodec_free_context(&m_acc);
    av_frame_free(&m_frame);
    av_free_packet(&m_pack);

    if (m_codec)
    {
        delete m_codec;
        m_codec = NULL;
    }
    if (m_stream)
    {
        delete m_stream;
        m_stream = NULL;
    }
}