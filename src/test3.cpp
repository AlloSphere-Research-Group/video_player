extern "C" {
#include <libavcodec/avcodec.h>
#include <libavformat/avformat.h>
#include <libavutil/hwcontext.h>
#include <libavutil/imgutils.h>
#include <libavutil/pixdesc.h>
}

#include <iostream>

enum AVPixelFormat hw_pix_fmt;
AVBufferRef *hw_device_ctx = NULL;

static int hw_decoder_init(AVCodecContext *ctx,
                           const enum AVHWDeviceType type) {
  int err = 0;

  if ((err = av_hwdevice_ctx_create(&hw_device_ctx, type, NULL, NULL, 0)) < 0) {
    std::cerr << "Failed to create specified HW device" << std::endl;
    return err;
  }

  ctx->hw_device_ctx = av_buffer_ref(hw_device_ctx);
  return err;
}

static AVPixelFormat get_hw_format(AVCodecContext *ctx,
                                   const enum AVPixelFormat *pix_fmts) {
  const enum AVPixelFormat *p;
  for (p = pix_fmts; *p != AV_PIX_FMT_NONE; p++) {
    if (*p == hw_pix_fmt) {
      return *p;
    }
  }
}

static int decode_write(AVCodecContext *avctx, AVPacket *packet) {
  AVFrame *frame = NULL;
  AVFrame *sw_frame = NULL;
  AVFrame *tmp_frame = NULL;
  uint8_t *buffer = NULL;
  int size;
  int ret = 0;

  ret = avcodec_send_packet(avctx, packet);
  if (ret < 0) {
    std::cerr << "Error sending packet" << std::endl;
    return ret;
  }

  while (true) {
    if (!(frame = av_frame_alloc()) || !(sw_frame = av_frame_alloc())) {
      std::cerr << "Cannot alloc frame" << std::endl;
      ret = AVERROR(ENOMEM);
      goto fail;
    }

    ret = avcodec_receive_frame(avctx, frame);
    if (ret == AVERROR(EAGAIN) || ret == AVERROR_EOF) {
      av_frame_free(&frame);
      av_frame_free(&sw_frame);
      return 0;
    } else if (ret < 0) {
      std::cerr << "Error receiving frame" << std::endl;
      goto fail;
    }

    if (frame->format == hw_pix_fmt) {
      if ((ret = av_hwframe_transfer_data(sw_frame, frame, 0)) < 0) {
        std::cerr << "Error transferring data to system memory" << std::endl;
        goto fail;
      }
      tmp_frame = sw_frame;
      std::cout << "Correct hw format: " << av_get_pix_fmt_name(hw_pix_fmt)
                << std::endl;
    } else {
      tmp_frame = frame;
    }

    size = av_image_get_buffer_size((AVPixelFormat)tmp_frame->format,
                                    tmp_frame->width, tmp_frame->height, 1);

    buffer = (uint8_t *)av_malloc(size);
    if (!buffer) {
      std::cerr << "Cannot allocate buffer" << std::endl;
      ret = AVERROR(ENOMEM);
      goto fail;
    }
    ret = av_image_copy_to_buffer(
        buffer, size, (const uint8_t *const *)tmp_frame->data,
        (const int *)tmp_frame->linesize, (AVPixelFormat)tmp_frame->format,
        tmp_frame->width, tmp_frame->height, 1);

    if (ret < 0) {
      std::cerr << "Cannot copy image to buffer" << std::endl;
      goto fail;
    }

    // use data here
    // std::cout << tmp_frame->best_effort_timestamp << std::endl;
  fail:
    av_frame_free(&frame);
    av_frame_free(&sw_frame);
    av_freep(&buffer);
    if (ret < 0) {
      return ret;
    }
  }
}

int main(int argc, char *argv[]) {
  AVFormatContext *input_ctx = NULL;
  int video_stream, ret;
  AVStream *video = NULL;
  AVCodecContext *decoder_ctx = NULL;
  const AVCodec *decoder = NULL;
  AVPacket *packet = NULL;
  enum AVHWDeviceType type;

  std::cout << "Available device types:" << std::endl;
  while ((type = av_hwdevice_iterate_types(type)) != AV_HWDEVICE_TYPE_NONE) {
    std::cout << " " << av_hwdevice_get_type_name(type) << std::endl;
  }

  type = av_hwdevice_find_type_by_name("qsv");
  if (type == AV_HWDEVICE_TYPE_NONE) {
    std::cerr << "HW Device not supported" << std::endl;
    return -1;
  }

  packet = av_packet_alloc();
  if (!packet) {
    std::cerr << "Failed to allocate AVPacket" << std::endl;
    return -1;
  }

  const char *url =
      // "C:/Users/kenny/code/video_player/data/LW_2K_ERP_noaudio.mp4";
      "/data/media/LastWhispers/LW_2K_ERP_noaudio.mp4";
  if (avformat_open_input(&input_ctx, url, NULL, NULL) != 0) {
    std::cerr << "Cannot open file: " << url << std::endl;
    return -1;
  }

  if (avformat_find_stream_info(input_ctx, NULL) < 0) {
    std::cerr << "Cannot find input stream information" << std::endl;
    return -1;
  }

  ret = av_find_best_stream(input_ctx, AVMEDIA_TYPE_VIDEO, -1, -1, &decoder, 0);
  if (ret < 0) {
    std::cerr << "Cannot find a video stream in the input file" << std::endl;
    return -1;
  }
  video_stream = ret;

  for (int i = 0;; ++i) {
    const AVCodecHWConfig *config = avcodec_get_hw_config(decoder, i);
    if (!config) {
      std::cerr << "Decoder " << decoder->name
                << " does not support device type "
                << av_hwdevice_get_type_name(type) << std::endl;
      return -1;
    }
    if (config->methods & AV_CODEC_HW_CONFIG_METHOD_HW_DEVICE_CTX &&
        config->device_type == type) {
      hw_pix_fmt = config->pix_fmt;
      break;
    }
  }

  if (!(decoder_ctx = avcodec_alloc_context3(decoder))) {
    std::cerr << "Failed to allocated codec context" << std::endl;
    return -1;
  }

  video = input_ctx->streams[video_stream];
  if (avcodec_parameters_to_context(decoder_ctx, video->codecpar) < 0) {
    std::cerr << "Failed to retrieve codec parameters" << std::endl;
    return -1;
  }

  decoder_ctx->get_format = get_hw_format;

  if (hw_decoder_init(decoder_ctx, type) < 0) {
    std::cerr << "Failed to init HW decoder" << std::endl;
    return -1;
  }

  if ((ret = avcodec_open2(decoder_ctx, decoder, NULL)) < 0) {
    std::cerr << "Failed to open codec for stream #" << video_stream
              << std::endl;
    return -1;
  }

  int temp = 0;
  while (ret >= 0 && temp++ < 3) {
    if ((ret = av_read_frame(input_ctx, packet)) < 0) {
      break;
    }

    if (video_stream == packet->stream_index) {
      ret = decode_write(decoder_ctx, packet);
    }

    av_packet_unref(packet);
  }

  ret = decode_write(decoder_ctx, NULL);

  av_packet_free(&packet);
  avcodec_free_context(&decoder_ctx);
  avformat_close_input(&input_ctx);
  av_buffer_unref(&hw_device_ctx);
}