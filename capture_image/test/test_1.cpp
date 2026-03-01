#include <libuvc/libuvc.h>
#include <chrono>
#include <csignal>
#include <iostream>
#include <opencv2/opencv.hpp>

namespace {
struct FrameStats {
    bool first = true;
    std::chrono::steady_clock::time_point last_tp{};
};

struct CaptureContext {
    FrameStats stats;
    uvc_frame_t* bgr_frame = nullptr;
    bool running = true;
    bool is_mjpeg = false;
};

void frame_callback(uvc_frame_t* frame, void* ptr) {
    auto* ctx = static_cast<CaptureContext*>(ptr);
    if (!ctx->running) {
        return;
    }

    cv::Mat image;
    if (ctx->is_mjpeg) {
        cv::Mat encoded(1, static_cast<int>(frame->data_bytes), CV_8UC1, frame->data);
        image = cv::imdecode(encoded, cv::IMREAD_COLOR);
        if (image.empty()) {
            std::cerr << "[UVC] Failed to decode MJPEG frame.\n";
            return;
        }
    } else {
        if (!ctx->bgr_frame || uvc_any2bgr(frame, ctx->bgr_frame) != UVC_SUCCESS) {
            std::cerr << "[UVC] Failed to convert frame to BGR.\n";
            return;
        }
        image = cv::Mat(frame->height, frame->width, CV_8UC3, ctx->bgr_frame->data);
    }

    auto now = std::chrono::steady_clock::now();
    if (ctx->stats.first) {
        ctx->stats.first = false;
    } else {
        auto delta = std::chrono::duration_cast<std::chrono::milliseconds>(now - ctx->stats.last_tp).count();
        std::cout << "[UVC] Frame interval: " << delta << " ms\n";
    }
    ctx->stats.last_tp = now;

    auto wall_time = std::chrono::system_clock::now();
    auto wall_ms = std::chrono::duration_cast<std::chrono::milliseconds>(wall_time.time_since_epoch()).count();
    std::cout << "[UVC] Frame #" << frame->sequence << " timestamp(ms): " << wall_ms << '\n';

    cv::imshow("Trigger Camera", image);
    cv::waitKey(1);
}

void handle_signal(int) {
    std::cout << "\n[UVC] Signal caught, stopping...\n";
}
}  // namespace

int main() {
    signal(SIGINT, handle_signal);

    uvc_context_t* ctx = nullptr;
    if (uvc_init(&ctx, nullptr) != UVC_SUCCESS) {
        std::cerr << "[UVC] Failed to init libuvc.\n";
        return 1;
    }

    uvc_device_t* dev = nullptr;
    if (uvc_find_device(ctx, &dev, 0x0bda, 0x5856, nullptr) != UVC_SUCCESS) {
        std::cerr << "[UVC] No UVC device found.\n";
        uvc_exit(ctx);
        return 1;
    }

    uvc_device_handle_t* devh = nullptr;
    if (uvc_open(dev, &devh) != UVC_SUCCESS) {
        std::cerr << "[UVC] Cannot open device.\n";
        uvc_unref_device(dev);
        uvc_exit(ctx);
        return 1;
    }

    uvc_stream_ctrl_t ctrl;
    if (uvc_get_stream_ctrl_format_size(devh, &ctrl, UVC_FRAME_FORMAT_MJPEG, 1280, 720, 30) != UVC_SUCCESS) {
        std::cerr << "[UVC] Failed to get stream control.\n";
        uvc_close(devh);
        uvc_unref_device(dev);
        uvc_exit(ctx);
        return 1;
    }

    CaptureContext capture_ctx;
    capture_ctx.is_mjpeg = true;
    if (!capture_ctx.is_mjpeg) {
        capture_ctx.bgr_frame = uvc_allocate_frame(ctrl.dwMaxVideoFrameSize);
        if (!capture_ctx.bgr_frame) {
            std::cerr << "[UVC] Unable to allocate BGR frame buffer.\n";
            uvc_close(devh);
            uvc_unref_device(dev);
            uvc_exit(ctx);
            return 1;
        }
    }

    if (uvc_start_streaming(devh, &ctrl, frame_callback, &capture_ctx, 0) != UVC_SUCCESS) {
        std::cerr << "[UVC] Failed to start streaming.\n";
        if (capture_ctx.bgr_frame) {
            uvc_free_frame(capture_ctx.bgr_frame);
        }
        uvc_close(devh);
        uvc_unref_device(dev);
        uvc_exit(ctx);
        return 1;
    }

    std::cout << "[UVC] Streaming... External trigger frames will appear.\nPress Enter to stop.\n";
    std::cin.get();

    capture_ctx.running = false;
    uvc_stop_streaming(devh);
    if (capture_ctx.bgr_frame) {
        uvc_free_frame(capture_ctx.bgr_frame);
    }
    uvc_close(devh);
    uvc_unref_device(dev);
    uvc_exit(ctx);
    cv::destroyAllWindows();
    return 0;
}
