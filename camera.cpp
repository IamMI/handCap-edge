#include <curl/curl.h>
#include <iostream>
#include "yolo.hpp"
#include <thread>
#include <mutex>
#include <condition_variable>
#include <queue>
#include <atomic>
#include <sys/mman.h>
#include <signal.h>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <fcntl.h>
#include <unistd.h>
#include <sys/ioctl.h>
#include <linux/videodev2.h>
#include <opencv2/opencv.hpp> 
#include <optional>

struct SharedFrame {
    std::mutex mtx;
    std::condition_variable cv;
    std::optional<cv::Mat> frame;
    std::atomic<bool> stopFlag{false};

    std::chrono::steady_clock::time_point lastFrameTime;
    int width = 0;
    int height = 0;
} sharedFrame;

std::mutex detectionFrameMtx;
cv::Mat latestDetectionFrame;                
std::atomic<bool> newFrameReady(false);    

const std::string RTMP_LIVE = "rtmp://Your server IP:Port1/stream1";
const int FPS_LIVE = 15;
const int FPS_IMG =1 ;
const size_t QUEUE_SIZE = 1;

std::queue<cv::Mat> frameQueue;
std::mutex mtx;
std::condition_variable cvNotEmpty;
std::atomic<bool> stopFlag(false);
bool sendAlertToServer() {
    CURL* curl = curl_easy_init();
    if (!curl) {
        std::cerr << "[ERROR] 初始化 curl 失败" << std::endl;
        return false;
    }

    // 报警 URL
    const std::string alertUrl = "http://Your server IP:Port2/alert";

    // 设置 POST 数据
    std::string postData = R"({"event":"person_detected","camera":"main_camera"})";

    curl_easy_setopt(curl, CURLOPT_URL, alertUrl.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDS, postData.c_str());
    curl_easy_setopt(curl, CURLOPT_POSTFIELDSIZE, (long)postData.size());

    // 设置 Content-Type 为 JSON
    struct curl_slist* headers = nullptr;
    headers = curl_slist_append(headers, "Content-Type: application/json");
    curl_easy_setopt(curl, CURLOPT_HTTPHEADER, headers);

    // 执行请求
    CURLcode res = curl_easy_perform(curl);
    if (res != CURLE_OK) {
        std::cerr << "[ERROR] curl_easy_perform() failed: " << curl_easy_strerror(res) << std::endl;
    } else {
        std::cout << "[INFO] 已发送报警信息到服务器！" << std::endl;
    }

    curl_easy_cleanup(curl);
    curl_slist_free_all(headers);
    return res == CURLE_OK;
}
// FFmpeg 进程结构体
struct FFMpegProcess {
    FILE* pipe = nullptr;
    std::string cmd;
};

FFMpegProcess proc_live;   // 全局变量
FFMpegProcess proc_gimg;   // 全局变量

// 判断 pipe 是否还可用
bool isPipeAlive(FILE* pipe) {
    return pipe && !feof(pipe) && !ferror(pipe);
}

// 创建 FFmpeg 推流子进程
FFMpegProcess startFFmpeg(const std::string& url, int fps, const std::string& tag, int width, int height) {
    char cmd[1024];
    snprintf(cmd, sizeof(cmd),
            "ffmpeg -loglevel verbose "
            "-f rawvideo -pix_fmt bgr24 -s %dx%d -r %d -i - "
            "-c:v libx264 -preset ultrafast -tune zerolatency "
            "-pix_fmt yuv420p -g 10 -b:v 512k -bf 0 "
            "-f flv %s",
            width, height, fps, url.c_str());

    std::cout << "[" << tag << "] " << cmd << std::endl;
    FILE* pipe = popen(cmd, "w");
    if (!pipe) {
        std::cerr << "无法启动 ffmpeg 子进程" << std::endl;
        exit(EXIT_FAILURE);
    }

    return {pipe, cmd};
}

// Ctrl+C 处理
void signalHandler(int s) {
   std::cerr << "\n收到终止信号，准备退出...\n";
    stopFlag = true;
    // 如果有其他线程需要通知，可以通过条件变量或其他机制通知它们
    sharedFrame.stopFlag = true; // 设置共享帧的退出标志
    if (proc_live.pipe) pclose(proc_live.pipe); // 关闭FFmpeg管道
    if (proc_gimg.pipe) pclose(proc_gimg.pipe);
}

// 使用 OpenCV 解码 MJPG 数据
cv::Mat decodeMJPEG(const unsigned char* jpegData, size_t jpegSize) {
    std::vector<uchar> data(jpegData, jpegData + jpegSize);
    cv::Mat frame = cv::imdecode(data, cv::IMREAD_COLOR); // OpenCV 自动解码
    return frame;
}
void yoloDetectionThread() {
    int frameCounter = 0;
    const int detectInterval = 60; // 每60帧检测一次

    while (!sharedFrame.stopFlag) {
        std::unique_lock<std::mutex> lock(sharedFrame.mtx);
        sharedFrame.cv.wait(lock, [&]{
            return sharedFrame.stopFlag || sharedFrame.frame.has_value();
        });

        if (sharedFrame.stopFlag)
            break;

        cv::Mat latestFrame = sharedFrame.frame.value();
        sharedFrame.frame.reset();  // 清空帧，避免重复处理
        lock.unlock();

        frameCounter++;
        if (frameCounter % detectInterval == 0) {
            std::cerr << "[INFO] 开始执行 YOLO 检测..." << std::endl;
            std::cerr << "[DEBUG] 即将调用 detect_yolov5_for_person()" << std::endl;
            bool personDetected = false;
            detect_yolov5_for_person(latestFrame, personDetected);
            std::cerr << "[DEBUG] detect_yolov5_for_person() 返回: " << personDetected << std::endl;
            if (personDetected) {
                std::cerr << "🤖 检测到人！正在发送报警信息..." << std::endl;
                sendAlertToServer();  // 发送HTTP报警
            } else {
                std::cerr << "[DEBUG] 未检测到人" << std::endl;
            }
        }
    }

    std::cout << "[INFO] YOLO 检测线程已退出" << std::endl;
}
int main() {
    signal(SIGINT, signalHandler);
    signal(SIGTERM, signalHandler);

    std::cout << "开始推流；Ctrl+C 退出..." << std::endl;

    const char* dev_name = "/dev/video0";
    int fd = open(dev_name, O_RDWR);
    if (fd < 0) {
        std::cerr << "无法打开摄像头设备：" << dev_name << std::endl;
        return -1;
    }

    struct v4l2_format fmt;
    memset(&fmt, 0, sizeof(fmt));
    fmt.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    fmt.fmt.pix.width = 640;
    fmt.fmt.pix.height = 480;
    fmt.fmt.pix.pixelformat = V4L2_PIX_FMT_MJPEG;
    fmt.fmt.pix.field = V4L2_FIELD_NONE;

    if (ioctl(fd, VIDIOC_S_FMT, &fmt) < 0) {
        std::cerr << "设置视频格式失败" << std::endl;
        close(fd);
        return -1;
    }

    int width = fmt.fmt.pix.width;
    int height = fmt.fmt.pix.height;
    sharedFrame.width = width;
    sharedFrame.height = height;
    std::cout << "摄像头分辨率：" << width << "x" << height << std::endl;
    
    // 启动 FFmpeg 子进程
    proc_live = startFFmpeg(RTMP_LIVE, FPS_LIVE, "LIVE", width, height);
    std::thread yoloThread(yoloDetectionThread);

    // 请求缓冲区
    struct v4l2_requestbuffers req;
    memset(&req, 0, sizeof(req));
    req.count = 4;
    req.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    req.memory = V4L2_MEMORY_MMAP;

    if (ioctl(fd, VIDIOC_REQBUFS, &req) < 0) {
        std::cerr << "请求缓冲区失败" << std::endl;
        close(fd);
        return -1;
    }

    void* buffers[4];
    for (unsigned int i = 0; i < req.count; ++i) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        buf.index = i;

        if (ioctl(fd, VIDIOC_QUERYBUF, &buf) < 0) continue;

        void* start = mmap(nullptr, buf.length, PROT_READ | PROT_WRITE, MAP_SHARED, fd, buf.m.offset);
        if (start == MAP_FAILED) continue;

        buffers[i] = start;

        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;
        ioctl(fd, VIDIOC_QBUF, &buf);
    }

    enum v4l2_buf_type type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
    ioctl(fd, VIDIOC_STREAMON, &type);
    
    int count = 0;
    while (!stopFlag) {
        struct v4l2_buffer buf;
        memset(&buf, 0, sizeof(buf));
        buf.type = V4L2_BUF_TYPE_VIDEO_CAPTURE;
        buf.memory = V4L2_MEMORY_MMAP;

        if (ioctl(fd, VIDIOC_DQBUF, &buf) < 0) {
            perror("取出缓冲区失败");
            continue;
        }

        // 使用 OpenCV 解码 MJPG 数据
        cv::Mat bgrFrame = decodeMJPEG((unsigned char*)buffers[buf.index], buf.bytesused);
        {
        std::lock_guard<std::mutex> lock(sharedFrame.mtx);
        sharedFrame.frame = bgrFrame.clone();  // 写入共享帧
        }
        sharedFrame.cv.notify_one(); 
        // 推送主直播流
        if (isPipeAlive(proc_live.pipe)) {
            fwrite(bgrFrame.data, 1, bgrFrame.total() * bgrFrame.elemSize(), proc_live.pipe);
            fflush(proc_live.pipe);
        } else {
            std::cerr << "[ERROR] proc_live.pipe 已断开，停止写入" << std::endl;
            stopFlag = true;
        }

        ioctl(fd, VIDIOC_QBUF, &buf);
    }
if (yoloThread.joinable())
    yoloThread.join();

    std::cout << "⏹ 正在关闭子进程..." << std::endl;
    if (proc_live.pipe) pclose(proc_live.pipe);
    if (proc_gimg.pipe) pclose(proc_gimg.pipe);


    close(fd);
    std::cout << "程序正常退出" << std::endl;

    return 0;
}