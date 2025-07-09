# 📹 Embedded YOLO-Based RTMP Streaming System
This project is a lightweight real-time video stream detection and streaming system deployed on the Loongson embedded development board, featuring:
- Capturing camera frames using V4L2 (MJPEG format)
- Decoding to BGR images using OpenCV
- Detecting human presence using YOLOv5
- Triggering detection every few frames and sending alerts via HTTP POST upon detection
- Encoding and streaming frames to RTMP server via FFmpeg


## 🧩 System Architecture
```text
[Camera (/dev/video0)] --> [V4L2 + OpenCV decoding] --> [YOLOv5 detection thread]
         |                                              |
         |-----> RTMP stream via FFmpeg ----------------|
         |-----> Alert system (curl POST JSON to server)|
```
## 🚀 Features
- 🎥 **MJPEG camera** frame capture
- 🤖 Lightweight **YOLOv5-based** human detection
- 🌐 Send alert events to remote server using **curl**
- 📡 Real-time streaming (RTMP via **FFmpeg** subprocess)
- 🧵 **Multithreaded** architecture with **condition variable** synchronization
- 🖥️ Compatible with embedded Linux (e.g., **Loongson** platform)

## 📦 Build Dependencies
- C++17 or later
- OpenCV
- FFmpeg (must be available via CLI)
- libcurl
- V4L2 (included in Linux kernel)

## 🔧 Usage
1. Prepare the camera device
Make sure the device `/dev/video0` is accessible and supports MJPEG format.

    > If the device `/dev/video0` is not recognized after connecting the camera, it may be due to missing drivers in your board's Linux system. In that case, a kernel replacement might be necessary.

2. Create your own server like aliyun, expose its IP and some ports and rewrite 
    ```C++
    const std::string RTMP_LIVE = "rtmp://Your server IP:Port1/stream1";
    ```
    and 
    ```C++
    const std::string alertUrl = "http://Your server IP:Port2/alert";
    ```


3. Compile the program (*adjust paths as needed*)
    ```bash 复制 编辑
    g++ -std=c++17 camera.cpp yolo.cpp \
        -I/home/loongson/ncnn/build/install/include/ncnn \
        -L/home/loongson/ncnn/build/install/lib \
        -L/usr/lib/loongarch64-linux-gnu \
        -lncnn -lopencv_core  \
        -lopencv_highgui -lopencv_videoio  \
        -lopencv_imgproc -lopencv_imgcodecs \
        -lavcodec -lavformat -lavutil -lswscale -fopenmp  \
        -march=loongarch64 \
        -lcurl -o camera
    ```
4. Run the program
    ```bash 复制 编辑
    ./camera
    ```

## ⚠️ Alert Mechanism
When a person is detected (every 60 frames), the system sends an alert via the following HTTP interface:

```http 复制 编辑
POST http://Your server IP:Port/alert
Content-Type: application/json

{
  "event": "person_detected",
  "camera": "main_camera"
}
```


## 📁 Project Structure (Partial)
```
.
├── camera.cpp        # Main program: video capture, multithreading, RTMP streaming
├── yolo.hpp          # YOLO model interface definitions
├── yolo.cpp          # YOLO model loading and inference implementation
└── build/            # Build output directory (optional)

```

## 📌 TODO
- [ ] Support multiple camera switching
- [ ] Add image compression and encoding optimization
- [ ] Accelerate YOLO inference with CUDA / NPU