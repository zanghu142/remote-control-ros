# dozer_rtc_console

从 rtc-framework 迁入的独立 RTC 操作台，使用 C++17。Jetson 上使用 ament_cmake / colcon，Windows x64 上使用独立 CMake / MSVC 构建。
应用源码位于 `src/app/`，回归测试位于 `test/`，不链接 `dozer_rtc_edge`，不加入车端默认启动流程。
本包编译和运行均不再依赖相邻的 rtc-framework 仓库。

保留上游 SDL3 / ImGui / OpenGL 界面、车辆槽位、控制发送、视图切换、音频和检测叠框。
console 不是 ROS 节点，不运行本地推理；它通过 `perception.fused.v1` 接收 edge 转发的 ROS 融合结果。

## 依赖

在 Jetson Orin 上使用 ROS 2 Humble、JetPack 6、OpenGL、protobuf 和本工作空间的 `dozer_audio`、`dozer_media_vendor`、`dozer_rtc_sdk`、`dozer_rtc_common`。colcon 从同仓库源码构建 vtsrtc，操作台通过 `$ORIGIN` 加载安装在可执行文件旁的动态库。`dozer_rtc_common` 提供与车端共用的车辆、视图切换和融合检测协议。

## Windows 构建与运行

支持 Windows 10（1903 或更新）/11 x64、Visual Studio 2026（安装“使用 C++ 的桌面开发”和 Windows SDK）、CMake 4.2 或更高版本。
Windows 不需要 ROS、ament、colcon 或 Jetson 开发环境。编译需要 CUDA Toolkit（验证版本 12.6）和 NVIDIA Video Codec SDK 11.0.10。RTC SDK、音频和共享协议均从本仓库源码编译。

先按 [Windows 开发包说明](../../../vendor/windows/README.md) 准备匹配的 WebRTC 和 Video Codec SDK。
默认位置为 `vendor/windows/webrtc-win/` 和 `vendor/windows/Video_Codec_SDK_11.0.10/`，也可以指定其它目录。
不能使用 Jetson 的 `.a` 库或任意版本的 WebRTC：头文件和 `webrtc.lib` 必须匹配本项目使用的接口与 ABI。

在工作空间根目录双击 `build_rtc_console.bat`，或在 PowerShell 中执行 `build_rtc_console.ps1`。两个入口接受相同参数：

```powershell
.\build_rtc_console.ps1
# 开发包在其它位置时：
.\build_rtc_console.ps1 -WebRtcRoot D:\deps\webrtc-win -NvCodecRoot D:\deps\Video_Codec_SDK_11.0.10
.\install\windows-console\bin\rtc_console.exe --help
.\install\windows-console\bin\rtc_console.exe
```

脚本默认编译 Release、运行测试并安装到 `install/windows-console/bin/`。可以传入
`-Configuration Debug` 或 `-Configuration RelWithDebInfo`、`-BuildDirectory`、`-InstallDirectory`；
`-SkipTests` 仅跳过执行测试。脚本不会自动启动 RTC 会话。

首次 CMake 配置通过 HTTPS 获取固定版本并校验 SHA-256：SDL 3.4.12、ImGui 1.92.6、
nlohmann/json 3.12.0、nanopb 0.4.9、Asio 1.32.0、spdlog 1.14.1、Protobuf 3.21.12。
后续增量构建复用缓存。离线构建可通过 `FETCHCONTENT_SOURCE_DIR_SDL3`、`_IMGUI`、
`_NLOHMANN_JSON`、`_NANOPB`、`_ASIO`、`_SPDLOG`、`_PROTOBUF` 指定各自解压后的源码目录。
Protobuf 编译器随构建生成，不要求预装 Python 或 protoc。

也可直接调用 CMake：

```powershell
cmake -S src/remote_drive/dozer_rtc_console -B build/windows-console `
  -G "Visual Studio 18 2026" -A x64 -DDOZER_WEBRTC_ROOT=D:/deps/webrtc-win `
  -DDOZER_NVCODEC_ROOT=D:/deps/Video_Codec_SDK_11.0.10 -DBUILD_TESTING=ON
cmake --build build/windows-console --config Release --parallel
ctest --test-dir build/windows-console -C Release --output-on-failure
cmake --install build/windows-console --config Release --prefix install/windows-console --component Runtime
```

若构建目录曾使用 VS 2022，先在上述配置命令中添加 `--fresh` 重新生成 CMake 缓存，或用 `-BuildDirectory` 指定新的构建目录。

构建目录中的入口为 `build/windows-console/bin/Release/rtc_console.exe`。
交付时复制整个安装目录（`bin/` 包含 EXE、`vtsrtc.dll`、`rtc.cfg`、`console.cfg`，`share/licenses/` 保存第三方许可证），
目标机需安装 x64 Visual C++ 2015–2022 Redistributable 和支持 H.264 NVDEC 的 NVIDIA 显卡驱动，无需 CUDA Toolkit 或开发机 PATH。配置放在 EXE 旁，也可通过 `--config` 指定。
Windows 默认 `use_NVDEC=true`：接收端协商 H.264，经 NVIDIA NVDEC 解码并转为 I420 交给显示模块。驱动或硬件初始化失败会报错，不会静默切换为软件解码。需要对照排查时，才显式设置 `use_NVDEC=false` 使用 WebRTC 内置解码器。发送端编码仍使用 WebRTC 内置编码器。

### Windows 双目与环视鱼眼矫正

Windows 客户端使用 OpenGL GPU 着色器对 `stereo` 左右拼接视频分别去畸变和校正极线，
采用标定文件中的 `K/D/R1/R2/P1/P2`，再进行显示或逐列像素交错。
四路环视各自读取 `camera_matrix`、`dist_coeffs`、`new_camera_matrix` 做单目去畸变，
按前、后、左、右分别应用到 `surround_grid` 的对应格子以及 `surround_front/rear/left/right` 单路视频。
四宫格、单路放大与全屏使用相同参数，GPU 采样限制在对应格子内部，检测框同步变换。
每路环视独立计算有效中心裁剪区域，自动去除矫正黑边。
Windows 每路视频只上传原始纹理，每次界面刷新中最多上传一次；普通视图、全屏和单眼裁剪
共享原始纹理，原始交错与矫正交错都在 GPU 完成，不再生成 CPU 交错图像。
标定映射按标定对象和接收分辨率缓存，着色器参数位置只在初始化时查询。
SDK 回调提供临时 CPU 图像，因此客户端仍需一次帧数据复制和一次纹理上传，并非端到端零拷贝。
普通视图与全屏使用相同的矫正流程；接收分辨率变化时按每眼宽高缩放内参和投影矩阵。
标定尺寸 `1920×1080` 指单眼，输入顺序为左眼在左、右眼在右。应使用同一对摄像头、
相同视场的画面；裁剪或更换镜头后应重新标定。

矫正时根据标定参数自动裁去边缘无效黑区，左右眼使用同一中心矩形并等比例放大，
保持双目极线对齐和画面比例。并排、像素交错及全屏均使用这一裁剪范围；
检测框同步映射到裁剪后的坐标。裁剪会减少边缘视野，按 `F` 可切回完整原始画面。
窗口与双目视频宽高比不同产生的外侧留白仍用于保持画面比例。

默认按 `F` 同时切换双目和环视的原始画面／矫正画面，按 `I` 切换双目的左右并排／逐列交错。
两种开关相互独立，支持原始并排、原始交错、矫正并排、矫正交错。
双目并排时，双击左眼或右眼可分别全屏放大；再次双击或按 `Esc` 返回。
在双目全屏中继续双击某一眼，返回时恢复双目全屏。单眼放大使用对应鱼眼参数，
保持该眼比例，并暂停像素交错显示；返回双目后恢复原交错开关状态。
交错模式中双击仍放大完整交错画面，可先按 `I` 切回并排以选择单眼。
设置 → 快捷键中的“切换原始／鱼眼矫正画面”可修改 `F`；保存后重启生效。
临时切换不会改写启动默认值，可在设置 → 常规中分别修改双目和环视的启动矫正开关及标定路径。

五份标定源文件统一放在 `src/system/dozer_bringup/config/`，与 `camera.yaml`、`extrinsics.yaml` 同级：
`stereo_fisheye_extrinsics.yml`、`surround_front_fisheye.yml`、`surround_rear_fisheye.yml`、
`surround_left_fisheye.yml`、`surround_right_fisheye.yml`。每次构建操作台时自动将它们复制到
`build/windows-console/bin/Release/fisheye_calibration/`；其它构建配置也会复制到对应 EXE 目录。
更新标定后再次编译即可同步，即使 EXE 无需重新链接也会执行复制。
安装后位于 `install/windows-console/bin/fisheye_calibration/`。
客户端直接读取 OpenCV YAML 标定中的矩阵、科学计数法及多行数组，不要求安装 OpenCV。
`console.cfg` 中的配置为：

```json
"stereo_fisheye": {
  "enabled": true,
  "calibration_file": "fisheye_calibration/stereo_fisheye_extrinsics.yml"
},
"surround_fisheye": {
  "enabled": true,
  "calibration_files": {
    "front": "fisheye_calibration/surround_front_fisheye.yml",
    "rear": "fisheye_calibration/surround_rear_fisheye.yml",
    "left": "fisheye_calibration/surround_left_fisheye.yml",
    "right": "fisheye_calibration/surround_right_fisheye.yml"
  }
},
"key_bindings": {
  "toggle_fisheye": ["F"]
}
```

相对标定路径以 EXE 所在目录为基准，也支持绝对路径。
标定加载或着色器初始化失败时记录原因并显示原始画面。
单路标定加载失败时只让该路保持原始画面，其它相机仍可正常矫正。

中文字体默认使用 Windows 的微软雅黑或黑体。Linux 专用 `readelf` / `ldd` 检查在 Windows 上由安装目录启动检查替代，
并增加 SDK ABI、SDL dummy 音频和 NVDEC 实机测试。可单独运行 `ctest --test-dir build/windows-console -C Release -L hardware --output-on-failure`：测试使用操作台相同的解码工厂，在 GPU 上解码两种分辨率的 H.264 帧并校验 I420 像素和 RTP 时间戳。无需信令服务器、相机或车辆。实际视频帧率、麦克风、方向盘和与车端的网络联调仍需在目标机器验收。

## Jetson 构建与运行

在工作空间根目录执行：

```bash
source /opt/ros/humble/setup.bash
colcon build --packages-up-to dozer_rtc_console \
  --cmake-clean-cache \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=ON
source install/setup.bash
ros2 run dozer_rtc_console rtc_console
```

GUI 需要可用的显示服务和 OpenGL 环境。可执行入口为 `rtc_console`，不是 `console`。
从旧版本切换时，上述 `--cmake-clean-cache` 会重新查找依赖，避免沿用旧 SDK/模型路径。
后续增量构建可省略该参数；不需要修改或移动外部仓库。
常用参数：

```bash
ros2 run dozer_rtc_console rtc_console --help
ros2 run dozer_rtc_console rtc_console --room test1
ros2 run dozer_rtc_console rtc_console \
  --config "$PWD/src/system/dozer_bringup/config/rtc_console/rtc.cfg"
ros2 run dozer_rtc_console rtc_console --headless
```

`--headless` / `--no-render` 只关闭窗口和 OpenGL 上下文，不是安全的离线检查：
仍会初始化 RTC 及相关音频功能。仅查看入口请使用 `--help`。
`--room` 覆盖房间名称，不覆盖默认的手动开房设置。

## 配置

集中配置位于 `src/system/dozer_bringup/config/rtc_console/`：

- `rtc.cfg`：信令、网络和认证，通过 `console.config` 引用同目录的 `console.cfg`。
- `console.cfg`：房间、设备槽位和方向盘设置；未指定的按键使用上游默认值。

本次信令与网络配置以现有 edge 配置为基线复制，随后两端各自独立维护；不要把认证内容写入日志。
默认房间为 `test1`，槽位为 `car_1` 至 `car_4`，`auto_open_room=false`，未假定舵机标定值。
这不会改变 edge 的 debug 模式或看门狗设置，也不代表已经接入实车。

不传 `--config` 时，程序固定使用可执行文件旁的 `rtc.cfg`，不搜索当前工作目录，避免误读 edge 配置。
构建时将集中配置复制到构建目录，Jetson 安装时复制到 `install/dozer_rtc_console/lib/dozer_rtc_console/`，Windows 安装到 `install/windows-console/bin/`。
修改源码配置后应重新构建，或直接用 `--config` 指向源码配置；GUI 保存设置会写入当前选用的配置文件。

## 验证与限制

构建成功后执行：

```bash
ctest --test-dir build/dozer_rtc_console --output-on-failure
```

提供参数/配置、控制发送、车辆状态日志、视图、合成布局、叠框、全屏状态、帮助入口和 SDK 链接检查。
融合检测测试覆盖 Protobuf 解码、重复和乱序丢弃、快照过期及空快照清理。
这些测试不启动 RTC 会话、不打开真实相机，不验证 GUI 显示、真实音频设备、RTC 网络链路或实车控制。

操作台使用本仓库 C ABI 提供的视频延迟统计接口，ABI 测试随 `dozer_rtc_sdk` 构建。
console 按设备和相机缓存叠框。`camera:<name>` 更新（包括空结果）只影响对应相机，各路独立在接收后 500 ms 过期。`radar_fusion` 周期快照独立保留，不覆盖单相机更新；未收到带范围标记的更新时兼容旧的整包叠框。带相机框的 3D 目标显示融合坐标，纯雷达目标不绘制视频框。叠框不按采集时间戳与视频做精确同步。修复叠框闪烁需同步更新 `dozer_fusion`、edge 和 console；旧版 `vision.detect.v1` 不兼容。
