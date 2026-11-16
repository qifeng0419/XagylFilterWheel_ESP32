# include/

工程级公共头文件目录。

当前所有头文件都放在 `src/` 旁边（模块与实现一一对应），本目录保留为空，
用于放置你打算跨 `src/` 与 `test/` 共享的头，例如：

- 仿真/回环测试用的桩接口声明
- 从设备端与主机端共用的报文常量定义

PlatformIO 会自动把本目录加入 `build_flags` 的 include 搜索路径
（见 `platformio.ini` 的 `include_dir`）。
