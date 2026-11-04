# Docker 使用与迁移

项目在 Raspberry Pi 的 Linux ARM64 上验证。宿主需要 Docker Engine、Compose v2 和 Bash。真机连接需要宿主 Linux 能识别 SocketCAN；离线处理与 mock 仿真无需连接机械臂。命令从项目根目录执行；若当前用户已有 Docker 权限，可省略 Docker 命令前的 `sudo`。

## 启动和构建

```bash
mkdir -p output
sudo PIPER_UID="$(id -u)" PIPER_GID="$(id -g)" docker compose up -d
sudo docker compose logs --tail 80 web
```

Web 默认监听宿主 `127.0.0.1:8080`。通过 SSH 访问时，用 `ssh -N -L 8080:127.0.0.1:8080 用户@目标主机` 转发。若 8080 已被占用，在 `up` 命令的 `docker` 前加 `PIPER_WEB_PORT=8081`。停止服务用 `sudo docker compose down`。

本机已有镜像时可以直接启动。联网的目标机可从同一份源码构建：

```bash
sudo docker compose build web      # piperdraw/ros:humble
bash scripts/build_hardware.sh    # piperdraw/hardware:0.1，可选
```

默认基础镜像是 `ros:humble-ros-base-jammy` 和 `python:3.10-slim-bookworm`。若目标网络需使用镜像代理，可分别用 `PIPER_ROS_BASE_IMAGE`（作为 `sudo` 的环境变量传入）和 `PIPER_BASE_IMAGE` 指定可访问的同版本基础镜像。`PIPER_ROS_APT_MIRROR` 和 `PIPER_PIP_INDEX` 可覆盖包源。`vendor/` 保存 `vendor.lock` 对应的依赖源码；原样复制可避免重新下载 Git 仓库。

## CLI 与仿真

```bash
./piper-draw prepare input/style_reference.png --from-line-art
./piper-draw --ros bringup --mode mock
# 仿真启动后，从另一个终端使用上面 prepare 打印的任务文件：
./piper-draw --ros plan output/任务目录/任务_job.yaml
```

如果只构建了 ROS 镜像，第一行也可用 `./piper-draw --ros prepare`。容器内项目固定在 `/workspace`，可传项目内相对路径；项目根目录只读，生成物写入 `output/`。Web 和 `--ros` 容器共享宿主网络与 IPC，让同机的 ROS 进程通过 DDS 通信。默认使用 ROS 域 106，同一台机器同时开独立后端时应选不同的 `ROS_DOMAIN_ID`。

接入真机后，先确认目标机的 CAN 接口名与 `config/device.yaml` 一致，再手动运行 `sudo bash scripts/can_up.sh can0` 和 `./piper-draw robot doctor`。历史设备诊断和标定随目录保留，应按目标设备与现场重新核对。

## 搬到另一台机器

复制整个项目目录，保留 `vendor/`、`config/`、`input/` 和 `output/`。`output/` 包含任务、诊断及 Web 的工作配置、标定采样和上传文件。复制后让启动用户拥有这些文件的访问权限。照片模式需要 Seedream 密钥：按需通过受信渠道单独复制 `api_key.txt` 并保持 `0600`。密钥、配置和项目数据通过目录挂载提供，镜像中没有这些文件。

例如，在源机的项目根目录将文件复制到目标机（先停止 Web 和其他写入任务）：

```bash
rsync -a --exclude='/api_key.txt' ./ 用户@目标主机:~/piperdraw/
```

同架构且无法访问镜像源时，可单独打包和导入已构建的镜像：

```bash
# 源机（只用 ROS 时可省略第二个镜像）
sudo docker save piperdraw/ros:humble piperdraw/hardware:0.1 | gzip > ../piperdraw-images.tar.gz
# 目标机
gzip -dc ../piperdraw-images.tar.gz | sudo docker load
sudo PIPER_UID="$(id -u)" PIPER_GID="$(id -g)" docker compose up -d --no-build --pull never
```

更换 CPU 架构时，在目标机按其架构构建镜像。若同一台主机运行两份项目，用 `docker compose -p 不同项目名` 区分，同时为第二份设置独立的 Web 端口和 ROS 域。
