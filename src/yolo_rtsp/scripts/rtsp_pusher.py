# rtsp_pusher.py - GStreamer GPU推流器（生产级）

# 特性：
# - GPU 硬编码 (nvv4l2h264enc)，不占 CPU
# - GPU 失败自动降级到 CPU (x264enc via GStreamer)
# - mediaMTX 重启后自动重连
# - 线程安全

# 推流流程 (GPU)：
#   OpenCV帧(BGR) → cvtColor(RGB) → appsrc → videoconvert → nvvidconv
#   → nvv4l2h264enc → h264parse → rtspclientsink → mediaMTX

# 推流流程 (CPU fallback)：
#   OpenCV帧(BGR) → appsrc → videoconvert → x264enc
#   → h264parse → rtspclientsink → mediaMTX

import sys
import time
import threading
import cv2
import config
import numpy as np

try:
    import rospy
except ImportError:
    rospy = None


def _log(msg, level="info"):
    if rospy:
        if level == "warn":
            rospy.logwarn(msg)
        elif level == "error":
            rospy.logerr(msg)
        else:
            rospy.loginfo(msg)
    else:
        print(msg, file=sys.stderr, flush=True)

try:
    import gi
    gi.require_version('Gst', '1.0')
    from gi.repository import Gst, GLib
    Gst.init(None)
    GST_AVAILABLE = True
except Exception as e:
    GST_AVAILABLE = False
    _log(f"[PUSH] GStreamer not available: {e}", "error")


class AnnotatedPusher:
    #''' GPU 推流器（带自动重连'''

    def __init__(self, url):
        self.url = url
        self.pipeline = None
        self.appsrc = None
        self.loop = None
        self.thread = None
        self.width = 0
        self.height = 0
        self.running = False
        self.lock = threading.Lock()
        self.reconnect_interval = 2.0
        self.use_gpu = True
        self._push_count = 0
        self._warn_count = 0
        self._frame_count = 0

    # =====================================================================
    # 改动一(改1)：start() 不再直接构建 pipeline，改为只启动后台线程，
    #             让 pipeline 在 _run_loop 线程内构建。
    # =====================================================================
    def start(self, width, height):
        self.width, self.height = width, height

        if not GST_AVAILABLE:
            _log("[PUSH] GStreamer not available, push disabled", "warn")
            return False

        self.use_gpu = self._check_gpu_encoder()
        if not self.use_gpu:
            _log("[PUSH] GPU encoder not available, using CPU", "warn")

        self.running = True
        self.thread = threading.Thread(target=self._run_loop, daemon=True)
        self.thread.start()

        # 等待后台线程完成 pipeline 构建，否则 write() 可能拿到 None
        while self.pipeline is None and self.running:
            time.sleep(0.01)

        if self.pipeline is None:
            _log("[PUSH] pipeline build failed, not started", "error")
            return False

        mode = "GPU" if self.use_gpu else "CPU"
        _log(f"[PUSH] streaming started -> {self.url} ({mode})")
        return True

    def _check_gpu_encoder(self):
        try:
            test = Gst.parse_launch('nvv4l2h264enc ! fakesink silent=true')
            test.set_state(Gst.State.READY)
            test.set_state(Gst.State.NULL)
            return True
        except Exception:
            return False

    def _build_pipeline(self):
        bitrate = config.PUSH_BITRATE * 1000
        fps = config.PUSH_FPS
        w, h = self.width, self.height

        if self.use_gpu:
            pipeline_str = (
                f'appsrc name=src is-live=true format=time do-timestamp=false ' #true
                f'caps=video/x-raw,format=RGB,width={w},height={h},framerate={fps}/1 '
                f'! queue '
                f'! videoconvert '
                f'! video/x-raw,format=I420,width={w},height={h},framerate={fps}/1 '
                f'! queue '
                f'! nvvidconv '
                f'! video/x-raw(memory:NVMM),width={w},height={h},framerate={fps}/1,format=NV12 '
                f'! queue '
                f'! nvv4l2h264enc bitrate={bitrate} iframeinterval={fps} '
                f'! h264parse config-interval=-1 '
                f'! queue '
                f'! rtspclientsink location={self.url} protocols=tcp sync=false'
            )
        else:
            pipeline_str = (
                f'appsrc name=src is-live=true format=time do-timestamp=false '
                f'caps=video/x-raw,format=BGR,width={w},height={h},framerate={fps}/1 '
                f'! queue '
                f'! videoconvert '
                f'! video/x-raw,format=I420,width={w},height={h},framerate={fps}/1 '
                f'! queue '
                f'! x264enc bitrate={int(bitrate/1000)} tune=zerolatency speed-preset=ultrafast '
                f'! video/x-h264,stream-format=byte-stream,alignment=au,profile=baseline '
                f'! h264parse config-interval=-1 '
                f'! queue '
                f'! rtspclientsink location={self.url} protocols=tcp sync=false'
            )

        try:
            _log(f"[PUSH] building pipeline: {pipeline_str[:120]}...")
            self.pipeline = Gst.parse_launch(pipeline_str)
            self.appsrc = self.pipeline.get_by_name('src')

            if self.appsrc is None:
                _log("[PUSH] ERROR: appsrc element not found!", "error")
                self.pipeline = None
                return False

            bus = self.pipeline.get_bus()
            bus.add_signal_watch()          # 关键：现在在 _run_loop 线程里调用，
                                            # 与 MainLoop 同属一个 GMainContext，信号才会被派发
            bus.connect('message::error', self._on_error)
            bus.connect('message::eos', self._on_eos)
            bus.connect('message::warning', self._on_warning)

            ret = self.pipeline.set_state(Gst.State.PLAYING)
            _log(f"[PUSH] pipeline PLAYING: {ret} (0=SUCCESS,1=NO_PREROLL,2=ASYNC,3=FAILURE)")

            if ret == Gst.StateChangeReturn.FAILURE:
                _log("[PUSH] ERROR: pipeline failed to enter PLAYING state!", "error")
                self.pipeline.set_state(Gst.State.NULL)
                self.pipeline = None
                self.appsrc = None
                return False

            return True
        except Exception as e:
            _log(f"[PUSH] pipeline build failed: {e}", "error")
            self.pipeline = None
            self.appsrc = None
            return False

    # =====================================================================
    # 改动一(改2)：_run_loop 里先构建 pipeline 再 run()，
    #             保证 bus watch 与 MainLoop 在同一线程上下文。
    # =====================================================================
    def _run_loop(self):
        self.loop = GLib.MainLoop()
        if not self._build_pipeline():
            _log("[PUSH] background thread pipeline build failed", "error")
            return
        self.loop.run()

    def _on_error(self, bus, msg):
        err, debug = msg.parse_error()
        _log(f"[PUSH] GStreamer ERROR: {err.message}", "error")
        _log(f"[PUSH]   debug: {debug}", "error")
        self._schedule_reconnect()

    def _on_eos(self, bus, msg):
        _log("[PUSH] GStreamer EOS, reconnecting")
        self._schedule_reconnect()

    def _on_warning(self, bus, msg):
        warn, debug = msg.parse_warning()
        _log(f"[PUSH] GStreamer WARNING: {warn.message}", "warn")

    def _schedule_reconnect(self):
        if self.running and self.loop:
            GLib.timeout_add_seconds(int(self.reconnect_interval), self._reconnect)

    def _reconnect(self):
        with self.lock:
            _log("[PUSH] reconnecting...")
            if self.pipeline:
            # 清理旧管道的监听指针残留，避免 MainLoop 残留指向旧 pipe 的 IO 源
                try:
                    self.pipeline.get_bus().remove_signal_watch()
                except Exception:
                    pass
                self.pipeline.set_state(Gst.State.NULL)
                self.pipeline = None
                self.appsrc = None
            if self._build_pipeline():
                _log("[PUSH] reconnect success")
            else:
                _log(f"[PUSH] reconnect failed, retry in {self.reconnect_interval}s", "warn")
                GLib.timeout_add_seconds(int(self.reconnect_interval), self._reconnect)
        return False

    # =====================================================================
    # 改动三：write() 里对 FLUSHING 处理 —— 下行满时直接丢弃该帧，不阻塞
    # =====================================================================
    def write(self, frame):
        if self.pipeline is None or self.appsrc is None:
            return False

        with self.lock:
            try:
                h, w = frame.shape[:2]
                if (w, h) != (self.width, self.height):
                    frame = cv2.resize(frame, (self.width, self.height))
                if not frame.flags['C_CONTIGUOUS']:
                    frame = np.ascontiguousarray(frame)

                if self.use_gpu:
                    rgb = cv2.cvtColor(frame, cv2.COLOR_BGR2RGB)
                    data = rgb.tobytes()
                else:
                    data = frame.tobytes()

                buf = Gst.Buffer.new_wrapped(data)
                            # ===== 新增：手动设置均匀递增的 PTS =====
                # pts = self._push_count * (Gst.SECOND / config.PUSH_FPS)
                # buf.pts = pts
                # buf.dts = pts
            # ========================================
                fps = config.PUSH_FPS                           # 帧率（如 30）
                pts_ns = self._frame_count * (Gst.SECOND // fps)  # 每帧间隔（纳秒）
                buf.pts = pts_ns                                 # 设置 PTS
                buf.dts = pts_ns                                 # 设置 DTS（与 PTS 相同）
                buf.duration = Gst.SECOND // fps                 # 设置帧时长
                self._frame_count += 1

                result = self.appsrc.emit('push-buffer', buf)

                self._push_count += 1
                if self._push_count % 30 == 1:
                    _log(f"[PUSH] pushed {self._push_count} frames, last: {w}x{h}")

                # FLUSHING 表示 appsrc 被下游阻塞，丢弃该帧不阻塞
                if int(result) == int(Gst.FlowReturn.FLUSHING):
                    self._warn_count += 1
                    return False

                if int(result) != int(Gst.FlowReturn.OK):
                    self._warn_count += 1
                    if self._warn_count <= 5:
                        _log(f"[PUSH] push-buffer failed: {result} (frame={w}x{h}, total_fails={self._warn_count})", "warn")

                return result == Gst.FlowReturn.OK
            except Exception as e:
                _log(f"[PUSH] push frame failed: {e}", "error")
                return False

    def release(self):
        self.running = False
        with self.lock:
            if self.pipeline:
                if self.appsrc:
                    self.appsrc.emit('end-of-stream')
                self.pipeline.set_state(Gst.State.NULL)
                self.pipeline = None
                self.appsrc = None
        if self.loop and self.loop.is_running():
            self.loop.quit()
        if self.thread and self.thread.is_alive():
            self.thread.join(timeout=2.0)
        _log("[PUSH] pusher stopped")