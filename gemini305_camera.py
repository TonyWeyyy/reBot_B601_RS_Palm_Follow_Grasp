#!/usr/bin/env python3
"""
Orbbec Gemini 305 RGB-D 相机采集封装模块（基于 Orbbec SDK v2 / pyorbbecsdk2）
===========================================================================
针对奥比中光 Gemini 305 超紧凑近距双目腕部相机（42×42×23 mm, ~68 g, 18 mm 基线,
理想深度 0.07 ~ 0.50 m, 工作范围 0.04 ~ 1.0 m+, 全局快门）优化，向上层视觉伺服与标定工具
提供统一采集接口：

    cam.fx, cam.fy, cam.cx, cam.cy    -> 硬件 D2C 对齐后的 RGB 相机内参（像素）
    cam.camera_matrix                 -> 3×3 float32 内参矩阵
    cam.dist_coeffs                   -> 5 维 float32 畸变系数（默认安全置零，可配置开关）
    cam.depth_scale                   -> 深度原始值到米的换算系数（m/unit，首帧自动读取）
    cam.get_frames(latest_only=True)  -> (color_bgr, depth_raw, depth_scale)
    cam.get_last_frame_timing()       -> (timestamp_s, source_str)

Gemini 305 关键适配特性：
  1. **原生 60fps 彩色/深度与独立分辨率解耦**：
     Gemini 305 原生支持 Color 1280×800@60fps / 848×480@60fps / 640×400@60fps 与 Depth
     848×480@60fps / 1280×800@30fps，不再依赖 Gemini 2 专属的 ``Binned Sparse`` 深度模式切换；
     初始化时按「目标 fps 优先 + 像素面积最小（低解码/MediaPipe 延迟）」自动选取最优 profile，
     并通过 ``get_d2c_depth_profile_list(..., HW_MODE)`` 自动匹配硬件 D2C 兼容深度流。
  2. **多彩色编码兼容（MJPG / RGB / BGR / YUYV / UYVY）**：
     自动识别实际枚举到的编码格式并转为 OpenCV BGR。
  3. **动态 ``depth_scale`` 自动换算**：
     首帧直接从 ``depth_frame.get_depth_scale()`` 读取 mm/unit 并除以 1000.0 转为 m/unit，
     兼容 1.0mm / 0.2mm / 0.1mm 等高精度近距深度尺度。
  4. **低延迟 ``latest_only=True`` 排空队列 + 可选闭环手动曝光压运动模糊**。
"""

from __future__ import annotations

import cv2
import numpy as np

from pyorbbecsdk import (
    Config,
    OBAlignMode,
    OBError,
    OBFormat,
    OBSensorType,
    Pipeline,
)


def _pick_best_color_profile(color_profiles, width: int, height: int, fps: int):
    """按优先级选择最适合实时视觉伺服的 Gemini 305 彩色流配置。"""
    for fmt in (OBFormat.MJPG, OBFormat.RGB, getattr(OBFormat, "YUYV", None)):
        if fmt is None:
            continue
        try:
            return color_profiles.get_video_stream_profile(width, height, fmt, fps)
        except OBError:
            continue

    supported_fmts = {OBFormat.MJPG, OBFormat.RGB}
    for extra in ("BGR", "YUYV", "UYVY"):
        val = getattr(OBFormat, extra, None)
        if val is not None:
            supported_fmts.add(val)

    all_profiles = [
        color_profiles.get_stream_profile_by_index(i)
        for i in range(color_profiles.get_count())
    ]

    exact_fps = [p for p in all_profiles if p.get_fps() == fps and p.get_format() in supported_fmts]
    if exact_fps:
        def _rank(p):
            fmt_prio = 0 if p.get_format() == OBFormat.MJPG else (1 if p.get_format() == OBFormat.RGB else 2)
            area = p.get_width() * p.get_height()
            too_small = 1 if p.get_width() < 480 else 0
            return (too_small, fmt_prio, area)

        return min(exact_fps, key=_rank)

    usable = [p for p in all_profiles if p.get_format() in supported_fmts]
    if usable:
        max_fps = max(p.get_fps() for p in usable)
        best_fps = [p for p in usable if p.get_fps() == max_fps]
        return min(
            best_fps,
            key=lambda p: (0 if p.get_format() == OBFormat.MJPG else 1, p.get_width() * p.get_height()),
        )

    return color_profiles.get_default_video_stream_profile()


def _decode_color_frame(color_frame) -> np.ndarray | None:
    """将 Orbbec ColorFrame（MJPG / RGB / BGR / YUYV / UYVY）解码为 OpenCV uint8 BGR 图像。"""
    fmt = color_frame.get_format()
    raw = np.asanyarray(color_frame.get_data())
    h, w = color_frame.get_height(), color_frame.get_width()

    if fmt == OBFormat.MJPG:
        return cv2.imdecode(raw, cv2.IMREAD_COLOR)
    if fmt == OBFormat.RGB:
        return cv2.cvtColor(raw.reshape(h, w, 3), cv2.COLOR_RGB2BGR)
    if getattr(OBFormat, "BGR", None) is not None and fmt == OBFormat.BGR:
        return raw.reshape(h, w, 3).copy()
    if getattr(OBFormat, "YUYV", None) is not None and fmt == OBFormat.YUYV:
        return cv2.cvtColor(raw.reshape(h, w, 2), cv2.COLOR_YUV2BGR_YUYV)
    if getattr(OBFormat, "UYVY", None) is not None and fmt == OBFormat.UYVY:
        return cv2.cvtColor(raw.reshape(h, w, 2), cv2.COLOR_YUV2BGR_UYVY)
    return cv2.cvtColor(raw.reshape(h, w, 3), cv2.COLOR_RGB2BGR)


class Gemini305Camera:
    """Orbbec Gemini 305 RGB-D 相机封装（Orbbec SDK v2 Pipeline + HW D2C）。"""

    def __init__(
        self,
        width: int = 848,
        height: int = 480,
        fps: int = 60,
        align_to_color: bool = True,
        color_exposure_ms: float = 0.0,
        force_zero_distortion: bool = True,
    ) -> None:
        self.width = int(width)
        self.height = int(height)
        self.force_zero_distortion = bool(force_zero_distortion)

        self.pipeline = Pipeline()
        config = Config()

        try:
            dev_info = self.pipeline.get_device().get_device_info()
            dev_name = dev_info.get_name() if hasattr(dev_info, "get_name") else "Orbbec"
            dev_sn = dev_info.get_serial_number() if hasattr(dev_info, "get_serial_number") else ""
            print(f"[Gemini305] 已连接设备: {dev_name} (SN={dev_sn})")
        except Exception:
            pass

        # ---- 1. 彩色流选择 ----
        color_profiles = self.pipeline.get_stream_profile_list(OBSensorType.COLOR_SENSOR)
        color_profile = _pick_best_color_profile(color_profiles, self.width, self.height, int(fps))
        config.enable_stream(color_profile)
        self.width = int(color_profile.get_width())
        self.height = int(color_profile.get_height())
        self.fps = int(color_profile.get_fps())
        print(f"[Gemini305] 彩色流: {self.width}x{self.height}@{self.fps} {color_profile.get_format()}")

        # ---- 2. 深度流选择（优先硬件 D2C 兼容列表 + 匹配帧率）----
        depth_profile = None
        if align_to_color:
            try:
                d2c_profiles = self.pipeline.get_d2c_depth_profile_list(color_profile, OBAlignMode.HW_MODE)
                d2c_list = [
                    d2c_profiles.get_stream_profile_by_index(i)
                    for i in range(d2c_profiles.get_count())
                ]
                if d2c_list:
                    same_fps = [p for p in d2c_list if p.get_fps() == self.fps]
                    pool = same_fps if same_fps else d2c_list
                    depth_profile = max(
                        pool,
                        key=lambda p: (
                            p.get_fps(),
                            -abs(p.get_width() - self.width) - abs(p.get_height() - self.height),
                        ),
                    )
            except OBError:
                depth_profile = None

        if depth_profile is None:
            depth_profiles = self.pipeline.get_stream_profile_list(OBSensorType.DEPTH_SENSOR)
            try:
                depth_profile = depth_profiles.get_video_stream_profile(
                    self.width, self.height, OBFormat.Y16, self.fps
                )
            except OBError:
                cands = [
                    depth_profiles.get_stream_profile_by_index(i)
                    for i in range(depth_profiles.get_count())
                ]
                y16 = [p for p in cands if p.get_format() == OBFormat.Y16]
                if y16:
                    same_fps = [p for p in y16 if p.get_fps() == self.fps]
                    pool = same_fps if same_fps else y16
                    depth_profile = max(pool, key=lambda p: (p.get_fps(), -abs(p.get_width() - self.width)))
                else:
                    depth_profile = depth_profiles.get_default_video_stream_profile()

        config.enable_stream(depth_profile)
        print(
            f"[Gemini305] 深度流: {depth_profile.get_width()}x"
            f"{depth_profile.get_height()}@{depth_profile.get_fps()} {depth_profile.get_format()}"
        )

        # ---- 3. 硬件 D2C 对齐 + 帧同步 ----
        if align_to_color:
            try:
                config.set_align_mode(OBAlignMode.HW_MODE)
            except OBError:
                config.set_align_mode(OBAlignMode.SW_MODE)
                print("[Gemini305] 硬件 D2C 不可用，已自动回退软件 D2C 对齐")
            try:
                self.pipeline.enable_frame_sync()
            except OBError:
                pass

        self.pipeline.start(config)

        # ---- 4. 读取对齐后 RGB 内参 ----
        camera_param = self.pipeline.get_camera_param()
        intr = camera_param.rgb_intrinsic
        self.fx, self.fy, self.cx, self.cy = (
            float(intr.fx),
            float(intr.fy),
            float(intr.cx),
            float(intr.cy),
        )
        self.camera_matrix = np.array(
            [[self.fx, 0.0, self.cx], [0.0, self.fy, self.cy], [0.0, 0.0, 1.0]],
            dtype=np.float32,
        )
        if self.force_zero_distortion:
            self.dist_coeffs = np.zeros(5, dtype=np.float32)
        else:
            d = camera_param.rgb_distortion
            self.dist_coeffs = np.array([d.k1, d.k2, d.p1, d.p2, d.k3], dtype=np.float32)

        self.depth_scale: float | None = None
        self.dropped_frames: int = 0
        self._last_frame_timestamp_s: float | None = None
        self._timestamp_source: str = ""

        print("[Gemini305] 预热相机...")
        for _ in range(20):
            self.pipeline.wait_for_frames(100)

        # ---- 5. 可选彩色手动曝光 ----
        if color_exposure_ms and color_exposure_ms > 0:
            self._apply_manual_exposure(float(color_exposure_ms))

        print(
            f"[Gemini305] 相机就绪  内参: fx={self.fx:.2f} fy={self.fy:.2f} "
            f"cx={self.cx:.2f} cy={self.cy:.2f} (zero_dist={self.force_zero_distortion})"
        )

    def _apply_manual_exposure(self, color_exposure_ms: float) -> None:
        try:
            from pyorbbecsdk import OBPropertyID

            device = self.pipeline.get_device()

            def _grab_brightness(discard: int = 6, take: int = 2) -> float:
                vals = []
                for i in range(discard + take):
                    c, _, _ = self.get_frames()
                    if c is not None and i >= discard:
                        vals.append(float(c.mean()))
                return float(max(vals)) if vals else 0.0

            auto_brightness = _grab_brightness()
            if auto_brightness < 10.0:
                auto_brightness = 80.0
            device.set_bool_property(OBPropertyID.OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, False)
            ok = False
            for us_per_unit in (100, 10, 1):
                units = max(1, int(round(color_exposure_ms * 1000.0 / us_per_unit)))
                device.set_int_property(OBPropertyID.OB_PROP_COLOR_EXPOSURE_INT, units)
                bri = _grab_brightness()
                print(
                    f"[Gemini305] 曝光试写 {units} ({us_per_unit}μs/单位) -> "
                    f"亮度 {bri:.0f} (基准 {auto_brightness:.0f})"
                )
                if bri >= max(25.0, 0.40 * auto_brightness):
                    print(f"[Gemini305] 手动彩色曝光生效: {color_exposure_ms}ms = {units} 单位")
                    ok = True
                    break
            if not ok:
                device.set_bool_property(OBPropertyID.OB_PROP_COLOR_AUTO_EXPOSURE_BOOL, True)
                print("[Gemini305] 三档亮度不足，已恢复自动曝光")
        except Exception as e:
            print(f"[Gemini305] 手动曝光设置失败({e})，保持自动曝光")

    @staticmethod
    def _read_timestamp_seconds(frame) -> tuple[float | None, str]:
        candidates = (
            ("get_timestamp_us", 1e-6),
            ("get_time_stamp_us", 1e-6),
            ("get_timestamp_ns", 1e-9),
            ("get_time_stamp_ns", 1e-9),
            ("get_timestamp", 1e-3),
            ("get_time_stamp", 1e-3),
        )
        for name, scale in candidates:
            method = getattr(frame, name, None)
            if not callable(method):
                continue
            try:
                value = float(method())
                if np.isfinite(value) and value >= 0.0:
                    return value * scale, f"{name}×{scale:g}"
            except Exception:
                continue
        return None, ""

    def get_last_frame_timing(self) -> tuple[float | None, str]:
        return self._last_frame_timestamp_s, self._timestamp_source

    def get_frames(self, latest_only: bool = True):
        """读取最新一帧硬件对齐的 ``(color_bgr, depth_raw, depth_scale)``。"""
        frames = self.pipeline.wait_for_frames(100)
        if frames is None:
            return None, None, None
        if latest_only:
            while True:
                newer = self.pipeline.wait_for_frames(0)
                if newer is None:
                    break
                frames = newer
                self.dropped_frames += 1

        color_frame = frames.get_color_frame()
        depth_frame = frames.get_depth_frame()
        if color_frame is None or depth_frame is None:
            self._last_frame_timestamp_s = None
            return None, None, None

        timestamp_s, source = self._read_timestamp_seconds(color_frame)
        if timestamp_s is None:
            timestamp_s, source = self._read_timestamp_seconds(depth_frame)
        self._last_frame_timestamp_s = timestamp_s
        if source and source != self._timestamp_source:
            self._timestamp_source = source
            print(f"[Gemini305] 帧硬件时间接口: {source}")

        color_image = _decode_color_frame(color_frame)
        h, w = depth_frame.get_height(), depth_frame.get_width()
        depth_raw = np.frombuffer(depth_frame.get_data(), dtype=np.uint16).reshape(h, w)

        if self.depth_scale is None:
            scale_mm = float(depth_frame.get_depth_scale())
            self.depth_scale = scale_mm / 1000.0
            print(f"[Gemini305] depth_scale = {scale_mm:.4f} mm/unit ({self.depth_scale:.6f} m/unit)")

        return color_image, depth_raw, self.depth_scale

    def stop(self) -> None:
        self.pipeline.stop()


# 向后兼容别名
Gemini2Camera = Gemini305Camera


if __name__ == "__main__":
    cam = Gemini305Camera()
    print("按 q 退出")
    try:
        while True:
            color, depth, scale = cam.get_frames()
            if color is None:
                continue
            depth_m = depth.astype(np.float32) * scale
            depth_vis = cv2.normalize(np.clip(depth_m, 0.0, 1.0), None, 0, 255, cv2.NORM_MINMAX).astype(np.uint8)
            depth_vis = cv2.applyColorMap(depth_vis, cv2.COLORMAP_JET)
            cv2.imshow("Gemini 305 Color", color)
            cv2.imshow("Gemini 305 Depth (0-1m)", depth_vis)
            if cv2.waitKey(1) & 0xFF == ord("q"):
                break
    finally:
        cam.stop()
        cv2.destroyAllWindows()
