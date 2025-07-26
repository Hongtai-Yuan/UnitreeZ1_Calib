import pyrealsense2 as rs
import numpy as np
#  def __init__(self,width=640,height=480,fps=15):
class RealsenseD435:
    def __init__(self):
        self.im_height, self.im_width = 720, 1280
        self.pipeline = rs.pipeline()
        config = rs.config()
        config.enable_stream(rs.stream.depth, self.im_width, self.im_height, rs.format.z16, 30)
        config.enable_stream(rs.stream.color, self.im_width, self.im_height, rs.format.bgr8, 30)
        self.profile = self.pipeline.start(config)

        # 提取 color 相机内参
        color_stream   = self.profile.get_stream(rs.stream.color)
        color_profile  = color_stream.as_video_stream_profile()
        intr           = color_profile.get_intrinsics()

        # 构造 3×3 内参矩阵
        self.cam_intrinsics = np.array([
            [intr.fx, 0.0,      intr.ppx],
            [0.0,      intr.fy, intr.ppy],
            [0.0,      0.0,      1.0]
        ], dtype=np.float64)

        # **新增**：把畸变系数存下来
        # intr.coeffs 是一个长度为 5（或更多）的列表：[k1, k2, p1, p2, k3, …]
        self.dist_coeffs = np.array(intr.coeffs, dtype=np.float64)

    def get_data(self):
        frames      = self.pipeline.wait_for_frames()
        depth_frame = frames.get_depth_frame()
        color_frame = frames.get_color_frame()
        depth_img   = np.asarray(depth_frame.get_data())
        color_img   = np.asarray(color_frame.get_data())
        return color_img, depth_img

    def __del__(self):
        try:
            self.pipeline.stop()
        except:
            pass
