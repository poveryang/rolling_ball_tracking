import cv2
import numpy as np
import json
import os
from typing import List, Tuple

class TrajectoryVisualizer:
    def __init__(self, video_path: str, trajectory_path: str):
        """
        初始化轨迹可视化器
        :param video_path: 视频文件路径
        :param trajectory_path: 轨迹数据文件路径
        """
        self.video_path = video_path
        self.trajectory_path = trajectory_path
        self.cap = None
        self.trajectory_data = []
        self.frame_width = 0
        self.frame_height = 0
        self.fps = 0
        self.total_frames = 0

    def load_trajectory(self) -> bool:
        """
        加载轨迹数据
        :return: 是否成功加载
        """
        if not os.path.exists(self.trajectory_path):
            print(f"轨迹文件不存在: {self.trajectory_path}")
            return False

        try:
            with open(self.trajectory_path, 'r') as f:
                data = json.load(f)
                self.trajectory_data = data["track_trajectory"]["points"]
            return True
        except Exception as e:
            print(f"加载轨迹文件失败: {e}")
            return False

    def initialize_video(self) -> bool:
        """
        初始化视频
        :return: 是否成功初始化
        """
        if not os.path.exists(self.video_path):
            print(f"视频文件不存在: {self.video_path}")
            return False

        self.cap = cv2.VideoCapture(self.video_path)
        if not self.cap.isOpened():
            print("无法打开视频文件")
            return False

        self.frame_width = int(self.cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        self.frame_height = int(self.cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        self.fps = int(self.cap.get(cv2.CAP_PROP_FPS))
        self.total_frames = int(self.cap.get(cv2.CAP_PROP_FRAME_COUNT))

        return True

    def visualize(self, output_path: str = "output.mp4"):
        """
        可视化轨迹
        :param output_path: 输出视频路径
        """
        if not self.initialize_video() or not self.load_trajectory():
            return

        # 创建视频写入器
        fourcc = cv2.VideoWriter_fourcc(*'mp4v')
        out = cv2.VideoWriter(output_path, fourcc, self.fps, (self.frame_width, self.frame_height))

        frame_idx = 0
        trajectory_points = []  # 存储轨迹点

        while True:
            ret, frame = self.cap.read()
            if not ret:
                break

            # 绘制当前帧的轨迹点
            if frame_idx < len(self.trajectory_data):
                point = self.trajectory_data[frame_idx]
                x, y = int(point["x"]), int(point["y"])
                trajectory_points.append((x, y))

                # 绘制轨迹线
                for i in range(1, len(trajectory_points)):
                    cv2.line(frame, trajectory_points[i-1], trajectory_points[i], (0, 255, 0), 2)

                # 绘制当前点
                cv2.circle(frame, (x, y), 5, (0, 0, 255), -1)

            # 添加帧号显示
            cv2.putText(frame, f"Frame: {frame_idx}", (10, 30), 
                       cv2.FONT_HERSHEY_SIMPLEX, 1, (255, 255, 255), 2)

            out.write(frame)
            frame_idx += 1

        # 释放资源
        self.cap.release()
        out.release()
        cv2.destroyAllWindows()

def main():
    # 使用示例
    video_path = "test/test_video.MOV"
    trajectory_path = "config/trajectory.json"
    output_path = "output.mp4"

    visualizer = TrajectoryVisualizer(video_path, trajectory_path)
    visualizer.visualize(output_path)

if __name__ == "__main__":
    main() 