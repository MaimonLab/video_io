import cv2
import numpy as np


original_video = "../videos/sample_multi_arena.m4v"
output_video = "../data/sample_multi_arena_v2.avi"


def main():
    cap_dict = {
        "original": cv2.VideoCapture(original_video),
        "output": cv2.VideoCapture(output_video),
    }

    for cap_name, cap in cap_dict.items():
        fps = cap.get(cv2.CAP_PROP_FPS)
        width = int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))
        height = int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))
        total_frames = int(cap.get(cv2.CAP_PROP_FRAME_COUNT))
        print(
            f"{cap_name}, total frames: {total_frames}, wxh:{width}x{height}, fps: {fps}"
        )

    # total_n_frames = cap.get(cv::CAP_PROP_FRAME_COUNT);
    # int width = cap.get(cv::CAP_PROP_FRAME_WIDTH);
    # int height = cap.get(cv::CAP_PROP_FRAME_HEIGHT);
    # double fps = cap.get(cv::CAP_PROP_FPS);


if __name__ == "__main__":
    main()