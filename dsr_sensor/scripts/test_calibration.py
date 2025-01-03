#!/usr/bin/env python3

import json
import numpy as np

def save_calibration(reference_pixel, scale, filename="calibration_params.json"):
    """캘리브레이션 결과 저장"""
    params = {
        "reference_pixel": reference_pixel.tolist(),
        "scale": scale
    }
    with open(filename, "w") as f:
        json.dump(params, f)
    print(f"Calibration parameters saved to {filename}.")

def main():
    reference_pixel = np.array([352.16, 205.20])  # 감지된 1번 코너 픽셀 좌표
    scale = 0.334857  # 계산된 픽셀-월드 변환 비율 (mm/pixel)
    
    save_calibration(reference_pixel, scale)

if __name__ == "__main__":
    main()
# import json

# json_path = "/home/mk/dev_ws/prject/doosan_robotics2/src/doosan-robot2/calibration_params.json"

# with open(json_path, "r") as f:
#     params = json.load(f)

# # calibration_origin 추가
# params["calibration_origin"] = [-518.42, -214.15]  # 기본값 설정

# with open(json_path, "w") as f:
#     json.dump(params, f, indent=4)

# print(f"Updated {json_path} with calibration_origin.")