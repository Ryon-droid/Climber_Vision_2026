import cv2
import yaml
import os

# 读取 sentry.yaml 
script_dir = os.path.dirname(os.path.abspath(__file__))
config_path = os.path.join(script_dir, '../configs/sentry.yaml')
with open(config_path, 'r', encoding='utf-8') as f:
    config = yaml.safe_load(f)

# 获取USB相机参数
image_width = int(config['image_width'])
image_height = int(config['image_height'])
usb_exposure = config['usb_exposure']
usb_frame_rate = config['usb_frame_rate']
usb_gamma = config['usb_gamma']
usb_gain = config['usb_gain']
camera_name_map = config['camera_name_map']

print("感知相机配置:")
print(f"  分辨率: {image_width}x{image_height}")
print(f"  曝光: {usb_exposure}")
print(f"  帧率: {usb_frame_rate}")
print(f"  伽马: {usb_gamma}")
print(f"  增益: {usb_gain}")
print(f"  相机映射: {camera_name_map}")

# 初始化相机
cap = cv2.VideoCapture('/dev/camera_back', cv2.CAP_V4L2)

# 设置相机参数
cap.set(cv2.CAP_PROP_FRAME_WIDTH, image_width)
cap.set(cv2.CAP_PROP_FRAME_HEIGHT, image_height)
cap.set(cv2.CAP_PROP_FPS, usb_frame_rate)
cap.set(cv2.CAP_PROP_EXPOSURE, usb_exposure)
cap.set(cv2.CAP_PROP_GAMMA, usb_gamma)
cap.set(cv2.CAP_PROP_GAIN, usb_gain)

print(f"\n实际相机参数:")
print(f"  分辨率: {int(cap.get(cv2.CAP_PROP_FRAME_WIDTH))}x{int(cap.get(cv2.CAP_PROP_FRAME_HEIGHT))}")
print(f"  帧率: {cap.get(cv2.CAP_PROP_FPS)}")
print(f"  曝光: {cap.get(cv2.CAP_PROP_EXPOSURE)}")

while True:
    ret, frame = cap.read()
    if not ret:
        print("Failed to grab frame")
        break
    cv2.imshow('USB Camera Test', frame)
    if cv2.waitKey(1) & 0xFF == ord('q'):
        break

cap.release()
cv2.destroyAllWindows()