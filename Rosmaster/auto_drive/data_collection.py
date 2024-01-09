import cv2
import numpy as np
import cvui

import torchvision.transforms as transforms
from xy_dataset import XYDataset



# V0.0.4
# initial cvui
WINDOW_NAME = 'CVUI DATA COLLECTION'
cvui.init(WINDOW_NAME)

TASK = 'road_following'
CATEGORIES = ['apex']
DATASETS = ['A', 'B']

TRANSFORMS = transforms.Compose([
    transforms.ColorJitter(0.2, 0.2, 0.2, 0.2),
    transforms.Resize((224, 224)),
    transforms.ToTensor(),
    transforms.Normalize([0.485, 0.456, 0.406], [0.229, 0.224, 0.225])
])

datasets = {}
for name in DATASETS:
    datasets[name] = XYDataset(TASK + '_' + name, CATEGORIES, TRANSFORMS, random_hflip=True)

cap = cv2.VideoCapture(0)
if cap.isOpened():
    cap.set(cv2.CAP_PROP_FOURCC, cv2.VideoWriter.fourcc('M', 'J', 'P', 'G'))
    cap.set(cv2.CAP_PROP_FRAME_WIDTH, 640)   # 640
    cap.set(cv2.CAP_PROP_FRAME_HEIGHT, 480)  # 480
    cap.set(cv2.CAP_PROP_FPS, 30)            # 30
ret, frame = cap.read()
frame = cv2.resize(frame, (int(224), int(224)))
# image size
img_h = 224
img_w = 224


# total size
total_h = img_h
total_w = img_w*2+10


total_frame = np.zeros((total_h,  total_w, 3), np.uint8)
# 设置背景色 Setting the background color
total_frame[:] = (150, 150, 150) 


# 初始化数据表 initialize active dataset
category_value = CATEGORIES[0]
dataset = datasets[DATASETS[0]]
count_value = dataset.get_count(category_value)
print("count_value:", count_value)

# 保存当前摄像头画面到本地和数据表 Saves the current camera frame to local and datasheet
def save_snapshot(x, y, snapshot):
    global total_frame, count_value
    # 保存到本地 save to disk
    dataset.save_entry(category_value, snapshot, x, y)
    # 画一个小圆并显示图像 Draw a small circle and display the image
    snapshot = snapshot
    snapshot = cv2.circle(snapshot, (x, y), 8, (0, 255, 0), 3)
    total_frame[0:img_h, img_w+10:img_w*2+10] = snapshot
    count_value = dataset.get_count(category_value)
    print("x=", x,"y=", y)
    print("count_value:", count_value)

# 鼠标事件处理 Mouse event handling
def onMouse(event, x, y, flags, param):
    if event == 1: # 鼠标按下 The mouse click
        if x <= 224 and y <= 224:
            global frame
            save_snapshot(x, y, frame)

try:
    while(1):
        # process image
        ret, frame = cap.read()
        frame = cv2.resize(frame, (int(224), int(224)))
        total_frame[0:img_h, 0:img_w] = frame
        cv2.setMouseCallback(WINDOW_NAME, onMouse, 0)
        # show
        cvui.imshow(WINDOW_NAME, total_frame)
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
except KeyboardInterrupt:
    pass
cap.release()
cv2.destroyAllWindows()
