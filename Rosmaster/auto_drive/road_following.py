import torch
import torchvision

from torch2trt import torch2trt
from torch2trt import TRTModule

from utils import preprocess
import numpy as np

from Rosmaster_Lib import Rosmaster

from jetcam.usb_camera import USBCamera


CATEGORIES = ['apex']
print("torchvision cuda")
device = torch.device('cuda')
model = torchvision.models.resnet18(pretrained=False)
model.fc = torch.nn.Linear(512, 2 * len(CATEGORIES))
model = model.cuda().eval().half()


print("load model")
model.load_state_dict(torch.load('road_following_model.pth'))


data = torch.zeros((1, 3, 224, 224)).cuda().half()
print("torch to trt")
model_trt = torch2trt(model, [data], fp16_mode=True)

print("save model trt")
torch.save(model_trt.state_dict(), 'road_following_model_trt.pth')

print("load model trt")
model_trt = TRTModule()
model_trt.load_state_dict(torch.load('road_following_model_trt.pth'))

print("load Rosmaster")
car = Rosmaster(com='/dev/ttyUSB0')
print("load camera")
camera = USBCamera(width=224, height=224)


STEERING_GAIN = 45
STEERING_BIAS = 0.00

speed = 0.3

car.set_beep(100)

print("start program")
try:
    while True:
        image = camera.read()
        image = preprocess(image).half()
        output = model_trt(image).detach().cpu().numpy().flatten()
        x = float(output[0])
        fvalue = x * STEERING_GAIN + STEERING_BIAS
        car.set_akm_steering_angle(fvalue, True)
        print("car:", fvalue)
        # car.set_car_motion(speed, fvalue, 0)
except:
    del camera

