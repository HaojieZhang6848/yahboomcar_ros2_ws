# encoding=utf-8

from PIL import Image

import numpy as np


def img2txt(img_path, txt_name):

    im = Image.open(img_path).convert('1').resize((32, 32))  # type:Image.Image

    data = np.asarray(im)

    np.savetxt(txt_name, data, fmt='%d', delimiter='')
img2txt("8.png","./knn-digits/testDigits/8_1.txt")
