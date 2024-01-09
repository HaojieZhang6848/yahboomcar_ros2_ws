#!/usr/bin/env python3
#!/usr/bin/env python3
# -*- coding: utf-8 -*-



import os
from PIL import Image



def file2img(filename, target):
    fr = open(filename)
    #name = filename.split('/')[-1][:-4]
    #print(name)
    image = Image.new("L", (32, 32))
    for i in range(32):
        lineStr = fr.readline()
        for j in range(32):
            colorValue = int(lineStr[j])
            if colorValue == 1:
                colorValue = 255
            image.putpixel((j, i), int(colorValue))
            #image.save(target + "/" + name + ".png")
            image.save(target + ".png")



def genImg(filename, target):
    fileList = os.listdir(filename)
    m = len(fileList)
    if not os.path.exists(target):
        os.mkdir(target)
    for i in range(m):
        file2img(filename + fileList[i], target)


#genImg("knn-digits/trainingDigits/", "training_img")
#genImg("knn-digits/testDigits/", "test_img")
file2img("/home/yahboom/opencv/this_is_2.txt","this_is")
os.system("say 'Finish!'")
