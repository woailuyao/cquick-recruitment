import numpy as np
import cv2 as cv
import sys
import os


def imread_zh(path, flags=cv.IMREAD_COLOR):
    data = np.fromfile(path, dtype=np.uint8)
    if data.size == 0:
        return None
    return cv.imdecode(data, flags)


def imwrite_zh(path, img):
    ext = os.path.splitext(path)[1] or ".png"
    ok, buf = cv.imencode(ext, img)
    if ok:
        buf.tofile(path)
    return ok
    

def detect(img, lo=(8, 120, 80), hi=(25, 255, 255), min_area=500, min_ratio=0.5,ksize=5):
    hsv = cv.cvtColor(img, cv.COLOR_BGR2HSV)
    mask = cv.inRange(hsv, lo, hi)
    kernel = cv.getStructuringElement(cv.MORPH_ELLIPSE, (ksize, ksize))
    mask = cv.morphologyEx(mask, cv.MORPH_OPEN, kernel)

    cnts, _ = cv.findContours(mask, cv.RETR_EXTERNAL, cv.CHAIN_APPROX_SIMPLE)
    if not cnts:
        return False, -1.0, -1.0, 0.0, None, mask

    total = sum(cv.contourArea(c) for c in cnts)
    c = max(cnts, key=cv.contourArea)
    area = cv.contourArea(c)
    if total <= 0 or area < min_area or area < min_ratio * total:
        return False, -1.0, -1.0, 0.0, None, mask

    M = cv.moments(c)
    if M["m00"] == 0:
        return False, -1.0, -1.0, 0.0, None, mask

    u = M["m10"] / M["m00"]
    v = M["m01"] / M["m00"]
    return True, u, v, area, c, mask
    
    
def draw_result(img,detected,u,v,c):
    if detected:
        x, y, w, h = cv.boundingRect(c)
        cv.drawContours(img, [c], -1, (0, 255, 0), 2)
        cv.rectangle(img, (x, y), (x + w, y + h), (255, 0, 0), 1)
        cv.circle(img, (int(u), int(v)), 6, (0, 0, 255), -1)
        cv.putText(img, "u=%d v=%d" % (u, v), (x, y - 8), cv.FONT_HERSHEY_SIMPLEX, 0.6, (0, 0, 255), 2)
    else:
        cv.putText(img, "no target", (20, 40), cv.FONT_HERSHEY_SIMPLEX, 1.0, (0, 0, 255), 2)
    
    
BASE=os.path.dirname(os.path.abspath(__file__))
IMG_DIR=os.path.join(BASE,'img')
OUT_DIR=os.path.join(BASE,'out')


#Another available path that you can choose the file you exactedly want
'''filename=input("The file you want to process:")
img = imread_zh(os.path.join(IMG_DIR, f"{filename}.png"))
if img is None:
    print("Found no file!")
    sys.exit(1)


cv.imshow('maskdisplay',mask)
k=cv.waitKey(0)
if k==ord("s"):
    imwrite_zh(os.path.join(OUT_DIR, f"{filename}_output.png"), mask)'''

def main():
    names = ["test4", "test5", "test6", "test7", "test8"]
    for name in names:
        path = os.path.join(IMG_DIR, name + ".png")
        img = imread_zh(path)
        if img is None:
            print(name, "Failed imread!")
            continue


        detected, u, v, area, c, mask = detect(img)
    
        draw_result(img,detected,u,v,c)

        print("%-6s detected=%-5s u=%6.1f v=%6.1f area=%7.0f" % (name, detected, u, v, area))
        cv.imshow(f'{name}display',img)
        cv.waitKey(0)
        
        imwrite_zh(os.path.join(OUT_DIR, name + "_result.png"), img)
        imwrite_zh(os.path.join(OUT_DIR, name + "_mask.png"), mask)
        
        
if __name__=="__main__":
    main()
        
        
        
        
