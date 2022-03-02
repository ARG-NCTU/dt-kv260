在terminal中輸入以下指令測試OpenCV是否成功安裝
```
python -c "import cv2; print(cv2.__version__)"
```

# 匯入OpenCV函式庫
import cv2

# 設定從哪顆鏡頭讀取影像，在括弧中填入先前查詢到的webcam編號
webcam = cv2.VideoCapture(0)

# 讀取影像
return_value, image = webcam.read()

# 開啟視窗顯示影像
cv2.imshow('img_gray',img_gray)

# 儲存名為Picture.png的照片
cv2.imwrite("Picture.png", image)

# 刪除webcam，避免影像佔用資源
del(webcam)

# Try this if the way above don't work
https://www.jetsonhacks.com/2019/04/02/jetson-nano-raspberry-pi-camera/
