import cv2
import numpy as np
import matplotlib.pyplot as plt


img = cv2.imread('test.png', 0)  
equ = cv2.equalizeHist(img)
res = np.hstack((img, equ))

print(type(res), type(img))

plt.figure(figsize=(10, 5))
plt.imshow(res, cmap='gray')  
plt.title("Original vs Equalized Image")
plt.axis('off')  
plt.show()