import cv2
import numpy as np
def zero_one():
    img = cv2.imread('C:/workspace/flask_server/gan_g/android_real_fake.png', cv2.IMREAD_GRAYSCALE)
    edge1 = cv2.Canny(img, 50, 200)
    #cv2.imshow('DounyGar', img)
    #cv2.imwrite('thounygar.jpg',edge1)
    img = np.zeros(edge1.shape, np.uint8)

    for i in range(edge1.shape[0]):
        for j in range(edge1.shape[1]):
            if edge1[i][j] == 255:
                edge1[i][j] = 1

    #cv2.imshow('result', img)
    #print(edge1.shape)
    #print(edge1.size)
    edge1 = edge1[100:124, 100:124]
    print(edge1)
    edge1=list(np.array(edge1).reshape(-1,))
    print(len(edge1))
    cv2.waitKey(0)

    return edge1

zero_one()