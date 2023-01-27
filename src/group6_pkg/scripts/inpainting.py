import cv2
import numpy as np

brightness_factor = 20

while(True):
    cv_image = cv2.imread('/home/robis/image_23012023/cv_img_161.jpg')
    # mask = cv2.inRange(cv_image, (0,0,0), (0,0,0))
    # mask = cv2.rectangle(mask, (0, 250), (1280, 300), (255, 255, 255), -1)

    # dst = cv2.inpaint(cv_image, mask, 3, cv2.INPAINT_TELEA)
    # dst = cv2.add(cv_image, brightness_factor)

    # rgb_planes = cv2.split(cv_image)

    # result_planes = []
    # result_norm_planes = []
    # for plane in rgb_planes:
    #     dilated_img = cv2.dilate(plane, np.ones((7,7), np.uint8))
    #     bg_img = cv2.medianBlur(dilated_img, 21)
    #     diff_img = 255 - cv2.absdiff(plane, bg_img)
    #     norm_img = cv2.normalize(diff_img,None, alpha=0, beta=255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)
    #     result_planes.append(diff_img)
    #     result_norm_planes.append(norm_img)
        
    # result = cv2.merge(result_planes)
    # result_norm = cv2.merge(result_norm_planes)
    norm_img = cv2.normalize(cv_image,None, 100, 255, norm_type=cv2.NORM_MINMAX, dtype=cv2.CV_8UC1)

    # cv2.imwrite('shadows_out.png', result)
    # cv2.imwrite('shadows_out_norm.png', result_norm)

    cv2.imshow("Inpainting", norm_img)
    cv2.imshow("Original", cv_image)
    if cv2.waitKey(10) & 0xFF == ord('q'):
        break

cv2.destroyAllWindows()
