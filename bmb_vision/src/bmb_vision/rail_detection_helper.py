import rospy
from bmb_msgs.msg import RailDetection
from sensor_msgs.msg import Image
import numpy as np
import cv2

class RailDetectionHelper:
    def __init__(self):
        self.raw_image_sub = rospy.Subscriber("/rectified_image", Image, self.image_callback)
        self.rail_detection_pub = rospy.Publisher("/rail_detection", RailDetection, queue_size=1)
    
    def Distanceperp(x1, x3, height, m1, b1, m2, b2):
        if m1 == 0 and m2 == 0:
            distanceperp = abs(x3 - x1)
        else:
            if m1 != 0:
                xone = 0 / m1 + b1
                xtwo = 200 / m1 + b1
                angle = np.arctan(height / (abs(xone - xtwo)))
                x1 = 50 / m1 + b1
            else:
                x1 = x1
            if m2 != 0:
                xthree = 0 / m2 + b2
                xfour = 200 / m2 + b2
                angle = np.arctan(height / (abs(xthree - xfour)))
                x2 = 50 / m2 + b2
            else:
                x2 = x3
            distanceperp = abs(x2 - x1) * np.sin(angle)
        return distanceperp


    def equationoflineVector(x1, y1, x2, y2):
        if (x2 - x1) == 0:
            m = 0
            b = 0
        else:
            m = (y2 - y1) / (x2 - x1)
            b = x2 - y2 / m
        return m, b
        # x=t
        # y=(m(t-b))


    def ccw(A, B, C):
        return (C[1] - A[1]) * (B[0] - A[0]) > (B[1] - A[1]) * (C[0] - A[0])


    # Return true if line segments AB and CD intersect
    def intersect(A, B, C, D):
        return ccw(A, C, D) != ccw(B, C, D) and ccw(A, B, C) != ccw(A, B, D)

    def gauge_measurement(m1, b1, m2, b2):
        y = 0
        gauge = 0
        while y < 100:
            x1 = y / m1 + b1
            x2 = y / m2 + b2
            gauge += x2 - x1
            y += 1
        meangauge = abs(gauge / 100)
        print(f'Gauge:{meangauge}')


    def railoffsetimagecrop(img, x0i, x0f, x1i, x1f):
        xin = round(min(x0i, x1i, x0f, x1f)) - 50
        xout = round(min(x0i, x1i, x0f, x1f)) + 65
        xin2 = round(max(x0i, x1i, x0f, x1f)) - 50
        xout2 = round(max(x0i, x1i, x0f, x1f)) + 65
        crop_img = img[300:500, xin:xout]  # leftrail
        crop_img2 = img[300:500, xin2:xout2]  # rightrail
        crop_img3 = cv2.cvtColor(crop_img2, cv2.COLOR_BGR2GRAY)
        crop_img1 = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
        xone = 0
        xtwo = 0
        mone = 0
        bone = 0
        x_leftinside=0
        b_leftinside=0
        m_leftinside=0
        left_rail_head_width=0
        ret, thresh = cv2.threshold(crop_img1, 50, 255, cv2.THRESH_BINARY_INV)
        ret, thresh2 = cv2.threshold(crop_img3, 80, 255, cv2.THRESH_BINARY_INV)
        thresh1 = cv2.Canny(thresh, 1500, 200, None, 3)
        thresh3 = cv2.Canny(thresh2, 200, 200, None, 3)
        lineshead = cv2.HoughLines(thresh1, 1, np.pi / 180, 50)
        lineshead2 = cv2.HoughLines(thresh3, 1, np.pi / 180, 60)

        for n2 in range(0, len(lineshead2)):
            for rho, theta in lineshead2[n2]:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
                pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                if n2 == 0:
                    pt1og = pt1
                    pt2og = pt2
                    theta1 = theta
                    rhoog = rho
                    xone = x1
                    xtwo = x2
                    mone, bone = equationoflineVector(x1, y1, x2, y2)
                    mright1, bright1 = equationoflineVector(x1 + xin2, y1, x2 + xin2, y2)
                else:
                    if n2==0:
                         continue
                    if rho < 0:
                        rho *= -1
                    closeness_rho = np.isclose(rho, rhoog, atol=1)

                    if closeness_rho==False and np.allclose(abs(theta),abs(theta1),rtol=1)==True and \
                            intersect(pt1og, pt2og, pt1, pt2) == False:
                        success=True
                        mtwo, btwo = equationoflineVector(x1, y1, x2, y2)
                        mright2, bright2 = equationoflineVector(x1 + xin2, y1, x2 + xin2, y2)
                        xthree = x1
                        xfour = x2
                        right_rail_head_width = Distanceperp(xone, xthree, 200, mone, bone, mtwo, btwo)
                        rightminvalue = min(xone, xtwo, xthree, xfour)
                        if rightminvalue == xone or rightminvalue == xtwo:
                            m_rightinside = mright1
                            b_rightinside = bright1
                            x_rightinside = xone
                        else:
                            m_rightinside = mright2
                            b_rightinside = bright2
                            x_rightinside = xthree

        for n1 in range(0, len(lineshead)):
            for rho, theta in lineshead[n1]:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
                pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                if n1 == 0:
                    pt1og = pt1
                    pt2og = pt2
                    theta1 = theta
                    rhoog=rho
                    xone = x1
                    xtwo = x2
                    mone, bone = equationoflineVector(x1, y1, x2, y2)
                    mleft1, bleft1 = equationoflineVector(x1 + xin, y1, x2 + xin, y2)
                    success=False
                else:
                    if rho < 0:
                        rho *= -1
                    closeness_rho = np.isclose(rho, rhoog, atol=3)
                    if closeness_rho==False and np.allclose(abs(theta), abs(theta1), rtol=1) == True and intersect(pt1og, pt2og,
                                                                                                             pt1,
                                                                                                             pt2) == False:
                        success=True
                        mtwo, btwo = equationoflineVector(x1, y1, x2, y2)
                        mleft2, bleft2 = equationoflineVector(x1 + xin, y1, x2 + xin, y2)
                        xthree = x1
                        xfour = x2
                        left_rail_head_width = Distanceperp(xone, xthree, 200, mone, bone, mtwo, btwo)
                        leftmaxvalue = max(xone, xtwo, xthree, xfour)
                        if leftmaxvalue == xone or leftmaxvalue == xtwo:
                            m_leftinside = mleft1
                            b_leftinside = bleft1
                            x_leftinside = xone
                        else:
                            m_leftinside = mleft2
                            b_leftinside = bleft2
                            x_leftinside = xthree
        meangauge = Distanceperp(x_leftinside + xin, x_rightinside + xin2, 200, m_leftinside, b_leftinside, m_rightinside,
                                     b_rightinside)
        if right_rail_head_width<3 or left_rail_head_width<3:
            success=False
        return crop_img, crop_img2, right_rail_head_width, left_rail_head_width, meangauge,success


    def linedetection(lines, img3, img, linekeep, x, crop_img, crop_img2, rightrailwidth, leftrailwidth, meangauge,success):
        n2 = 0
        crop_img3=np.copy(crop_img)
        crop_img4=np.copy(crop_img2)
        rightrailwidth1=rightrailwidth
        leftrailwidth1=leftrailwidth
        meangauge1=meangauge
        for n1 in range(0, len(lines)):
            for rho, theta in lines[n1]:
                a = np.cos(theta)
                b = np.sin(theta)
                x0 = a * rho
                y0 = b * rho
                pt1 = (int(x0 + 1000 * (-b)), int(y0 + 1000 * (a)))
                pt2 = (int(x0 - 1000 * (-b)), int(y0 - 1000 * (a)))
                x1 = int(x0 + 1000 * (-b))
                y1 = int(y0 + 1000 * (a))
                x2 = int(x0 - 1000 * (-b))
                y2 = int(y0 - 1000 * (a))
                if n2 == 0 and abs(linekeep - x1) < 200:
                    strong_lines[n2] = lines[n1]
                    linekeep = x1
                    m1, b1 = equationoflineVector(x1, y1, x2, y2)
                    if m1==0:
                        x0i=x1
                        x0f=x2
                    else:
                        x0i = 300 / m1 + b1
                        x0f = 500 / m1 + b1
                    n2 = n2 + 1
                    pt1og = pt1
                    pt2og = pt2
                    theta1 = theta
                else:
                    if n2 == 0:
                        continue
                    if rho < 0:
                        rho *= 1
                        # theta -= np.pi
                    closeness_rho = np.isclose(rho, strong_lines[0:n2, 0, 0], atol=50)
                    closeness = np.all([closeness_rho], axis=0)
                    if not any(closeness) and np.allclose(abs(theta), abs(theta1), rtol=0.000001) == True and intersect(
                            pt1og, pt2og, pt1, pt2) == False and abs(
                        x1 - x0i) < 150:  # takes away all lines that are in proximity and only keeps other lines with similar theta
                        strong_lines[n2] = lines[n1]
                        m2, b2 = equationoflineVector(x1, y1, x2, y2)
                        if m2 == 0:
                            x1i = x1
                            x1f = x2
                        else:
                            x1i = 300 / m2 + b2
                            x1f = 500 / m2 + b2
                        x1i = x1
                        x1f = x2
                        n2 = n2 + 1
                        
        if success == False or x % 61 == 0:
            crop_img3, crop_img4, rightrailwidth1, leftrailwidth1, meangauge1,success = railoffsetimagecrop(img, x0i, x0f, x1i, x1f)
        if success==True:
            crop_img=crop_img3
            crop_img2=crop_img4
            rightrailwidth=rightrailwidth1
            leftrailwidth=leftrailwidth1
            meangauge=meangauge1
        thetafinal=(theta1+theta)/2
        ymid=(m1*50-b1)+(m2*50-b2)
        xmid=50
        return crop_img, crop_img2, rightrailwidth, leftrailwidth, meangauge,success, xmid,ymid,thetafinal


    def image_callback(self, image):
        # TODO: perform rail detection
        rail_detection = RailDetection()
        x = 0
        linekeep = 500
        crop_img = np.zeros((200, 45, 3), np.uint8)
        crop_img2 = np.zeros((200, 45, 3), np.uint8)
        rightrailheadwidth = 0
        leftrailheadwidth = 0
        meangauge = 0
        success=False
        img = cv2.imread(image)
        img3 = np.copy(img)
        ret, img2 = cv2.threshold(img, 75, 255, cv2.THRESH_BINARY_INV)
        dst = cv2.Canny(img2, 50, 200, None, 3)
        cdst = cv2.cvtColor(dst, cv2.COLOR_GRAY2BGR)
        cdstP = np.copy(cdst)

        lines = cv2.HoughLines(dst, 1, np.pi / 180, 100, None, 500, 0)
        strong_lines = np.zeros([50, 1, 2])
        if int(0 if lines is None else 1) == 0:
            found=False

        else:
            crop_img, crop_img2, rightrailheadwidth, leftrailheadwidth, meangauge,success,xmid,ymid,thetafinal = linedetection(lines, img3, img,
                                                                                                  linekeep, x, crop_img,
                                                                                                  crop_img2,
                                                                                                  rightrailheadwidth,
                                                                                                  leftrailheadwidth,
                                                                                                  meangauge,success)
        rail_detection=(thetafinal,meangauge,xmid,ymid,success)
        self.rail_detection_pub.publish(rail_detection)

    def spin(self):
        rospy.spin()
