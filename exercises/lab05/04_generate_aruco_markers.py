#!/usr/bin/env python
#
# Copyright (c) CTU -- All Rights Reserved
# Created on: 2023-11-9
#     Author: Vladimir Petrik <vladimir.petrik@cvut.cz>
#

import cv2
from fpdf import FPDF

pdf = FPDF()

w = 210
h = 297
ms = 50
aruco_size = 40

nw = w // ms
nh = h // ms

margin_w = (w - nw * ms) / 2
margin_h = (h - nh * ms) / 2


def add_page_with_grid():
    pdf.add_page()
    for i in range(nw + 1):
        pdf.line(margin_w + i * ms, margin_h, margin_w + i * ms, h - margin_h)
    for j in range(nh + 1):
        pdf.line(margin_w, margin_h + j * ms, w - margin_w, margin_h + j * ms)


def get_center(i, j):
    return margin_w + i * ms + ms / 2, margin_h + j * ms + ms / 2


aruco_dict = cv2.aruco.getPredefinedDictionary(cv2.aruco.DICT_6X6_50)
for i in range(50):
    img = aruco_dict.generateImageMarker(i, sidePixels=100)
    cv2.imwrite(f"/tmp/aruco_{i}.png", img)

for _ in range(3):
    add_page_with_grid()
    for i in range(nw):
        for j in range(nh):
            xc, yc = get_center(i, j)
            x = xc - aruco_size / 2
            y = yc - aruco_size / 2
            pdf.image(f"/tmp/aruco_{i}.png", x, y, aruco_size, aruco_size)

add_page_with_grid()
for i in range(nw):
    for j in range(nh):
        xc, yc = get_center(i, j)
        x = xc - aruco_size / 2
        y = yc - aruco_size / 2
        ind = i * nh + j + nw
        print(ind)
        pdf.image(f"/tmp/aruco_{ind}.png", x, y, aruco_size, aruco_size)

pdf.output("markers.pdf")
