
import cv2
import math
import time
import numpy as np
import shapely
from shapely.geometry import Point
from shapely.geometry import Polygon
# Function which returns subset or r length from n
from itertools import combinations

import warnings
warnings.filterwarnings("error")

def crop_rect(img, rect):
    # get the parameter of the small rectangle
    center = rect[0]
    size = rect[1]
    angle = rect[2]
    center, size = tuple(map(int, center)), tuple(map(int, size))

    # get row and col num in img
    height, width = img.shape[0], img.shape[1]

    M = cv2.getRotationMatrix2D(center, angle, 1)
    img_rot = cv2.warpAffine(img, M, (width, height), flags=cv2.INTER_NEAREST)
    img_crop = cv2.getRectSubPix(img_rot, size, center)
    return img_crop

def circleColonyDetect(image):

    start_time = time.time()
    # applying clahe on the image
    lab = cv2.cvtColor(image, cv2.COLOR_BGR2LAB)
    l_channel = lab[:, :, 0]
    a = lab[:, :, 1]
    b = lab[:, :, 2]
    # Applying CLAHE to L-channel
    clahe = cv2.createCLAHE(clipLimit=2.0, tileGridSize=(8, 8))
    cl = clahe.apply(l_channel)
    # merge the CLAHE enhanced L-channel with the a and b channel
    limg = cv2.merge((cl, a, b))
    # Converting image from LAB Color model to BGR color spcae
    enhanced_img = cv2.cvtColor(limg, cv2.COLOR_LAB2BGR)
    image = enhanced_img

    print("CLAHE time: {:.4f} seconds".format(time.time() - start_time))

    circles = circle_detection(image)
    downscaled_circles1 = circle_detection(
        image, scale=0.5, params=(100, 28, 10, 50))
    downscaled_circles2 = circle_detection(
        image, scale=1.5, params=(100, 30, 20, 50))
    final_circles = [*circles, *downscaled_circles1, *downscaled_circles2]
    print(len(final_circles))
    return final_circles

def circle_detection(image, scale=1.0, params=(100, 30, 25, 50)):
    image = cv2.resize(image, (0,0), fx=scale, fy=scale)
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
    gray = cv2.medianBlur(gray, 5)

    circles = cv2.HoughCircles(gray, cv2.HOUGH_GRADIENT, 1, 10,
                               param1=params[0], param2=params[1],
                               minRadius=params[2], maxRadius=params[3])
    if circles is not None:
        circles = circles[0]
        # return circles
        if scale != 1.:
            rescaled_circles = []
            for circle in circles:
                rescaled_circles.append([i * (1./scale) for i in circle])
            return rescaled_circles
        else:
            return circles
    else:
        return []

def circle_intersection_area(circle1, circle2):
    
    circle1 = np.array(circle1)
    circle2 = np.array(circle2)
    radius1 = circle1[2]
    radius2 = circle2[2]

    d = ((circle1[0] - circle2[0])**2 + (circle1[1] - circle2[1])**2)**0.5
    if d < radius1 or d < radius2:
        return 0, 100

    circlePoly1 = Point(circle1[0], circle1[1]).buffer(circle1[2])
    # circlePoly1 = Polygon(circlePoly1)
    circlePoly2 = Point(circle2[0], circle2[1]).buffer(circle2[2])

    intersection_1 = 0
    intersection_2 = 0
    if circlePoly1.intersects(circlePoly2):
        intersection_1 = circlePoly1.intersection(circlePoly2).area
        intersection_2 = circlePoly2.intersection(circlePoly1).area
    
    # union_area = circlePoly1.union(circlePoly2).area
    base_area = min(circlePoly1.area, circlePoly2.area)

    iou = min(intersection_1, intersection_2) / base_area * 100

    return max(intersection_1, intersection_2), iou

    intersection = 0
    iou = 0

    d = ((circle1[0] - circle2[0])**2 + (circle1[1] - circle2[1])**2)**0.5
    if (d < (radius1 + radius2)):
        if d < radius1 or d < radius2:
            return 0, 100
        if radius1 > radius2:
            x = (circle1[2]**2 - circle2[2]**2 + d**2) / (2*d)
        else:
            x = (circle2[2]**2 - circle1[2]**2 + d**2) / (2*d)
        if d < x:
            y = x - d
        else:
            y = d - x
        a = (circle1[2]**2 - x**2) ** 0.5
        b = (circle2[2]**2 - y**2) ** 0.5
        theta1 = np.arctan2(a, x)
        theta2 = np.arctan2(b, y)

        arcarea1 = (np.pi * circle1[2]**2) * ((2 * theta1) / (2 * np.pi))
        arcarea2 = (np.pi * circle2[2]**2) * ((2 * theta2) / (2 * np.pi))
        triarea1 = x * a
        triarea2 = y * b
        intersection = arcarea1 + arcarea2 - triarea1 - triarea2
        if circle1[2] > circle2[2]:
            smaller_area = (np.pi * circle2[2]**2)
        else:
            smaller_area = (np.pi * circle1[2]**2)

        iou = (intersection / smaller_area) * 100.

    return intersection, iou

def iou_filtering(filtered_circles, ref_circle, idx):
    if idx < len(filtered_circles):
        for circle in filtered_circles[idx:]:
            intersection, iou = circle_intersection_area(ref_circle, circle)
            if iou > 80:
                del filtered_circles[idx]
                filtered_circles = iou_filtering(filtered_circles, ref_circle, idx)
                return filtered_circles
            idx += 1
    return filtered_circles

def loop_iou_filtering(circles, final_circles=[]):
    if len(circles) > 0:
        ref_circle = circles[0]
        final_circles.append(ref_circle)
        filtered_circles = circles.copy()
        del filtered_circles[0]
        filtered_circles = iou_filtering(filtered_circles, ref_circle, idx=0)
        final_circles = loop_iou_filtering(filtered_circles, final_circles=final_circles)
    return final_circles


def remove_overlapping_box(image, candidates_list, num_data, counter, IOU_THRESHOLD=50):
    if len(candidates_list) > 0:
        reference = candidates_list[0]
        candidates_list = candidates_list[1:]

        reference_cropped_img = crop_texture(image, reference)
        reference_cropped_img = cv2.cvtColor(reference_cropped_img, cv2.COLOR_BGR2HSV)
        reference_hist = compute_colorHist(reference_cropped_img, None, excluding_v=False)

        idx_list = []
        overlapping_list = [(0, reference_hist, reference_cropped_img, reference)]
        for idx in range(len(candidates_list)):
            candidate = candidates_list[idx]

            intersection, iou = circle_intersection_area(reference, candidate)
            
            if iou > IOU_THRESHOLD:
                candidate_cropped_img = crop_texture(image, candidate)
                candidate_cropped_img = cv2.cvtColor(candidate_cropped_img, cv2.COLOR_BGR2HSV)
                candidate_hist = compute_colorHist(candidate_cropped_img, None, excluding_v=False)
                overlapping_list.append((idx, candidate_hist, candidate_cropped_img, candidate))
                
                idx_list.append(idx)
        if len(idx_list) > 0:
            idx_list = sorted(idx_list, reverse=True)
            for idx in idx_list:
                del candidates_list[idx]
            best_candiate = sorted(overlapping_list, key=lambda x: x[0])[0]
            candidates_list.append(best_candiate[3])
            num_data = len(candidates_list)
            counter = 0
        else:
            best_candiate = overlapping_list[0]
            candidates_list.append(best_candiate[6])
            counter = counter + 1

            if num_data*2 == counter:
                return candidates_list

        final_candidates = remove_overlapping_box(image, candidates_list, num_data, counter, IOU_THRESHOLD=IOU_THRESHOLD)
        return final_candidates
    else:
        return candidates_list

def jeffrey_divergence(hist1, hist2):
    hist1 = hist1 + 1e-20
    hist1 = hist1/hist1.sum()
    hist2 = hist2 + 1e-20
    hist2 = hist2/hist2.sum()
    kl_1_2 = (hist1 * np.log2(hist1 / hist2)).sum()
    kl_2_1 = (hist2 * np.log2(hist2 / hist1)).sum()
    return kl_1_2 + kl_2_1

def loop_color_similarity(image, circle_with_hist, reference, histogram_visualization=False):
    output_div_list = []
    data_with_div = []
    circle_ref, hist1 = reference
    r1 = circle_ref[2]
    patch_ref = crop_texture(image, circle_ref)
    for data in circle_with_hist:
        
        circle_data, hist2 = data
        r2 = circle_data[2]
        # patch_data = crop_texture(image, circle_data)

        # print(patch_ref.shape)
        # print(patch_data.shape)
        # cv2.imwrite("patch_ref.png", patch_ref)
        # cv2.imwrite("patch_data.png", patch_data)
        # exit()
        r_diff = abs(float(r1) - float(r2))
        j_div_avg = 0

        for i in range(3):
            j_div = jeffrey_divergence(hist1[i, :], hist2[i, :])
            j_div_avg += j_div
        j_div_avg /= 3
        output_div_list.append(j_div_avg)
        data_with_div.append((data[0], data[1], j_div_avg, r_diff))
    # print(sorted(output_div_list))
    # output_div_list = np.array(output_div_list)
    # import matplotlib.pyplot as plt
    # plt.hist(output_div_list, density=True, bins=50)
    # plt.savefig("histogram.png")
    # plt.show()
    if histogram_visualization:
        return data_with_div, output_div_list
    else:
        return data_with_div

def compute_colorHist(image, circles):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)
    circle_with_hist = []
    for circle in circles:
        circle = np.uint16(np.around(circle))
        mask = np.zeros(image.shape[:2]).astype(np.uint8)
        center = (circle[0], circle[1])
        radius = circle[2]
        cv2.circle(mask, center, radius, 255, -1)
        mask = np.expand_dims(mask, axis=2)

        color = ('h','s','v')
        hist = []
        for i,col in enumerate(color):
            hist_item = cv2.calcHist([image],[i],mask,[256],[0,255])
            hist_item = hist_item[:,0]
            hist.append(hist_item)
        circle_with_hist.append((circle, np.array(hist)))
    return circle_with_hist

def crop_texture(image, circle):
    center = (circle[0], circle[1])
    radius = circle[2]
    
    side = radius / (2**0.5)
    p1 = (math.floor(center[0] - (side/2)), math.floor(center[1] - (side/2)))
    p2 = (math.floor(center[0] + (side/2)), math.floor(center[1] + (side/2)))

    patch = image[p1[1]:p2[1], p1[0]:p2[0],...]
    return patch