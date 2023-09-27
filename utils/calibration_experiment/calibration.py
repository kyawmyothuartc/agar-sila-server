import numpy as np
import cv2

# image = cv2.imread("calibration.bmp")

# print(image.shape)

#checkerboard detection here

# point2dList = np.array([[4619, 3332], [4627, 479], [343, 479]])
# point3dlist = np.array([[741.72, 19.67, 3.29], [662.27, 19.88, 2.18], [661.36, - 99.92, 2.17]])


# def locate_chessboard_corners(gray, chessboard_rows, chessboard_cols, criteria):
#     # Find the chess board corners.
#     ret, corners = cv2.findChessboardCorners(
#         gray, (chessboard_rows, chessboard_cols))
#     if ret == True:
#         corners = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
#     return ret, corners

# def detect_checkerboard(image, chessboard_rows, chessboard_cols, criteria):
#     calib_image = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
#     calib_image[calib_image > 60] = 255
#     cv2.imwrite("calib_image.png", calib_image)

#     print(calib_image.shape)
#     # Detect chessboard corner points.
#     result, detected_corners = locate_chessboard_corners(
#         calib_image, chessboard_rows, chessboard_cols, criteria)
#     print(result)
#     if result:
#         corners2 = cv2.cornerSubPix(
#             calib_image, detected_corners, (11, 11), (-1, -1), criteria)
#         corners2 = corners2[::-1, :, :]

#         image_checkerboard_vis = cv2.drawChessboardCorners(
#             image.copy(), (chessboard_rows, chessboard_cols), corners2, result)
#         return corners2, image_checkerboard_vis


# chessboard_rows = 5
# chessboard_cols = 3
# criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 35, 0.001)
# image = cv2.imread("calibration_experiment\calibration.bmp")
# image = cv2.resize(image, (0, 0), fx=0.2, fy=0.2)

# print(image.shape)
# corners, image_vis = detect_checkerboard(image, chessboard_rows, chessboard_cols, criteria)
# cv2.imwrite("detected_board.png", image_vis)

point3d_dict = {}
point2d_dict = {}

with open("calibration_experiment/robot_points.txt") as file:
    for line in file:
        data = line.split(": ")
        point3d_dict[int(data[0].replace("p", "")) - 1] = [ float(val) for val in data[1].strip().replace("\n","").split(" ")]

with open("calibration_experiment/image_points.txt") as file:
    for line in file:
        data = line.split(": ")
        point2d_dict[int(data[0].replace("p", "")) - 1] = [ float(val) for val in data[1].strip().replace("\n","").split(" ")]

print(point3d_dict)
print(point2d_dict)

xlist = []

for n in range(3):
    A = []
    b = []
    for i in range(len(point2d_dict.keys())):
        A.append([point2d_dict[i][0], point2d_dict[i][1], 1])
        b.append(point3d_dict[i][n])
    A = np.array(A)
    b = np.array(b)

    x, residual, _, _ = np.linalg.lstsq(A, b, rcond=None)
    print(x)
    xlist.append(x)

xlist = np.array(xlist)
np.save("coeff.npy", xlist)
print(xlist)

# point2dList = np.array([[645, 615], [810,3060] ,[4535, 3110]])
# point3dlist = np.array([[667.05, -94.65, -3.50], [737.78, -90.56, -2.22], [739.29, 14.65, -1.39]])

# point2dList_testset = np.array([[4630,710],[2360,1900]])
# point3dList_testset = np.array([[671.7, 16.99, -2.82], [703.25, -47.63, -2.91]])

# xlist = []

# for n in range(3):
#     # solving linear system here
#     A = np.array([[point2dList[0,0], point2dList[0,1], 1],
#                 [point2dList[1,0], point2dList[1,1], 1],
#                 [point2dList[2,0], point2dList[2,1], 1]])
#     b = np.array([point3dlist[0,n], point3dlist[1,n], point3dlist[2,n]])


#     x = np.linalg.solve(A, b)
#     xlist.append(x)
#     # print(x)

#     for i in range(point2dList_testset.shape[0]):
#         test_x_3d = (point2dList_testset[i,0] * x[0]) + (point2dList_testset[i,1] * x[1]) + x[2]
#         print(point3dList_testset[i,n], test_x_3d, point3dList_testset[i,n] - test_x_3d)

#     print(n, "-----------")

# xlist = np.array(xlist)
# np.save("coeff.npy", xlist)
# print(xlist)