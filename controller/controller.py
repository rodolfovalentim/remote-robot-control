import math
import socket
from PIL import Image

import cv2
import numpy
import numpy as np
import requests
import yaml
from cv2 import aruco


def eulerAnglesToRotationMatrix(theta):
    cx = math.cos(theta[0])
    cy = math.cos(theta[1])
    cz = math.cos(theta[2])

    sx = math.sin(theta[0])
    sy = math.sin(theta[1])
    sz = math.sin(theta[2])

    # Calculate rotation about x axis - roll
    R_x = np.matrix([[1, 0, 0],
                     [0, cx, -sx],
                     [0, sx, cx]])

    # Calculate rotation about y axis - pitch
    R_y = np.matrix([[cy, 0, sy],
                     [0, 1, 0],
                     [-sy, 0, cy]])

    # Calculate rotation about z axis - yaw
    R_z = np.matrix([[cz, -sz, 0],
                     [sz, cz, 0],
                     [0, 0, 1]])

    # Combined rotation matrix
    return reduce(numpy.dot, [R_x, R_y, R_z])


def feedback_control(robot, goal):
    pd_x, pd_y, theta_jog = robot['x'], robot['y'], robot['theta']  # unidade das coordenadas eh cm, angulo em radianos
    xb, yb = goal[0], goal[1]  # unidade das coordenadas eh cm

    distance = math.sqrt((xb - pd_x) ** 2 + (yb - pd_y) ** 2)

    M_rot = np.array([[math.cos(theta_jog), math.sin(theta_jog)], [-math.sin(theta_jog), math.cos(theta_jog)]])
    M_trans = np.array([[pd_x], [pd_y]])

    oldcoords_goal = np.array([[xb], [yb]])
    newcoords_goal = M_rot.dot(oldcoords_goal - M_trans)

    theta_erro = math.atan2(newcoords_goal[1][0], newcoords_goal[0][0])

    # distancia das rodas em metros
    D = 0.23

    # tempo de amostragem
    T = 90

    # dado o sistema y = pseudoA*Matriz_erro obtem-se y que eh
    # a velocidade da roda direita e velocidade da roda esquerda

    A = np.array(
        [
            [math.cos(theta_jog) / 2.0, math.cos(theta_jog) / 2.0],
            [math.sin(theta_jog) / 2.0, math.sin(theta_jog) / 2.0],
            [1 / D, -1 / D]
        ])

    pseudoA = np.linalg.pinv(A)
    matrix_erro = np.array([[xb - pd_x], [yb - pd_y], [(-30) * theta_erro]])
    y = pseudoA.dot(matrix_erro + 0.01)

    v_max = max(abs(y[0][0]), abs(y[1][0]))  # pega a maior velocidade

    # como a velocidade foi parametrizada pela maior, K eh a maior velocidade que a roda pode assumir
    K = 90

    if v_max == 0:
        return 0, 0, False

    vl = y[0][0] * (K / v_max)
    vr = y[1][0] * (K / v_max)

    if distance < 0.1:
        return 0, 0, True

    if math.isnan(vl) or math.isnan(vr):
        return 0, 0, False

    return int(vl), int(vr), False


UDP_IP = "192.168.1.114"
UDP_PORT = 5005
MESSAGE = '{"vl": 0, "vr": 0}'

print("UDP target IP:", UDP_IP)
print("UDP target port:", UDP_PORT)

with open('calibration.yaml') as f:
    _dict = yaml.load(f)

camera_matrix = numpy.array(_dict.get('camera_matrix'))
dist_coeffs = numpy.array(_dict.get('dist_coeff'))
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
markerLength = 2.2  # Here, our measurement unit is centimetre.
arucoParams = aruco.DetectorParameters_create()

while True:
    r = requests.get('http://192.168.1.99:8080/?action=snapshot', stream=True)
    r.raise_for_status()
    r.raw.decode_content = True  # Required to decompress gzip/deflate compressed responses

    img = Image.open(r.raw)
    gray = cv2.cvtColor(numpy.array(img), cv2.COLOR_BGR2GRAY)

    corners, ids, rejectedImgPoints = aruco.detectMarkers(gray, aruco_dict, parameters=arucoParams)  # Detect aruco

    # if aruco marker detected
    if ids is not None:
        # posture estimation from a single marker
        rvec, tvec, _objPoints = aruco.estimatePoseSingleMarkers(corners, markerLength, camera_matrix, dist_coeffs)
        imgWithAruco = aruco.drawDetectedMarkers(gray, corners, ids, (0, 255, 0))

        ref_rot, ref_trans = None, None
        robot_rot, robot_trans = None, None
        dst_rot, dst_trans = None, None

        origin_coord = np.matrix([[0], [0], [0]])
        ref_coord = None
        robot_coord = None
        dst_coord = None

        for i in range(len(ids)):
            # axis length 100 can be changed according to your requirement
            imgWithAruco = aruco.drawAxis(imgWithAruco, camera_matrix, dist_coeffs, rvec[i], tvec[i], 1)

            if ids[i] == 8:
                ref_rot = numpy.zeros((3, 3))
                cv2.Rodrigues(rvec[i], ref_rot)
                ref_trans = tvec[i].transpose()
                ref_coord = ref_rot.dot(origin_coord + ref_trans)

            elif ids[i] == 13:
                robot_rot = numpy.zeros((3, 3))
                cv2.Rodrigues(rvec[i], robot_rot)
                robot_trans = tvec[i].transpose()
                robot_coord = robot_rot.dot(origin_coord + robot_trans)

            elif ids[i] == 5:
                dst_rot = numpy.zeros((3, 3))
                cv2.Rodrigues(rvec[i], dst_rot)
                dst_trans = tvec[i].transpose()
                dst_coord = dst_rot.dot(origin_coord + dst_trans)

        if ref_coord is not None:
            print("origin", str(ref_rot.transpose().dot(origin_coord - ref_trans)))
            # print("robot", ref_rot.dot(robot_coord + ref_trans))
            # print("dstin", ref_rot.dot(dst_coord + ref_trans))

        sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
        # print("message:", MESSAGE)
        sock.sendto(MESSAGE, (UDP_IP, UDP_PORT))

    else:
        # if aruco marker is NOT detectede
        imgWithAruco = gray  # assign imRemapped_color to imgWithAruco directly

    cv2.imshow("Aruco", imgWithAruco)  # display

    if cv2.waitKey(2) & 0xFF == ord('q'):  # if 'q' is pressed, quit.
        break
