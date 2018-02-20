import math
import socket
from PIL import Image

import cv2
import numpy
import numpy as np
import requests
import yaml
import json
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
    xb, yb = goal['x'], goal['y']  # unidade das coordenadas eh cm

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

np.set_printoptions(precision=3, suppress=True, formatter={'float': '{: 0.3f}'.format})

camera_matrix = numpy.array(_dict.get('camera_matrix'))
dist_coeffs = numpy.array(_dict.get('dist_coeff'))
aruco_dict = aruco.Dictionary_get(aruco.DICT_4X4_1000)
markerLength = 3.4  # Here, our measurement unit is centimetre.
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

            if ids[i] == 11:
                ref_rot = numpy.zeros((3, 3))
                cv2.Rodrigues(rvec[i], ref_rot)
                ref_trans = tvec[i].transpose()
                # ref_coord = ref_rot.dot(origin_coord + ref_trans)
                imgWithAruco = aruco.drawAxis(imgWithAruco, camera_matrix, dist_coeffs, rvec[i], tvec[i], 1)

            elif ids[i] == 12:
                robot_rot = numpy.zeros((3, 3))
                cv2.Rodrigues(rvec[i], robot_rot)
                robot_trans = tvec[i].transpose()
                # robot_coord = robot_rot.dot(origin_coord + robot_trans)
                imgWithAruco = aruco.drawAxis(imgWithAruco, camera_matrix, dist_coeffs, rvec[i], tvec[i], 1)

            elif ids[i] == 13:
                dst_rot = numpy.zeros((3, 3))
                cv2.Rodrigues(rvec[i], dst_rot)
                dst_trans = tvec[i].transpose()
                # dst_coord = dst_rot.dot(origin_coord + dst_trans)
                imgWithAruco = aruco.drawAxis(imgWithAruco, camera_matrix, dist_coeffs, rvec[i], tvec[i], 1)

        if ref_rot is not None and dst_rot is not None and robot_rot is not None:
            # coordenada da camera em relacao ao marcador de origem
            refmarker_ref_cam = ref_rot.dot(origin_coord + ref_trans)
            cam_ref_refmarker = ((ref_rot.transpose()).dot(refmarker_ref_cam)) - ref_trans
            print("  ref", 11, cam_ref_refmarker.transpose())

            robot_ref_cam = robot_rot.dot(origin_coord + robot_trans)
            robot_ref_refmarker = ((ref_rot.transpose()).dot(robot_ref_cam)) - ref_trans
            print("robot", 12, robot_ref_refmarker.transpose())

            dst_ref_cam = dst_rot.dot(origin_coord + dst_trans)
            dst_ref_refmarker = ((ref_rot.transpose()).dot(dst_ref_cam)) - ref_trans
            print("dstin", 13, dst_ref_refmarker.transpose())

            robot = dict()
            robot['x'] = robot_ref_refmarker[0]
            robot['y'] = robot_ref_refmarker[1]
            robot['theta'] = 0

            goal = dict()
            goal['x'] = dst_ref_refmarker.item(0,0)
            goal['y'] = dst_ref_refmarker.item(1,0)

            vl, vr, ret = feedback_control(robot, goal)

            MESSAGE = {'vl': vl, 'vr': vr}
            sock = socket.socket(socket.AF_INET, socket.SOCK_DGRAM)
            sock.sendto(json.dumps(MESSAGE), (UDP_IP, UDP_PORT))

            print MESSAGE
            print "++++++++++++++++++++++++++++++++"


    else:
        # if aruco marker is NOT detectede
        imgWithAruco = gray  # assign imRemapped_color to imgWithAruco directly

    cv2.imshow("Aruco", imgWithAruco)  # display

    if cv2.waitKey(2) & 0xFF == ord('q'):  # if 'q' is pressed, quit.
        break
