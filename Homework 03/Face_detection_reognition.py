import face_recognition
from PIL import Image, ImageDraw
import cv2
import numpy as np
import dlib
from imutils import face_utils
import depthai as dai
import time
import os.path

pipeline = dai.Pipeline()

# Define source and output
camRgb = pipeline.create(dai.node.ColorCamera)
xoutVideo = pipeline.create(dai.node.XLinkOut)

xoutVideo.setStreamName("video")

# Propertiesq

xoutVideo.input.setBlocking(False)
xoutVideo.input.setQueueSize(1)

# Linking
camRgb.video.link(xoutVideo.input)
camRgb.setVideoSize(800, 800)
count = 0

detector = dlib.get_frontal_face_detector()
predictor = dlib.shape_predictor("shape_predictor_68_face_landmarks.dat")

# Load a sample picture and learn how to recognize it.
dul_image = face_recognition.load_image_file("Dulmini.jpg")
dul_face_encoding = face_recognition.face_encodings(dul_image)[0]

known_face_encodings = [
    dul_face_encoding,
]
known_face_names = [
    "Dulmini Diss",
]

face_locations = []
face_encodings = []
face_names = []
process_this_frame = True

mouth_array = []

with dai.Device(pipeline) as device:

    video = device.getOutputQueue(name="video", maxSize=1, blocking=False)
    print('Usb speed: ', device.getUsbSpeed().name)

    while True:
        videoIn = video.get()
        frame = videoIn.getCvFrame()

        small_frame = cv2.resize(frame, (0, 0), fx=0.25, fy=0.25)

        rgb_small_frame = small_frame[:, :, ::-1]

        if process_this_frame:

            face_locations = face_recognition.face_locations(rgb_small_frame)
            face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)
            face_landmarks_list = face_recognition.face_landmarks(frame)

            face_names = []
            for face_encoding in face_encodings:

                matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
                name = "Unknown"

                face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
                best_match_index = np.argmin(face_distances)
                if matches[best_match_index]:
                    name = known_face_names[best_match_index]

                face_names.append(name)

        process_this_frame = not process_this_frame

        for (top, right, bottom, left), name in zip(face_locations, face_names):

            top *= 4
            right *= 4
            bottom *= 4
            left *= 4

            cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

            cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
            font = cv2.FONT_HERSHEY_DUPLEX
            cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)

            if name == 'Dulmini Diss':
                for face_landmarks in face_landmarks_list:
                    mouth_array.append(face_landmarks['top_lip'][3])

                for i in mouth_array:
                    cv2.circle(img=frame, center=(i[0], i[1] + 8), radius=1, color=(0, 255, 0), thickness=10)

        cv2.imshow('Video', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):
            break

video_capture.release()
cv2.destroyAllWindows()