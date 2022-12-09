#!/usr/bin/env python

import rospy
import cv2
import face_recognition
import numpy as np
from sensor_msgs.msg import Image
from cv_bridge import CvBridge, CvBridgeError
from pyzbar.pyzbar import decode

class camera_1:

  def __init__(self):
    self.image_sub = rospy.Subscriber("/camera/color/image_raw", Image, self.callback)
    self.process_this_image = True

  def callback(self,data):
    bridge = CvBridge()

    try:
      cv_image = bridge.imgmsg_to_cv2(data, "bgr8")
    except CvBridgeError as e:
      rospy.logerr(e)

    (rows,cols,channels) = cv_image.shape
    
    image = cv_image

    resized_image = image # cv2.resize(image, (640, 640)) 
    
    if self.process_this_image:  
      do_face_recognition(resized_image)

    self.process_this_image = not self.process_this_image

    gray = cv2.cvtColor(resized_image, cv2.COLOR_BGR2GRAY)
    thresh = 40
    img_bw = cv2.threshold(gray, thresh, 255, cv2.THRESH_BINARY)[1]

    #cv2.imshow("B&W Image", gray)
    #cv2.imshow("B&W Image /w threshold", img_bw)

    qr_result = decode(img_bw)

    #print (qr_result)
    if qr_result:
      qr_data = qr_result[0].data
      print(qr_data)

      (x, y, w, h) = qr_result[0].rect

      cv2.rectangle(resized_image, (x, y), (x + w, y + h), (0, 0, 255), 4)

      text = "{}".format(qr_data)
      cv2.putText(resized_image, text, (x, y - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (0, 0, 255), 2)
    
    
    
    

    # cv2.imshow("Camera output", resized_image)
    cv2.imshow("Face recog output", resized_image)


    cv2.waitKey(5)


import os
import glob
script_path = os.path.dirname(os.path.realpath(__file__))

image_files = glob.glob(script_path + '/brivo-people/*.jpg')

# Create arrays of known face encodings and their names
known_face_encodings = [
    face_recognition.face_encodings(face_recognition.load_image_file(image_file))[0] for image_file in image_files
]
known_face_names = [x.split('/')[-1].split('.')[0].replace('-', ' ') for x in image_files]

print(known_face_names)
# Initialize some variables
face_locations = []
face_encodings = []
face_names = []

def do_face_recognition(frame):
  # Resize frame of video to 1/4 size for faster face recognition processing
  small_frame = cv2.resize(frame, (0, 0), fx=0.5, fy=0.5)

  # Convert the image from BGR color (which OpenCV uses) to RGB color (which face_recognition uses)
  rgb_small_frame = small_frame[:, :, ::-1]
  
  # Find all the faces and face encodings in the current frame of video
  face_locations = face_recognition.face_locations(rgb_small_frame)
  face_encodings = face_recognition.face_encodings(rgb_small_frame, face_locations)

  face_names = []
  for face_encoding in face_encodings:
      # See if the face is a match for the known face(s)
      matches = face_recognition.compare_faces(known_face_encodings, face_encoding)
      name = "Unknown"

      # # If a match was found in known_face_encodings, just use the first one.
      # if True in matches:
      #     first_match_index = matches.index(True)
      #     name = known_face_names[first_match_index]

      # Or instead, use the known face with the smallest distance to the new face
      face_distances = face_recognition.face_distance(known_face_encodings, face_encoding)
      best_match_index = np.argmin(face_distances)
      if matches[best_match_index]:
          name = known_face_names[best_match_index]

      face_names.append(name)

  # Display the results
  for (top, right, bottom, left), name in zip(face_locations, face_names):
      # Scale back up face locations since the frame we detected in was scaled to 1/4 size
      top *= 2
      right *= 2
      bottom *= 2
      left *= 2

      # Draw a box around the face
      cv2.rectangle(frame, (left, top), (right, bottom), (0, 0, 255), 2)

      # Draw a label with a name below the face
      cv2.rectangle(frame, (left, bottom - 35), (right, bottom), (0, 0, 255), cv2.FILLED)
      font = cv2.FONT_HERSHEY_DUPLEX
      cv2.putText(frame, name, (left + 6, bottom - 6), font, 1.0, (255, 255, 255), 1)


def main():
	camera_1()
	
	try:
		rospy.spin()
	except KeyboardInterrupt:
		rospy.loginfo("Shutting down")
	
	cv2.destroyAllWindows()

if __name__ == '__main__':
    rospy.init_node('camera_read', anonymous=False)
    main()
