import cv2
import numpy as np
from ultralytics import YOLO
from shared_objects.ROS_utils import Topics, SHOW
import time, torch

device = torch.device('cpu')
topics=Topics()
topic_names=topics.topic_names
row, col, overlap = 5, 5, 0.3
img_size = (640, 360)
threshold = 5


# creating yolo model
model = YOLO("yolov8m.pt")

def slice_frame(frame, rows, cols, overlap):
    """ takes the data for the slicing from the global variables 
    and cuts in that number of images the arleady cut(ted?) image"""
    height, width, _ = frame.shape
    step_height = height // rows
    step_width = width // cols

    # Calculate the overlap in pixels
    overlap_height = int(step_height * overlap)
    overlap_width = int(step_width * overlap)

    return (frame[max(0, step_height*i - overlap_height):min(height, step_height*(i+1) + overlap_height),
                  max(0, step_width*j - overlap_width):min(width, step_width*(j+1) + overlap_width)]
            for i in range(rows) for j in range(cols))

def analysis(frame):
    # Slice the frame into 12 pieces (3 rows x 4 columns)
    sliced_frames = list(slice_frame(frame, row, col, overlap))
    start=time.time()
    # we give our results to yolo
    results = model.predict(sliced_frames, save=True, imgsz=320,conf=0.5)#, show_labels=True, show_conf=True, show_boxes=True)
    print(f"result took {time.time()-start}")
    # Calculate the width and height of each slice
    slice_height, slice_width = frame.shape[0] // row, frame.shape[1] // col

    if results is not None:
        for num, result in enumerate(results):
            colors = np.random.randint(0, 255, size=(len(result.boxes.conf), 3), dtype=np.uint8)
            for i in range(len(result.boxes.conf)):
                xy = result.boxes.xyxy[i]
                xy_np = xy.cpu().numpy()
                x1, y1, x2, y2 = map(int, xy_np)
                confidence = result.boxes.conf[i]
                label = result.names[int(result.boxes.cls[i])]
                if label in ["stop sign"]:
                    if SHOW:
                        # Calculate the offset based on the slice's position
                        offset_x, offset_y = (num % row) * slice_width, (num // col) * slice_height

                        # Adjust the bounding box coordinates
                        x1 += offset_x
                        y1 += offset_y
                        x2 += offset_x
                        y2 += offset_y

                        # Set the position for the label text
                        label_position = (x1, y1 - 10) # a little bit above

                        # Set the font, font scale, and thickness
                        font, font_scale, thickness = cv2.FONT_HERSHEY_SIMPLEX, 1, 3

                        # Use a different color for each box
                        color = tuple(map(int, colors[i]))

                        # Draw the bounding box
                        frame = cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
                        label_text = f"{label}: {confidence:.2f}"
                        print(f"Labels: {label_position}\nBoxes conf: {results.boxes.conf}")
                        # Put the label text on the image
                        frame = cv2.putText(frame, label_text, label_position, font, font_scale, color, thickness)
                        # Display the image with bounding boxes and labels
                        cv2.imshow('All for one, one for all', frame)

                        if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
                            break
                    return (True, frame)
    return (False, frame)