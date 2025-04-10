from ultralytics import YOLO
import cv2
import numpy as np

# Load a pretrained model
model = YOLO("yolov8n.pt")

#model=YOLO("/home/alemomo/AUG_SEM/src/test_files/best.pt")

# Initialize video capture from the first camera device
#video = cv2.VideoCapture(0)
video = cv2.VideoCapture(4)

# Check if video is opened successfully
if video.isOpened():
    while True:
        ret, frame = video.read()  # read video frame

        if not ret:
            print("Failed to grab frame")
            break

        # Perform inference using the predict function
        results = model.predict(frame, save=False, imgsz=320, conf=0.5, show_labels= True, show_conf= True, boxes=True)[0]

        # Define a list of colors
        colors = np.random.randint(0, 255, size=(len(results.boxes.conf), 3), dtype=np.uint8)

        # Draw bounding boxes and labels with confidence on the image
        print(results.boxes.conf)
        for i in range(len(results.boxes.conf)):
            xy = results.boxes.xyxy[i]
            xy_np = xy.cpu().numpy()
            x1, y1, x2, y2 = map(int, xy_np)
            confidence = results.boxes.conf[i]
            label = results.names[int(results.boxes.cls[i])]
            # Set the position for the label text
            label_position = (x1, y1 - 10) # a little bit above
            
            # Set the font, font scale, and thickness
            font, font_scale, thickness = cv2.FONT_HERSHEY_SIMPLEX, 1, 3
            
            # Use a different color for each box
            color = tuple(map(int, colors[i]))
            
            # Draw the bounding box
            frame = cv2.rectangle(frame, (x1, y1), (x2, y2), color, thickness)
            
            # Put the label text on the image
            label_text = f"{label}: {confidence:.2f}"
            print(f"Probs: {results.names}\nLabels: {label_position}Boxes conf:{results.boxes.conf}")
            frame = cv2.putText(frame, label_text, label_position, font, font_scale, color, thickness)
        # Display the image with bounding boxes and labels
        cv2.imshow('All for one, one for all', frame)

        if cv2.waitKey(1) & 0xFF == ord('q'):  # Press 'q' to quit
            break

video.release()
cv2.destroyAllWindows()