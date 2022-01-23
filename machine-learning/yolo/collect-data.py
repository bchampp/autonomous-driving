import pyrealsense2 as rs
import numpy as np
import cv2

classname = 'slow'

pipeline = rs.pipeline()
config = rs.config()

config.enable_stream(rs.stream.color, 1280, 720, rs.format.bgr8, 30)

profile = pipeline.start(config)


align_to = rs.stream.color
align = rs.align(align_to)

count = 20
for i in range(count):
    frames = pipeline.wait_for_frames()

    aligned_frames = align.process(frames)
    color_frame = aligned_frames.get_color_frame()

    color_image = np.asanyarray(color_frame.get_data())

    # Saving the image 
    cv2.imwrite(f'{classname}-{i}.jpg', color_image)


pipeline.stop()