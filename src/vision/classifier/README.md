# Classifier 

Two types of classifiers built into OpenCV

- HAAR
- LBP

## HAAR
Looks for features inside an image in different layers
- Top layer is looking at image as a whole and recurse using divide and conquer
- Bottom layers are usually very fine details 

Propose to use YoloV3 algorithm due to real time accuracy - we don't need 100% accuracy, but instead real time analysis. 
https://machinelearningmastery.com/how-to-perform-object-detection-with-yolov3-in-keras/