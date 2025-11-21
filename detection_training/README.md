# Detection training
This folder contains the scripts and the data used for the holes and flange detection training.

## Intro

### DATASET GENERATION: 
The first step is the collection of a dataset of flange images. Since the detection will always take place in a stable and controlled environment, there are no particular requirements for the dataset. If a new flange should be detected, a 30 images dataset from varying perspectives should be more than sufficient. I already labeled more than 200 flange images, so my dataset should be used as "starting dataset", and the new images should be added to my dataset, in order to enrich the dataset every time that a flange is added.

### MANUAL LABELING: 
The second step is the manual annotation of the dataset. To "label" an image, it means to create a file associated to an image, where it’s specified where the bounding box Each image must be labeled with bounding boxes around flange and hole, saved in YOLO format (.txt file per image). There are many tools on the internet; the one that I used is also provided by Roboflow. The format is: class_id x_center y_center width height (all values normalized to [0, 1]).

### DATASET AUGMENTATION: 
The third step is the augmentation of the dataset. Image augmentation is the process of applying transformations to the original images in order to increase the variability of the dataset without collecting new data. This helps the model become more robust to real-world variations and improves generalization. Here is a list of some augmentation techniques that I used:

• Rotations and flips (horizontal/vertical)

• Translations and zooming

• Adjustments of brightness, contrast, or color

• Adding noise or blur

• Cropping and resizing

A practical way to perform augmentation is by using tools like Roboflow, which pro- vide built-in pipelines to automatically apply and manage these transformations.

### COMMENTS: 
Since Ultralytics is quickly updating the YOLO model, there might be many better model by the time someone will use again the program. Since it’s very easy to change the YOLO model, my advice is to do so. The same goes for the data augmentation and labeling tool: since in the future there might be better tool, I would advice to look up on the internet which one is the best at the moment.


## Project structure

- **bags/**  
  Contains `.bag` files recorded with Intel RealSense camera. Used as raw input for frame extraction.

- **config/**  
  Holds configuration files for YOLO training.  
  - `configTraining.yaml`: dataset and training parameters.

- **images/**  
  Contains images used for training and validation.  
  - `train/`: training set images.  
  - `val/`: validation set images.

- **labels/**  
  YOLO-format `.txt` annotation files corresponding to images.

- **original/**  
  Extracted raw frames from `.bag` files before any processing.

- **runs/**  
  Output directory for YOLO training, evaluation, and inference results (e.g., weights, logs, predictions).

- **scripts/**  
  Utility and training scripts:  
  - `bag_to_png.py`: convert RealSense `.bag` to PNG frames.  
  - `picture_check.py`: check for mismatches between images and labels.  
  - `play_images.py`: simple frame player with keyboard controls.  
  - `video_detection_recorder.py`: run YOLO inference on images, save/display detections.  
  - `yolo_training.py`: training script for YOLO model.  
  - `videoPlayer.py`: generic video player utility.

- **yolo11n.pt**  
  Pretrained YOLOv11 model weights (nano version).

## Tutorial

1. **Get inside an environment with Ultralytics installed**

For example

```bash
# create a new venv
python3 -m venv detection_training_venv

# activate it
source detection_training_venv/bin/activate

# update pip
pip install --upgrade pip

# install ultralytics
# at the moment, 8.3.189 is the last version
# consider that installing the last version may results in some compatibility issue in the future

pip install ultralytics==8.3.189 # or pip install ultralytics
```

2. **[IF NEEDED] Extract frames from `.bag`**
If the images has been recorded in a .bag file using realsense-viewer, use this script to convert to png
```bash
python detection_training/scripts/bag_to_png.py
```

3. **Check images <-> labels**
```bash
python detection_training/scripts/picture_check.py
```
If you get "All images and labels match ✅", you can continue.


4. **Train**
Tune the training parameters inside the script as needed
```bash
python detection_training/scripts/yolo_training.py
```


5. **Inference on a folder**
To play the detection / record a video, use the script:
```bash
python detection_training/scripts/video_detection_recorder.py
```
"""
