## Tutorial

1. **Activate environment**

   Make sure you have an environment with the required dependencies:
   ```bash
   pip install ultralytics opencv-python numpy pillow pycocotools
   ```
   Depending on when the code will be used, some versions could be changed. 

2. **Get roi from images using a detection model**
   Inside roi_extraction_from_imgs folder you can add the desired flange images inside roi_extraction_from_imgs/imgs
   Once done that use roi_extraction_from_imgs/roi_extractor.py to get the roi. They're provided inside roi_extraction_from_imgs/roi_extracted
   Use roi_extraction_from_imgs/config.yaml to specify the name of the detection model (.pt) used, and place that inside roi_extraction_from_imgs

3. **Label the roi using roboflow**
   Copy and paste the extracted roi in a roboflow dataset. Then use the polygon tool to annotate the hole edge in the roi.

   ***Pay attention to be as precise as possible in this process***

   Once done that, export the labels in a "JSON - COCO segmentation" format.

4. **Get the masks from the JSON file**
   Take the JSON labels file and move that to json_to_mask/json_labels.
   Rename that as you wish and then configure the same name in the json_to_mask/config.yaml
   Then run json_to_mask/json_to_mask.py you will get the òabels inside json_to_mask/mask_labels

5. **Train**
   Copy and paste the masks inside train_dataset/masks
   Copy and paste the roi, downloaded from roboflow, inside train_dataset/roi

   ***Pay attention: corresponding roi and mask should have corresponding file name***

   Change the parameters inside train_config.yaml
   Run train.py, and once finished, you will find the results and the net inside runs/result_X 


