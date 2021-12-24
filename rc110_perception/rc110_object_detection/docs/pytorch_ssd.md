Go to jetson-inference/python/training/detection/ssd/data and create an empty directory to store the dataset and a text file to define the class labels.

Stop robocar movement with:  
$ cd ros/src/robocar110_ros/  
$ make stop

Open a new terminal and type the following command to start the camera.

$ camera-capture csi://0       # using default MIPI CSI camera  
$ camera-capture /dev/video0   # using V4L2 camera /dev/video0

When Data Capture Control is opened, change Dataset Type to Detection, and link Dataset Path and Class label to the ones you created above.  
![image](https://user-images.githubusercontent.com/90881576/147323346-61db06ac-86e5-41fe-b2b9-00529c07b91c.png)  
Make sure that the object is within the frame of the camera, and press the Freeze/Edit button (or the space key) to freeze the image and draw a bounding box on the image.  
![image](https://user-images.githubusercontent.com/90881576/147323686-b79b816c-26c3-4657-8a35-79ae7e2e7752.png)  
Labeling the image by selecting the appropriate object class for each bounding box. After labeling is finished, click the Freeze/Edit button to save the data and release freeze for the next labeling.

Once you have enough data, you will train the model using train_ssd.py.

$ cd jetson-inference/python/training/detection/ssd  
$ python3 train_ssd.py --dataset-type=voc --data=data/<YOUR-DATASET> --model-dir=models/<YOUR-MODEL>

If the software does not exist to do train_ssd.py, you necessary to install the packages and then enter the following command to set it up.

$ cd jetson-inference/python/training/detection/ssd  
$ wget https://nvidia.box.com/shared/static/djf5w54rjvpqocsiztzaandq1m3avr7c.pth -O models/mobilenet-v1-ssd-mp-0_675.pth  
$ pip3 install -v -r requirements.txt

After training, the PyTorch model needs to be converted to Onnx data.

$ python3 onnx_export.py --model-dir=models/<YOUR-MODEL>

The converted model will be saved as <YOUR-MODEL>/ssd-mobilenet.onnx and can be loaded by entering the following command.

NET=models/<YOUR-MODEL>

detectnet --model=$NET/ssd-mobilenet.onnx --labels=$NET/labels.txt \  
          --input-blob=input_0 --output-cvg=scores --output-bbox=boxes \  
            csi://0


Referenced URLs: https://github.com/dusty-nv/jetson-inference/blob/master/docs/pytorch-collect-detection.md  
		 https://github.com/dusty-nv/jetson-inference/blob/master/docs/pytorch-ssd.md
