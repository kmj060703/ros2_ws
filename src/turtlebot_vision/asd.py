from ultralytics import YOLO
import torch
import cv2

print(torch.cuda.is_available())
print(torch.__version__)
# nvidia-smi
# sudo apt install nvidia-driver-580 