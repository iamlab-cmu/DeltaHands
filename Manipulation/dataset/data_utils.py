import numpy as np
import math
import torch
import pickle
import os
import torchvision.transforms as transforms

image_transforms = transforms.Compose([transforms.CenterCrop(size=(216,288))])

def normalize_data(data, stats):
    # nomalize to [0,1]
    ndata = (data - stats['min']) / (stats['max'] - stats['min'])
    # normalize to [-1, 1]
    ndata = ndata * 2 - 1
    return ndata

def unnormalize_data(ndata, stats):
    ndata = (ndata + 1) / 2
    data = ndata * (stats['max'] - stats['min']) + stats['min']
    return data

def normalize_images(images):
    # resize image to (120, 160)
    # nomalize to [0,1]
    nimages = image_transforms(torch.from_numpy(images / 255.0))
    return nimages
