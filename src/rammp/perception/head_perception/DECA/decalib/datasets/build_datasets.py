import os
import sys
from glob import glob

import cv2
import numpy as np
import scipy
import torch
import torchvision.transforms as transforms
from skimage.io import imread, imsave
from skimage.transform import estimate_transform, rescale, resize, warp
from torch.utils.data import ConcatDataset, Dataset

from .aflw2000 import AFLW2000
from .ethnicity import EthnicityDataset
from .now import NoWDataset
from .vggface import VGGFace2Dataset
from .vox import VoxelDataset


def build_train(config, is_train=True):
    data_list = []
    if 'vox2' in config.training_data:
        data_list.append(VoxelDataset(dataname='vox2', K=config.K, image_size=config.image_size, scale=[config.scale_min, config.scale_max], trans_scale=config.trans_scale, isSingle=config.isSingle))
    if 'vggface2' in config.training_data:
        data_list.append(VGGFace2Dataset(K=config.K, image_size=config.image_size, scale=[config.scale_min, config.scale_max], trans_scale=config.trans_scale, isSingle=config.isSingle))
    if 'vggface2hq' in config.training_data:
        data_list.append(VGGFace2HQDataset(K=config.K, image_size=config.image_size, scale=[config.scale_min, config.scale_max], trans_scale=config.trans_scale, isSingle=config.isSingle))
    if 'ethnicity' in config.training_data:
        data_list.append(EthnicityDataset(K=config.K, image_size=config.image_size, scale=[config.scale_min, config.scale_max], trans_scale=config.trans_scale, isSingle=config.isSingle))
    if 'coco' in config.training_data:
        data_list.append(COCODataset(image_size=config.image_size, scale=[config.scale_min, config.scale_max], trans_scale=config.trans_scale))
    if 'celebahq' in config.training_data:
        data_list.append(CelebAHQDataset(image_size=config.image_size, scale=[config.scale_min, config.scale_max], trans_scale=config.trans_scale))
    dataset = ConcatDataset(data_list)
    
    return dataset

def build_val(config, is_train=True):
    data_list = []
    if 'vggface2' in config.eval_data:
        data_list.append(VGGFace2Dataset(isEval=True, K=config.K, image_size=config.image_size, scale=[config.scale_min, config.scale_max], trans_scale=config.trans_scale, isSingle=config.isSingle))
    if 'now' in config.eval_data:
        data_list.append(NoWDataset())
    if 'aflw2000' in config.eval_data:
        data_list.append(AFLW2000())
    dataset = ConcatDataset(data_list)

    return dataset
