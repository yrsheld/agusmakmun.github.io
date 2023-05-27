---
layout: post
title:  "Loading image dataset in Pytorch"
date:   2023-03-02 14:30:00 +0700
categories: [Pytorch, ML]
---
## Imports
``` python
import torch
from torchvision import datasets, transforms
from torch.utils.data import DataLoader
```
## 1. Transform

Common transforms include image resizing, data augmentation, value standarization and normalization.

### 1.1 Data Augmentation
* Ex: RandomCrop, RandomRotate, RandomHorizontalFlip...
* In general, only for **training dataset**.
* But if there includes **resizing the image to specific size**, then it **must also be done on the testing data**.
* Example:
If `transforms.CenterCrop(224)` is done on training, then it must also be done on the testing to make them both at the same size: 224 x 224.

### 1.2 Convert to torch tensor
Convert a PIL image or ndarray to tensor, and scale the values to **[0.0, 1.0]** accordingly.
* Necessary for both **training & testing dataset**

### 1.3 Normalization
For better and more stable training, normalize each channel from [0.0, 1.0] to **[-1.0, 1.0]**. This is done by transforms.Normalize(means, stds).
``` python
# For rgb images (3 channels)
# Normalize each channel
transforms.Normalize([0.5, 0.5, 0.5],[0.5, 0.5, 0.5])
```
* Necessary for both **training & testing dataset**

### 1.4 Example:
``` python
train_T = transforms.Compose([
    transforms.Resize(255),
    transforms.RandomRotation(30),
    transforms.CenterCrop(224),                     # random crop and resize to 224 x 224
    transforms.ToTensor(),                                 # convert image to tensor and scale values to [0, 1]
    transforms.Normalize([0.5, 0.5, 0.5],
                         [0.5, 0.5, 0.5])                  # normalize each channel value to [-1,1]
])

val_T = transforms.accimageCompose([
    transforms.Resize(255),
    transforms.CenterCrop(224),
    transforms.ToTensor(),                                 # convert image to tensor and scale values to [0, 1]
    transforms.Normalize([0.5, 0.5, 0.5],
                         [0.5, 0.5, 0.5])                  # normalize each channel value to [-1,1]
])
```

> Choosing 224 here is just due to the fact that most pretrained models accept input of this size. So by resizing images to this size would allow using pretrained models for transfer learning afterwards.

## 2. ImageFolder
``` python
ds = datasets.ImageFolder('path/to/data', transform=transform)
```

Images of each class are in separate folders. The file level should be like

```
root/dog/dog_0.png
root/dog/dog_1.png
...

root/cat/cat_0.png
root/cat/cat_1.png
...
```
## 3. DataLoader
``` python
dataloader = DataLoader(ds, batch_size=32, shuffle=True)
```
Every time we loop through the dataloader, it outputs a batch of data with labels

``` python
for images, labels in dataloader:
    ...
```