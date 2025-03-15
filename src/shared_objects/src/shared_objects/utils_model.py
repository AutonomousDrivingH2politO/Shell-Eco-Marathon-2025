import cv2
import numpy as np
import torch
from torchvision import transforms
import matplotlib.pyplot as plt
import time

use_cuda = torch.cuda.is_available()
#use_cuda = False
print("CUDA Available: ", use_cuda)
shapes = [((720, 1280), ((0.5, 0.5), (0.0, 12.0)))]
color_list_seg = {}

def letterbox(combination, new_shape=(384, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True):
    # Resize image to a 32-pixel-multiple rectangle https://github.com/ultralytics/yolov3/issues/232
    img, seg = combination
    shape = img.shape[:2]  # current shape [height, width]
    if isinstance(new_shape, int):
        new_shape = (new_shape, new_shape)

    # Scale ratio (new / old)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup:  # only scale down, do not scale up (for better test mAP)
        r = min(r, 1.0)

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad = int(round(shape[1] * r)), int(round(shape[0] * r))
    dw, dh = new_shape[1] - new_unpad[0], new_shape[0] - new_unpad[1]  # wh padding
    if auto:  # minimum rectangle
        dw, dh = np.mod(dw, 32), np.mod(dh, 32)  # wh padding
    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad = (new_shape[1], new_shape[0])
        ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]  # width, height ratios

    dw /= 2  # divide padding into 2 sides
    dh /= 2

    if shape[::-1] != new_unpad:  # resize
        img = cv2.resize(img, new_unpad, interpolation=cv2.INTER_LINEAR)
        if seg:
            for seg_class in seg:
                seg[seg_class] = cv2.resize(seg[seg_class], new_unpad, interpolation=cv2.INTER_LINEAR)

    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))

    img = cv2.copyMakeBorder(img, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    if seg:
        for seg_class in seg:
            seg[seg_class] = cv2.copyMakeBorder(seg[seg_class], top, bottom, left, right, cv2.BORDER_CONSTANT, value=0)  # add border

    combination = (img, seg)
    return combination, ratio, (dw, dh)

for seg_class in ['road','lane']:
    color_list_seg[seg_class] = list(np.random.choice(range(256), size=3))

def preprocessing_image(image,half=False):
    image = cv2.cvtColor(image, cv2.COLOR_BGR2RGB)

    resized_shape = 640

    normalize = transforms.Normalize(
        mean=[0.485, 0.456, 0.406],
        std=[0.229, 0.224, 0.225]
    )
    transform = transforms.Compose([
        transforms.ToTensor(),
        normalize,
    ])

    h0, w0 = image.shape[:2]
    r = resized_shape / max(h0, w0)
    input_img = cv2.resize(image, (int(w0 * r), int(h0 * r)), interpolation=cv2.INTER_AREA)


    (input_img, _), _, _ = letterbox((input_img, None), resized_shape, auto=True,
                                            scaleup=False)

    if use_cuda:
        input_tensor = transform(input_img).unsqueeze(0).cuda()
    else:
        input_tensor = transform(input_img).unsqueeze(0).cpu()
    return input_tensor

def preprocessing_mask(seg, show=False,improve=True):
    _, seg_mask = torch.max(seg, 1)
    seg_mask_ = seg_mask[0].squeeze().cpu().numpy()
    pad_h = int(shapes[0][1][1][1])
    pad_w = int(shapes[0][1][1][0])
    seg_mask_ = seg_mask_[pad_h:seg_mask_.shape[0]-pad_h, pad_w:seg_mask_.shape[1]-pad_w]
    seg_mask_ = cv2.resize(seg_mask_, dsize=shapes[0][0][::-1], interpolation=cv2.INTER_NEAREST)
    color_seg = np.zeros((seg_mask_.shape[0], seg_mask_.shape[1], 3), dtype=np.uint8)
    for index, seg_class in enumerate(['road','lane']):
        if seg_class == 'road': # 'road', 'lane', or remove this line for both 'road' and 'lane
            color_seg[seg_mask_ == index+1] = color_list_seg[seg_class]
    color_seg = color_seg[..., ::-1]  
    color_mask = np.mean(color_seg, 2)
    _, end_mask = cv2.threshold(color_mask,0,255, cv2.THRESH_BINARY)

    if improve:
        _,labeled_image, stats, _ = cv2.connectedComponentsWithStats(image=np.uint8(end_mask))
        if len(stats)>2:
            wanted_label=np.argmax(stats[1::,4])+1
            end_mask=np.array(np.where(labeled_image==wanted_label,255,0),dtype=np.uint8)

    if show:
        plt.imshow(end_mask)
        plt.show()
    return end_mask.astype('uint8')