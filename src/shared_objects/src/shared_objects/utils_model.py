import cv2
import numpy as np
import torch
from torchvision import transforms
import matplotlib.pyplot as plt
import time
import torch.nn.functional as F
use_cuda = torch.cuda.is_available()
#use_cuda = False
print("CUDA Available: ", use_cuda)
shapes = [((720, 1280), ((0.5, 0.5), (0.0, 12.0)))]
color_list_seg = {}
# In shared_objects/utils_model.py
import cv2
import numpy as np

def letterbox_single_twin(img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True):
    # Ensure new_shape elements are integers if they come from an external source
    if isinstance(new_shape, tuple):
        new_h, new_w = new_shape
        if not (isinstance(new_h, int) and isinstance(new_w, int)):
            # This is a guard; ideally, new_shape is always passed correctly
            raise TypeError(f"new_shape dimensions must be integers. Got: {new_shape}")
    elif isinstance(new_shape, int):
        new_shape = (new_shape, new_shape) # Convert to tuple
    else:
        raise TypeError(f"new_shape must be an int or a tuple of ints. Got: {type(new_shape)}")

    shape = img.shape[:2]  # current shape [height, width]
    h, w = shape

    # Scale ratio (new / old)
    r = min(new_shape[0] / h, new_shape[1] / w) # DIVISION HERE (h, w must be int)
    if not scaleup:  # only scale down, do not scale up
        r = min(r, 1.0)

    # Compute padding
    ratio = r, r  # width, height ratios
    new_unpad_w = int(round(w * r))
    new_unpad_h = int(round(h * r))
    new_unpad_shape_wh = (new_unpad_w, new_unpad_h)

    dw = new_shape[1] - new_unpad_w  # wh padding (width)
    dh = new_shape[0] - new_unpad_h  # wh padding (height)

    if auto:  # minimum rectangle
        dw = np.mod(dw, 32)  # wh padding
        dh = np.mod(dh, 32)  # wh padding
    elif scaleFill:  # stretch
        dw, dh = 0.0, 0.0
        new_unpad_shape_wh = (new_shape[1], new_shape[0])
        # Ensure shape dimensions are not zero before division
        if w == 0 or h == 0:
            raise ValueError("Original image dimensions cannot be zero for scaleFill.")
        ratio = new_shape[1] / w, new_shape[0] / h  # width, height ratios DIVISION HERE

    dw /= 2  # divide padding into 2 sides (DIVISION HERE)
    dh /= 2  # (DIVISION HERE)

    if shape[::-1] != new_unpad_shape_wh:  # resize
        img_resized = cv2.resize(img, new_unpad_shape_wh, interpolation=cv2.INTER_LINEAR)
    else:
        img_resized = img.copy() # No resize needed

    top = int(round(dh - 0.1))
    bottom = int(round(dh + 0.1))
    left = int(round(dw - 0.1))
    right = int(round(dw + 0.1))

    img_padded = cv2.copyMakeBorder(img_resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)  # add border
    
    padding_tblr = (top, bottom, left, right)
    unpadded_shape_after_letterbox_resize_hw = (new_unpad_h, new_unpad_w) # (height, width)

    return img_padded, ratio, padding_tblr, unpadded_shape_after_letterbox_resize_hw
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


    (input_img, _), ratio, (dw, dh)= letterbox((input_img, None), resized_shape, auto=True,
                                            scaleup=False)

    if use_cuda:
        input_tensor = transform(input_img).unsqueeze(0).cuda()
    else:
        input_tensor = transform(input_img).unsqueeze(0).cpu()
    return input_tensor, dw,dh
def letterbox_single_twin(img, new_shape=(640, 640), color=(114, 114, 114), auto=True, scaleFill=False, scaleup=True):
    shape = img.shape[:2]
    if isinstance(new_shape, int): new_shape = (new_shape, new_shape)
    r = min(new_shape[0] / shape[0], new_shape[1] / shape[1])
    if not scaleup: r = min(r, 1.0)
    ratio = r, r
    new_unpad_shape_wh = (int(round(shape[1] * r)), int(round(shape[0] * r)))
    dw, dh = new_shape[1] - new_unpad_shape_wh[0], new_shape[0] - new_unpad_shape_wh[1]
    if auto: dw, dh = np.mod(dw, 32), np.mod(dh, 32)
    elif scaleFill: dw, dh = 0.0, 0.0; new_unpad_shape_wh = (new_shape[1], new_shape[0]); ratio = new_shape[1] / shape[1], new_shape[0] / shape[0]
    dw /= 2; dh /= 2
    if shape[::-1] != new_unpad_shape_wh:
        img_resized = cv2.resize(img, new_unpad_shape_wh, interpolation=cv2.INTER_LINEAR)
    else: img_resized = img.copy()
    top, bottom = int(round(dh - 0.1)), int(round(dh + 0.1))
    left, right = int(round(dw - 0.1)), int(round(dw + 0.1))
    img_padded = cv2.copyMakeBorder(img_resized, top, bottom, left, right, cv2.BORDER_CONSTANT, value=color)
    padding_tblr = (top, bottom, left, right)
    unpadded_shape_after_letterbox_resize_hw = (new_unpad_shape_wh[1], new_unpad_shape_wh[0])
    return img_padded, ratio, padding_tblr, unpadded_shape_after_letterbox_resize_hw,dw,dh


def preprocess_image_twinlitenetplus(image_bgr, target_img_size=640, device=None, half_precision=False):
    if device is None: device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    original_shape_hw = image_bgr.shape[:2]
    img_rgb = cv2.cvtColor(image_bgr, cv2.COLOR_BGR2RGB)
    img_padded, _ratio_orig_to_padded_content, padding_tblr, unpadded_content_shape_hw,dw,dh = \
        letterbox_single_twin(img_rgb, new_shape=(target_img_size, target_img_size), auto=True, scaleup=True)
    img_tensor = torch.from_numpy(img_padded.transpose((2, 0, 1))).contiguous().float() / 255.0
    img_tensor = img_tensor.unsqueeze(0).to(device)
    if half_precision and device.type == 'cuda': img_tensor = img_tensor.half()
    padded_tensor_shape_hw = tuple(img_tensor.shape[2:])
    return (img_tensor, _ratio_orig_to_padded_content, padding_tblr,
            unpadded_content_shape_hw, padded_tensor_shape_hw, original_shape_hw,dw,dh)

def process_twinlitenetplus_masks(
    da_seg_out_raw: torch.Tensor,
    ll_seg_out_raw: torch.Tensor,
    original_image_shape_hw: tuple,
    padded_tensor_shape_hw: tuple,
    padding_tblr: tuple,
    unpadded_content_shape_hw: tuple
):
    pad_t, pad_b, pad_l, pad_r = padding_tblr
    model_out_h, model_out_w = padded_tensor_shape_hw

    # --- Drivable Area Segmentation ---
    da_predict_unpadded_logits = da_seg_out_raw[:, :, pad_t : model_out_h - pad_b, pad_l : model_out_w - pad_r]
    da_seg_intermediate_logits = F.interpolate( # <--- F is used here
        da_predict_unpadded_logits,
        size=unpadded_content_shape_hw,
        mode='bilinear',
        align_corners=False
    )
    _, da_labels_intermediate = torch.max(da_seg_intermediate_logits, 1)
    da_seg_mask_final = F.interpolate( # <--- F is used here
        da_labels_intermediate.float().unsqueeze(1),
        size=original_image_shape_hw,
        mode='nearest'
    ).squeeze().cpu().numpy().astype(np.uint8)

    # --- Lane Line Segmentation ---
    ll_predict_unpadded_logits = ll_seg_out_raw[:, :, pad_t : model_out_h - pad_b, pad_l : model_out_w - pad_r]
    ll_seg_intermediate_logits = F.interpolate( # <--- F is used here
        ll_predict_unpadded_logits,
        size=unpadded_content_shape_hw,
        mode='bilinear',
        align_corners=False
    )
    _, ll_labels_intermediate = torch.max(ll_seg_intermediate_logits, 1)
    ll_seg_mask_final = F.interpolate( # <--- F is used here
        ll_labels_intermediate.float().unsqueeze(1),
        size=original_image_shape_hw,
        mode='nearest'
    ).squeeze().cpu().numpy().astype(np.uint8)

    return da_seg_mask_final, ll_seg_mask_final

def preprocessing_image_no_normalisation(image, model,half=False):
    # No mean/std unlike other models
    rgb = image[:, :, ::-1]                 # BGR to RGB
    resized_shape = 640
    if model == "twin":
        boxed = cv2.resize(image, (640, 360), interpolation=cv2.INTER_AREA)
        dw, dh = 0.0, 0.0
    else:
        h0, w0 = rgb.shape[:2]
        r = resized_shape / max(h0, w0)                 # scale factor
        resized = cv2.resize(rgb, (int(w0*r), int(h0*r)), interpolation=cv2.INTER_AREA)

        (boxed, _),ratio, (dw, dh) = letterbox((resized, None),
                                    resized_shape, auto=True, scaleup=False)

    tensor = torch.from_numpy(boxed.transpose(2,0,1)).cuda().float() / 255.0
    tensor = tensor.unsqueeze(0)
    if use_cuda:
        tensor = tensor.cuda() 
    return tensor ,dw , dh


import cv2
import numpy as np
import torch
from shared_objects.utils_model import letterbox, color_list_seg
import matplotlib.pyplot as plt

def preprocessing_image_no_normalisation(image, model, half=False):
    """
    Resize+letterbox the BGR input into a 640×640 box, return:
      - normalized tensor [1×3×HxW] (float32 or float16 if half=True),
      - the pad widths (dw, dh) so we can undo the letterbox exactly.
    """
    # Note: image is BGR
    rgb = image[:, :, ::-1]  # BGR->RGB just for resizing consistency
    resized_shape = 640

    if model == "twin":
        # twin expects non-square input (640×360), no letterbox
        boxed = cv2.resize(image, (640, 360), interpolation=cv2.INTER_AREA)
        dw, dh = 0.0, 0.0
    else:
        h0, w0 = rgb.shape[:2]
        r = resized_shape / max(h0, w0)
        resized = cv2.resize(rgb, (int(w0 * r), int(h0 * r)), interpolation=cv2.INTER_AREA)

        # letterbox returns ((img, _), ratio, (dw, dh))
        (boxed, _), (ratio_w, ratio_h), (dw, dh) = letterbox(
            (resized, None),
            new_shape=resized_shape,
            auto=True,
            scaleup=False
        )

    # convert to tensor 1×3×H×W and normalize to [0,1]
    tensor = torch.from_numpy(boxed.transpose(2, 0, 1)).float() / 255.0
    if half:
        tensor = tensor.half()
    tensor = tensor.unsqueeze(0).cuda() if torch.cuda.is_available() else tensor.unsqueeze(0)

    return tensor, dw, dh


def preprocessing_mask(seg, dw0,dh0 , orig_shape,show=False,improve=True):
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

### Adding classes and methods for TwinLite
import torch
import torch.nn as nn


from torch.nn import Module, Conv2d, Parameter, Softmax

class PAM_Module(Module):
    """ Position attention module"""
    #Ref from SAGAN
    def __init__(self, in_dim):
        super(PAM_Module, self).__init__()
        self.chanel_in = in_dim

        self.query_conv = Conv2d(in_channels=in_dim, out_channels=in_dim//8, kernel_size=1)
        self.key_conv = Conv2d(in_channels=in_dim, out_channels=in_dim//8, kernel_size=1)
        self.value_conv = Conv2d(in_channels=in_dim, out_channels=in_dim, kernel_size=1)
        self.gamma = Parameter(torch.zeros(1))

        self.softmax = Softmax(dim=-1)
    def forward(self, x):
        """
            inputs :
                x : input feature maps( B X C X H X W)
            returns :
                out : attention value + input feature
                attention: B X (HxW) X (HxW)
        """
        m_batchsize, C, height, width = x.size()
        proj_query = self.query_conv(x).view(m_batchsize, -1, width*height).permute(0, 2, 1)
        proj_key = self.key_conv(x).view(m_batchsize, -1, width*height)
        energy = torch.bmm(proj_query, proj_key)
        attention = self.softmax(energy)
        proj_value = self.value_conv(x).view(m_batchsize, -1, width*height)

        out = torch.bmm(proj_value, attention.permute(0, 2, 1))
        out = out.view(m_batchsize, C, height, width)

        out = self.gamma*out + x
        return out
class CAM_Module(Module):
    """ Channel attention module"""
    def __init__(self, in_dim):
        super(CAM_Module, self).__init__()
        self.chanel_in = in_dim


        self.gamma = Parameter(torch.zeros(1))
        self.softmax  = Softmax(dim=-1)
    def forward(self,x):
        """
            inputs :
                x : input feature maps( B X C X H X W)
            returns :
                out : attention value + input feature
                attention: B X C X C
        """
        m_batchsize, C, height, width = x.size()
        proj_query = x.view(m_batchsize, C, -1)
        proj_key = x.view(m_batchsize, C, -1).permute(0, 2, 1)
        energy = torch.bmm(proj_query, proj_key)
        energy_new = torch.max(energy, -1, keepdim=True)[0].expand_as(energy)-energy
        attention = self.softmax(energy_new)
        proj_value = x.view(m_batchsize, C, -1)

        out = torch.bmm(attention, proj_value)
        out = out.view(m_batchsize, C, height, width)

        out = self.gamma*out + x
        return out


class UPx2(nn.Module):
    '''
    This class defines the convolution layer with batch normalization and PReLU activation
    '''
    def __init__(self, nIn, nOut):
        '''

        :param nIn: number of input channels
        :param nOut: number of output channels
        :param kSize: kernel size
        :param stride: stride rate for down-sampling. Default is 1
        '''
        super().__init__()
        self.deconv = nn.ConvTranspose2d(nIn, nOut, 2, stride=2, padding=0, output_padding=0, bias=False)
        self.bn = nn.BatchNorm2d(nOut, eps=1e-03)
        self.act = nn.PReLU(nOut)

    def forward(self, input):
        '''
        :param input: input feature map
        :return: transformed feature map
        '''
        output = self.deconv(input)
        output = self.bn(output)
        output = self.act(output)
        return output
    def fuseforward(self, input):
        output = self.deconv(input)
        output = self.act(output)
        return output

class CBR(nn.Module):
    '''
    This class defines the convolution layer with batch normalization and PReLU activation
    '''
    def __init__(self, nIn, nOut, kSize, stride=1):
        '''

        :param nIn: number of input channels
        :param nOut: number of output channels
        :param kSize: kernel size
        :param stride: stride rate for down-sampling. Default is 1
        '''
        super().__init__()
        padding = int((kSize - 1)/2)
        #self.conv = nn.Conv2d(nIn, nOut, kSize, stride=stride, padding=padding, bias=False)
        self.conv = nn.Conv2d(nIn, nOut, (kSize, kSize), stride=stride, padding=(padding, padding), bias=False)
        #self.conv1 = nn.Conv2d(nOut, nOut, (1, kSize), stride=1, padding=(0, padding), bias=False)
        self.bn = nn.BatchNorm2d(nOut, eps=1e-03)
        self.act = nn.PReLU(nOut)

    def forward(self, input):
        '''
        :param input: input feature map
        :return: transformed feature map
        '''
        output = self.conv(input)
        #output = self.conv1(output)
        output = self.bn(output)
        output = self.act(output)
        return output
    def fuseforward(self, input):
        output = self.conv(input)
        output = self.act(output)
        return output
    




class CB(nn.Module):
    '''
       This class groups the convolution and batch normalization
    '''
    def __init__(self, nIn, nOut, kSize, stride=1):
        '''
        :param nIn: number of input channels
        :param nOut: number of output channels
        :param kSize: kernel size
        :param stride: optinal stide for down-sampling
        '''
        super().__init__()
        padding = int((kSize - 1)/2)
        self.conv = nn.Conv2d(nIn, nOut, (kSize, kSize), stride=stride, padding=(padding, padding), bias=False)
        self.bn = nn.BatchNorm2d(nOut, eps=1e-03)

    def forward(self, input):
        '''

        :param input: input feature map
        :return: transformed feature map
        '''
        output = self.conv(input)
        output = self.bn(output)
        return output

class C(nn.Module):
    '''
    This class is for a convolutional layer.
    '''
    def __init__(self, nIn, nOut, kSize, stride=1):
        '''

        :param nIn: number of input channels
        :param nOut: number of output channels
        :param kSize: kernel size
        :param stride: optional stride rate for down-sampling
        '''
        super().__init__()
        padding = int((kSize - 1)/2)
        # print(nIn, nOut, (kSize, kSize))
        self.conv = nn.Conv2d(nIn, nOut, (kSize, kSize), stride=stride, padding=(padding, padding), bias=False)

    def forward(self, input):
        '''
        :param input: input feature map
        :return: transformed feature map
        '''
        output = self.conv(input)
        return output

class CDilated(nn.Module):
    '''
    This class defines the dilated convolution.
    '''
    def __init__(self, nIn, nOut, kSize, stride=1, d=1):
        '''
        :param nIn: number of input channels
        :param nOut: number of output channels
        :param kSize: kernel size
        :param stride: optional stride rate for down-sampling
        :param d: optional dilation rate
        '''
        super().__init__()
        padding = int((kSize - 1)/2) * d
        self.conv = nn.Conv2d(nIn, nOut, (kSize, kSize), stride=stride, padding=(padding, padding), bias=False, dilation=d)

    def forward(self, input):
        '''
        :param input: input feature map
        :return: transformed feature map
        '''
        output = self.conv(input)
        return output

class DownSamplerB(nn.Module):
    def __init__(self, nIn, nOut):
        super().__init__()
        n = int(nOut/5)
        n1 = nOut - 4*n
        self.c1 = C(nIn, n, 3, 2)
        self.d1 = CDilated(n, n1, 3, 1, 1)
        self.d2 = CDilated(n, n, 3, 1, 2)
        self.d4 = CDilated(n, n, 3, 1, 4)
        self.d8 = CDilated(n, n, 3, 1, 8)
        self.d16 = CDilated(n, n, 3, 1, 16)
        self.bn = nn.BatchNorm2d(nOut, eps=1e-3)
        self.act = nn.PReLU(nOut)

    def forward(self, input):
        output1 = self.c1(input)
        d1 = self.d1(output1)
        d2 = self.d2(output1)
        d4 = self.d4(output1)
        d8 = self.d8(output1)
        d16 = self.d16(output1)

        add1 = d2
        add2 = add1 + d4
        add3 = add2 + d8
        add4 = add3 + d16

        combine = torch.cat([d1, add1, add2, add3, add4],1)
        #combine_in_out = input + combine
        output = self.bn(combine)
        output = self.act(output)
        return output
class BR(nn.Module):
    '''
        This class groups the batch normalization and PReLU activation
    '''
    def __init__(self, nOut):
        '''
        :param nOut: output feature maps
        '''
        super().__init__()
        self.nOut=nOut
        self.bn = nn.BatchNorm2d(nOut, eps=1e-03)
        self.act = nn.PReLU(nOut)

    def forward(self, input):
        '''
        :param input: input feature map
        :return: normalized and thresholded feature map
        '''
        # print("bf bn :",input.size(),self.nOut)
        output = self.bn(input)
        # print("after bn :",output.size())
        output = self.act(output)
        # print("after act :",output.size())
        return output
class DilatedParllelResidualBlockB(nn.Module):
    '''
    This class defines the ESP block, which is based on the following principle
        Reduce ---> Split ---> Transform --> Merge
    '''
    def __init__(self, nIn, nOut, add=True):
        '''
        :param nIn: number of input channels
        :param nOut: number of output channels
        :param add: if true, add a residual connection through identity operation. You can use projection too as
                in ResNet paper, but we avoid to use it if the dimensions are not the same because we do not want to
                increase the module complexity
        '''
        super().__init__()
        n = max(int(nOut/5),1)
        n1 = max(nOut - 4*n,1)
        # print(nIn,n,n1,"--")
        self.c1 = C(nIn, n, 1, 1)
        self.d1 = CDilated(n, n1, 3, 1, 1) # dilation rate of 2^0
        self.d2 = CDilated(n, n, 3, 1, 2) # dilation rate of 2^1
        self.d4 = CDilated(n, n, 3, 1, 4) # dilation rate of 2^2
        self.d8 = CDilated(n, n, 3, 1, 8) # dilation rate of 2^3
        self.d16 = CDilated(n, n, 3, 1, 16) # dilation rate of 2^4
        # print("nOut bf :",nOut)
        self.bn = BR(nOut)
        # print("nOut at :",self.bn.size())
        self.add = add

    def forward(self, input):
        '''
        :param input: input feature map
        :return: transformed feature map
        '''
        # reduce
        output1 = self.c1(input)
        # split and transform
        d1 = self.d1(output1)
        d2 = self.d2(output1)
        d4 = self.d4(output1)
        d8 = self.d8(output1)
        d16 = self.d16(output1)
        

        # heirarchical fusion for de-gridding
        add1 = d2
        add2 = add1 + d4
        add3 = add2 + d8
        add4 = add3 + d16
        # print(d1.size(),add1.size(),add2.size(),add3.size(),add4.size())

        #merge
        combine = torch.cat([d1, add1, add2, add3, add4], 1)
        # print("combine :",combine.size())
        # if residual version
        if self.add:
            # print("add :",combine.size())
            combine = input + combine
        # print(combine.size(),"-----------------")
        output = self.bn(combine)
        return output

class InputProjectionA(nn.Module):
    '''
    This class projects the input image to the same spatial dimensions as the feature map.
    For example, if the input image is 512 x512 x3 and spatial dimensions of feature map size are 56x56xF, then
    this class will generate an output of 56x56x3
    '''
    def __init__(self, samplingTimes):
        '''
        :param samplingTimes: The rate at which you want to down-sample the image
        '''
        super().__init__()
        self.pool = nn.ModuleList()
        for i in range(0, samplingTimes):
            #pyramid-based approach for down-sampling
            self.pool.append(nn.AvgPool2d(3, stride=2, padding=1))

    def forward(self, input):
        '''
        :param input: Input RGB Image
        :return: down-sampled image (pyramid-based approach)
        '''
        for pool in self.pool:
            input = pool(input)
        return input

class ESPNet_Encoder(nn.Module):
    '''
    This class defines the ESPNet-C network in the paper
    '''
    def __init__(self, p=5, q=3):
    # def __init__(self, classes=20, p=1, q=1):
        '''
        :param classes: number of classes in the dataset. Default is 20 for the cityscapes
        :param p: depth multiplier
        :param q: depth multiplier
        '''
        super().__init__()
        self.level1 = CBR(3, 16, 3, 2)
        self.sample1 = InputProjectionA(1)
        self.sample2 = InputProjectionA(2)

        self.b1 = CBR(16 + 3,19,3)
        self.level2_0 = DownSamplerB(16 +3, 64)

        self.level2 = nn.ModuleList()
        for i in range(0, p):
            self.level2.append(DilatedParllelResidualBlockB(64 , 64))
        self.b2 = CBR(128 + 3,131,3)

        self.level3_0 = DownSamplerB(128 + 3, 128)
        self.level3 = nn.ModuleList()
        for i in range(0, q):
            self.level3.append(DilatedParllelResidualBlockB(128 , 128))
        # self.mixstyle = MixStyle2(p=0.5, alpha=0.1)
        self.b3 = CBR(256,32,3)
        self.sa = PAM_Module(32)
        self.sc = CAM_Module(32)
        self.conv_sa = CBR(32,32,3)
        self.conv_sc = CBR(32,32,3)
        self.classifier = CBR(32, 32, 1, 1)

    def forward(self, input):
        '''
        :param input: Receives the input RGB image
        :return: the transformed feature map with spatial dimensions 1/8th of the input image
        '''
        output0 = self.level1(input)
        inp1 = self.sample1(input)
        inp2 = self.sample2(input)

        output0_cat = self.b1(torch.cat([output0, inp1], 1))
        output1_0 = self.level2_0(output0_cat) # down-sampled
        
        for i, layer in enumerate(self.level2):
            if i==0:
                output1 = layer(output1_0)
            else:
                output1 = layer(output1)

        output1_cat = self.b2(torch.cat([output1,  output1_0, inp2], 1))
        output2_0 = self.level3_0(output1_cat) # down-sampled
        for i, layer in enumerate(self.level3):
            if i==0:
                output2 = layer(output2_0)
            else:
                output2 = layer(output2)
        cat_=torch.cat([output2_0, output2], 1)

        output2_cat = self.b3(cat_)
        out_sa=self.sa(output2_cat)
        out_sa=self.conv_sa(out_sa)
        out_sc=self.sc(output2_cat)
        out_sc=self.conv_sc(out_sc)
        out_s=out_sa+out_sc
        classifier = self.classifier(out_s)

        return classifier

class TwinLiteNet(nn.Module):
    '''
    This class defines the ESPNet network
    '''

    def __init__(self, p=2, q=3, ):

        super().__init__()
        self.encoder = ESPNet_Encoder(p, q)

        self.up_1_1 = UPx2(32,16)
        self.up_2_1 = UPx2(16,8)

        self.up_1_2 = UPx2(32,16)
        self.up_2_2 = UPx2(16,8)

        self.classifier_1 = UPx2(8,2)
        self.classifier_2 = UPx2(8,2)



    def forward(self, input):

        x=self.encoder(input)
        x1=self.up_1_1(x)
        x1=self.up_2_1(x1)
        classifier1=self.classifier_1(x1)
        
        

        x2=self.up_1_2(x)
        x2=self.up_2_2(x2)
        classifier2=self.classifier_2(x2)

        return (classifier1,classifier2)

