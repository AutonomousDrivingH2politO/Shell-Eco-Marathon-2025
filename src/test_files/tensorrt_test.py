import tensorrt as trt

TRT_LOGGER=trt.Logger(trt.Logger.WARNING)

def build_engine_onnx(model_file):
    