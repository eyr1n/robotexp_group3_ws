import cv2
import torch

from nanodet.data.batch_process import stack_batch_img
from nanodet.data.collate import naive_collate
from nanodet.data.transform import Pipeline
from nanodet.model.arch import build_model
from nanodet.util import Logger, cfg, load_config, load_model_weight


class Predictor(object):
    def __init__(self, cfg, model_path):
        self.cfg = cfg
        self.device = torch.device("cpu")
        model = build_model(cfg.model)
        ckpt = torch.load(model_path, map_location=lambda storage, loc: storage)
        logger = Logger(0, use_tensorboard=False)
        load_model_weight(model, ckpt, logger)
        self.model = model.to(self.device).eval()
        self.pipeline = Pipeline(cfg.data.val.pipeline, cfg.data.val.keep_ratio)

    def inference(self, img):
        height, width = img.shape[:2]
        img_info = {
            "id": 0,
            "file_name": None,
            "height": height,
            "width": width,
        }
        meta = {
            "img_info": img_info, 
            "raw_img": img, 
            "img": img,
        }
        meta = self.pipeline(None, meta, self.cfg.data.val.input_size)
        meta["img"] = torch.from_numpy(meta["img"].transpose(2, 0, 1)).to(self.device)
        meta = naive_collate([meta])
        meta["img"] = stack_batch_img(meta["img"], divisible=32)
        with torch.no_grad():
            results = self.model.inference(meta)
        return results

    def visualize(self, dets, meta, class_names, score_thres, wait=0):
        self.model.head.show_result(
            meta, dets, class_names, score_thres=score_thres, show=True
        )
