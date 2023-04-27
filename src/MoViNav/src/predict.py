import cv2
import numpy as np
import torch

def depth_norm(x, max_depth):
    return max_depth / x

def predict(model, device, image, min_depth, max_depth, batch_size):

    # Compute predictions
    with torch.no_grad():
        predictions = model(torch.from_numpy(image).permute(2,0,1).unsqueeze(0).to(device)).cpu().numpy()[0]

    # Put in expected range
    return np.clip(depth_norm(predictions, max_depth=max_depth), min_depth, max_depth) / max_depth