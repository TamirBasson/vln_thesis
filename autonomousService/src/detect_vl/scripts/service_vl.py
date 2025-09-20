import warnings
import torch
from PIL import Image
from transformers import AutoProcessor, AutoModelForZeroShotObjectDetection 
import numpy as np
import cv2

# Suppress PyTorch deprecation warnings
warnings.filterwarnings("ignore", message=".*encoder_attention_mask.*is deprecated.*", category=FutureWarning)

_MODEL_ID = "IDEA-Research/grounding-dino-tiny"
class GroundingDINOInfer:
    def __init__(self, model_id=_MODEL_ID, device=None):
        self.device = device if device else ("cuda" if torch.cuda.is_available() else "cpu")
        print(f"Using device: {self.device}")
        self.processor = AutoProcessor.from_pretrained(model_id)
        self.model = AutoModelForZeroShotObjectDetection.from_pretrained(model_id).to(self.device)
        
        # Default thresholds (good for simulation)
        self.box_threshold = 0.35
        self.text_threshold = 0.35
        
    def set_thresholds(self, box_threshold=0.35, text_threshold=0.35):
        """Set detection thresholds. Lower values = more permissive detection"""
        self.box_threshold = box_threshold
        self.text_threshold = text_threshold
        print(f"🎯 Updated thresholds: box={box_threshold}, text={text_threshold}")
        
    def set_real_robot_mode(self):
        """Set thresholds optimized for real robot (higher quality images)"""
        self.set_thresholds(box_threshold=0.5, text_threshold=0.5)
        
    def set_simulation_mode(self):
        """Set thresholds optimized for simulation (more permissive)"""
        self.set_thresholds(box_threshold=0.3, text_threshold=0.3)

    def infer(self, img, task):
        # image_resize= cv2.resize(img, (img.shape[1] // 2, img.shape[0] // 2))
        image_rgb = cv2.cvtColor(img, cv2.COLOR_BGR2RGB)
        image = Image.fromarray(image_rgb)
        print(task)
        inputs = self.processor(images=image, text=task, return_tensors="pt").to(self.device)
        with torch.no_grad():
            outputs = self.model(**inputs)

        results = self.processor.post_process_grounded_object_detection(
            outputs,
            inputs.input_ids,
            box_threshold = self.box_threshold,
            text_threshold = self.text_threshold,
            target_sizes=[image.size[::-1]]
        )

        image_cv = cv2.cvtColor(np.array(image), cv2.COLOR_RGB2BGR)
        
        # Debug: print available keys
        print("Available keys in results[0]:", results[0].keys())
        
        boxes = results[0]['boxes'].cpu().numpy()
        
        # Try different possible keys for labels
        if 'labels' in results[0]:
            labels = results[0]['labels']
        elif 'text_labels' in results[0]:
            labels = results[0]['text_labels']
        else:
            print("Warning: No labels found in results. Available keys:", list(results[0].keys()))
            labels = []
            
        scores = results[0]['scores'].cpu().numpy()
        
        # Enhanced debugging
        print(f"🔍 Detection Results for '{task}':")
        print(f"   - Found {len(boxes)} detections above threshold")
        if len(scores) > 0:
            print(f"   - Confidence scores: {[f'{score:.3f}' for score in scores]}")
            print(f"   - Best score: {max(scores):.3f}")
        else:
            print("   - No detections found (all below threshold)")

        rect = None
        center = None

        for box, label, score in zip(boxes, labels, scores):
            x1, y1, x2, y2 = map(int, box)
            rect=[x1, y1, x2, y2]
            center=[(x1 + x2) // 2, (y1 + y2) // 2]
            
            print(f"   - Detection: {label} at ({x1}, {y1}, {x2}, {y2}) with confidence {score:.3f}")

            cv2.rectangle(image_cv, (x1, y1), (x2, y2), (0, 0, 255), 2)
            text = f"{label} {score:.2f}"
            cv2.putText(image_cv, text, (x1, y1 - 10), cv2.FONT_HERSHEY_SIMPLEX, 0.7, (0, 0, 255), 2)

        return image_cv, rect, center
