"""
Perception Agent - YOLO/ONNX-based Object Detection for UAV/USV/UGV
Provides real-time object detection and tracking using YOLOv8 with ONNX Runtime.
"""

from dataclasses import dataclass, field
from typing import List, Dict, Any, Optional, Tuple
import asyncio
import numpy as np
import time
import uuid

from ...core.base_agent import BaseAgent, AgentMessage, MessagePriority, AgentState
from ...utils.config import PerceptionConfig, AsyncLogger


@dataclass
class Detection:
    """Single object detection result."""
    bbox: Tuple[float, float, float, float]  # x1, y1, x2, y2 (normalized)
    class_id: int
    class_name: str
    confidence: float
    tracking_id: Optional[int] = None
    timestamp: float = field(default_factory=time.time)
    
    @property
    def center(self) -> Tuple[float, float]:
        """Get bounding box center."""
        return ((self.bbox[0] + self.bbox[2]) / 2, 
                (self.bbox[1] + self.bbox[3]) / 2)
    
    @property
    def area(self) -> float:
        """Get bounding box area."""
        return (self.bbox[2] - self.bbox[0]) * (self.bbox[3] - self.bbox[1])


@dataclass
class DetectionFrame:
    """Complete detection frame with all detections."""
    frame_id: str
    timestamp: float
    detections: List[Detection]
    image_shape: Tuple[int, int]
    processing_time_ms: float
    
    def get_by_class(self, class_name: str) -> List[Detection]:
        """Get detections filtered by class name."""
        return [d for d in self.detections if d.class_name == class_name]
    
    def get_by_confidence(self, threshold: float) -> List[Detection]:
        """Get detections above confidence threshold."""
        return [d for d in self.detections if d.confidence >= threshold]


class SORTTracker:
    """Simple Online Realtime Tracking (SORT) implementation."""
    
    def __init__(self, max_age: int = 30, min_hits: int = 3, iou_threshold: float = 0.3):
        self.max_age = max_age
        self.min_hits = min_hits
        self.iou_threshold = iou_threshold
        self.trackers: Dict[int, Dict] = {}
        self.next_id = 0
        self.frame_count = 0
        
    def iou(self, bbox1: Tuple, bbox2: Tuple) -> float:
        """Calculate IoU between two bounding boxes."""
        x1 = max(bbox1[0], bbox2[0])
        y1 = max(bbox1[1], bbox2[1])
        x2 = min(bbox1[2], bbox2[2])
        y2 = min(bbox1[3], bbox2[3])
        
        inter_area = max(0, x2 - x1) * max(0, y2 - y1)
        bbox1_area = (bbox1[2] - bbox1[0]) * (bbox1[3] - bbox1[1])
        bbox2_area = (bbox2[2] - bbox2[0]) * (bbox2[3] - bbox2[1])
        
        return inter_area / (bbox1_area + bbox2_area - inter_area + 1e-6)
    
    def update(self, detections: List[Detection]) -> List[Detection]:
        """Update trackers with new detections."""
        self.frame_count += 1
        
        # Match detections to existing trackers
        matched = set()
        used_trackers = set()
        
        # Simple Hungarian-like matching based on IoU
        for det in detections:
            best_iou = self.iou_threshold
            best_tracker = None
            
            for tid, tracker in self.trackers.items():
                if tid in used_trackers:
                    continue
                iou = self.iou(det.bbox, tracker['bbox'])
                if iou > best_iou:
                    best_iou = iou
                    best_tracker = tid
            
            if best_tracker is not None:
                det.tracking_id = best_tracker
                self.trackers[best_tracker] = {
                    'bbox': det.bbox,
                    'hits': self.trackers[best_tracker]['hits'] + 1,
                    'age': 0
                }
                matched.add(det)
                used_trackers.add(best_tracker)
            else:
                # Create new tracker
                det.tracking_id = self.next_id
                self.trackers[self.next_id] = {
                    'bbox': det.bbox,
                    'hits': 1,
                    'age': 0
                }
                self.next_id += 1
        
        # Update unmatched trackers
        for tid in list(self.trackers.keys()):
            if tid not in used_trackers:
                self.trackers[tid]['age'] += 1
                if self.trackers[tid]['age'] > self.max_age:
                    del self.trackers[tid]
        
        return detections


class PerceptionAgent(BaseAgent):
    """Perception Agent for camera-based object detection."""
    
    def __init__(self, agent_id: str, config: PerceptionConfig, message_bus):
        super().__init__(agent_id, "perception", message_bus)
        self.config = config
        self.logger = AsyncLogger(f"perception_{agent_id}")
        
        # Model components
        self.session = None
        self.input_name = None
        self.input_shape = None
        self.class_names = config.classes_of_interest
        
        # Tracking
        self.tracker = SORTTracker(
            max_age=config.track_history,
            min_hits=3,
            iou_threshold=0.3
        ) if config.tracking_enabled else None
        
        # State
        self.latest_detections: Optional[DetectionFrame] = None
        self.detection_history: List[DetectionFrame] = []
        self._detection_task: Optional[asyncio.Task] = None
        self._running = False
        
    async def initialize(self) -> bool:
        """Initialize the perception agent."""
        await self.set_state(AgentState.INITIALIZING)
        await self.logger.info("Initializing Perception Agent...")
        
        try:
            # Import onnxruntime
            import onnxruntime as ort
            
            # Setup providers
            providers = []
            if self.config.onnx_runtime == "cuda":
                providers = ['CUDAExecutionProvider', 'CPUExecutionProvider']
            elif self.config.onnx_runtime == "tensorrt":
                providers = ['TensorrtExecutionProvider', 'CUDAExecutionProvider', 'CPUExecutionProvider']
            else:
                providers = ['CPUExecutionProvider']
            
            # Load model
            self.session = ort.InferenceSession(self.config.model_path, providers=providers)
            self.input_name = self.session.get_inputs()[0].name
            self.input_shape = self.session.get_inputs()[0].shape
            
            await self.logger.info(f"Loaded ONNX model: {self.config.model_path}")
            await self.logger.info(f"Input shape: {self.input_shape}")
            await self.logger.info(f"Providers: {providers}")
            
            # Subscribe to message bus
            await self._message_bus.subscribe(self.agent_id, self._message_handler)
            
            await self.set_state(AgentState.IDLE)
            return True
            
        except Exception as e:
            await self.logger.error(f"Initialization failed: {e}")
            await self.set_state(AgentState.ERROR)
            return False
    
    async def process(self, message: AgentMessage) -> Optional[AgentMessage]:
        """Process incoming messages."""
        msg_type = message.message_type
        payload = message.payload
        
        if msg_type == "DETECT_REQUEST":
            # On-demand detection request
            frame_data = payload.get("frame_data")
            if frame_data is not None:
                detections = await self.detect(frame_data)
                return AgentMessage(
                    source=self.agent_id,
                    target=message.source,
                    message_type="DETECTION_RESULT",
                    payload={
                        "detections": self._detections_to_dict(detections),
                        "request_id": payload.get("request_id")
                    }
                )
        
        elif msg_type == "START_DETECTION":
            camera_source = payload.get("camera_source", self.config.camera_source)
            await self.start_detection_loop(camera_source)
            return AgentMessage(
                source=self.agent_id,
                target=message.source,
                message_type="DETECTION_STARTED",
                payload={"camera_source": camera_source}
            )
        
        elif msg_type == "STOP_DETECTION":
            await self.stop_detection_loop()
            return AgentMessage(
                source=self.agent_id,
                target=message.source,
                message_type="DETECTION_STOPPED"
            )
        
        elif msg_type == "GET_DETECTIONS":
            return AgentMessage(
                source=self.agent_id,
                target=message.source,
                message_type="DETECTION_RESULT",
                payload={
                    "detections": self._detections_to_dict(self.latest_detections) 
                    if self.latest_detections else None
                }
            )
        
        elif msg_type == "UPDATE_CLASSES":
            self.class_names = payload.get("classes", self.class_names)
            await self.logger.info(f"Updated classes of interest: {self.class_names}")
        
        return None
    
    async def detect(self, frame: np.ndarray) -> DetectionFrame:
        """Run YOLO inference on a frame."""
        start_time = time.time()
        
        try:
            # Preprocess
            input_tensor = self._preprocess(frame)
            
            # Run inference
            outputs = self.session.run(None, {self.input_name: input_tensor})
            
            # Postprocess
            detections = self._postprocess(outputs, frame.shape[:2])
            
            # Apply tracking
            if self.tracker:
                detections = self.tracker.update(detections)
            
            # Create detection frame
            det_frame = DetectionFrame(
                frame_id=str(uuid.uuid4()),
                timestamp=time.time(),
                detections=detections,
                image_shape=frame.shape[:2],
                processing_time_ms=(time.time() - start_time) * 1000
            )
            
            # Update state
            self.latest_detections = det_frame
            self.detection_history.append(det_frame)
            if len(self.detection_history) > self.config.track_history:
                self.detection_history.pop(0)
            
            # Update metrics
            await self.update_metric("detection_count", len(detections))
            await self.update_metric("processing_time_ms", det_frame.processing_time_ms)
            await self.update_metric("fps", 1000.0 / max(det_frame.processing_time_ms, 1))
            
            return det_frame
            
        except Exception as e:
            await self.logger.error(f"Detection error: {e}")
            return DetectionFrame(
                frame_id=str(uuid.uuid4()),
                timestamp=time.time(),
                detections=[],
                image_shape=frame.shape[:2] if frame is not None else (0, 0),
                processing_time_ms=(time.time() - start_time) * 1000
            )
    
    def _preprocess(self, frame: np.ndarray) -> np.ndarray:
        """Preprocess frame for YOLO inference."""
        # Resize
        input_size = self.config.input_size
        resized = cv2.resize(frame, input_size)
        
        # Normalize
        normalized = resized.astype(np.float32) / 255.0
        
        # HWC to CHW
        transposed = np.transpose(normalized, (2, 0, 1))
        
        # Add batch dimension
        batched = np.expand_dims(transposed, axis=0)
        
        return batched
    
    def _postprocess(self, outputs: List[np.ndarray], 
                     image_shape: Tuple[int, int]) -> List[Detection]:
        """Post-process YOLO outputs to detections."""
        detections = []
        
        # Parse outputs (Yolov8 format)
        predictions = outputs[0]  # Shape: [batch, num_boxes, num_classes + 5]
        
        for pred in predictions[0]:  # First batch
            confidence = pred[4]
            if confidence < self.config.detection_threshold:
                continue
            
            class_scores = pred[5:]
            class_id = int(np.argmax(class_scores))
            class_confidence = class_scores[class_id]
            
            if class_id >= len(self.class_names):
                continue
            
            # Convert normalized coordinates to image coordinates
            x_center, y_center, width, height = pred[0:4]
            x1 = (x_center - width / 2) / self.config.input_size[0]
            y1 = (y_center - height / 2) / self.config.input_size[1]
            x2 = (x_center + width / 2) / self.config.input_size[0]
            y2 = (y_center + height / 2) / self.config.input_size[1]
            
            detections.append(Detection(
                bbox=(x1, y1, x2, y2),
                class_id=class_id,
                class_name=self.class_names[class_id],
                confidence=float(confidence * class_confidence)
            ))
        
        # Apply NMS
        detections = self._nms(detections)
        
        # Limit max detections
        return detections[:self.config.max_detections]
    
    def _nms(self, detections: List[Detection], iou_threshold: float = 0.5) -> List[Detection]:
        """Apply Non-Maximum Suppression."""
        if not detections:
            return []
        
        # Sort by confidence
        sorted_dets = sorted(detections, key=lambda x: x.confidence, reverse=True)
        
        keep = []
        while sorted_dets:
            current = sorted_dets.pop(0)
            keep.append(current)
            
            sorted_dets = [
                d for d in sorted_dets 
                if self.tracker.iou(current.bbox, d.bbox) < iou_threshold
            ]
        
        return keep
    
    async def start_detection_loop(self, camera_source: str = None):
        """Start continuous detection loop."""
        if self._running:
            await self.logger.warning("Detection loop already running")
            return
        
        self._running = True
        await self.set_state(AgentState.RUNNING)
        
        self._detection_task = asyncio.create_task(
            self._detection_loop(camera_source or self.config.camera_source)
        )
        
        await self.logger.info(f"Started detection loop with source: {camera_source}")
    
    async def stop_detection_loop(self):
        """Stop continuous detection loop."""
        self._running = False
        
        if self._detection_task:
            self._detection_task.cancel()
            try:
                await self._detection_task
            except asyncio.CancelledError:
                pass
            self._detection_task = None
        
        await self.set_state(AgentState.IDLE)
        await self.logger.info("Stopped detection loop")
    
    async def _detection_loop(self, camera_source: str):
        """Main detection loop."""
        try:
            # Import cv2 for camera capture
            import cv2
            
            cap = cv2.VideoCapture(camera_source)
            if not cap.isOpened():
                await self.logger.error(f"Failed to open camera: {camera_source}")
                await self.set_state(AgentState.ERROR)
                return
            
            while self._running:
                ret, frame = cap.read()
                if not ret:
                    await self.logger.warning("Failed to capture frame")
                    await asyncio.sleep(0.01)
                    continue
                
                # Run detection
                detections = await self.detect(frame)
                
                # Broadcast results
                await self.broadcast(
                    payload={
                        "type": "detection",
                        "frame_id": detections.frame_id,
                        "detection_count": len(detections.detections),
                        "detections": self._detections_to_dict(detections),
                        "processing_time_ms": detections.processing_time_ms
                    },
                    message_type="DETECTION",
                    priority=MessagePriority.NORMAL
                )
                
                # Rate limiting
                await asyncio.sleep(self.config.inference_interval)
                
        except asyncio.CancelledError:
            await self.logger.info("Detection loop cancelled")
        except Exception as e:
            await self.logger.error(f"Detection loop error: {e}")
            await self.set_state(AgentState.ERROR)
        finally:
            if 'cap' in locals():
                cap.release()
    
    def _detections_to_dict(self, det_frame: DetectionFrame) -> Dict[str, Any]:
        """Convert detection frame to dictionary."""
        if det_frame is None:
            return None
        
        return {
            "frame_id": det_frame.frame_id,
            "timestamp": det_frame.timestamp,
            "image_shape": det_frame.image_shape,
            "processing_time_ms": det_frame.processing_time_ms,
            "detections": [
                {
                    "bbox": d.bbox,
                    "class_id": d.class_id,
                    "class_name": d.class_name,
                    "confidence": d.confidence,
                    "tracking_id": d.tracking_id
                }
                for d in det_frame.detections
            ]
        }
    
    async def get_detections(self) -> Optional[DetectionFrame]:
        """Get latest detections."""
        return self.latest_detections
    
    async def shutdown(self) -> bool:
        """Shutdown the perception agent."""
        await self.set_state(AgentState.SHUTTING_DOWN)
        await self.logger.info("Shutting down Perception Agent...")
        
        await self.stop_detection_loop()
        
        if self.session:
            del self.session
            self.session = None
        
        await self.set_state(AgentState.SHUTDOWN)
        await self.logger.info("Perception Agent shutdown complete")
        return True


# Import cv2 for preprocessing
try:
    import cv2
except ImportError:
    cv2 = None
