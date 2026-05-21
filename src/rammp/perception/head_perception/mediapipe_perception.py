"""MediaPipe-based head perception — replacement for the DECA pipeline.

Detects face landmarks with MediaPipe Face Landmarker, back-projects a rigid
subset through the depth image, and tracks head motion relative to a one-time
calibration to produce a cup-tip target pose.
"""

import os

import cv2
import numpy as np
from mediapipe import Image, ImageFormat
from mediapipe.tasks import python as mp_python
from mediapipe.tasks.python import vision as mp_vision

from rammp.perception.head_perception import head_geometry as hg
from rammp.perception.head_perception.landmark_indices import (
    JAW_OPEN_BLENDSHAPE,
    RIGID_LANDMARK_INDICES,
)
from rammp.utils.timing import timer

_THIS_DIR = os.path.dirname(os.path.abspath(__file__))
_DEFAULT_MODEL_PATH = os.path.join(_THIS_DIR, "models", "face_landmarker.task")
_CONFIG_DIR = os.path.join(_THIS_DIR, "mediapipe_config")

# Minimum number of rigid landmarks with valid depth required to trust a frame.
_MIN_VALID_LANDMARKS = 60


class MediaPipeHeadPerception:
    """Head-pose / cup-target perception backed by MediaPipe Face Landmarker."""

    def __init__(self, model_path: str | None = None) -> None:
        model_path = model_path or _DEFAULT_MODEL_PATH
        options = mp_vision.FaceLandmarkerOptions(
            base_options=mp_python.BaseOptions(model_asset_path=model_path),
            output_face_blendshapes=True,
            output_facial_transformation_matrixes=False,
            num_faces=1,
            running_mode=mp_vision.RunningMode.IMAGE,
        )
        self._detector = mp_vision.FaceLandmarker.create_from_options(options)

        self.tool: str | None = None
        self._rigid_indices = np.array(RIGID_LANDMARK_INDICES, dtype=int)
        self._smoother = hg.TransformSmoother()

        self.reference_points: np.ndarray | None = None
        self.reference_head_frame: np.ndarray | None = None
        self.tool_tip_transform: np.ndarray | None = None

    def set_tool(self, tool: str) -> None:
        """Load the calibration files recorded for the given tool."""
        self.tool = tool
        self._smoother = hg.TransformSmoother()
        tool_dir = os.path.join(_CONFIG_DIR, tool)
        required = {
            "reference_points": "reference_landmarks_camera.npy",
            "reference_head_frame": "reference_head_frame.npy",
            "tool_tip_transform": "tool_tip_transform.npy",
        }
        missing = [
            name
            for name in required.values()
            if not os.path.exists(os.path.join(tool_dir, name))
        ]
        if missing:
            raise FileNotFoundError(
                f"Head-perception calibration for tool '{tool}' is missing "
                f"{missing} in {tool_dir}. Run the calibration first:\n"
                f"  python -m rammp.perception.head_perception.calibrate_head "
                f"--tool {tool}"
            )
        self.reference_points = np.load(
            os.path.join(tool_dir, required["reference_points"])
        )
        self.reference_head_frame = np.load(
            os.path.join(tool_dir, required["reference_head_frame"])
        )
        self.tool_tip_transform = np.load(
            os.path.join(tool_dir, required["tool_tip_transform"])
        )
        expected_rows = len(self._rigid_indices)
        if self.reference_points.shape != (expected_rows, 3):
            raise ValueError(
                f"reference_landmarks_camera.npy for tool '{tool}' has shape "
                f"{self.reference_points.shape}, expected ({expected_rows}, 3). "
                f"Re-run the calibration."
            )

    def detect_landmarks(self, bgr_image: np.ndarray):
        """Run MediaPipe on a BGR image.

        Returns (landmarks_px (478, 2), jaw_open_score float) or None if no
        face was detected.
        """
        rgb_image = cv2.cvtColor(bgr_image, cv2.COLOR_BGR2RGB)
        mp_image = Image(image_format=ImageFormat.SRGB, data=rgb_image)
        with timer("head/mediapipe_detect"):
            result = self._detector.detect(mp_image)
        if not result.face_landmarks:
            return None

        height, width = bgr_image.shape[:2]
        landmarks = result.face_landmarks[0]
        landmarks_px = np.array(
            [[lm.x * width, lm.y * height] for lm in landmarks], dtype=np.float64
        )

        jaw_open_score = 0.0
        if result.face_blendshapes:
            for category in result.face_blendshapes[0]:
                if category.category_name == JAW_OPEN_BLENDSHAPE:
                    jaw_open_score = float(category.score)
                    break

        return landmarks_px, jaw_open_score

    def rigid_landmark_points(self, bgr_image, depth_image, camera_info):
        """Back-project the rigid landmark subset to camera-frame 3D points.

        Returns (rigid_points (N, 3) with NaN for invalid, landmarks_px,
        jaw_open_score) or None if no face was detected.
        """
        detection = self.detect_landmarks(bgr_image)
        if detection is None:
            return None
        landmarks_px, jaw_open_score = detection

        rigid_px = landmarks_px[self._rigid_indices]
        rigid_points = hg.backproject_landmarks(
            rigid_px,
            depth_image,
            fx=camera_info.k[0],
            fy=camera_info.k[4],
            cx=camera_info.k[2],
            cy=camera_info.k[5],
        )
        return rigid_points, landmarks_px, jaw_open_score

    def run(self, bgr_image, camera_info, depth_image, base_to_camera: np.ndarray | None):
        """Run one head-perception cycle.

        Returns a dict with keys head_pose, tool_tip_target_pose, landmarks2d,
        jaw_open_score, noisy_reading — or None if perception failed this frame.
        """
        if self.reference_points is None:
            raise RuntimeError(
                "MediaPipeHeadPerception.set_tool() must be called before run()."
            )
        if base_to_camera is None:
            return None

        result = self.rigid_landmark_points(bgr_image, depth_image, camera_info)
        if result is None:
            return None
        rigid_points, landmarks_px, jaw_open_score = result

        valid = ~np.isnan(rigid_points).any(axis=1) & ~np.isnan(
            self.reference_points
        ).any(axis=1)
        if int(valid.sum()) < _MIN_VALID_LANDMARKS:
            return None

        rotation, translation, _ = hg.kabsch_with_rejection(
            rigid_points[valid], self.reference_points[valid]
        )
        trans = hg.make_transform(rotation, translation)
        trans, noisy_reading = self._smoother.update(trans)

        tool_tip_target_camera = trans @ self.tool_tip_transform
        tool_tip_target_base = base_to_camera @ tool_tip_target_camera

        head_frame_camera = trans @ self.reference_head_frame
        head_frame_base = base_to_camera @ head_frame_camera

        return {
            "head_pose": hg.head_frame_to_pose(head_frame_base),
            "tool_tip_target_pose": tool_tip_target_base,  # 4x4 ndarray, base frame
            "landmarks2d": landmarks_px,
            "jaw_open_score": jaw_open_score,
            "noisy_reading": noisy_reading,
        }
