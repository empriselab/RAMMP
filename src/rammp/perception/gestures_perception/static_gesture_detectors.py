import time

function_name_to_label = {
    "mouth_open": "mouth open",
    "head_nod": "head nod",
}

# jawOpen blendshape score above which the mouth is considered open.
MOUTH_OPEN_THRESHOLD = 0.4


def lip_gap_ratio(face_keypoints):
    """Inner-lip gap / face height from the 478 MediaPipe landmarks (diagnostic only).

    Pose-robust alternative to jawOpen: ~0.15-0.17 on logged open-mouth frames
    whose jawOpen ranged 0.38-0.84. Returns None if keypoints are unavailable.
    """
    if face_keypoints is None or len(face_keypoints) < 478:
        return None
    kp = face_keypoints
    gap = ((kp[13][0] - kp[14][0]) ** 2 + (kp[13][1] - kp[14][1]) ** 2) ** 0.5
    face_h = ((kp[10][0] - kp[152][0]) ** 2 + (kp[10][1] - kp[152][1]) ** 2) ** 0.5
    return gap / face_h if face_h > 0 else None


def mouth_open(perception_interface, termination_event, timeout):
    """Block until the user opens their mouth, the timeout elapses, or cancel.

    Returns True if a mouth-open gesture was detected, False otherwise.
    """
    start_time = time.time()
    last_log_time = 0.0
    frames = failed = 0
    while time.time() - start_time < timeout and (
        termination_event is None or not termination_event.is_set()
    ):
        head_perception_data = perception_interface.run_head_perception()
        frames += 1
        if head_perception_data is None:
            failed += 1
            score = None
        else:
            score = head_perception_data["jaw_open_score"]
            ratio = lip_gap_ratio(head_perception_data.get("face_keypoints"))
            if score > MOUTH_OPEN_THRESHOLD:
                print(
                    f"[mouth_open] detected: jawOpen={score:.2f} > {MOUTH_OPEN_THRESHOLD} "
                    f"(lip_gap_ratio={'n/a' if ratio is None else f'{ratio:.3f}'})",
                    flush=True,
                )
                return True
        if time.time() - last_log_time >= 1.0:
            last_log_time = time.time()
            score_str = "n/a" if score is None else f"{score:.2f}"
            ratio_str = "n/a" if score is None or ratio is None else f"{ratio:.3f}"
            print(
                f"[mouth_open] jawOpen={score_str} (threshold {MOUTH_OPEN_THRESHOLD}) "
                f"lip_gap_ratio={ratio_str}; "
                f"{failed}/{frames} frames returned no head perception",
                flush=True,
            )
    print(f"[mouth_open] gave up after {frames} frames ({failed} failed)", flush=True)
    return False
