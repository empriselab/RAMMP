import time

function_name_to_label = {
    "mouth_open": "mouth open",
    "head_nod": "head nod",
}

# jawOpen blendshape score above which the mouth is considered open.
MOUTH_OPEN_THRESHOLD = 0.4


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
            if score > MOUTH_OPEN_THRESHOLD:
                print(f"[mouth_open] detected: jawOpen={score:.2f} > {MOUTH_OPEN_THRESHOLD}", flush=True)
                return True
        if time.time() - last_log_time >= 1.0:
            last_log_time = time.time()
            score_str = "n/a" if score is None else f"{score:.2f}"
            print(
                f"[mouth_open] jawOpen={score_str} (threshold {MOUTH_OPEN_THRESHOLD}); "
                f"{failed}/{frames} frames returned no head perception",
                flush=True,
            )
    print(f"[mouth_open] gave up after {frames} frames ({failed} failed)", flush=True)
    return False
