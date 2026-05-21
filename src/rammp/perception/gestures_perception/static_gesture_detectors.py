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
    while time.time() - start_time < timeout and (
        termination_event is None or not termination_event.is_set()
    ):
        head_perception_data = perception_interface.run_head_perception()
        if head_perception_data is None:
            continue
        if head_perception_data["jaw_open_score"] > MOUTH_OPEN_THRESHOLD:
            return True
    return False
