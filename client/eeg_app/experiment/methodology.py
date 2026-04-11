"""Methodology definitions and session timing helpers.

Ported from data_collection.py — defines the six BCI gesture paradigm
methodologies (M1-M6) and helper functions for computing default session
timing parameters.
"""
from dataclasses import dataclass

from eeg_app.core.constants import GESTURES


# ---------------------------------------------------------------------------
# Methodology definition
# ---------------------------------------------------------------------------
@dataclass
class MethodologyDef:
    id: int
    name: str
    modality: str          # "physical" or "imagery"
    has_video: bool
    phases: list           # [(phase_name, duration_ms), ...]
    rest_phases: list      # phases for rest trial
    instruction_text: str
    prep_subtext: str
    execute_prefix: str    # "Perform:" or "Imagine:"


METHODOLOGIES = {
    1: MethodologyDef(
        id=1,
        name="M1: Physical Execution (Cued)",
        modality="physical",
        has_video=False,
        phases=[("Prep", 2000), ("Go", 500), ("Execute", 4000), ("End", 300)],
        rest_phases=[("Rest", 8000)],
        instruction_text=(
            "PHYSICAL EXECUTION WITH CUES\n\n"
            "You will see the name of a hand gesture on screen.\n\n"
            "1. PREPARE phase (blue): Get ready, but do NOT move yet.\n"
            "2. GO phase (green): Begin performing the gesture with your "
            "designated hand. Hold the position.\n"
            "3. EXECUTE phase: Continue holding the gesture.\n"
            "4. STOP phase (red): Relax your hand immediately.\n"
            "5. REST phase: Keep still and relax until the next trial.\n\n"
            "The five gestures are: Open (spread fingers), Close (make a fist), "
            "Pinch (thumb to index finger), Key Grip (thumb against side "
            "of index finger), and Rotate (rotate wrist).\n\n"
            "Try to keep the rest of your body still throughout."
        ),
        prep_subtext="Prepare to perform this gesture",
        execute_prefix="Perform:",
    ),
    2: MethodologyDef(
        id=2,
        name="M2: Motor Imagery (Cued)",
        modality="imagery",
        has_video=False,
        phases=[("Prep", 2000), ("Go", 500), ("Execute", 4000), ("End", 300)],
        rest_phases=[("Rest", 8000)],
        instruction_text=(
            "MOTOR IMAGERY WITH CUES\n\n"
            "You will see the name of a hand gesture on screen.\n\n"
            "1. PREPARE phase (blue): Get ready to imagine the gesture.\n"
            "2. GO phase (green): Begin IMAGINING performing the gesture. "
            "Do NOT actually move your hand.\n"
            "3. EXECUTE phase: Continue imagining the movement vividly.\n"
            "4. STOP phase (red): Stop imagining.\n"
            "5. REST phase: Relax your mind until the next trial.\n\n"
            "Imagine the movement as vividly as possible \u2014 feel the muscles "
            "contracting, the fingers moving \u2014 but keep your hand completely "
            "still.\n\n"
            "The five gestures are: Open, Close, Pinch, Key Grip, and Rotate."
        ),
        prep_subtext="Prepare to imagine this gesture",
        execute_prefix="Imagine:",
    ),
    3: MethodologyDef(
        id=3,
        name="M3: Physical Execution (No Cue)",
        modality="physical",
        has_video=False,
        phases=[("Execute", 4000), ("End", 300)],
        rest_phases=[("Rest", 8000)],
        instruction_text=(
            "PHYSICAL EXECUTION \u2014 NO PREPARATION CUE\n\n"
            "The gesture name will appear immediately.\n\n"
            "1. When you see the gesture name (green), perform it right away "
            "and hold the position.\n"
            "2. STOP phase (red): Relax your hand immediately.\n"
            "3. REST phase: Keep still and relax.\n\n"
            "There is no preparation phase \u2014 react as quickly as you can.\n\n"
            "The five gestures are: Open, Close, Pinch, Key Grip, and Rotate."
        ),
        prep_subtext="",
        execute_prefix="Perform:",
    ),
    4: MethodologyDef(
        id=4,
        name="M4: Motor Imagery (No Cue)",
        modality="imagery",
        has_video=False,
        phases=[("Execute", 4000), ("End", 300)],
        rest_phases=[("Rest", 8000)],
        instruction_text=(
            "MOTOR IMAGERY \u2014 NO PREPARATION CUE\n\n"
            "The gesture name will appear immediately.\n\n"
            "1. When you see the gesture name (green), begin IMAGINING the "
            "gesture. Do NOT move your hand.\n"
            "2. STOP phase (red): Stop imagining.\n"
            "3. REST phase: Relax your mind.\n\n"
            "React as quickly as you can when the gesture appears.\n\n"
            "The five gestures are: Open, Close, Pinch, Key Grip, and Rotate."
        ),
        prep_subtext="",
        execute_prefix="Imagine:",
    ),
    5: MethodologyDef(
        id=5,
        name="M5: Physical + Video Demo",
        modality="physical",
        has_video=True,
        phases=[("Prep", 2000), ("Execute", 4000), ("End", 300)],
        rest_phases=[("Rest", 8000)],
        instruction_text=(
            "PHYSICAL EXECUTION WITH VIDEO DEMONSTRATION\n\n"
            "You will see a video demonstration of each gesture.\n\n"
            "1. PREPARE phase (blue): Watch the demonstration video.\n"
            "2. EXECUTE phase (green): Perform the gesture as shown in the "
            "video. Hold the position.\n"
            "3. STOP phase (red): Relax your hand.\n"
            "4. REST phase: Keep still and relax.\n\n"
            "Match the gesture shown in the video as closely as possible.\n\n"
            "The five gestures are: Open, Close, Pinch, Key Grip, and Rotate."
        ),
        prep_subtext="Watch the demonstration",
        execute_prefix="Perform:",
    ),
    6: MethodologyDef(
        id=6,
        name="M6: Imagery + Video Demo",
        modality="imagery",
        has_video=True,
        phases=[("Prep", 2000), ("Execute", 4000), ("End", 300)],
        rest_phases=[("Rest", 8000)],
        instruction_text=(
            "MOTOR IMAGERY WITH VIDEO DEMONSTRATION\n\n"
            "You will see a video demonstration of each gesture.\n\n"
            "1. PREPARE phase (blue): Watch the demonstration video.\n"
            "2. EXECUTE phase (green): IMAGINE performing the gesture as "
            "shown. Do NOT actually move.\n"
            "3. STOP phase (red): Stop imagining.\n"
            "4. REST phase: Relax your mind.\n\n"
            "Watch the video carefully, then imagine the movement vividly.\n\n"
            "The five gestures are: Open, Close, Pinch, Key Grip, and Rotate."
        ),
        prep_subtext="Watch the demonstration",
        execute_prefix="Imagine:",
    ),
}


# ---------------------------------------------------------------------------
# Session timing helpers
# ---------------------------------------------------------------------------
SESSION_DURATION_S = 1200    # 20 minutes
BLOCK_DURATION_S = 120       # 2 minutes
BREAK_DURATION_S = 30        # 30 seconds between blocks
DEFAULT_ITI_S = 0.5


def compute_session_defaults(methodology_id: int, iti: float = DEFAULT_ITI_S):
    """Compute trials_per_gesture and block_count for a ~20-min session."""
    m = METHODOLOGIES[methodology_id]
    # Trial cycle = gesture phases + rest phases + 2 ITIs
    gesture_time = sum(d / 1000 for _, d in m.phases)
    rest_time = sum(d / 1000 for _, d in m.rest_phases)
    cycle_time = gesture_time + rest_time + 2 * iti

    # How many trials fit in one 2-min block
    trials_per_block = max(1, int(BLOCK_DURATION_S / cycle_time))

    # How many blocks fit in 20 min (including 30s breaks)
    # N_blocks * block_time + (N_blocks - 1) * break = session_time
    block_active_time = trials_per_block * cycle_time
    block_count = max(1, round(
        (SESSION_DURATION_S + BREAK_DURATION_S)
        / (block_active_time + BREAK_DURATION_S)
    ))

    # Total trials, rounded to nearest multiple of gesture count
    total_trials = trials_per_block * block_count
    n = len(GESTURES)
    total_trials = max(n, (total_trials // n) * n)
    trials_per_gesture = total_trials // n

    return trials_per_gesture, block_count
