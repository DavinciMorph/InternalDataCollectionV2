"""Trial sequencer — generates balanced, shuffled experiment trial sequences.

Ported from data_collection.py TrialSequencer class.
"""
import hashlib
import random

from eeg_app.core.constants import GESTURES
from eeg_app.core.types import SessionConfig


class TrialSequencer:
    """Generates one shuffled session with equal gesture counts, split into blocks.

    Gestures are balanced across the whole session (not per-block) and
    randomly dispersed. Blocks are equal-sized time chunks.
    """

    def __init__(self, config: SessionConfig):
        self.config = config
        self.blocks: list[list[dict]] = []
        self._total_active = 0
        self._generate()

    def _generate(self):
        seed_raw = f"{self.config.participant_id}_{self.config.session_id}"
        seed = int(hashlib.sha256(seed_raw.encode()).hexdigest()[:8], 16)
        rng = random.Random(seed)

        # Build full gesture list -- equal counts, shuffled across session
        total_per_gesture = self.config.trials_per_gesture
        all_gestures = []
        for g in GESTURES:
            all_gestures.extend([g] * total_per_gesture)
        rng.shuffle(all_gestures)

        self._total_active = len(all_gestures)

        # Interleave rest after each gesture
        all_trials = []
        for g in all_gestures:
            all_trials.append({"gesture": g, "is_rest": False})
            all_trials.append({"gesture": "Rest", "is_rest": True})

        # Split into blocks (pairs stay together: gesture + its rest)
        num_blocks = self.config.block_count
        pairs_total = len(all_gestures)
        pairs_per_block = max(1, pairs_total // num_blocks)

        self.blocks = []
        idx = 0
        for b in range(num_blocks):
            if b == num_blocks - 1:
                # Last block gets remainder
                block = all_trials[idx:]
            else:
                count = pairs_per_block * 2  # 2 entries per pair
                block = all_trials[idx:idx + count]
                idx += count
            if block:
                self.blocks.append(block)

    def total_active_trials(self) -> int:
        return self._total_active

    def total_trials_per_block(self) -> int:
        if not self.blocks:
            return 0
        # Return count of gesture (non-rest) trials in first block
        return sum(1 for t in self.blocks[0] if not t["is_rest"])

    def flat_trial_count(self) -> int:
        return sum(len(b) for b in self.blocks)
