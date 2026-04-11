"""Application configuration — load/save from JSON."""
import json
import os
from dataclasses import dataclass, asdict


_CONFIG_PATH = os.path.join(os.path.dirname(os.path.dirname(__file__)), "config.json")


@dataclass
class AppConfig:
    # Pi connection
    pi_host: str = "10.42.0.1"
    ssh_user: str = "morph"
    ssh_password: str = "morph"
    tcp_port: int = 8888
    binary_path: str = "~/ads1299-cpp/build/ads1299_acquire"

    # Acquisition
    default_sps: int = 250
    extra_args: str = ""

    # Display
    window_seconds: float = 5.0
    y_range_uv: int = 200

    # Experiment
    output_dir: str = "experiment_data"
    last_participant_id: str = ""
    default_methodology: int = 1
    default_hand: str = "Right"

    @classmethod
    def load(cls, path: str = None) -> "AppConfig":
        path = path or _CONFIG_PATH
        if not os.path.exists(path):
            return cls()
        try:
            with open(path, "r") as f:
                data = json.load(f)
            # Flatten nested structure if present
            flat = {}
            for key, val in data.items():
                if isinstance(val, dict):
                    flat.update(val)
                else:
                    flat[key] = val
            valid = {k: v for k, v in flat.items() if k in cls.__dataclass_fields__}
            return cls(**valid)
        except Exception:
            return cls()

    def save(self, path: str = None):
        path = path or _CONFIG_PATH
        with open(path, "w") as f:
            json.dump(asdict(self), f, indent=2)

    @property
    def binary_dir(self) -> str:
        """Directory containing the binary (for cd before execution)."""
        return os.path.dirname(self.binary_path)

    @property
    def binary_name(self) -> str:
        """Just the binary filename."""
        return os.path.basename(self.binary_path)
