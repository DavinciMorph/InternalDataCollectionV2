"""Shared data types used across the application."""
from dataclasses import dataclass, field
from typing import List, Optional, Dict


@dataclass
class PortConfig:
    """Configuration for one SPI port, received from Pi metadata."""
    name: str
    num_devices: int


@dataclass
class StreamMetadata:
    """Metadata received from the Pi TCP server on connect."""
    format: str
    compression: str
    batch_size: int
    sample_rate: int
    num_channels: int
    num_devices: int
    ports: List[str]
    port_config: List[PortConfig]
    sample_size: int
    sample_struct: str

    @classmethod
    def from_json(cls, data: dict) -> "StreamMetadata":
        port_configs = [
            PortConfig(name=pc["name"], num_devices=pc["num_devices"])
            for pc in data.get("port_config", [])
        ]
        return cls(
            format=data.get("format", "binary_raw"),
            compression=data.get("compression", "none"),
            batch_size=data.get("batch_size", 1),
            sample_rate=data.get("sample_rate", 250),
            num_channels=data.get("num_channels", 0),
            num_devices=data.get("num_devices", 0),
            ports=data.get("ports", []),
            port_config=port_configs,
            sample_size=data.get("sample_size", 0),
            sample_struct=data.get("sample_struct", ""),
        )


@dataclass
class Sample:
    """One sample from the acquisition system."""
    timestamp: float
    sample_number: int
    channels: List[int]


@dataclass
class PortStatus:
    """Status of a single SPI port during init."""
    name: str
    num_devices: int
    status: str = "waiting"  # waiting, initializing, recovering, ready, failed
    tier: int = 0
    attempt: int = 0
    max_attempts: int = 0


@dataclass
class InitProgress:
    """Overall initialization progress."""
    phase: str = ""
    detail: str = ""
    percent: float = 0.0
    ports: Dict[str, PortStatus] = field(default_factory=dict)
    retry_count: int = 0
    max_retries: int = 3


@dataclass
class SessionConfig:
    """Configuration for one experiment session."""
    methodology_id: int = 1
    participant_id: str = ""
    session_id: str = ""
    hand: str = "Right"
    trials_per_gesture: int = 16
    iti_duration: float = 0.5
    block_count: int = 8
    break_duration: int = 30
    audio_cues: bool = False
    output_dir: str = ""
    latin_square_num: int = 1
    randomization_seed: int = 0

    def methodology(self):
        """Return the MethodologyDef for this config's methodology_id."""
        from eeg_app.experiment.methodology import METHODOLOGIES
        return METHODOLOGIES[self.methodology_id]
