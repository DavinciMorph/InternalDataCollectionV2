"""Parse ads1299_acquire stdout to extract structured status events."""
import re
from dataclasses import dataclass
from typing import Optional


@dataclass
class ParsedEvent:
    type: str
    data: dict


class StdoutParser:
    """Parses ads1299_acquire stdout lines into structured events."""

    PATTERNS = [
        ("phase1_start",     re.compile(r"PHASE 1|CONFIGURING DEVICES")),
        ("port_configured",  re.compile(r"\[OK\] All (\d+) ports configured")),
        ("phase2_start",     re.compile(r"PHASE 2|STARTING CONVERSIONS")),
        ("initial_health",   re.compile(r"Initial result:\s*(\d+)/(\d+)\s*ports healthy")),
        ("tier1_start",      re.compile(r"Tier 1.*RDATAC cycling")),
        ("tier1_attempt",    re.compile(r"(\w+):\s*Tier 1.*attempt\s*(\d+)")),
        ("tier1_recover",    re.compile(r"(\w+):\s*Tier 1 recovered on attempt (\d+)")),
        ("tier2_start",      re.compile(r"Tier 2.*software RESET")),
        ("tier2_attempt",    re.compile(r"(\w+):\s*Tier 2.*attempt\s*(\d+)")),
        ("tier2_recover",    re.compile(r"(\w+):\s*Tier 2 recovered on attempt (\d+)")),
        ("tier3_start",      re.compile(r"Tier 3.*full re-init")),
        ("system_ready",     re.compile(r"\[OK\] ALL (\d+) PORTS ACTIVE -- SYSTEM READY \((\d+) channels?\)")),
        ("init_failed",      re.compile(r"INITIALIZATION FAILED")),
        ("warmup",           re.compile(r"Warmup.*discarding")),
        ("final_health",     re.compile(r"(\w+):\s*([\d.]+)% valid.*\[(OK|WARN)\]")),
        ("stream_listen",    re.compile(r"\[STREAM\] Listening on\s*([\d.]+):(\d+)")),
        ("stream_connected", re.compile(r"\[STREAM\] Client connected")),
        ("stream_disconn",   re.compile(r"\[STREAM\] Client disconnected")),
        ("stats_line",       re.compile(r"\[\s*([\d.]+)s\]\s*samples=(\d+)\s*rate=([\d.]+)Hz")),
        ("acquisition_run",  re.compile(r"ACQUISITION RUNNING")),
        ("shutdown",         re.compile(r"^Shutdown")),
    ]

    def parse_line(self, line: str) -> Optional[ParsedEvent]:
        line = line.strip()
        if not line:
            return None

        for event_type, pattern in self.PATTERNS:
            m = pattern.search(line)
            if m:
                return ParsedEvent(type=event_type, data=self._extract(event_type, m))

        return None

    def _extract(self, event_type: str, m: re.Match) -> dict:
        if event_type == "system_ready":
            return {"ports": int(m.group(1)), "channels": int(m.group(2))}
        elif event_type == "port_configured":
            return {"count": int(m.group(1))}
        elif event_type == "initial_health":
            return {"healthy": int(m.group(1)), "total": int(m.group(2))}
        elif event_type in ("tier1_recover", "tier2_recover", "tier1_attempt", "tier2_attempt"):
            return {"port": m.group(1), "attempt": int(m.group(2))}
        elif event_type == "final_health":
            return {"port": m.group(1), "valid_pct": float(m.group(2)), "status": m.group(3)}
        elif event_type == "stream_listen":
            return {"host": m.group(1), "port": int(m.group(2))}
        elif event_type == "stats_line":
            return {"time_s": float(m.group(1)), "samples": int(m.group(2)), "rate": float(m.group(3))}
        return {}
