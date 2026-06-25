"""
Episode Writer
==============
Single-responsibility module for serialising recorded episodes to disk.

Separated from the node to keep the recorder node focused on data capture
and the writer focused on persistence and quality validation. This follows
the Single Responsibility principle (one module, one reason to change).

Episode structure on disk (JSON):

    {
        "metadata": { ... },
        "timesteps": [ { "timestamp_ns": ..., "observation": {...}, "action": {...} }, ... ]
    }
"""

from __future__ import annotations

import json
import logging
import os
from datetime import datetime
from typing import Any, Dict, List, Optional

import numpy as np


logger = logging.getLogger(__name__)


class Episode:
    """Structured container that accumulates timesteps and metadata.

    Each episode represents one demonstration recording session with
    observations (sensors) and actions (commands) at every timestep,
    plus rich metadata for downstream training pipelines.
    """

    def __init__(
        self,
        episode_id: int,
        task_name: str,
        operator_name: str,
        start_time: Optional[datetime] = None,
    ) -> None:
        self.episode_id = episode_id
        self.task_name = task_name
        self.operator_name = operator_name
        self.start_time = start_time or datetime.now()
        self.timesteps: List[Dict[str, Any]] = []
        self.dropped_frames: int = 0
        self.sync_offsets_ms: List[float] = []
        self._start_monotonic: Optional[float] = None

    def set_start_monotonic(self, t: float) -> None:
        """Record the monotonic clock value when recording began."""
        self._start_monotonic = t

    @property
    def duration_sec(self) -> float:
        """Elapsed time since episode start (monotonic)."""
        import time
        if self._start_monotonic is None:
            return 0.0
        return time.monotonic() - self._start_monotonic

    @property
    def num_timesteps(self) -> int:
        return len(self.timesteps)

    def add_timestep(self, timestep: Dict[str, Any]) -> None:
        self.timesteps.append(timestep)

    def to_dict(self, success_label: bool, notes: str = "") -> Dict[str, Any]:
        """Serialise the episode to a plain dict suitable for JSON encoding."""
        avg_sync = (
            float(np.mean(self.sync_offsets_ms))
            if self.sync_offsets_ms
            else 0.0
        )
        return {
            "metadata": {
                "episode_id": self.episode_id,
                "task_name": self.task_name,
                "operator": self.operator_name,
                "start_time": self.start_time.isoformat(),
                "duration_sec": round(self.duration_sec, 3),
                "success": success_label,
                "notes": notes,
                "num_timesteps": self.num_timesteps,
                "dropped_frames": self.dropped_frames,
                "avg_sync_offset_ms": round(avg_sync, 3),
            },
            "timesteps": self.timesteps,
        }


class EpisodeWriter:
    """Handles episode persistence and quality validation.

    Quality gate checks are applied before saving:
      - Minimum timestep count
      - Maximum dropped frame ratio

    Episodes are saved as JSON with a filename that includes the episode ID
    and wall-clock start time for easy identification.
    """

    def __init__(
        self,
        save_directory: str,
        min_episode_timesteps: int = 10,
        max_dropped_frame_ratio: float = 0.05,
    ) -> None:
        self._save_dir = save_directory
        self._min_timesteps = min_episode_timesteps
        self._max_drop_ratio = max_dropped_frame_ratio
        os.makedirs(self._save_dir, exist_ok=True)

    def save(
        self,
        episode: Episode,
        success_label: bool,
        notes: str = "",
    ) -> str:
        """Validate and save the episode to disk.

        Returns the file path on success, or an empty string on failure.
        Quality warnings are logged but do not prevent saving.
        """
        self._run_quality_checks(episode)

        ts_str = episode.start_time.strftime("%Y%m%d_%H%M%S")
        filename = f"episode_{episode.episode_id:04d}_{ts_str}.json"
        filepath = os.path.join(self._save_dir, filename)

        episode_dict = episode.to_dict(
            success_label=success_label, notes=notes
        )
        try:
            with open(filepath, "w") as fh:
                json.dump(episode_dict, fh, indent=2)
            logger.info("Episode saved to %s", filepath)
        except OSError as exc:
            logger.error("Failed to save episode: %s", exc)
            return ""

        return filepath

    def _run_quality_checks(self, episode: Episode) -> None:
        """Log warnings for episodes that fail quality gates."""
        if episode.num_timesteps < self._min_timesteps:
            logger.warning(
                "Episode has only %d timesteps (minimum: %d). "
                "Saving anyway with quality warning.",
                episode.num_timesteps,
                self._min_timesteps,
            )

        total = episode.num_timesteps + episode.dropped_frames
        if total > 0:
            drop_ratio = episode.dropped_frames / total
            if drop_ratio > self._max_drop_ratio:
                logger.warning(
                    "Dropped frame ratio %.1f%% exceeds threshold %.1f%%.",
                    drop_ratio * 100,
                    self._max_drop_ratio * 100,
                )
