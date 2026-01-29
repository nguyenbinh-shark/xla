"""
Data Logger Module - Log robot data for analysis and debugging.
Records control commands, sensor data, and mode states.
"""

import csv
import json
import time
import logging
import threading
from pathlib import Path
from typing import Optional, Dict, Any, List
from dataclasses import dataclass, asdict
from datetime import datetime

from src.communication import RobotFeedback

logger = logging.getLogger(__name__)


@dataclass
class LogEntry:
    """Single log entry with all relevant data."""
    timestamp: float
    
    # Commands sent
    cmd_velocity: float = 0.0
    cmd_yaw_rate: float = 0.0
    
    # Feedback received
    fb_velocity: float = 0.0
    fb_position: float = 0.0
    fb_yaw: float = 0.0
    fb_yaw_rate: float = 0.0
    
    # Mode info
    mode: str = ""
    mode_state: str = ""
    
    # Detection info (for line following)
    line_detected: bool = False
    position_error: float = 0.0
    heading_error: float = 0.0
    confidence: float = 0.0


class DataLogger:
    """
    Logs robot data to CSV file for analysis.
    
    Usage:
        logger = DataLogger("robot_log.csv")
        logger.start()
        
        # In control loop:
        logger.log(cmd_v, cmd_y, feedback, mode_output)
        
        logger.stop()
    """
    
    def __init__(
        self, 
        filename: Optional[str] = None,
        log_dir: str = "logs",
        buffer_size: int = 100
    ):
        """
        Initialize data logger.
        
        Args:
            filename: Log file name (auto-generated if None)
            log_dir: Directory for log files
            buffer_size: Number of entries to buffer before writing
        """
        # Create log directory
        self.log_dir = Path(log_dir)
        self.log_dir.mkdir(exist_ok=True)
        
        # Generate filename if not provided
        if filename is None:
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"robot_log_{timestamp}.csv"
        
        self.filepath = self.log_dir / filename
        
        self._buffer: List[LogEntry] = []
        self._buffer_size = buffer_size
        self._lock = threading.Lock()
        self._file = None
        self._writer = None
        self._running = False
        self._start_time = 0.0
        self._entry_count = 0
        
        logger.info(f"DataLogger initialized: {self.filepath}")
    
    def start(self) -> None:
        """Start logging."""
        self._start_time = time.time()
        self._file = open(self.filepath, 'w', newline='')
        self._writer = csv.writer(self._file)
        
        # Write header
        header = [
            'time', 'cmd_v', 'cmd_yaw', 
            'fb_v', 'fb_pos', 'fb_yaw', 'fb_yaw_rate',
            'mode', 'state', 
            'line_detected', 'pos_error', 'heading_error', 'confidence'
        ]
        self._writer.writerow(header)
        
        self._running = True
        logger.info(f"Logging started: {self.filepath}")
    
    def stop(self) -> None:
        """Stop logging and flush buffer."""
        self._running = False
        
        # Flush remaining buffer
        self._flush_buffer()
        
        if self._file is not None:
            self._file.close()
            self._file = None
        
        logger.info(f"Logging stopped. {self._entry_count} entries written to {self.filepath}")
    
    def log(
        self,
        cmd_velocity: float = 0.0,
        cmd_yaw_rate: float = 0.0,
        feedback: Optional[RobotFeedback] = None,
        mode_name: str = "",
        mode_state: str = "",
        line_detected: bool = False,
        position_error: float = 0.0,
        heading_error: float = 0.0,
        confidence: float = 0.0
    ) -> None:
        """
        Log a single entry.
        
        Args:
            cmd_velocity: Commanded velocity (m/s)
            cmd_yaw_rate: Commanded yaw rate (rad/s)
            feedback: Robot feedback (optional)
            mode_name: Current mode name
            mode_state: Current mode state
            line_detected: Whether line was detected
            position_error: Line position error
            heading_error: Line heading error
            confidence: Detection confidence
        """
        if not self._running:
            return
        
        entry = LogEntry(
            timestamp=time.time() - self._start_time,
            cmd_velocity=cmd_velocity,
            cmd_yaw_rate=cmd_yaw_rate,
            mode=mode_name,
            mode_state=mode_state,
            line_detected=line_detected,
            position_error=position_error,
            heading_error=heading_error,
            confidence=confidence
        )
        
        if feedback is not None:
            entry.fb_velocity = feedback.velocity
            entry.fb_position = feedback.position
            entry.fb_yaw = feedback.yaw
            entry.fb_yaw_rate = feedback.yaw_rate
        
        with self._lock:
            self._buffer.append(entry)
            
            if len(self._buffer) >= self._buffer_size:
                self._flush_buffer()
    
    def _flush_buffer(self) -> None:
        """Write buffer to file."""
        if self._writer is None:
            return
        
        with self._lock:
            for entry in self._buffer:
                row = [
                    f"{entry.timestamp:.3f}",
                    f"{entry.cmd_velocity:.3f}",
                    f"{entry.cmd_yaw_rate:.3f}",
                    f"{entry.fb_velocity:.3f}",
                    f"{entry.fb_position:.3f}",
                    f"{entry.fb_yaw:.3f}",
                    f"{entry.fb_yaw_rate:.3f}",
                    entry.mode,
                    entry.mode_state,
                    int(entry.line_detected),
                    f"{entry.position_error:.3f}",
                    f"{entry.heading_error:.3f}",
                    f"{entry.confidence:.3f}"
                ]
                self._writer.writerow(row)
                self._entry_count += 1
            
            self._buffer.clear()
        
        if self._file is not None:
            self._file.flush()
    
    def get_entry_count(self) -> int:
        """Get number of logged entries."""
        return self._entry_count + len(self._buffer)


class SessionSummary:
    """
    Creates a summary of a robot session.
    """
    
    @staticmethod
    def generate(
        log_file: Path,
        output_file: Optional[Path] = None
    ) -> Dict[str, Any]:
        """
        Generate summary from log file.
        
        Args:
            log_file: Path to CSV log file
            output_file: Path to save summary JSON (optional)
            
        Returns:
            Summary dictionary
        """
        import pandas as pd
        
        df = pd.read_csv(log_file)
        
        summary = {
            'file': str(log_file),
            'duration': df['time'].max() - df['time'].min(),
            'total_entries': len(df),
            'modes_used': df['mode'].unique().tolist(),
            
            'velocity': {
                'mean': df['cmd_v'].mean(),
                'max': df['cmd_v'].max(),
                'min': df['cmd_v'].min(),
            },
            
            'yaw_rate': {
                'mean': df['cmd_yaw'].mean(),
                'max': df['cmd_yaw'].max(),
                'min': df['cmd_yaw'].min(),
            },
            
            'line_detection_rate': df['line_detected'].mean() if 'line_detected' in df else None,
        }
        
        if output_file is not None:
            with open(output_file, 'w') as f:
                json.dump(summary, f, indent=2)
        
        return summary
