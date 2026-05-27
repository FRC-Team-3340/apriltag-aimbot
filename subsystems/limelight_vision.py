from __future__ import annotations

import time

import commands2
import ntcore
import wpilib

import constants
from tracking.apriltag_tracker import VisionTarget


class LimelightVisionSubsystem(commands2.Subsystem):
    def __init__(self) -> None:
        super().__init__()
        commands2.CommandScheduler.getInstance().registerSubsystem(self)

        self._nt_instance = ntcore.NetworkTableInstance.getDefault()
        self._table = self._nt_instance.getTable(constants.Limelight.TABLE_NAME)

        self._tv = self._table.getDoubleTopic(constants.Limelight.TARGET_VALID_KEY).subscribe(0.0)
        self._tx = self._table.getDoubleTopic(
            constants.Limelight.HORIZONTAL_OFFSET_KEY
        ).subscribe(0.0)
        self._ty = self._table.getDoubleTopic(
            constants.Limelight.VERTICAL_OFFSET_KEY
        ).subscribe(0.0)
        self._tid = self._table.getDoubleTopic(constants.Limelight.TARGET_ID_KEY).subscribe(-1.0)
        self._tl = self._table.getDoubleTopic(
            constants.Limelight.PIPELINE_LATENCY_KEY
        ).subscribe(0.0)
        self._cl = self._table.getDoubleTopic(
            constants.Limelight.CAPTURE_LATENCY_KEY
        ).subscribe(0.0)
        self._heartbeat = self._table.getDoubleTopic(constants.Limelight.HEARTBEAT_KEY).subscribe(0.0)

        self._last_heartbeat = float("nan")
        self._last_heartbeat_update_time = time.monotonic()
        self._latest_target = VisionTarget()

    def get_latest_target(self) -> VisionTarget:
        return self._latest_target

    def periodic(self) -> None:
        now = time.monotonic()
        heartbeat = self._heartbeat.get()

        if heartbeat != self._last_heartbeat:
            self._last_heartbeat = heartbeat
            self._last_heartbeat_update_time = now

        is_stale = (
            now - self._last_heartbeat_update_time
        ) > constants.Limelight.STALE_TIMEOUT_SECONDS

        limelight_has_target = self._tv.get() >= 1.0
        has_target = limelight_has_target and not is_stale

        tx = self._tx.get() if has_target else 0.0
        ty = self._ty.get() if has_target else 0.0
        raw_tag_id = self._tid.get()
        tag_id = int(raw_tag_id) if has_target and raw_tag_id >= 0.0 else None

        self._latest_target = VisionTarget(
            has_target=has_target,
            tx_degrees=tx,
            ty_degrees=ty,
            tag_id=tag_id,
            timestamp_seconds=now,
            is_stale=is_stale,
            pipeline_latency_ms=self._tl.get(),
            capture_latency_ms=self._cl.get(),
        )

        wpilib.SmartDashboard.putBoolean("Vision/HasTarget", self._latest_target.has_target)
        wpilib.SmartDashboard.putBoolean("Vision/IsStale", self._latest_target.is_stale)
        wpilib.SmartDashboard.putNumber("Vision/TxDegrees", self._latest_target.tx_degrees)
        wpilib.SmartDashboard.putNumber("Vision/TyDegrees", self._latest_target.ty_degrees)
        wpilib.SmartDashboard.putString(
            "Vision/TagId", "" if self._latest_target.tag_id is None else str(self._latest_target.tag_id)
        )
        wpilib.SmartDashboard.putNumber(
            "Vision/PipelineLatencyMs", self._latest_target.pipeline_latency_ms
        )
        wpilib.SmartDashboard.putNumber(
            "Vision/CaptureLatencyMs", self._latest_target.capture_latency_ms
        )
