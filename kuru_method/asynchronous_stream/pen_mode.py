"""
pen_mode.py — Priority 5 pen-up state machine (hybrid A→B policy)
==================================================================
Single source of truth for the four pen-mode states the EKF needs:

    COLD_START   — EKF not yet initialised
    PEN_DOWN     — debounced contact = 1; normal UWB EKF update
    HOVER_SHORT  — debounced contact = 0, hover_age < T_SHORT_S
                   (UWB update with R inflated by ALPHA_R_HOVER)
    HOVER_LONG   — debounced contact = 0, hover_age >= T_SHORT_S
                   (UWB update entirely skipped — pure IMU propagation)
    HOVER_H      — feature-flagged 7-state hover-height EKF branch
                   (only entered when ENABLE_HOVER_H, n_valid>=3, q_mean>=Q_H)

Transitions are driven by:
  - debounced FSR contact changes (rising / falling edges),
  - sender-clock hover age (so the policy is invariant to capture rate).

The manager itself is stateless w.r.t. the EKF. The fusion engine asks
it `update_imu(...)` for every IMU packet (to advance hover age and pick
up touchdowns) and `decide_uwb_mode(...)` at every UWB packet (to apply
the per-mode quality / anchor-count gating). Touchdown reacquisition is
also tracked here as a small countdown the EKF respects on the next few
UWB updates.
"""

from __future__ import annotations

from enum import Enum
from typing import Optional

from config import (T_SHORT_S, T_H_S, Q_SHORT, Q_H,
                    HOVER_MIN_VALID_ANCHORS, HOVER_H_MIN_VALID_ANCHORS,
                    TOUCHDOWN_REACQ_UPDATES, ENABLE_HOVER_H)


class PenMode(Enum):
    COLD_START  = 'cold_start'
    PEN_DOWN    = 'pen_down'
    HOVER_SHORT = 'hover_short'
    HOVER_LONG  = 'hover_long'
    HOVER_H     = 'hover_h'


class PenModeManager:
    """Drives the four-mode state machine using FSR contact + hover age."""

    def __init__(self):
        self.mode: PenMode               = PenMode.COLD_START
        self._hover_start_ts: Optional[int] = None    # micros
        self._last_ts:        Optional[int] = None
        self._was_contact:    bool       = False
        # Reacquisition window after a long-hover → pen-down transition.
        self._touchdown_remain: int      = 0
        # Diagnostics — surfaced in verifier output.
        self.last_touchdown_ts: Optional[int] = None
        self.last_lift_ts:      Optional[int] = None

    # ------------------------------------------------------------------
    # IMU-side updates
    # ------------------------------------------------------------------
    def mark_initialized(self) -> None:
        """Call once when the EKF transitions out of cold-start."""
        if self.mode == PenMode.COLD_START:
            # Default to PEN_DOWN unless a subsequent update_imu corrects us.
            self.mode = PenMode.PEN_DOWN

    def update_imu(self, contact: bool, ts: Optional[int]) -> PenMode:
        """
        Drive transitions based on debounced FSR contact and sender ts.
        Returns the resolved PenMode after this packet.
        """
        if self.mode == PenMode.COLD_START:
            # Cold-start path — track contact for stats, do not transition
            # until the EKF tells us via mark_initialized().
            self._was_contact = bool(contact)
            self._last_ts = ts
            return self.mode

        # Rising edge — touchdown.
        if contact and not self._was_contact:
            from_long = (self.mode == PenMode.HOVER_LONG
                         or self.mode == PenMode.HOVER_H)
            self.mode = PenMode.PEN_DOWN
            self._hover_start_ts = None
            self.last_touchdown_ts = ts
            if from_long:
                self._touchdown_remain = TOUCHDOWN_REACQ_UPDATES

        # Falling edge — lift.
        elif (not contact) and self._was_contact:
            self.mode = PenMode.HOVER_SHORT
            self._hover_start_ts = ts
            self.last_lift_ts = ts

        # Continuing hover — promote SHORT → LONG (or HOVER_H) at thresholds.
        elif (not contact) and self._hover_start_ts is not None and ts is not None:
            hover_s = (ts - self._hover_start_ts) / 1_000_000.0
            if self.mode == PenMode.HOVER_SHORT and hover_s >= T_SHORT_S:
                # Default promotion is to HOVER_LONG; HOVER_H is opt-in and
                # only chosen at the UWB packet boundary where we know
                # n_valid / q_mean (decide_uwb_mode handles that).
                self.mode = PenMode.HOVER_LONG
            elif (ENABLE_HOVER_H
                  and self.mode == PenMode.HOVER_LONG
                  and hover_s >= (T_SHORT_S + T_H_S)):
                # Stay in HOVER_LONG until decide_uwb_mode confirms anchor
                # quality is high enough; only then promote to HOVER_H.
                pass

        self._was_contact = bool(contact)
        self._last_ts = ts
        return self.mode

    # ------------------------------------------------------------------
    # UWB-side decision
    # ------------------------------------------------------------------
    def decide_uwb_mode(self, n_valid: int, q_mean: float) -> PenMode:
        """
        Apply the hover gates (anchor-count + mean quality). The UWB-update
        path uses the *returned* mode, which may downgrade HOVER_SHORT →
        HOVER_LONG (skip) when the gate fails, or promote HOVER_LONG →
        HOVER_H when ENABLE_HOVER_H is on and quality is high enough.
        """
        m = self.mode

        if m == PenMode.HOVER_SHORT:
            if (n_valid >= HOVER_MIN_VALID_ANCHORS and q_mean >= Q_SHORT):
                return PenMode.HOVER_SHORT
            return PenMode.HOVER_LONG          # fall through to skip

        if m == PenMode.HOVER_LONG:
            if (ENABLE_HOVER_H
                    and n_valid >= HOVER_H_MIN_VALID_ANCHORS
                    and q_mean >= Q_H):
                return PenMode.HOVER_H
            return PenMode.HOVER_LONG

        return m

    # ------------------------------------------------------------------
    # Touchdown reacquisition window
    # ------------------------------------------------------------------
    def consume_touchdown_credit(self) -> bool:
        """Returns True iff the next UWB update should run with the
        touchdown-reacq R inflation + widened χ² gate. Decrements the
        counter on consumption."""
        if self._touchdown_remain > 0:
            self._touchdown_remain -= 1
            return True
        return False

    @property
    def hover_age_s(self) -> float:
        if self._hover_start_ts is None or self._last_ts is None:
            return 0.0
        return max(0.0, (self._last_ts - self._hover_start_ts) / 1_000_000.0)

    @property
    def is_hover(self) -> bool:
        return self.mode in (PenMode.HOVER_SHORT, PenMode.HOVER_LONG,
                             PenMode.HOVER_H)
