#!/usr/bin/env python3
"""Readable live discrete closed-loop pole dashboard for micro_lqr_controller."""

from __future__ import annotations

import math
import sys
from dataclasses import dataclass
from typing import Iterable, List, Sequence, Tuple

import matplotlib.pyplot as plt
from matplotlib.patches import Circle, FancyBboxPatch
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import (
    DurabilityPolicy,
    HistoryPolicy,
    QoSProfile,
    ReliabilityPolicy,
)
from std_msgs.msg import Float64MultiArray


# Deliberately use a fixed dashboard palette rather than inheriting an arbitrary
# desktop Matplotlib style. This keeps the ROS launch window consistent.
BG = "#07111f"
PANEL = "#0d1b2a"
PANEL_ALT = "#102235"
TEXT = "#e8eef7"
MUTED = "#98a8bb"
GRID = "#2a4158"
UNIT = "#d5deea"
BLUE = "#38bdf8"
PURPLE = "#a78bfa"
GREEN = "#34d399"
AMBER = "#fbbf24"
RED = "#fb7185"
WHITE = "#f8fafc"




def _signature_value(value: float) -> float:
    value = float(value)
    if math.isnan(value):
        return 9.87654321e307
    if math.isinf(value):
        return math.copysign(9.12345678e307, value)
    return round(value, 12)


@dataclass(frozen=True)
class Pole:
    real: float
    imag: float
    magnitude: float
    damping_ratio: float
    frequency_hz: float


@dataclass(frozen=True)
class PoleSnapshot:
    stable: bool
    max_abs: float
    radial_margin: float
    gain_scale: float
    sample_time: float
    manual_gain: bool
    gain: np.ndarray
    poles: List[Pole]


@dataclass(frozen=True)
class DisplayPole:
    label: str
    mode_label: str
    color: str
    pole: Pole


class PoleDashboard:
    """Matplotlib renderer independent from ROS subscription details."""

    def __init__(self, window_title: str) -> None:
        plt.ion()
        plt.rcParams.update(
            {
                "font.size": 10,
                "axes.titleweight": "semibold",
                "axes.labelcolor": TEXT,
                "axes.edgecolor": GRID,
                "xtick.color": MUTED,
                "ytick.color": MUTED,
                "text.color": TEXT,
                "figure.facecolor": BG,
                "savefig.facecolor": BG,
                "axes.unicode_minus": False,
            }
        )

        # Fixed normalized positions are intentional. They remain readable when
        # resized and avoid constrained_layout/tight_layout collapse warnings.
        self.figure = plt.figure(figsize=(15.8, 9.0), dpi=100, facecolor=BG)
        try:
            self.figure.canvas.manager.set_window_title(window_title)
        except AttributeError:
            pass

        self.main_axis = self.figure.add_axes([0.045, 0.265, 0.54, 0.640])
        self.zoom_axis = self.figure.add_axes([0.625, 0.565, 0.345, 0.340])
        self.metric_axis = self.figure.add_axes([0.625, 0.385, 0.345, 0.135])
        self.gauge_axis = self.figure.add_axes([0.655, 0.285, 0.285, 0.070])
        self.table_axis = self.figure.add_axes([0.045, 0.055, 0.655, 0.155])
        self.gain_axis = self.figure.add_axes([0.735, 0.055, 0.235, 0.155])

        self.figure.text(
            0.045,
            0.973,
            "MICRO LQR  ·  CLOSED-LOOP STABILITY",
            ha="left",
            va="top",
            fontsize=15,
            fontweight="bold",
            color=WHITE,
        )
        self.figure.text(
            0.97,
            0.973,
            "Discrete-time nominal model",
            ha="right",
            va="top",
            fontsize=9.5,
            color=MUTED,
        )

    @staticmethod
    def _style_plot_axis(axis: plt.Axes, title: str) -> None:
        axis.set_facecolor(PANEL)
        axis.set_title(title, color=WHITE, fontsize=12, pad=11, loc="left")
        axis.grid(True, color=GRID, alpha=0.52, linewidth=0.75)
        axis.tick_params(colors=MUTED, labelsize=9)
        for spine in axis.spines.values():
            spine.set_color(GRID)
            spine.set_linewidth(0.9)

    @staticmethod
    def _style_panel_axis(axis: plt.Axes) -> None:
        axis.set_facecolor(PANEL)
        axis.set_xticks([])
        axis.set_yticks([])
        for spine in axis.spines.values():
            spine.set_color(GRID)
            spine.set_linewidth(0.9)

    def draw_waiting(self, pole_topic: str) -> None:
        for axis in (
            self.main_axis,
            self.zoom_axis,
            self.metric_axis,
            self.gauge_axis,
            self.table_axis,
            self.gain_axis,
        ):
            axis.clear()

        self._style_plot_axis(self.main_axis, "Full unit-circle view")
        self.main_axis.set_aspect("equal", adjustable="box")
        self.main_axis.set_xlim(-1.12, 1.12)
        self.main_axis.set_ylim(-1.12, 1.12)
        self.main_axis.set_xlabel(r"Real part  $\mathrm{Re}(\lambda)$")
        self.main_axis.set_ylabel(r"Imaginary part  $\mathrm{Im}(\lambda)$")
        self._draw_unit_circle(self.main_axis, show_inner_guide=True)

        self._style_plot_axis(self.zoom_axis, "Dominant-pole zoom")
        self.zoom_axis.set_xlim(0.96, 1.005)
        self.zoom_axis.set_ylim(-0.03, 0.03)
        self.zoom_axis.set_xlabel(r"$\mathrm{Re}(\lambda)$")
        self.zoom_axis.set_ylabel(r"$\mathrm{Im}(\lambda)$")
        self._draw_unit_arc(self.zoom_axis)

        self._style_panel_axis(self.metric_axis)
        self.metric_axis.text(
            0.04,
            0.72,
            "Waiting for pole data",
            fontsize=17,
            fontweight="bold",
            color=WHITE,
            transform=self.metric_axis.transAxes,
        )
        self.metric_axis.text(
            0.04,
            0.37,
            f"Topic  {pole_topic}",
            fontsize=10.5,
            color=MUTED,
            transform=self.metric_axis.transAxes,
        )
        self.metric_axis.text(
            0.04,
            0.13,
            "The dashboard updates when the monitor receives controller parameters.",
            fontsize=9.2,
            color=MUTED,
            transform=self.metric_axis.transAxes,
        )

        self._style_panel_axis(self.gauge_axis)
        self.gauge_axis.text(
            0.5,
            0.5,
            "No spectral-radius sample yet",
            ha="center",
            va="center",
            color=MUTED,
            transform=self.gauge_axis.transAxes,
        )

        self._style_panel_axis(self.table_axis)
        self.table_axis.text(
            0.02,
            0.78,
            "Pole details will appear here",
            color=MUTED,
            fontsize=10,
            transform=self.table_axis.transAxes,
        )
        self._style_panel_axis(self.gain_axis)
        self.gain_axis.text(
            0.06,
            0.78,
            "Active gain",
            color=MUTED,
            fontsize=10,
            transform=self.gain_axis.transAxes,
        )
        self.figure.canvas.draw_idle()

    def draw_snapshot(self, snapshot: PoleSnapshot) -> None:
        display_poles = self._build_display_poles(snapshot.poles)
        self._draw_main(snapshot, display_poles)
        self._draw_zoom(snapshot, display_poles)
        self._draw_metrics(snapshot)
        self._draw_gauge(snapshot)
        self._draw_pole_table(display_poles)
        self._draw_gain_table(snapshot)
        self.figure.canvas.draw_idle()

    def _draw_main(
        self, snapshot: PoleSnapshot, display_poles: Sequence[DisplayPole]
    ) -> None:
        axis = self.main_axis
        axis.clear()
        self._style_plot_axis(axis, "Full unit-circle view")

        limit = max(1.10, snapshot.max_abs * 1.08)
        limit = min(limit, 2.5)
        axis.set_xlim(-limit, limit)
        axis.set_ylim(-limit, limit)
        axis.set_aspect("equal", adjustable="box")
        axis.set_xlabel(r"Real part  $\mathrm{Re}(\lambda)$", labelpad=7)
        axis.set_ylabel(r"Imaginary part  $\mathrm{Im}(\lambda)$", labelpad=7)
        axis.axhline(0.0, color=GRID, linewidth=0.9, zorder=0)
        axis.axvline(0.0, color=GRID, linewidth=0.9, zorder=0)
        self._draw_unit_circle(axis, show_inner_guide=True)

        for item in display_poles:
            pole = item.pole
            axis.scatter(
                pole.real,
                pole.imag,
                marker="X",
                s=135,
                linewidths=1.4,
                edgecolors=WHITE,
                color=item.color,
                zorder=5,
            )
            label_offsets = {
                "P1": (12, 15),
                "P2": (12, -20),
                "P3": (-34, 15),
                "P4": (-34, -20),
            }
            axis.annotate(
                item.label,
                xy=(pole.real, pole.imag),
                xytext=label_offsets.get(item.label, (10, 10)),
                textcoords="offset points",
                fontsize=9.6,
                fontweight="bold",
                color=item.color,
                bbox={
                    "boxstyle": "round,pad=0.20",
                    "facecolor": PANEL_ALT,
                    "edgecolor": item.color,
                    "alpha": 0.96,
                },
                zorder=6,
            )

        axis.text(
            0.025,
            0.035,
            "Inside unit circle  →  nominally asymptotically stable",
            transform=axis.transAxes,
            color=MUTED,
            fontsize=9.4,
            ha="left",
            va="bottom",
        )

    def _draw_zoom(
        self, snapshot: PoleSnapshot, display_poles: Sequence[DisplayPole]
    ) -> None:
        axis = self.zoom_axis
        axis.clear()
        self._style_plot_axis(axis, "Dominant-pole zoom  ·  distance to |λ| = 1")

        dominant_mag = max(item.pole.magnitude for item in display_poles)
        dominant_items = [
            item
            for item in display_poles
            if abs(item.pole.magnitude - dominant_mag) <= 2.0e-5
        ]
        if not dominant_items:
            dominant_items = [max(display_poles, key=lambda item: item.pole.magnitude)]

        center_real = float(np.mean([item.pole.real for item in dominant_items]))
        max_imag = max(abs(item.pole.imag) for item in dominant_items)
        x_padding = max(0.0025, min(0.030, abs(1.0 - center_real) * 2.8 + 0.0015))
        y_padding = max(0.006, max_imag * 1.55 + 0.002)
        x_left = min(center_real - x_padding, 0.995)
        x_right = max(1.0025, center_real + x_padding * 0.65)
        x_left = max(-1.05, x_left)
        x_right = min(1.25, x_right)
        axis.set_xlim(x_left, x_right)
        axis.set_ylim(-y_padding, y_padding)
        axis.set_xlabel(r"$\mathrm{Re}(\lambda)$")
        axis.set_ylabel(r"$\mathrm{Im}(\lambda)$")
        axis.axhline(0.0, color=GRID, linewidth=0.8)
        self._draw_unit_arc(axis)

        for item in dominant_items:
            pole = item.pole
            axis.scatter(
                pole.real,
                pole.imag,
                marker="X",
                s=155,
                linewidths=1.5,
                edgecolors=WHITE,
                color=item.color,
                zorder=6,
            )

            if pole.magnitude > 1.0e-12:
                boundary_real = pole.real / pole.magnitude
                boundary_imag = pole.imag / pole.magnitude
                axis.plot(
                    [pole.real, boundary_real],
                    [pole.imag, boundary_imag],
                    color=AMBER if snapshot.stable else RED,
                    linewidth=2.4,
                    zorder=4,
                )
                axis.scatter(
                    boundary_real,
                    boundary_imag,
                    marker="o",
                    s=32,
                    color=UNIT,
                    zorder=5,
                )
                midpoint = (
                    0.5 * (pole.real + boundary_real),
                    0.5 * (pole.imag + boundary_imag),
                )
                axis.annotate(
                    f"radial gap\n{1.0 - pole.magnitude:+.7f}",
                    xy=midpoint,
                    xytext=(8, 8 if pole.imag >= 0.0 else -30),
                    textcoords="offset points",
                    fontsize=8.8,
                    color=AMBER if snapshot.stable else RED,
                    bbox={
                        "boxstyle": "round,pad=0.25",
                        "facecolor": PANEL_ALT,
                        "edgecolor": "none",
                        "alpha": 0.96,
                    },
                )

        axis.text(
            0.025,
            0.04,
            "The short segment is the nominal radial stability margin.",
            transform=axis.transAxes,
            fontsize=8.9,
            color=MUTED,
            ha="left",
            va="bottom",
        )

    def _draw_metrics(self, snapshot: PoleSnapshot) -> None:
        axis = self.metric_axis
        axis.clear()
        self._style_panel_axis(axis)

        status_title, status_detail, status_color = self._status(snapshot)
        mode = "MANUAL K" if snapshot.manual_gain else "AUTO LQR"
        min_zeta = self._minimum_damping(snapshot.poles)

        axis.text(
            0.035,
            0.78,
            status_title,
            transform=axis.transAxes,
            fontsize=15.2,
            fontweight="bold",
            color=status_color,
            va="center",
        )
        axis.text(
            0.035,
            0.49,
            status_detail,
            transform=axis.transAxes,
            fontsize=9.1,
            color=MUTED,
            va="center",
        )

        cards = [
            ("ρ max", f"{snapshot.max_abs:.7f}"),
            ("1 − ρ", f"{snapshot.radial_margin:+.7f}"),
            ("ζ min", "n/a" if not math.isfinite(min_zeta) else f"{min_zeta:.3f}"),
            (mode, f"scale {snapshot.gain_scale:.3f}"),
        ]
        card_y = 0.05
        card_w = 0.222
        for index, (label, value) in enumerate(cards):
            left = 0.035 + index * 0.238
            patch = FancyBboxPatch(
                (left, card_y),
                card_w,
                0.25,
                boxstyle="round,pad=0.008,rounding_size=0.025",
                linewidth=0.8,
                edgecolor=GRID,
                facecolor=PANEL_ALT,
                transform=axis.transAxes,
            )
            axis.add_patch(patch)
            axis.text(
                left + 0.015,
                card_y + 0.177,
                label,
                transform=axis.transAxes,
                fontsize=8.3,
                color=MUTED,
                va="center",
            )
            axis.text(
                left + 0.015,
                card_y + 0.075,
                value,
                transform=axis.transAxes,
                fontsize=10.7,
                fontweight="semibold",
                color=WHITE,
                va="center",
            )

    def _draw_gauge(self, snapshot: PoleSnapshot) -> None:
        axis = self.gauge_axis
        axis.clear()
        axis.set_facecolor(PANEL)

        rho = snapshot.max_abs
        lower = min(0.985, rho - 0.010)
        lower = max(0.0, lower)
        upper = max(1.003, rho + 0.003)
        axis.set_xlim(lower, upper)
        axis.set_ylim(0.0, 1.0)
        axis.set_yticks([])
        axis.tick_params(axis="x", colors=MUTED, labelsize=8.5, pad=3)
        for spine in axis.spines.values():
            spine.set_visible(False)

        axis.axvspan(lower, min(0.995, upper), color=GREEN, alpha=0.22)
        if upper > 0.995:
            axis.axvspan(max(lower, 0.995), min(1.0, upper), color=AMBER, alpha=0.28)
        if upper > 1.0:
            axis.axvspan(max(lower, 1.0), upper, color=RED, alpha=0.25)
        axis.axvline(1.0, color=UNIT, linewidth=1.6, linestyle="--", zorder=3)
        marker_color = GREEN if rho < 0.995 else AMBER if rho < 1.0 else RED
        axis.scatter(
            [rho],
            [0.52],
            marker="v",
            s=115,
            color=marker_color,
            edgecolors=WHITE,
            linewidths=0.8,
            zorder=5,
        )
        axis.text(
            rho,
            0.90,
            f"ρ = {rho:.7f}",
            color=WHITE,
            fontsize=9.1,
            ha="center",
            va="top",
            bbox={
                "boxstyle": "round,pad=0.2",
                "facecolor": PANEL_ALT,
                "edgecolor": marker_color,
                "alpha": 0.96,
            },
        )
        axis.text(
            1.0,
            0.05,
            "unit-circle boundary",
            color=MUTED,
            fontsize=8.1,
            ha="right",
            va="bottom",
        )

    def _draw_pole_table(self, display_poles: Sequence[DisplayPole]) -> None:
        axis = self.table_axis
        axis.clear()
        self._style_panel_axis(axis)
        axis.set_title("Pole details", color=WHITE, fontsize=11.5, loc="left", pad=8)

        rows: List[List[str]] = []
        for item in display_poles:
            pole = item.pole
            zeta = "n/a" if not math.isfinite(pole.damping_ratio) else f"{pole.damping_ratio:.3f}"
            rows.append(
                [
                    item.label,
                    f"{pole.real:+.7f} {pole.imag:+.7f}j",
                    f"{pole.magnitude:.7f}",
                    f"{1.0 - pole.magnitude:+.7f}",
                    zeta,
                    f"{pole.frequency_hz:.3f}",
                    item.mode_label,
                ]
            )

        table = axis.table(
            cellText=rows,
            colLabels=["Pole", "λ", "|λ|", "1−|λ|", "ζ", "f_d [Hz]", "Mode"],
            cellLoc="center",
            colLoc="center",
            loc="center",
            bbox=[0.015, 0.05, 0.97, 0.80],
            colWidths=[0.07, 0.27, 0.13, 0.14, 0.09, 0.13, 0.17],
        )
        table.auto_set_font_size(False)
        table.set_fontsize(9.0)
        for (row, col), cell in table.get_celld().items():
            cell.set_edgecolor(GRID)
            cell.set_linewidth(0.65)
            if row == 0:
                cell.set_facecolor(PANEL_ALT)
                cell.get_text().set_color(MUTED)
                cell.get_text().set_fontweight("semibold")
            else:
                cell.set_facecolor(PANEL)
                cell.get_text().set_color(TEXT)
                if col == 0:
                    cell.get_text().set_color(display_poles[row - 1].color)
                    cell.get_text().set_fontweight("bold")

    def _draw_gain_table(self, snapshot: PoleSnapshot) -> None:
        axis = self.gain_axis
        axis.clear()
        self._style_panel_axis(axis)
        axis.set_title("Active feedback gain", color=WHITE, fontsize=11.5, loc="left", pad=8)

        effective = snapshot.gain_scale * snapshot.gain
        names = ["K_pitch", "K_rate", "K_pos", "K_vel"]
        rows = [
            [names[index], f"{snapshot.gain[index]:+.5f}", f"{effective[index]:+.5f}"]
            for index in range(4)
        ]
        table = axis.table(
            cellText=rows,
            colLabels=["Term", "Configured", "scale × K"],
            cellLoc="right",
            colLoc="center",
            loc="center",
            bbox=[0.04, 0.16, 0.92, 0.69],
            colWidths=[0.22, 0.39, 0.39],
        )
        table.auto_set_font_size(False)
        table.set_fontsize(9.0)
        for (row, col), cell in table.get_celld().items():
            cell.set_edgecolor(GRID)
            cell.set_linewidth(0.65)
            if row == 0:
                cell.set_facecolor(PANEL_ALT)
                cell.get_text().set_color(MUTED)
                cell.get_text().set_fontweight("semibold")
            else:
                cell.set_facecolor(PANEL)
                cell.get_text().set_color(TEXT)
                if col == 0:
                    cell.get_text().set_color(BLUE)
                    cell.get_text().set_fontweight("bold")

        axis.text(
            0.04,
            0.035,
            f"Ts = {snapshot.sample_time * 1000.0:.3f} ms",
            transform=axis.transAxes,
            fontsize=8.3,
            color=MUTED,
            ha="left",
            va="bottom",
        )

    @staticmethod
    def _draw_unit_circle(axis: plt.Axes, show_inner_guide: bool) -> None:
        interior = Circle(
            (0.0, 0.0),
            1.0,
            facecolor=GREEN,
            edgecolor="none",
            alpha=0.035,
            zorder=0,
        )
        axis.add_patch(interior)
        theta = np.linspace(0.0, 2.0 * np.pi, 1000)
        axis.plot(
            np.cos(theta),
            np.sin(theta),
            color=UNIT,
            linewidth=1.8,
            label=r"unit circle  $|\lambda|=1$",
            zorder=2,
        )
        if show_inner_guide:
            axis.plot(
                0.995 * np.cos(theta),
                0.995 * np.sin(theta),
                color=AMBER,
                linewidth=1.0,
                linestyle=(0, (4, 4)),
                alpha=0.8,
                label=r"low-margin guide  $|\lambda|=0.995$",
                zorder=1,
            )
        legend = axis.legend(
            loc="upper left",
            frameon=True,
            fontsize=8.7,
            facecolor=PANEL_ALT,
            edgecolor=GRID,
            labelcolor=TEXT,
        )
        legend.get_frame().set_alpha(0.94)

    @staticmethod
    def _draw_unit_arc(axis: plt.Axes) -> None:
        theta = np.linspace(-0.5 * np.pi, 0.5 * np.pi, 1000)
        axis.plot(
            np.cos(theta),
            np.sin(theta),
            color=UNIT,
            linewidth=2.0,
            zorder=3,
        )
        axis.text(
            0.985,
            0.93,
            r"$|\lambda|=1$",
            transform=axis.transAxes,
            ha="right",
            va="top",
            color=UNIT,
            fontsize=8.8,
        )

    @staticmethod
    def _minimum_damping(poles: Iterable[Pole]) -> float:
        values = [
            pole.damping_ratio
            for pole in poles
            if math.isfinite(pole.damping_ratio) and pole.imag >= -1.0e-10
        ]
        return min(values) if values else float("nan")

    @staticmethod
    def _status(snapshot: PoleSnapshot) -> Tuple[str, str, str]:
        min_zeta = PoleDashboard._minimum_damping(snapshot.poles)
        if not snapshot.stable:
            return (
                "UNSTABLE",
                "At least one nominal pole lies on or outside the unit circle.",
                RED,
            )
        if snapshot.max_abs >= 0.999:
            return (
                "NOMINALLY STABLE  ·  CRITICAL MARGIN",
                "The model is stable, but a small delay or parameter error can cross |λ| = 1.",
                RED,
            )
        if snapshot.max_abs >= 0.995 or (
            math.isfinite(min_zeta) and min_zeta < 0.10
        ):
            return (
                "STABLE  ·  LOW DAMPING / LOW MARGIN",
                "Expect visible oscillation or slow decay; keep practical margin from the boundary.",
                AMBER,
            )
        return (
            "NOMINALLY STABLE",
            "All model poles are comfortably inside the unit circle.",
            GREEN,
        )

    @staticmethod
    def _build_display_poles(poles: Sequence[Pole]) -> List[DisplayPole]:
        """Group approximate conjugate pairs and put the dominant pair first."""
        unused = set(range(len(poles)))
        pairs: List[List[int]] = []
        while unused:
            first = min(unused)
            unused.remove(first)
            if not unused:
                pairs.append([first])
                break
            partner = min(
                unused,
                key=lambda index: (
                    abs(poles[first].real - poles[index].real)
                    + abs(poles[first].imag + poles[index].imag)
                    + abs(poles[first].magnitude - poles[index].magnitude)
                ),
            )
            unused.remove(partner)
            pairs.append([first, partner])

        pairs.sort(
            key=lambda pair: max(poles[index].magnitude for index in pair),
            reverse=True,
        )
        colors = [BLUE, PURPLE, GREEN, AMBER]
        result: List[DisplayPole] = []
        pole_counter = 1
        for pair_index, pair in enumerate(pairs):
            pair.sort(key=lambda index: poles[index].imag, reverse=True)
            mode_name = "dominant mode" if pair_index == 0 else f"mode {pair_index + 1}"
            for index in pair:
                result.append(
                    DisplayPole(
                        label=f"P{pole_counter}",
                        mode_label=mode_name,
                        color=colors[pair_index % len(colors)],
                        pole=poles[index],
                    )
                )
                pole_counter += 1
        return result


class PoleVisualizer(Node):
    def __init__(self) -> None:
        super().__init__("micro_lqr_pole_visualizer")
        self.declare_parameter("pole_topic", "/micro_lqr/poles")
        self.declare_parameter("window_title", "Micro LQR stability dashboard")

        pole_topic = str(self.get_parameter("pole_topic").value)
        window_title = str(self.get_parameter("window_title").value)

        qos = QoSProfile(
            history=HistoryPolicy.KEEP_LAST,
            depth=1,
            reliability=ReliabilityPolicy.RELIABLE,
            durability=DurabilityPolicy.TRANSIENT_LOCAL,
        )
        self.create_subscription(Float64MultiArray, pole_topic, self._on_poles, qos)

        self.dashboard = PoleDashboard(window_title)
        self.dashboard.draw_waiting(pole_topic)
        self.dashboard.figure.canvas.mpl_connect("close_event", self._on_close)

        self._snapshot: PoleSnapshot | None = None
        self._last_rendered_signature: Tuple[float, ...] | None = None
        self._dirty = False
        self._closed = False
        self.get_logger().info(f"waiting for pole data on {pole_topic}")

    def _on_close(self, _event: object) -> None:
        self._closed = True

    def _on_poles(self, message: Float64MultiArray) -> None:
        data = list(message.data)
        if len(data) < 31:
            self.get_logger().error(
                f"invalid pole message: expected at least 31 values, got {len(data)}"
            )
            return
        if not math.isclose(data[0], 1.0, rel_tol=0.0, abs_tol=1.0e-9):
            self.get_logger().error(f"unsupported pole schema version: {data[0]}")
            return

        poles: List[Pole] = []
        for index in range(4):
            base = 11 + 5 * index
            poles.append(
                Pole(
                    real=float(data[base]),
                    imag=float(data[base + 1]),
                    magnitude=float(data[base + 2]),
                    damping_ratio=float(data[base + 3]),
                    frequency_hz=float(data[base + 4]),
                )
            )

        snapshot = PoleSnapshot(
            stable=bool(data[1] > 0.5),
            max_abs=float(data[2]),
            radial_margin=float(data[3]),
            gain_scale=float(data[4]),
            sample_time=float(data[5]),
            manual_gain=bool(data[6] > 0.5),
            gain=np.asarray(data[7:11], dtype=float),
            poles=poles,
        )
        signature = tuple(_signature_value(value) for value in data[:31])
        if signature == self._last_rendered_signature:
            return
        self._snapshot = snapshot
        self._dirty = True

    def redraw_if_needed(self) -> None:
        if not self._dirty or self._snapshot is None or self._closed:
            return
        self._dirty = False
        snapshot = self._snapshot
        self.dashboard.draw_snapshot(snapshot)
        signature_values: List[float] = [
            1.0,
            1.0 if snapshot.stable else 0.0,
            snapshot.max_abs,
            snapshot.radial_margin,
            snapshot.gain_scale,
            snapshot.sample_time,
            1.0 if snapshot.manual_gain else 0.0,
            *snapshot.gain.tolist(),
        ]
        for pole in snapshot.poles:
            signature_values.extend(
                [
                    pole.real,
                    pole.imag,
                    pole.magnitude,
                    pole.damping_ratio,
                    pole.frequency_hz,
                ]
            )
        self._last_rendered_signature = tuple(
            _signature_value(value) for value in signature_values
        )


def main(args: list[str] | None = None) -> int:
    rclpy.init(args=args)
    node: PoleVisualizer | None = None
    try:
        node = PoleVisualizer()
        plt.show(block=False)
        while rclpy.ok() and not node._closed:
            rclpy.spin_once(node, timeout_sec=0.05)
            node.redraw_if_needed()
            plt.pause(0.01)
    except KeyboardInterrupt:
        pass
    except Exception as exception:
        print(f"micro_lqr_pole_visualizer failed: {exception}", file=sys.stderr)
        return 1
    finally:
        if node is not None:
            node.destroy_node()
        if rclpy.ok():
            rclpy.shutdown()
        plt.close("all")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
