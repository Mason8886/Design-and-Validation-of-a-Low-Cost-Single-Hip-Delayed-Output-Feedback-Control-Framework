from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def _has_column(data: pd.DataFrame, name: str) -> bool:
    return name in data.columns


def main() -> None:
    parser = argparse.ArgumentParser(
        description="Plot full-scale DOFC partial-assistance simulation CSV results."
    )
    parser.add_argument("csv", type=Path, help="CSV produced by simulate_dofc")
    parser.add_argument("--out", type=Path, default=Path("data/simulation_plot.png"))
    args = parser.parse_args()

    data = pd.read_csv(args.csv)
    time_s = data["time_s"]

    human_peak = data["human_torque_nm"].abs().max()
    exo_peak = data["exo_torque_nm"].abs().max()
    assist_ratio = exo_peak / human_peak if human_peak > 0.0 else 0.0

    fig, axes = plt.subplots(4, 1, sharex=True, figsize=(11, 8.5))
    fig.suptitle(
        "Full-scale hip torque simulation with limited partial exoskeleton assistance "
        f"(peak assist ratio {assist_ratio:.2%})"
    )

    axes[0].plot(time_s, data["hip_angle_rad"], color="tab:blue")
    axes[0].set_ylabel("hip angle [rad]")
    axes[0].grid(True)

    axes[1].plot(
        time_s,
        data["human_torque_nm"],
        color="tab:gray",
        linewidth=1.2,
        label="biological hip torque",
    )
    axes[1].plot(
        time_s,
        data["exo_torque_nm"],
        color="tab:orange",
        linewidth=1.8,
        label="limited exoskeleton torque",
    )
    if _has_column(data, "dofc_raw_torque_nm"):
        axes[1].plot(
            time_s,
            data["dofc_raw_torque_nm"],
            color="tab:red",
            alpha=0.45,
            linewidth=1.0,
            label="raw DOFC request",
        )
    axes[1].set_ylabel("torque [Nm]")
    axes[1].legend(loc="upper right")
    axes[1].grid(True)

    axes[2].plot(time_s, data["assistance_scale"], color="tab:green")
    axes[2].set_ylabel("assist scale")
    axes[2].set_ylim(-0.05, 1.05)
    axes[2].grid(True)

    plotted_flag = False
    if _has_column(data, "saturated"):
        axes[3].step(time_s, data["saturated"], where="post", label="saturated")
        plotted_flag = True
    if _has_column(data, "rate_limited"):
        axes[3].step(
            time_s,
            data["rate_limited"],
            where="post",
            label="rate limited",
        )
        plotted_flag = True
    if plotted_flag:
        axes[3].set_ylim(-0.05, 1.05)
        axes[3].legend(loc="upper right")
    axes[3].set_ylabel("safety flag")
    axes[3].set_xlabel("time [s]")
    axes[3].grid(True)

    fig.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=160)
    print(f"saved {args.out}")


if __name__ == "__main__":
    main()
