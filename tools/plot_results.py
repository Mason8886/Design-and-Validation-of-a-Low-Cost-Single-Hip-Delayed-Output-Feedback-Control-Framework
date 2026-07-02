from __future__ import annotations

import argparse
from pathlib import Path

import matplotlib.pyplot as plt
import pandas as pd


def main() -> None:
    parser = argparse.ArgumentParser(description="Plot DOFC simulation CSV results.")
    parser.add_argument("csv", type=Path, help="CSV produced by simulate_dofc")
    parser.add_argument("--out", type=Path, default=Path("data/simulation_plot.png"))
    args = parser.parse_args()

    data = pd.read_csv(args.csv)

    fig, axes = plt.subplots(3, 1, sharex=True, figsize=(10, 7))
    axes[0].plot(data["time_s"], data["hip_angle_rad"])
    axes[0].set_ylabel("hip angle [rad]")
    axes[0].grid(True)

    axes[1].plot(data["time_s"], data["dofc_raw_torque_nm"], label="raw")
    axes[1].plot(data["time_s"], data["exo_torque_nm"], label="limited")
    axes[1].set_ylabel("torque [Nm]")
    axes[1].legend()
    axes[1].grid(True)

    axes[2].plot(data["time_s"], data["assistance_scale"])
    axes[2].set_ylabel("assist scale")
    axes[2].set_xlabel("time [s]")
    axes[2].grid(True)

    fig.tight_layout()
    args.out.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(args.out, dpi=160)
    print(f"saved {args.out}")


if __name__ == "__main__":
    main()
