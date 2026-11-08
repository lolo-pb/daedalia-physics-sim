"""Plot an altitude-test CSV beside its source file (requires matplotlib)."""

import argparse
import csv
from pathlib import Path
import sys


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("csv_path", type=Path)
    args = parser.parse_args()
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
    except ImportError:
        sys.exit("Plotting requires matplotlib. Install it for this interpreter: "
                 + sys.executable + " -m pip install matplotlib")

    try:
        with args.csv_path.open(newline="") as source:
            rows = list(csv.DictReader(source))
        times = [float(row["time_seconds"]) for row in rows]
        heights = [float(row["height_change_ft"]) for row in rows]
        if not rows:
            raise ValueError("CSV has no samples")
        fig, ax = plt.subplots(figsize=(10, 5), layout="constrained")
        ax.plot(times, heights, linewidth=1.5)
        ax.set(xlabel="Simulated time (seconds)", ylabel="Height change (feet)",
               title="Altitude over time")
        ax.grid(True, alpha=0.3)
        destination = args.csv_path.with_suffix(".png")
        fig.savefig(destination, dpi=160)
        plt.close(fig)
        print(f"Altitude plot: {destination}")
    except (OSError, ValueError, KeyError) as error:
        sys.exit(f"Cannot plot altitude CSV: {error}")


if __name__ == "__main__":
    main()
