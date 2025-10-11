import pandas as pd
import matplotlib.pyplot as plt
import pandas as pd
import matplotlib.pyplot as plt


def plot_log(df: pd.DataFrame, title: str):
    """Plot GPS and altimeter altitudes from a log DataFrame."""
    print("Sanity check..")
    print(df.head(), "\n")
    plt.figure(figsize=(10, 5))
    plt.plot(df["timestamp"], df["gps_altitude"], label="GPS altitude", color="blue")
    plt.plot(df["timestamp"], df["altimeter_1_altitude"], label="Altimeter 1", color="orange")
    plt.plot(df["timestamp"], df["altimeter_2_altitude"], label="Altimeter 2", color="green")
    plt.title(title)
    plt.xlabel("Time (s)")
    plt.ylabel("Altitude (m)")
    plt.legend()
    plt.grid(True)
    plt.tight_layout()
    plt.show()


def main():
    # Load logs
    log1 = pd.read_csv("log1.csv")
    log2 = pd.read_csv("log2.csv")

    # Plot each with appropriate title
    plot_log(log1, "Log 1 — Faulty Altimeter Behavior")
    plot_log(log2, "Log 2 — normal Altimeter Behavior")


if __name__ == "__main__":
    main()



# Run from data directory:  ../.venv/bin/python3 plotting_data.py