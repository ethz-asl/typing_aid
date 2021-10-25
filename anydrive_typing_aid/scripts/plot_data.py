import os
from copy import deepcopy
import pandas as pd
import matplotlib.pyplot as plt


def main():
    save_dir = os.path.expanduser("~/Data/TypingAid")
    files = os.listdir(save_dir)
    log_files = deepcopy(files)

    files_to_remove = set()
    for f in log_files:
        if not f.endswith(".csv"):
            files_to_remove.add(f)
    for f in files_to_remove:
        log_files.remove(f)

    log_files.sort()

    print("Select which one to plot.")
    for i in range(len(log_files)):
        print("{}: {}".format(i, log_files[i]))

    idx = raw_input("Enter index: ")

    f = log_files[int(idx)]
    # datestring = "_".join(f.split("_")[:2])
    # plot_name = datestring + "_plot.pdf"
    # if plot_name in files:
    #     continue
    log_data = pd.read_csv(os.path.join(save_dir, f))
    if "ImpedanceController" in f:
        plt.plot(log_data["t"], log_data["tau_cmd"])
        plt.plot(log_data["t"], log_data["tau_meas"])
        plt.title("{}".format(f))
        plt.legend(["tau_cmd", "tau_meas"])
        plt.show()


if __name__ == "__main__":
    main()
