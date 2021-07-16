from anydrive_typing_aid.utils.utilities import polynomial5, sigmoid
import matplotlib.pyplot as plt


def main():
    # t, y, y_d, y_dd = polynomial5(0.0, 4.0, 1.0, 4.0, 3.0, 1.0 / 100.0)
    t, y = sigmoid(0.0, 0.5, 0.5, 2.5, 0.05, 1.0 / 150.0)
    plt.plot(t, y)
    # plt.plot(t, y_d)
    # plt.plot(t, y_dd)
    plt.grid(True)
    plt.show()


if __name__ == "__main__":
    main()
