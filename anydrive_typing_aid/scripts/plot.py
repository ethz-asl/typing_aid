#!/usr/bin/env python
# coding=utf-8

import matplotlib.pyplot as plt

class plot:
    # plot(f, labelx = False) default setting
    def plot (f,labely):
        plt.plot(f)
        if labely:
            label = input("name of label y")
            plt.ylabel(label)
        plt.xlabel('iter')
        plt.show()