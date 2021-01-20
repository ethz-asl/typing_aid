#!/usr/bin/env python
# coding=utf-8

import matplotlib.pyplot as plt

class plot:
    # plot(f, labelx = False) default setting
    def plot(self, x,f,labely):
        plt.plot(x,f)
        if labely:
            label = input("name of label y")
            plt.ylabel(label)
        plt.xlabel('iter')
        plt.show()
    
    def arg(self):
        x = input("x values")
        f = input("y values")
        labely = input("y label")
        return x, f , labely
    