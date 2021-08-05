#!/usr/bin/env python

from balance_toy import BalanceToy

env = BalanceToy()

for i in range(1000):
    env.render()