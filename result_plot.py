#!/usr/bin/env python3

import pip
import numpy as np

def install(package):
    pip.main(['install', package])

if __name__ == '__main__':
    try:
        import pandas as pd
    except ImportError:
        install('pandas')
        import pandas as pd

    try:
        import plotly
        import plotly.express as px
    except ImportError:
        install('plotly')
        import plotly
        import plotly.express as px

    try:
        import psutil
    except ImportError:
        install('psutil')
        import psutil

a = pd.read_csv('./build/results.csv')
a.replace([np.inf, -np.inf], np.nan).dropna(subset=["TTC"], how="all")
a = a[a['TTC'] < 100]


fig = px.box(a, x="Detector",color="Descriptor", y="TTC", points="all")
fig.write_html("ttc_all.html")
fig.write_image("ttc_all.svg")

fig = px.box(a, x="Descriptor",color="Detector", y="TTC", points="all")
fig.write_html("ttc_all_desc.html")
fig.write_image("ttc_all_desc.svg")


