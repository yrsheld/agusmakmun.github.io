---
layout: post
title:  "Add conda environment kernel to jupyter notebook"
date:   2022-04-20 10:32:00 +0700
categories: [conda]
---

## Problem
Already installed a bunch of packages in conda environment. Try to run jupyter notebook in that kernel in order to make use of the packages.

However, by first activating the conda environment and start jupyter notebook does not solve the problem. There conda environment kernel does not show up in the kernel list, and actually it is not even created yet...

## Solution

> replace all $(myenv) with the name of conda environment

### Step1. Activate conda environment
```
conda activate $(myenv)
```

### Step2. Install ipykernel in the conda environment

ipykernel provides Ipython kernel for jupyter

```
pip install --user ipykernel
```

### Step3. Add virtual env

```
python -m ipykernel install --user --name=$(myenv)
```

### Done!
The kernel is now created and could be used regardless whether the conda environment is activated or not. 

You can see it in the kernel menu after starting Jupyter Notebook.


