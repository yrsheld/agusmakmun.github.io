---
layout: post
title:  "Add MathJax to Jekyll websites"
date:   2023-05-30 03:25:00 +0700
categories: [others]
---

## Overview
* MathJax: a javascript library that renders LaTeX expressions to display math formulas and notations in HTML pages.
* However, some Mardown processors or web browsers (ex: Jekyll websites) don't support MathJax. 
* Luckily, this problem could be solved easily with the following 2 steps.

## Solution
### 1. Add in `_config.yml`

``` yml
kramdown:
  math_engine: mathjax
```

### 2. Add to layout html

Add below in the `<head>` section of html.
This includes specifying the MathJax configuration, telling MathJax how you plan to indicate the mathematics and notions. By defining it based on LaTex syntax (ex: `$...$` as default in-line delimiters), MathJax is enabled to intepret original LaTeX in Markdown.

``` xml
<script type="text/javascript"
  src="https://cdnjs.cloudflare.com/ajax/libs/mathjax/2.7.0/MathJax.js?config=TeX-AMS_CHTML">
</script>
<script type="text/x-mathjax-config">
  MathJax.Hub.Config({
    tex2jax: {
      inlineMath: [['$','$'], ['\\(','\\)']],
      processEscapes: true},
      jax: ["input/TeX","input/MathML","input/AsciiMath","output/CommonHTML"],
      extensions: ["tex2jax.js","mml2jax.js","asciimath2jax.js","MathMenu.js","MathZoom.js","AssistiveMML.js", "[Contrib]/a11y/accessibility-menu.js"],
      TeX: {
      extensions: ["AMSmath.js","AMSsymbols.js","noErrors.js","noUndefined.js"],
      equationNumbers: {
      autoNumber: "AMS"
      }
    }
  });
</script>
```

## Result
### In Markdown

```
* $v = J(q) * \dot{q}$
* $\dot{q} = J^{+}(q) * v$, where $J^{+}(q)$ is the pseudo-inverse of Jacobian
```

### Display in web browser
<img src="https://raw.githubusercontent.com/yrsheld/yrsheld.github.io/master/static/img/_posts/latex.png" alt= “” width="500" height="">

## Reference
1. [https://www.fabriziomusacchio.com/blog/2021-08-10-How_to_use_LaTeX_in_Markdown/](https://www.fabriziomusacchio.com/blog/2021-08-10-How_to_use_LaTeX_in_Markdown/)
2. [https://docs.mathjax.org/en/v2.7-latest/start.html](https://docs.mathjax.org/en/v2.7-latest/start.html)

    
