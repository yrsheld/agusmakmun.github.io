---
layout: post
title:  "Random number generator in C"
date:   2023-07-10 15:30:00 +0700
categories: [C]
---

We can either use function from C library, or implement it on ourselves.

## Use C library

From <stdlib.h>

* `int rand(void)`: return random integer in range between 0 ~ RAND_MAX

* `void srand(unsigned int seed)`: set random seed


Example:

``` c
#include <stdio.h>
#include <stdlib.h>

int main(){
    srand(0);
    for(int i=0;i<5;i++){
        printf("%d\n", rand());
    }
}
```


To have different random seeds each time, we can make use of the current time. Note that, it is a simple but imperfect way. (Ex: making use of the computer time may be better)

``` c
#include <stdio.h>
#include <stdlib.h>
#include <time.h>

int main(){
    srand(time(0));
    for(int i=0;i<5;i++){
        printf("%d\n", rand());
    }
}
```

## Self-defined random number generator

### Linear congruential generator:

$x_0$ = 1

$x_{n+1}$ = ($x_{n}$ * a + c) % m

* $x_0$ is the random seed.
* a, c, m should follow some relation.


#### Example

$x_{n+1}$ = ($x_{n}$ * 1103515245 + 12345) % 4294967296, where **4294967296** is **UINT_MAX + 1**.
By doing so, we can get (unsigned int) random number, ranging from 0 ~ UINT_MAX (4294967295).

```c
unsigned int next = 1;
for(int i=0;i<5;i++){
    next = next * 1103515245 + 12345;
    printf("%u ", next);
}
```

It is not necessary to explicitly state `% 4294967296`. 

Reason: It is automatically done in unsigned int overflow. When the value of an unsigned int exceeds UINT_MAX, C would automatically calculate the result as `value % (UINT_MAX+1)`.

#### Result

This would **generate sequences of number, ranging from 0~n, appearing again and again starting from the same first number**. In a single sequence, there is **no repitition**.


#### Further modification

As seen from the result, the same sequence shows up again and again. In more realistic random number generation, we actually don't care about whether a same number shows up multiple times in the sequence, and having a same sequence showing up reptitively is somewhat not that "random."

Hence, to add more randomness, we can modify the original approach by **only taking the MSBs** (i.e., don't take all 32 bits from the calculated unsigned int value). By doing so, we may be able to see repitition in a single round, and the result becomes more random.

Example:
Only take the 1st~15th bit.
i.e., $rand_n$ = $x_n$ / $2^{16}$ % $2^{15}$


#### Final implementation of self-defined random function

We can make use of the aforementioned ideas to **implement our own random function** - `srand()`, `rand()`

```c
#include <stdio.h>
#include <time.h>

static unsigned int _next = 0;

int rand(){
    _next = _next * 1103515245 + 12345;
    return (_next / 65536) % 32768;  //(_next / 2^16) % (2^15)
}

void srand(unsigned int seed){
    _next = seed;    
}

int main(){
    srand(time(0));
    for(int i=0;i<5;i++){
        printf("%d\n", rand());  //0 ~ 2^15-1
    }
    return 0;
}
```

## Summary

To generate (pseudo) random integers, one can make use of the exsiting function in <stdlib.h> `rand`, `srand`, or we could define our own versions of these functions, following the concept of linear congruential generator.

