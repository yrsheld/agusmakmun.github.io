---
layout: post
title:  "Leetcode 201. Bitwise AND of Numbers Range"
date:   2023-09-06 17:57:00 +0700
categories: [Leetcode]
---

[Link to Problem](https://leetcode.com/problems/bitwise-and-of-numbers-range/description/)

## Idea 1. Brute Force (TLE)

Simply and over all values.

One thing worth notice is that, we take the and over left & right outside the loop. This is due to the fact that, they can possibly be INT_MAX ($2^{31}-1$). If `x = INT_MAX`, then x+1 would result in a negative value due to integer overflow. The condition `x < right` would never be true and would thus end up in an infinite loop.

```cpp
int rangeBitwiseAnd(int left, int right){
    int ans=left;
    for(int x=left+1;x<right;x++){
        ans &= x;
    }
    ans &= right;
    
    return ans;
}
```

## Idea 2. Extract the common digits

Extract the part where the digits of binary(left) & binary(right) are the same. Because when there is difference in i-th digit, it means that there have been  0/1 transitions in all 0~(i-1)-th digits, and thus taking and over these digits are all zero.

So we just need to extract the part where they share common digit values.

For instance:

```python
# left 5, right 7
5: 101
7: 111
   ^
   100
 
# left 82, right 87
82: 1010010
87: 1010111
       ^ 
    1010000
```

### Implementation 1.

We shall look from left to right, and stop until we encounter a difference in digit value.

To check whether the n-th digit of a number x is 1, simply take bitwise and with the bitmask, i.e., `x & (1<<n)`.
The largest bitmask would be $2^{30}$.


```cpp
int rangeBitwiseAnd(int left, int right){
    int ans=0;
    
    for(int n=30;n>=0;n--){
        int dleft = left & (1<<n);
        int dright = right & (1<<n);
        if(dleft == dright){
            ans += dleft;
        }
        else break;
    }
    return ans;
}
```


### Implementation 2.

We can think in another way. The goal is basically to retain the leftmost common part.

We can keep right-shifting both left and right values, and check whether they are of the same value. If not, it means that there are still difference in the remaining digits. If yes, then we have succesffuly found the common part, but just need to left-shift back to its original position.

For instance,

```python
# left 82, right 87
82: 1010010
87: 1010111        
# right shift (1)
     101001
     101011
# right shift (2)
      10100
      10101
# right shift (3)
       1010
       1010
# Done!
# left shift the 1010 << 3
    1010000
       
```

Final code:

```cpp
int rangeBitwiseAnd(int left, int right){
    int count = 0;
    
    while(left!=right){
        left = left >> 1;
        right = right >>1;
        count++;
    }

    return left << count;
}
```