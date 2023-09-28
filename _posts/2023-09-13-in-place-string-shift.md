---
layout: post
title:  "In-place string shift"
date:   2023-09-13 10:30:00 +0700
categories: [C]
---

## Goal
Shift the string with a certain amount of characters to the right/left side.

For instance, given a string "abcde", shift 3 characters to the left side, becomes "deabc".

## Insights
Instead of copying substrings and reconcatenate the whole string, we try to do it in-place.

Actually, it can be thought of as a series of reverse operations

abc | de

ed | cba  (reverse the whole string)

de | abc  (reverse each substring)


## Implementation
```cpp
void reverse(char* s, int start, int end){
    int i = start, j = end-1;
    while(i<j){
        //swap s[i] & s[j]
        char tmp = s[i];
        s[i] = s[j];
        s[j] = tmp;
    }
}

void shiftString(char* s, int leftShift){
    int length = strlen(s);
    leftShift %= length;
    
    reverse(s, 0, length);          //[0, length)
    reverse(s, 0, leftShift);       //[0, leftShift)
    reverse(s, leftShift, length);  //[leftShift, length)
}
```

## Reference
* Leetcode 1427. Perform String Shift