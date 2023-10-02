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

```cpp
// leftShift = 3
abc | de

// reverse whole string
ed | cba

// reverse each substring
// length of the second substring ("abc") = leftShift
// length of the first substring ("de) = length - leftShift
de | abc
```

## Implementation

```cpp
void reverse(char* s, int start, int end){
    int i = start, j = end-1;
    while(i<j){
        //swap s[i] & s[j]
        char tmp = s[i];
        s[i] = s[j];
        s[j] = tmp;
        i++; j--;
    }
}

void shiftString(char* s, int leftShift){
    int length = strlen(s);
    int front = length - (leftShift % length);
    
    reverse(s, 0, length);          //[0, length)
    reverse(s, 0, front);       //[0, leftShift)
    reverse(s, front, length);  //[leftShift, length)
}
```

Note: right shift x characters == left shift (length-x) characters.

## Reference
* Leetcode 1427. Perform String Shift