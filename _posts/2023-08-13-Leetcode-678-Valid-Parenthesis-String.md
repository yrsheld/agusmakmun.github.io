---
layout: post
title:  "Leetcode 678 Valid Parenthesis String"
date:   2023-08-20 17:57:00 +0700
categories: [C, Leetcode]
---

[Link to problem](https://leetcode.com/problems/valid-parenthesis-string/description/)

Here discuss two solutions, written in C, to this problem.
1. Recursion + Memoization
2. Count the parentheses

## Solution 1. Recursion + Memoization
Given a string, we look at the first character.

### Case1: `)`

Definitely invalid.

### Case2: `(`

Try to pair with a right parenthesis on the right side. But how to choose the corresponding right parenthesis?

#### Idea (x): Pair with the closest one?

This is ok if there are only left and right parentheses in the string. But now that there are also `*` involved, which can be treated as empty string or right parenthesis. A valid string may become invalid if a `*` is mistreated as right parenthesis instead of empty string.  For instance:

```
               idx 0 1 2 3 4 5 6
                   ( ( * ) ) ( )
pair 0 & 2  -->      (   ) ) ( )        
pair 1 & 3  -->            ) ( )
idx 4 is ')'-->   invalid
```

#### Idea (o): Try all possible pairs!

Search on the right side and try to pair with all possibilities (i.e., try with all `)` and `*`) until we end up finding a valid string.
The string is valid iff the substring within the pair, and the substring on the right side is also valid, i.e.,

```cpp
(left)right   //isValid(left) && isValid(right)
```

### Case3: `*`

It can be treated as `(` or `""` or `)`, and we would try our best to determine its value in a way such that the string can become valid.

Hence, we would try to see if by treating it as `(` or `""`  can end up a valid result, if not, then it is definitely invalid. So now we end up two subcases.


#### Case3-1: treated as `(`

If we treat it as `(`, then it is same as case 2.

#### Case3-2: treated as `""` (empty string)
Then we just go on and see if the string on the right side is valid, i.e.,

```cpp
*right   //isValid(right)
```

### Memoization

A function would check whether a given substring s[i:j] is valid. There would be many repetivie calculations and would TLE. To deal with this, we can build up a table to record all calculated values.
`valid[i][j]`: the validity of s[i:j]. 

* 0: not yet calculated
* 1: valid
* -1: invalid

### Implementation

```cpp
bool isValid(char * s, int** valid, int start, int end){  //[start, end]
    if(start>end) return true;  //empty
    if(valid[start][end]==1) return true;
    if(valid[start][end]==-1) return false;

    //case1: ')'
    if(s[start]==')'){
        valid[start][end] = -1;  //invalid
        return false;
    }

    //case3-2: '*'
    if(s[start]=='*'){
        //treat as ""
        if(isValid(s, valid, start+1, end)){
            valid[start][end] = 1;  //valid
            return true;
        }
    }
    
    //case2 & case3-1: '('
    //try to pair with ')' or '*' on the right
    for(int i=start+1; i<=end;i++){
        if(s[i]==')' || s[i]=='*'){
            //(left)right, both left & right needs to be valid
            if(isValid(s, valid, start+1, i-1) && isValid(s, valid, i+1, end)){
                valid[start][end] = 1;  //valid
                return true;
            }
        }
    }
    
    valid[start][end] = -1;  //invalid
    return false;
}

bool checkValidString(char * s){
    int length = strlen(s);
    int** valid = malloc(sizeof(int*)*length);
    //0: not yet visited, 1: valid, -1: invalid
    for(int i=0;i<length;i++){
        valid[i] = calloc(length, sizeof(int));
    }
    
    bool ans = isValid(s, valid, 0, strlen(s)-1);
    
    //free memory
    for(int i=0;i<length;i++){
        free(valid[i]);
    }
    free(valid);
    
    return ans;
}
```

## Solution 2. Count the parentheses

### Idea 1. (x)

Maintain a count value, +1 when encounter `'('`, and -1 when encounter `')'`. See at the end, whether the count == 0.

Problem: it only sums up the number of left/right parentheses, and does not care about the ordering. Also, the `'*'` is not taken care of.


### Idea 2. (o)

We see that we could not determine the validity only based on counting it to the end. (Because parentheses can only be cancelled out if left parenthesis is on the left side of the right parenthesis).

Instead, we should check along the way to check whether the cancellation makes sense.

#### Consider only left/right parentheses in the string

Whenever the count < 0, means there are too many `')'` that cannot be cancelled out by far and also could never be cancelled out further on.

Rule - **whenever count becomes < 0, then invalid!** 

You can think of the stack, and the count denotes **the number of elements inside the stack**.

* Whenver we meet a left parenthesis, put inside the stack (**count+1**). 
* If it is a right parenthesis, we cancel out with the top of the stack (i.e., the last left parenthesis we pushed into the stack.) (**count-1**). So if the stack is already empty, count becomes -1,  (**count<0**) --> it's clearly invalid!!

#### Consider star sign in the string

The fact that the star sign can be rendered as 3 possibilities make the whole thing trickier, because it leads to more possible combinations and different choices would affect the validity of string.

So instead of maintaining a single count, we should maintain a range of value instead , i.e., the **lower & upper value** of the range.

* For left/right parentheses, we would +1/-1 for both lower and upper bounds.
* For star sign, **the lower bound treats all star as right parentheses**, and thus -1, while **the upper treats all stars as left parenthesis**, thus +1.

If any point, the lower<0 (i.e. lower==-1), it means that too many star signs were rendered as right parentheses. So we switch a star sign from right parentheses to empty string, and thus +1 to lower --> set lower as 0.

> Note: if the star sign can only be treated as either left/right parentheses and not empty string, then we would need to switch a star sign from right parentheses to left parentheses, meaning to +2 --> in this case, we need to set lower as +1.

If any point, the upper<0, it means that even we had treated all stars as left parentheses in the past, there are still not enough left parentheses to cancel out with the right ones. Hence, must be invalid!

In the end, we should check **whether zero is within the range of [lower, upper]**. But since lower would always be >=0, we just need to check whether lower == 0.

### Implementation

```cpp
bool checkValidString(char * s){
    int length = strlen(s);
    int upper = 0, lower = 0;

    for(int i=0;i<length;i++){
        if(s[i]=='('){
            lower++;
            upper++;
        }
        else if(s[i]==')'){
            lower--;
            upper--;
        }
        else{
            lower--; //treat as ')' 
            upper++; //treat as '('
        }
        
        if(upper<0) 
            return false;
        if(lower<0)    //lower == -1
            lower = 0; 
    }

    return lower == 0;
}
```