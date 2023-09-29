---
layout: post
title:  "Leetcode 238. Product of Array Except Self"
date:   2023-09-02 17:57:00 +0700
categories: [C, Leetcode]
---

[Link to problem](https://leetcode.com/problems/product-of-array-except-self/)
Here discuss a solution, written in C, to this problem.

## Challenge
* No division
* O(n) time
* O(1) extract space, aside from the returned array

## Idea
For the i-th element, the result 
= (the product from 0~(i-1)-th) $\times$ (the product from (i+1)~(n-1)th)

= left product $\times$ right product

So we can formulate two arrays. One is for storing the left product, the other is for storing for the right product.

```cpp
int* product = malloc(sizeof(int)*n);

int* left = malloc(sizeof(int)*n);
int* right = malloc(sizeof(int)*n);

left[0] = 1;
for(int i=1;i<n;i++){
    left[i] = left[i-1] * nums[i-1];
}

right[n-1] = 1;
for(int i=n-2;i>=0;i--){
    right[i] = right[i+1] * nums[i+1];
}

for(int i=0;i<n;i++){
    product[i] = left[i] * right[i];
}
```

Complexity:
* Time: O(n), iterates 3 times
* Space: O(n), with two arrays (left& right) created.

But our goal is to have O(1) extra space. So next steps are to get rid of those two arrays.
### 1. Get rid of the left array
We currently have 3 for loops. The loop for calculating left and final product result are both going from beginning to the end.
So we can actually merge them together and update the left product on the fly, instead of storing the result in the array.

```cpp!
//merge the 1st & 3rd for loop
for(int i=1;i<n;i++){
    left = left * nums[i-1];
    product[i] = left * right[i];
}
```
### 2. Get rid of the right array
We cannot merge the for loop for calculating right product with the others, since they are operating in different direction. So we still need to calculate the right product beforehand.

But now the question is, do we really need an additional array to store the right product?

Actually, NO! We can just store inside the final product array.
```cpp!
//calculate right product
product[n-1] = 1;
for(int i=n-2;i>=0;i--){
    product[i] = product[i+1] * nums[i+1]; 
}

//merge the 1st & 3rd for loop
for(int i=1;i<n;i++){
    left = left * nums[i-1];
    product[i] = left * product[i];
}
```


## Implementation
```cpp
int* productExceptSelf(int* nums, int numsSize, int* returnSize){
    int* product = malloc(sizeof(int)*numsSize);

    *returnSize = numsSize;
    
    product[numsSize-1] = 1;
    for(int i=numsSize-2;i>=0;i--){
        product[i] = product[i+1]*nums[i+1];
    }

    int left = 1;
    for(int i=1;i<numsSize;i++){
        left = left * nums[i-1];
        product[i] = left * product[i];
    }

    return product;
}
```

## Reference
* [https://www.youtube.com/watch?v=uvlHzm6N0Y8&list=PLY_qIufNHc29OLToHki4t0Z5KIhUzdYxD&index=15](https://www.youtube.com/watch?v=uvlHzm6N0Y8&list=PLY_qIufNHc29OLToHki4t0Z5KIhUzdYxD&index=15)