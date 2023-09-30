---
layout: post
title:  "Leetcode 33. Search in Rotated Sorted Array"
date:   2023-09-28 17:57:00 +0700
categories: [C, Leetcode]
---


[Link to problem](https://leetcode.com/problems/search-in-rotated-sorted-array/description/)


## Challenge

* O(logn) time complexity

Apparently, it's about binary search....so that we can rule out half of the array each time, hence logn.

For a normal sorted array, it is sufficient to determine which part to rule out based on comparing the target value with the value in the middle of the array. (i.e., target > nums[mid] ?)

But here the array is rotated sorted, so even if the target is larger than nums[mid], the target could still possibly be on either side. To be certain about continuing the search in the left/right side, we need to also consider the values of the start or end.

For instance, given target > nums[mid]

* if nums[mid] > nums[start]: target must be on the right side
* else (nums[mid] < nums[start])
   * if target < nums[start]: right
   * else: left

So the tricky part is that we need to consider more subcases.

## Idea
By comparing nums[mid] & nums[start], we could determine where the actual minimum of the array is, and also, whether the left/right side is sorted. From this, we differentiate between these two cases, with two subcases each.

### Case1: `nums[mid]>nums[start]`

* the left part is sorted. 
* the actual minimum is on the right.
* `start-----mid   <min>    end`
   *  Case1.1: target on the left 
Only possibility: nums[start] <= target && target < nums[mid]
   * Case1.2: target on the right
Else

### Case2: `nums[mid]<nums[start]`

* the right part is sorted. 
* the actual minimum is on the left.
* `start <min> mid----end`
   * Case2.1: target on the right 
Only possibility: nums[mid] < target && target < nums[start]
   *  Case2.2: target on the left
Else


## Implementation

### Recursion

```cpp
int solve(int* nums, int numsSize, int target, int start, int end){
    if(start>=end) return -1;  //not found
    int mid = (start+end)/2;
    if(nums[mid] == target) return mid;
    
    if(nums[mid]>nums[start]){   //start~mid sorted (left sorted)
        //  start --------- mid  <min>  end
        if(target<nums[mid] && target>=nums[start])  //left
            return solve(nums, numsSize, target, start, mid);
        else   // right
            return solve(nums, numsSize, target, mid+1, end);
    }
    else{    //mid~end sorted (right sorted)
        //  start   <min>   mid -------- end
        if(target>nums[mid] && target<nums[start])   //right
            return solve(nums, numsSize, target, mid+1, end);
        else   //left
            return solve(nums, numsSize, target, start, mid);
    }
}

int search(int* nums, int numsSize, int target){
    return solve(nums, numsSize, target, 0, numsSize);
}
```


### Rewrite into loop

The recursion is of type - **tail call recusion** (尾端遞迴), i.e., the recursion call is at the end of the function. For some compilers, they can optimize the process without creating recursion stack. So maybe we don't need to worry about the additional memory space created because of the recursion.

But actually, tail call recursion **can be easily rewritten into loop**!

```cpp
int search(int* nums, int numsSize, int target){
    int start = 0, end = numsSize;
    int mid;
    while(start<end){
        mid = (start+end)/2;
        if(nums[mid] == target) return mid;

        if(nums[mid] >= nums[start]){   //start~mid sorted (left sorted)
            //  start --------- mid  <min>  end
            if(target<nums[mid] && target>=nums[start])  // left
                end = mid;
            else   // right
                start = mid+1;
        }
        else{    //mid~end sorted (right sorted)
            //  start   <min>   mid -------- end
            if(target>nums[mid] && target<nums[start])   // right
                start = mid+1;
            else   // left
                end = mid;
        }
    }

    return -1;
}
```

## Reference

* [https://www.youtube.com/live/DWA0HR_7RVY?si=8WtYIcbsNbYs9hS9](https://www.youtube.com/live/DWA0HR_7RVY?si=8WtYIcbsNbYs9hS9)

