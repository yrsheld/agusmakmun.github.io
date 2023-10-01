---
layout: post
title:  "Leetcode 560. Subarray Sum Equals K"
date:   2023-09-12 17:57:00 +0700
categories: [Leetcode]
---

[Link to Problem](https://leetcode.com/problems/subarray-sum-equals-k/description/)

## Challenge

> Constraints:
> * $1 <= nums.length <= 2 * 10^4$
> * $-1000 <= nums[i] <= 1000$
> * $-10^7 <= k <= 10^7$

Note:
* nums[i] can be negative
* k (target sum) can be negative
* The value ranges are large....


## Idea 1. Brute force

Define sum(i, j) = nums[i]+...nums[j-1].
Instead of a 2D array, we can use a 1D array to store the prefix sum.
sum(i, j) = sum(0, j) - sum(0, i) = prefsum(j) - prefsum(i).

Our goal is to find (i, j) such that sum(i, j) == k. In another way, when we are at index j, **we try to find i such that prefsum(j) - prefsum(i) == k**.

An easy way to find such i is to just **examine the prefsum(idx) for all idx before j.** Hence, a double for-loop: $O(n^2)$

```cpp
int subarraySum(int* nums, int numsSize, int k){
    //nums
    int count = 0, sum = 0;
    int pref[numsSize+1];
    pref[0] = 0;
    //pref[i] = sum(0, i) = nums[0]+nums[1]+...nums[i-1]
    for(int i=1;i<=numsSize;i++){
        pref[i] = pref[i-1] + nums[i-1];
    }

    for(int j=0;j<=numsSize;j++){
        //search for pref[i] s.t. pref[j]-pref[i] == k
        int target = pref[j] - k;
        for(int i=0;i<j;i++){
            if(pref[i]==target) count++;
        }
    }

    return count;
}
```

## Idea 2. Lookup table

Intead of looking through all the prefsum(i) for all index i before j, we create a lookup table to record the occurences of each sum value.

```cpp
int subarraySum(int* nums, int numsSize, int k){
    int counter[SIZE] = {0}; 

    int sum = 0, count = 0;
    
    for(int j=0;j<=numsSize;j++){
        counter[sum]++;
        sum += nums[j];
        count += counter[sum-k];
    }

    return count;
}
```

Problem: The size of lookup table (SIZE) needs to be at least larger than the value range($-2*10^7$~$2*10^7$)   --> too huge....


## Idea 3. Hashtable

Size of hashtable needs to be at least larger than the array. Here, the array is of size $2*10^4$, we can set the size of the hashtable $10*10^4$.

### Define the hashtable

Each element contains the sum & its corresponding count.

```cpp
struct Entry{
    int sum;
    int count;
}

int CAPACITY = 100000;
//Hashtable (size: CAPACITY)
struct Entry** counters = calloc(CAPACITY, sizeof(struct Entry*));
```

* Key: sum
* Index: idx of the table array

So we need to design a **hashfunction for mapping key to index**` [0, CAPACITY)`. To keep within this value range, we take mod over the CAPACITY. But since the sum can be of negative value, `sum % CAPACITY` would be in value range `(-CAPACITY, CAPACITY)`. Hence, we need to add CAPACITY to it, and take the mod again, i.e., `(sum % CAPACITY + CAPACITY) % CAPACITY`.

As for the storing and querying the hashtable, there are many different designs, but they have to be designed in the exact same manner.


### Update hashtable

Given a value, to store in the hashtable
* Calculate the corresponding index with the hashfunction.
* Check if the counters[idx] is already occupied.
* If yes, check whether the sum value stored in there is indeed our current sum. 
    * If counter[idx]->sum == sum: have found the correct entry! Increment 1 to the count value.
    * else: **Collision** **(i.e., different key values being mapped to the same index)** A simple way to handle this is to go to the next idx. (i.e., keep going on until it finds its correct entry or it reaches an empty entry.)
* If not, meaning an empty entry, we can create an entry here and set the sum to current sum with count equals to 1.

> Note that there are many different design choices of the hashtable and the mechanism for handling the collision. What shown here is only a simple example.

### Implementation

```cpp
int CAPACITY = 100000;
struct Entry{
    int sum;
    int count;
};

int getIndex(int sum){
    //map sum to [0, CAPACITY)
    return (sum%CAPACITY+CAPACITY)%CAPACITY;
}

void addCount(struct Entry** counter, int sum){
    int idx = getIndex(sum);
    while(counter[idx]!=NULL){
        if(counter[idx]->sum==sum){
            counter[idx]->count++;
            return;
        }
        idx = getIndex(idx+1);
    }
    counter[idx] = malloc(sizeof(struct Entry*));
    counter[idx]->sum = sum;
    counter[idx]->count = 1;
}

int query(struct Entry** counter, int sum){
    int idx = getIndex(sum);
    while(counter[idx]!=NULL){
        if(counter[idx]->sum == sum){
            return counter[idx]->count;
        }
        idx = getIndex(idx+1);
    }
    return 0;
}

int subarraySum(int* nums, int numsSize, int k){
    struct Entry** counter = calloc(CAPACITY, sizeof(struct Entry*));
    int count = 0, sum = 0;

    for(int i=0;i<numsSize;i++){
        addCount(counter, sum);
        sum += nums[i];
        count += query(counter, sum-k);
    }

    for(int i=0;i<CAPACITY;i++){
        free(counter[i]);
    }
    free(counter);
    
    return count;
}
```

### SideNote

Many languages already have the built-in hashtable that is ready-to-use. For instance, **unoredered_map** in C++ and **dict** in Python.

So we can rewrite it in C++ as below without the need for creating the hashtable from scratch.

```cpp
int subarraySum(vector<int>& nums, int k) {
    int count = 0, prefsum = 0;
    unordered_map<int, int> m; //(sum, count)
        
    for(int i=0;i<nums.size();i++){
        m[prefsum]++;
        prefsum += nums[i];
        if(m.find(prefsum-k) != m.end())
            count += m[prefsum-k];
    }
        
    return count;
}
```