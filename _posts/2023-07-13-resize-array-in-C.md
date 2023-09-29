---
layout: post
title:  "Resize array in C"
date:   2023-07-13 10:30:00 +0700
categories: [C]
---

## Challenge

Array elements are stored in contiguous locations. The size of array is determined at initialization.

```cpp
int arr[3];

// or after C99, 
// int variables can be used to declare array size
int length = 3;
int arr[length];
```

But in many cases, the array size is unknown or can change dynamically.
For instance, read in user input integers, and store in integer array.

## Idea1: create a larger array and copy the values to it

```cpp
#include <stdio.h>
#include <stdlib.h>

int main(){
    int numbers[0];  //1. invalid
    int length = 0;
    while(1){
        int input;
        scanf("%d", &input);
        if(input==0) break;
        int larger[length+1];
        for(int i=0;i<length;i++){
            larger[i] = numbers[i];
        }
        numbers = larger;  //2. invalid 
        numbers[length] = input;
        length++;
    }
    
    printf("Numbers: ");
    for(int i=0;i<length;i++){
        printf("%d ", numbers[i]);
    }
    
    printf("\n");
    return 0;
}
```

However, there are multiple problems with this program.
1. array size could not be set as 0.
2. array can not be directly assigned. (i.e., array cannot be directly copied, and thus cannot be put at the left side of the equal sign.)

## Idea2: (Better) Dynamically allocate memory space
### malloc

From stdlib.h,

```cpp
void* malloc(size_t size);
```

* size: number of bytes to allocate
* return type is void*, which could be implicitly convert to any data type.


#### Insights 1.

Before C99, if we want to use integer to declare array size, we would use malloc.

```cpp
int length = 3;
int* arr = malloc(sizeof(int)*length);
```

#### Insights 2.

We should not return the addresses of local variables. The reason is that, the lifetime ends as soon as the function execution finishes.

For instance,

```cpp
int* f(){
    int arr[3] = {0, 1, 2};
    return arr;
}

int main(){
    int* result = f();  //(x)
    
    for(int i=0;i<3;i++){
        printf("%d ", result[i]); //would not be 0 1 2
    }
    return 0;
}
```

The returned result would not be the same as the local array `int arr[3]` in the `int* f()` function, because the local array is released as the function finishes.

To prevent the local array from being automatically released, we should allocate its space with malloc.

```cpp
int* f(){
    int* arr = malloc(sizeof(int)*3); //(O)
    return arr;
}

int main(){
    int* result = f();
    
    for(int i=0;i<3;i++){
        printf("%d ", result[i]); //0 1 2
    }
    return 0;
}
```

### calloc
```cpp
void* calloc(size_t size, size_t size);
```
The only difference is that, calloc would **initialize all bytes as zero**.
For instance:
```cpp
//1D array (100)
int* arr = calloc(100, sizeof(int));

//2D array (100x100)
int** mat = malloc(sizeof(int*)*100);
for(int i=0;i<100;i++){
    mat[i] = calloc(100, sizeof(int));
}
```
### free

While malloc allocates memory space that would not automatically released, it could cause memory leak if not released properly.

Hence, it is important to take care of this issue by freeing spaces that are no longer needed.

Take the example above, we should free the `result`in the end.

```cpp
int* f(){
    //....
}

int main(){
    int* result = f();
    
    for(int i=0;i<3;i++){
        printf("%d ", result[i]); //0 1 2
    }
    
    free(result);
    return 0;
}
```


* If pointer is not given a value (i.e., no value assigned to it), freeing the pointer yields undefined behavior.
* To prevent this, we should **initialize pointers as NULL** (ex: `int* numbers = 0`; or `int* numbers = NULL;`). The free method would check whether the pointer is NULL and would to nothing if it's NULL.

### realloc
Back to the dynamic resizing of array, the idea is to reallocate a larger memory space.

In stdlib.h,

```cpp
void* realloc(void* ptr, size_t size);
```

* ptr: points to the start of the original array
* size: the desired new memory space (bytes)
* returns the address of the start of reallocated memory space. (It's possible the address is still the same, in case there is enough space.)

With this, we can increase the size of array by reallocating a larger memory space and move the whole array into it.

Back to the original problem, that is, reading an indefinite number of user inputs, and store them into integer array. From Idea1, we moved each array element one by one to a larger array. Now we can rewrite the program from using realloc.

```cpp
#include <stdio.h>
#include <stdlib.h>

int main(){
    int* numbers = 0;  //initialize as NULL
    int length = 0;
    
    while(1){
        int input;
        scanf("%d", &input);
        if(input==0) break;
        numbers = realloc(numbers, sizeof(int)*(length+1));
        numbers[length] = input;
        length++;
    }
    
    printf("Numbers: ");
    for(int i=0;i<length;i++){
        printf("%d ", numbers[i]);
    }
    
    printf("\n");
    return 0;
}
```

## Application

[Leetcode 49 Group Anagram](https://leetcode.com/problems/group-anagrams/description/)

The problem is not difficult, but rather tricky to write in C...

The challenge is that, we don't know 
* number of anagram groups
* number of elements in each group
* size of each element (i.e., length of each string)

All of the above would be determined along the way, and so the size of array is continuously changing.

Example solution:

```cpp
typedef struct{
    char* original;
    char* sorted;
 }Pair;

int cmpChar(const void * a, const void * b){
    return *(const char*)a - *(const char*)b;
}

int cmpPair(const void * a, const void *b){
    const Pair* pa = (const Pair*) a;
    const Pair* pb = (const Pair*) b;

    return strcmp(pa->sorted, pb->sorted);
}

char *** groupAnagrams(char ** strs, int strsSize, int* returnSize, int** returnColumnSizes){
    
    Pair* pairs = malloc(sizeof(Pair)*strsSize);
    //sort each string
    for(int i=0;i<strsSize;i++){
        int strlength = strlen(strs[i])+1;
        char* original = malloc(sizeof(char)*strlength);
        char* sorted = malloc(sizeof(char)*strlength);
        strcpy(original, strs[i]);
        strcpy(sorted, strs[i]);
        
        qsort(sorted, strlength-1, sizeof(char), cmpChar);

        pairs[i].original = original;
        pairs[i].sorted = sorted;
    }

    //sort the Pairs based on the sorted string
    qsort(pairs, strsSize, sizeof(Pair), cmpPair);

    //fill in the first string
    int groupIdx = 0;
    int subIdx = 0;

    char*** result = malloc(sizeof(char**)*1);
    result[groupIdx] = malloc(sizeof(char*)*1);
    result[groupIdx][0] = malloc(sizeof(char)*(strlen(pairs[0].original)+1));
    strcpy(result[groupIdx][0], pairs[0].original);

    char* last = malloc(sizeof(char)*(strlen(pairs[0].original)+1));
    strcpy(last, pairs[0].sorted);
    
    *returnColumnSizes = malloc(sizeof(int)*1);
    *returnColumnSizes[0] = 1;
    *returnSize = groupIdx+1;

    //fill in result
    for(int i=1;i<strsSize;i++){
        if(strcmp(pairs[i].sorted, last)==0){  //add to the last group
            subIdx++;
            (*returnColumnSizes)[groupIdx] = subIdx+1;
            result[groupIdx] = realloc(result[groupIdx], sizeof(char*)*(subIdx+1));
            result[groupIdx][subIdx] = malloc(sizeof(char)*(strlen(pairs[i].original)+1));
            strcpy(result[groupIdx][subIdx], pairs[i].original);
        }
        else{                                  //start a new group
            groupIdx++;
            subIdx=0;
            *returnSize = groupIdx+1;
            *returnColumnSizes = realloc(*returnColumnSizes, sizeof(int)*(groupIdx+1));
            (*returnColumnSizes)[groupIdx] = 1;

            result = realloc(result, sizeof(char**)*(groupIdx+1));
            result[groupIdx] = malloc(sizeof(char*)*1);
            result[groupIdx][0] = malloc(sizeof(char)*(strlen(pairs[i].original)+1));
            strcpy(result[groupIdx][0], pairs[i].original);

            //update the last anagram
            last = realloc(last, sizeof(char)*(strlen(pairs[i].original)+1));
            strcpy(last, pairs[i].sorted);
        }
    }
    
    return result;
}
```
