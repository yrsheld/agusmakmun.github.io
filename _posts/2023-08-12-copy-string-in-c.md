---
layout: post
title:  "Copy string in C"
date:   2023-08-12 10:30:00 +0700
categories: [C]
---

## How to store a string
### Variable type
1. const char *
2. char [] (can either specify size or not)

The main difference is that, const char * is not possible to be modified further. (i.e., an immutable string).

### Given value
The value assigned can either be a 
- **string literal** (ex: "Hello", and C would automatically add '\0' at the end)
* **an array of characters** (ex: {'H','e','l','l','o', '\0'})

``` c
const char* str1 = "Hello";
char str2[6] = "Hello";
char str3[6] = {'H', 'e', 'l', 'l', 'o'};
```

## Manual copy
### Shallow copy
``` c
char str1[] = "test";
char* str2 = str1;  //shallow copy

//modify str2
str2[0] = "T";

//check result
printf(str1);  //Test, also modified
printf("\n");
printf(str2);  //Test
```

### Deep copy
```c
char str1[5] = "test";
char str2[5];

for(int i=0;i<5;i++){
    str2[i] = str1[i];  //Deep copy
}
```

## Use Function to copy
From string.h - ***strcpy***.
This would **Deep Copy**!
```c
char * strcpy (char * destination, const char * source);
//copy source to destination
```


Note that, although the function specifies that destination of type `char*`, directly declare it as this type is invalid. (Reason: destination is going to be modified.)

Hence, destination should be of type: `char []`, so that actual memory space is allocated to it, and thus able to be modified.

```c
const char* source = "test"; 
char destination[5]; //char * destination: invalid

strcpy(destination, source);

printf("%s\n", destination);
return 0;
```