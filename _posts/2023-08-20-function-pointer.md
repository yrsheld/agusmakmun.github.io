---
layout: post
title:  "Function pointer"
date:   2023-08-20 17:57:00 +0700
categories: [C]
---

## How are functions called?
Actually, in C, functions are called via function pointer.

Ex: 

* `printf`: function name / designator
* `&printf`: function pointer

When we call `printf`, it would be implicitly converted to `&printf`.
i.e.,

```c
printf("Hello");

//is actually
(&printf)("Hello"); //printf -> &printf

//or
(*printf)("Hello"); //*printf -> *&printf -> printf -> &printf
```


So `printf`, `&printf`, and `*printf` all work, and they are essentially doing the same thing, that is `&printf`.


## Usage: for copying functions, and function pointers can be passed as function argument.


### Copy function
Functions cannot be directly copied. For instance,


```c
void hello();
void func() = hello;   //(x) compile error
```


But with function pointers, it is possible to copy the address of the function and call it via pointer.


```c
void hello();
void (*func)() = hello;  //(o) hello->&hello

func();  //call function via pointer
```

Some points worth notice: 

* when a function pointer is declared, it needs to be specified as a pointer. (i.e., need to add * before it)
* When calling a function via pointer, `func()` = `(*func)()` = `(**func)()` = `(********func)()`. The reason is that func would always be converted to &func implicitly.


### Pass function pointers as arguments


```c
void A(int (*B)(int))
```

As pointer B refer to different functions, the function A becomes general and could perform different tasks.

Example:

We would like to define a function that performs calculation, including addition and multiplication of numbers from 1 to 5.

```c
int add(int, int);
int multiply(int, int);

int main(){
    int addition = calculate(add); //int (*op)(int, int) = add  
    int multiplication = calculate(multiply); //int (*op)(int, int) = multiply
}

int calculate(int (*op)(int, int)){
    int result = 1;
    for(int i=2; i<=5; i++){
        result = op(result, i);
    }
    return result;
}

int add(int a, int b){
    return a+b;
}

int multiply(int a, int b){
    return a*b;
}
```