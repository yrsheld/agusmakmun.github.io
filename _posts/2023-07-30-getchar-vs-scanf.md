---
layout: post
title:  "getchar vs. scanf"
date:   2023-07-30 10:30:00 +0700
categories: [C]
---

## standard input stream (sdtin)

* `FILE * stdin`
* Buffered I/O
i.e., keyboard -> Buffer -> getc, getchar, scanf
So we are actually reading from the buffer


## getchar (from <stdio.h>)

`int getchar(void)`: get character from stdin

* cannot take argument
* return integer (ASCII code of character)

```c
# include<stdio.h>

int main(){
    int ch = getchar();
    printf("%d\n", ch);
    return 0;
}
```

## getc (from <stdio.h>)

`int getc(FILE * stream)`: get character from stream

* similar to getchar, but now takes a file pointer as argument (hence more general)

So we can also read from standard input stream


```c
int ch = getc(stdin);  //i.e., int ch = getchar();
```


## scanf

`int scanf (const char * format, ...)`: read formatted data (有格式的資料) from stdin

* arguments: specified data format, indefinite number of pointers
* return number of successfully read data

ex: 

```c
int res = scanf("%d%d", &a, &b);  //res == 2, if successful
```


### scanf vs. getchar

Ex: for reading in number as input

getchar read character by character, and we need to specify the end of current data.

```c
int main(){
    int input = getchar();
    int grade = 0;
    while(input >= '0' && input <= '9'){
        grade = grade*10 + (input-'0');
        input = getchar();
    }
    
    printf("%d", grade);
    retrun 0;
}
```

scanf is more convenient. Although in the implementation, it also reads character by character, but on the outside, it would automatically determine the split points of input string, and assign value to each via pointer.

ex:
```c 
//input: 68192asedasej...
scanf("%d", &grade);
printf("%d\n", grade); //68192
```

## Implement scanf using getchar

The number of pointers is indefinite. To deal with this:

1. `void myscanf(const char * format, ...)` : ... denotes infinite number of arguments
2. use `va_list` (from <stdargs.h>) to handle the arguments

Now we could implement the simplified version of scanf, that only handles format: int & float. 


```c
# include <stdio.h>
# include <stdargs.h>
# include <math.h>

void myscanf(const char* format, ...) {
    const char* fc = format;
    va_list args;
    va_start(args, format);

    while (*fc!='\0') {
	if (*fc == '%') {
	    fc++;
	    continue;
	}
        char c = *fc;
	fc++;

	int ch = getchar();

	switch (c) {
	case 'd':
	{
	    int num_int = 0;
	    while (ch >= '0' && ch <= '9') {
	        num_int = num_int * 10 + (ch - '0');
	        ch = getchar();
	    }
	    *va_arg(args, int*) = num_int;
	}
	break;
	case 'f':
	{
	    float num_float = 0;
	    while (ch >= '0' && ch <= '9') {
		num_float = num_float * 10 + (ch - '0');
		ch = getchar();
	    }
	    if (ch == '.') {  //handle the decimal points
		ch = getchar();
		int n = 1;
		while (ch >= '0' && ch <= '9') {
		    num_float += (ch - '0') / pow(10, n++);
		    ch = getchar();
		}
	    }
	    *va_arg(args, float*) = num_float;
	}
	break;
        }
    }
}

int main() {
	int a; 
	float b;
	myscanf("%d%f", &a, &b);  //12 86.4
	printf("%d\n", a);  //12
	printf("%f\n", b);  //86.400002
}
```


## Summary

* getchar() & getc(stdin) has the same effect, both reading character by character from stdin, and return ASCII.
* scanf can read formatted data, and it already handls lots of things. (ex: for scanning intergers, the space or even end of line signs could be ignored)
* scanf itself is also implemented to read input char by char, but it finds out where to cut based on specified datatype.