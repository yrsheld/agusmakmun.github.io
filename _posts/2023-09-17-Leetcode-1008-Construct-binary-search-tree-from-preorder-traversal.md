---
layout: post
title:  "Leetcode 1008. Create Binary Search Tree from preorder traversal"
date:   2023-09-17 17:57:00 +0700
categories: [Leetcode]
---

[Link to Problem](https://leetcode.com/problems/construct-binary-search-tree-from-preorder-traversal/description/)

## Idea 1.

### Implementation

```cpp
struct TreeNode* createTree(int* preorder, int start, int end){
    if(start==end) return NULL;

    struct TreeNode* root = malloc(sizeof(struct TreeNode));
    root->val = preorder[start];

    int upper = start+1;
    while(upper<end){
        if(preorder[start] < preorder[upper]) break;
        upper++;
    }
    
    root->left = createTree(preorder, start+1, upper);
    root->right = createTree(preorder,  upper, end);

    return root;
}

struct TreeNode* bstFromPreorder(int* preorder, int preorderSize){
    return createTree(preorder, 0, preorderSize);
}
```


### Complexity

Time: $O(n^2)$

Worst case: The whole preorder traversal is in descending order, ex: [5,4,3,2,1]. In this case, we need to look till the end of the array to seek the upper bound (i.e., the end of left subtree) every time.

## Idea 2. (Better!)

The goal is to traverse the preorder array only once s.t. time complexity becomes of $O(n)$.

So we need to do two things at the same time:

1. Create the tree
2. Tell me which index it has reached (i.e. where is the upper bound)

This can be achieved by either recursion as before or can also be rewritten as loop.

### Recursion

In C, we could only return one single value. So we can make use of pointer `int* idx` (or a global variable) to denote the current index value. After constructing the left subtree, the idx pointer would be pointing at the upper bound, i.e., the start of the right subtree.

```cpp
struct TreeNode* createTree(int* preorder, int* idx, int preorderSize, int upperBound){
    if(*idx==preorderSize || preorder[*idx]>upperBound) return NULL;

    struct TreeNode* root = malloc(sizeof(struct TreeNode));
    root->val = preorder[*idx];
    (*idx)++;
    root->left = createTree(preorder, idx, preorderSize, root->val); //8 //5
    root->right = createTree(preorder, idx, preorderSize, upperBound);  //INT_MAX

    return root;
}

struct TreeNode* bstFromPreorder(int* preorder, int preorderSize){
    int* start = calloc(1, sizeof(int));
    return createTree(preorder, start, preorderSize, INT_MAX);
}
```

### Loop
We would maintain an array (actually a stack) to store all nodes that have been created but have not grown both their left and right children yet.

Going through the preorder traversal, **when it comes a value that is smaller than the top of the stack (i.e., the latest node that is pushed in the stack), it becomes the left child of that node**. And we push this node in the stack, making it the top of the stack.

On the other hand, if the current node is **larger than the top of the stack**, we need to go back to examine the nodes inside the stack one by one, to see where could we insert this node as a right child. The correct position would be: **the last node that is smaller than the current node. (i.e., the next node would be larger than current node). Then we append this node as the right child.** And this node becomes the new top of the stack! 

```cpp
struct TreeNode* bstFromPreorder(int* preorder, int preorderSize){
    struct TreeNode* root = malloc(sizeof(struct TreeNode));
    root->val = preorder[0];
    root->left = root->right = NULL;

    struct TreeNode* st[100];
    st[0] = root;    //push in the root
    int topIdx = 0;  //index within st

    for(int i=1;i<preorderSize;i++){
        struct TreeNode* node = malloc(sizeof(struct TreeNode));
        node->val = preorder[i];
        node->left = node->right = NULL;

        if(preorder[i]<st[topIdx]->val){  //to left
            st[topIdx]->left = node;
            topIdx++;
            st[topIdx] = node;
        }
        else{     //to right
            while(topIdx>0 && st[topIdx-1]->val<preorder[i]){
                topIdx--;
            }
            //st[topIdx]: the last one smaller than node
            st[topIdx]->right = node;
            st[topIdx] = node;
        }
    }

    return root;
}
```


