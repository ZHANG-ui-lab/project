#include <stdio.h>
#include <stdlib.h>

// 比较函数
int compare(const void *a, const void *b) {
    return (*(int *)a - *(int *)b);
}

int main() {
    int N;
    scanf("%d", &N);

    // 动态分配内存存储N个整数
    int *arr = (int *)malloc(N * sizeof(int));
    if (arr == NULL) {
        printf("内存分配失败\n");
        return 1;
    }

    // 读取N个整数到数组里
    for (int i = 0; i < N; i++) {
        scanf("%d", &arr[i]);
    }

    // 使用 qsort 对数组进行升序排序
    qsort(arr, N, sizeof(int), compare);

    // 输出排序后的数组
    for (int i = 0; i < N; i++) {
        //除了第一个数，其他数前加空格
        if (i != 0) {
            printf(" ");
        }
        printf("%d", arr[i]);
    }
    printf("\n");

    // 释放动态分配的内存
    free(arr);
    return 0;
}