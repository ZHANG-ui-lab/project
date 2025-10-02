#include <stdio.h>

// 递归计算阶乘
int factorial(int n) {
    // 基本情况： 1! = 1
    if (n == 1) {
        return 1;
    }

    else if (n < 1 || n > 12) {
        return 0; // 表示错误
    }

    // n! = n * (n - 1)!
    else {
        return n * factorial(n - 1);
    }
}

int main() {
    int n;
    scanf("%d", &n);
    printf("%d\n", factorial(n));
    return 0;
}