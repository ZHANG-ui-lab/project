#include <stdio.h>

// a代表第几项
int fibonacci(int a) {
    if (a == 1 || a == 2) {
        return 1;
    }
    int fib_1 = 1, fib_2 = 1, fib;
    for (int i = 3; i <= a; i++) {
        fib_2 = fib_1 + fib_2;
        fib_1 = fib_2 - fib_1;
    }
    return fib_2;
}
// fib_1 代表该项的前前项，fib_2 代表该项的前一项

int main() {
    int n;
    // n代表测试数据的组数
    scanf("%d", &n);
    for (int i = 0; i < n; i++) {
        int a;
        scanf("%d", &a);
        printf("%d\n", fibonacci(a));
    }
    return 0;
}