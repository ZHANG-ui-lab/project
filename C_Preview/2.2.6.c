#include <stdio.h>
#include <string.h>

// 定义一个足够大的数字来存储结果的位数。
#define MAX_DIGITS 10000

// 函数：将数组 b 加到数组 a 上，结果存储在 a 中。
void add(int a[], int b[], int *len_a, int len_b) {
    int carry = 0; // 进位
    // 从个位开始，逐位相加
    for (int i = 0; i < *len_a || i < len_b; i++) {
        int sum = a[i] + b[i] + carry;
        a[i] = sum % 10; // 当前位的结果
        carry = sum / 10;  // 计算此处需不需要进位
    }
    // 如果最后还有进位
    if (carry > 0) {
        a[*len_a] = carry;
        (*len_a)++;
    }
}

int main() {
    int n;
    scanf("%d", &n);

    // 基础情况，直接输出
    if (n == 1) {
        printf("1\n");
        return 0;
    }
    if (n == 2) {
        printf("2\n");
        return 0;
    }

    // a = f(n-2), b = f(n-1), c = f(n);   f(n) = f(n-1) + f(n-2)
    int a[MAX_DIGITS] = {0};
    int b[MAX_DIGITS] = {0};
    int c[MAX_DIGITS] = {0};
    
    // 初始化：f(1) = 1, f(2) = 2
    a[0] = 1; int len_a = 1;
    b[0] = 2; int len_b = 1;

    // 从 f(3) 开始计算，直到 f(n)
    for (int i = 3; i <= n; i++) {
        // 将 a 的值复制到 c
        memcpy(c, a, sizeof(int) * MAX_DIGITS);
        int len_c = len_a;
        add(c, b, &len_c, len_b);

        memcpy(a, b, sizeof(int) * MAX_DIGITS);
        len_a = len_b;

        memcpy(b, c, sizeof(int) * MAX_DIGITS);
        len_b = len_c;
    }

    for (int i = len_b - 1; i >= 0; i--) {
        printf("%d", b[i]);
    }
    printf("\n");

    return 0;
}