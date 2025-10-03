#include <stdio.h>

// 判断一个数是否为质数
int determine(int n) {
    if (n <= 1) {
        return 0;
    }
    for (int i = 2; i * i <= n; i++) {
        if (n % i == 0) {
            return 0;
        }
    }
    return 1;
}

int main() {
    int L;
    scanf("%d", &L);
    if (L < 1 || L > 100000) {
        printf("错误：L 超出范围！须满足 1 ≤ L ≤ 100000。\n");
        return 1; // 程序异常退出
    }

    int sum = 0;
    int count = 0;
    int n = 2; // 从2开始判断质数

    while (1) {
        if (determine(n)) {
            if (sum + n <= L) {
                sum += n;
                count++;
                printf("%d\n", n);
            } else {
                break;
            }
        }
        n++; // 不管n是不是质数，继续检查下一个数
    }

    printf("%d\n", count);

    return 0;
}