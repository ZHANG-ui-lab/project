#include <stdio.h>

int check(int k, int m) {
    int total = 2 * k; // 总人数
    int pos = 0;       // 当前数数的人
    int killed[k];     // 记录前k个被杀的人的编号
    int killCount = 0; // 已杀人数

    while (killCount < k) {
        // 数m个人，找到第m个的位置，因为是圈，所以用取模
        pos = (pos + m - 1) % total;
        // 将被杀的人的编号放入数组
        killed[killCount++] = pos;
        total--;
        // 下一次从被杀的人的下一个位置开始数
        pos = pos % total;
    }

     // 检查前k个被杀的人的编号是否大于等于K
    for (int i = 0; i < k; i++) {
        if (killed[i] < k) {
            return 0; 
        }
    }
    return 1; 
}

int main() {
    int k;
    scanf("%d", &k);

    if (k <= 0 || k >= 14) {
        printf("输入的 k 值超出范围！请输入 0 < k < 14 的整数。\n");
        return 1; 
    }

    int m = 1;
    while (1) {
        if (check(k, m)) {
            printf("%d\n", m);
            break;
        }
        m++;
    }

    return 0;
}