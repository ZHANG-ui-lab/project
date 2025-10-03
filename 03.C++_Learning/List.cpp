#include <iostream>
#include <map>
#include <vector>
#include <algorithm>
using namespace std;

int main() {
    int n;
    cin >> n;
    // 用map存储id和对应的整数序列，vector<int>存储序列中的整数
    map<int, vector<int>> sequences;

    for (int i = 0; i < n; ++i) {
        string cmd;
        cin >> cmd;

        if (cmd == "new") {
            int id;
            cin >> id;
            // 新建一个id的序列，初始为空
            sequences[id] = vector<int>();
        } else if (cmd == "add") {
            int id, num;
            cin >> id >> num;
            // 向id对应的序列中添加整数num
            sequences[id].push_back(num);
        } else if (cmd == "merge") {
            int id1, id2;
            cin >> id1 >> id2;
            if (id1 != id2) {
                // 对两个序列进行排序
                sort(sequences[id1].begin(), sequences[id1].end());
                sort(sequences[id2].begin(), sequences[id2].end());

                vector<int> merged;
                int i = 0, j = 0;
                // 归并两个有序序列
                while (i < sequences[id1].size() && j < sequences[id2].size()) {
                    if (sequences[id1][i] < sequences[id2][j]) {
                        merged.push_back(sequences[id1][i++]);
                    } else {
                        merged.push_back(sequences[id2][j++]);
                    }
                }
                // 处理剩余元素
                while (i < sequences[id1].size()) {
                    merged.push_back(sequences[id1][i++]);
                }
                while (j < sequences[id2].size()) {
                    merged.push_back(sequences[id2][j++]);
                }
                // 更新id1的序列，清空id2的序列
                sequences[id1] = merged;
                sequences[id2].clear();
            }
        } else if (cmd == "unique") {
            int id;
            cin >> id;
            // 先排序
            sort(sequences[id].begin(), sequences[id].end());
            // 去重
            auto last = unique(sequences[id].begin(), sequences[id].end());
            sequences[id].erase(last, sequences[id].end());
        } else if (cmd == "out") {
            int id;
            cin >> id;
            // 先排序
            sort(sequences[id].begin(), sequences[id].end());
            // 输出序列元素，以空格分隔
            for (size_t j = 0; j < sequences[id].size(); ++j) {
                if (j > 0) {
                    cout << " ";
                }
                cout << sequences[id][j];
            }
            cout << endl;
        }
    }

    return 0;
}