#include <cstdlib>
#include <iostream>
using namespace std;

int strlen(const char *s) {
    int i = 0;
    for (; s[i]; ++i);
    return i;
}

void strcpy(char *d, const char *s) {
    int i = 0;
    for (i = 0; s[i]; ++i)
        d[i] = s[i];
    d[i] = 0;
}

int strcmp(const char *s1, const char *s2) {
    for (int i = 0; s1[i] && s2[i]; ++i) {
        if (s1[i] < s2[i])
            return -1;
        else if (s1[i] > s2[i])
            return 1;
    }
    return 0;
}

void strcat(char *d, const char *s) {
    int len = strlen(d);
    strcpy(d + len, s);
}

class MyString {
private:
    char *data;

public:
    // 构造函数
    MyString(const char *s = "") {
        data = new char[strlen(s) + 1];
        strcpy(data, s);
    }

    // 拷贝构造函数
    MyString(const MyString &other) {
        data = new char[strlen(other.data) + 1];
        strcpy(data, other.data);
    }

    // 析构函数
    ~MyString() {
        delete[] data;
    }

    // 赋值运算符重载
    MyString &operator=(const MyString &other) {
        if (this != &other) {
            delete[] data;
            data = new char[strlen(other.data) + 1];
            strcpy(data, other.data);
        }
        return *this;
    }

    // 复合赋值加法运算符重载
    MyString &operator+=(const MyString &other) {
        char *newData = new char[strlen(data) + strlen(other.data) + 1];
        strcpy(newData, data);
        strcat(newData, other.data);
        delete[] data;
        data = newData;
        return *this;
    }

    // 加法运算符重载（利用 += 简化，更高效）
    MyString operator+(const MyString &other) const {
        MyString result(*this);
        result += other;
        return result;
    }

    // 下标运算符重载
    char &operator[](int index) {
        return data[index];
    }

    // 用于常量对象的下标运算符重载（只读）
    const char &operator[](int index) const {
        return data[index];
    }

    // 子串获取方法，模拟函数调用形式 s1(0,4)
    MyString operator()(int start, int length) const {
        MyString result;
        delete[] result.data;
        result.data = new char[length + 1];
        for (int i = 0; i < length; ++i) {
            if (start + i < strlen(data)) {
                result.data[i] = data[start + i];
            } else {
                result.data[i] = '\0';
                break;
            }
        }
        result.data[length] = '\0';
        return result;
    }

    // 友元函数，用于输出
    friend ostream &operator<<(ostream &os, const MyString &str) {
        os << str.data;
        return os;
    }

    // 友元函数，用于输入（本题未用到，可根据需要完善）
    friend istream &operator>>(istream &is, MyString &str) {
        char buffer[1000];
        is >> buffer;
        delete[] str.data;
        str.data = new char[strlen(buffer) + 1];
        strcpy(str.data, buffer);
        return is;
    }

    // 比较运算符重载
    bool operator<(const MyString &other) const {
        return strcmp(data, other.data) < 0;
    }

    bool operator==(const MyString &other) const {
        return strcmp(data, other.data) == 0;
    }

    bool operator>(const MyString &other) const {
        return strcmp(data, other.data) > 0;
    }
};

// 非成员函数：支持 const char* + MyString
MyString operator+(const char *left, const MyString &right) {
    MyString result(left);
    result += right;
    return result;
}

int CompareString(const void *e1, const void *e2) {
    MyString *s1 = (MyString *)e1;
    MyString *s2 = (MyString *)e2;
    if (*s1 < *s2)
        return -1;
    else if (*s1 == *s2)
        return 0;
    else
        return 1;
}

int main() {
    MyString s1("abcd-"), s2, s3("efgh-"), s4(s1);
    MyString SArray[4] = {"big", "me", "about", "take"};
    cout << "1. " << s1 << s2 << s3 << s4 << endl;
    s4 = s3;
    s3 = s1 + s3;
    cout << "2. " << s1 << endl;
    cout << "3. " << s2 << endl;
    cout << "4. " << s3 << endl;
    cout << "5. " << s4 << endl;
    cout << "6. " << s1[2] << endl;
    s2 = s1;
    s1 = "ijkl-";
    s1[2] = 'A';
    cout << "7. " << s2 << endl;
    cout << "8. " << s1 << endl;
    s1 += "mnop";
    cout << "9. " << s1 << endl;
    s4 = "qrst-" + s2; // 现在可正确执行
    cout << "10. " << s4 << endl;
    s1 = s2 + s4 + " uvw " + "xyz";
    cout << "11. " << s1 << endl;
    qsort(SArray, 4, sizeof(MyString), CompareString);
    for (int i = 0; i < 4; i++)
        cout << SArray[i] << endl;
    // s1的从下标0开始长度为4的子串
    cout << s1(0, 4) << endl;
    // s1的从下标5开始长度为10的子串
    cout << s1(5, 10) << endl;
    return 0;
}