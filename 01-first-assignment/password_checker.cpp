/*brief: The wrong judgement in line 11 has been added equality signs to.
 *       The opposite inequality sign in line 21 has been corrected.
 *       The wrong input in line 24 has been modified to char type.
 *       The useless variable in line 28 has been commented out.
 */

#include <iostream>
#include <string>

bool contains_between(const std::string& text, char lower, char upper) {
    for (char ch : text) {
        if (lower <= ch && ch <= upper) {
            return true;
        }
    }
    return false;
}

bool check_password(const std::string& first_name,
                    const std::string& last_name,
                    const std::string& password) {
    bool length_ok = password.size() >= 10;
    bool upper_ok = contains_between(password, 'A', 'Z');
    bool lower_ok = contains_between(password, 'a', 'z');
    bool digit_ok = contains_between(password, '0', '9');
    bool name_ok = password.find(first_name) == std::string::npos &&
                   password.find(last_name) == std::string::npos;
    //int checked = 0;
    return length_ok && upper_ok && lower_ok && digit_ok && name_ok;
}

int main() {
    //self example: Donghui Chen Az114514Chen
    //expected output: Invalid
    bool res = check_password("Donghui", "Chen", "Az114514Chen");
    std::cout << (res ? "VALID" : "INVALID") << std::endl;

    std::string first_name, last_name, password;
    if (!(std::cin >> first_name >> last_name >> password)) {
        std::cerr << "请输入：名 姓 密码\n";
        return 1;
    }
    std::cout << (check_password(first_name, last_name, password)
                      ? "VALID" : "INVALID") << '\n';
    return 0;
}
