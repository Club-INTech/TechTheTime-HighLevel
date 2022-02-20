import sys
from typing import Dict

DATA_SEP = "---"
NAME_TYPE_SEP = " "


def extract_vars(srv_file_path: str) -> Dict[str, str]:
    srv_vartypes_dict = {}
    with open(srv_file_path, "r") as srv_file:
        for srv_var in srv_file:
            if srv_var.__contains__(DATA_SEP):
                break
            vartype, var = srv_var.split(NAME_TYPE_SEP)
            if vartype == "int64":
                vartype += "_t"
            srv_vartypes_dict[var[:len(var)-1]] = vartype
    return srv_vartypes_dict


def match_template(content: list, request_template_regex: str) -> int:
    for i, line in enumerate(content):
        if line.__contains__("struct " + request_template_regex):
            return i+2
    return -1


def generate_lines(content: list, srv_vartypes_dict: Dict[str, str], index: int, request_template_regex: str) -> list:
    result_content = []
    for i in range(index):
        result_content.append(content[i])
    constructor_begin = "explicit " + request_template_regex + "("
    counter_ = 1

    for k, v in srv_vartypes_dict.items():
        constructor_begin = constructor_begin + v + " " + k
        if counter_ != len(srv_vartypes_dict):
            constructor_begin = constructor_begin + ", "
        counter_ += 1

    result_content.append(constructor_begin + ") : " + request_template_regex + "(rosidl_runtime_cpp::MessageInitialization::ALL) {\n")

    for k, _ in srv_vartypes_dict.items():
        result_content.append("this->" + k + " = " + k + ";\n")

    result_content.append("}\n")

    for i in range(index, len(content)):
        result_content.append(content[i])

    return result_content


def generate_constructor(cpp_request_file_path: str, srv_vartypes_dict: Dict[str, str],
        request_template_regex: str) -> None:

    content = None
    with open(cpp_request_file_path, "r") as cpp_request_file:
        content = cpp_request_file.readlines()
    index = match_template(content, request_template_regex)

    with open(cpp_request_file_path, "w") as cpp_request_file:
        new_content = "".join(generate_lines(content, srv_vartypes_dict, index, request_template_regex))
        cpp_request_file.write(new_content)


if __name__ == "__main__":
    generate_constructor(sys.argv[1], extract_vars(sys.argv[2]), sys.argv[3])
    