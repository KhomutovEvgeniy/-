#!/bin/sh

function check_style {
    file1=$1
    file2=$2

    echo
    echo  "------------------------------------------------------------------------------------------------------------------------------"

    echo "Comparing '${file1} and '${file2}'"
    echo


    sdiff "${file1}" "${file2}" --ignore-trailing-space

    if [[ $? -eq 0 ]]; then
	echo
        echo -e "\e[30;42mMatching style found\e[0m"
        exit 0
    fi
}

input=$1

astyle --style=allman --indent=spaces=4 --fill-empty-lines < "${input}" > main_allman_spaces_4.c
astyle --style=allman --indent=spaces=2 --fill-empty-lines < "${input}" > main_allman_spaces_2.c
astyle --style=allman --indent=tab      --fill-empty-lines < "${input}" > main_allman_spaces_tab.c

astyle --style=java --indent=spaces=4 --fill-empty-lines < "${input}" > main_java_spaces_4.c
astyle --style=java --indent=spaces=2 --fill-empty-lines < "${input}" > main_java_spaces_2.c
astyle --style=java --indent=tab      --fill-empty-lines < "${input}" > main_java_spaces_tab.c


check_style "${input}" main_allman_spaces_4.c
check_style "${input}" main_allman_spaces_2.c
check_style "${input}" main_allman_spaces_tab.c
check_style "${input}" main_java_spaces_4.c
check_style "${input}" main_java_spaces_2.c
check_style "${input}" main_java_spaces_tab.c

echo
echo -e "\e[41mStyle match not found!\e[0m"
exit 1


