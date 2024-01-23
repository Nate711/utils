#!/bin/bash

COLOR="\033[0;34m"  # Replace 0;31 with your chosen color code
RESET="\033[0m"

while IFS= read -r line; do
    echo -e "${COLOR}${line}${RESET}"
done < "/home/pi/utils/banner.txt"
