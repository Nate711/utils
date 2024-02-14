#!/bin/bash

# Check if a file path argument is provided
if [ "$#" -eq 0 ]; then
    text_file="banner_regular_eyes.txt" # Default file path
else
    text_file="$1" # Use the provided argument
fi


# Assign the first command line argument to a variable
# text_file="$1"

# Iterate over each logged-in session
while IFS= read -r session; do
    # Extract username and TTY
    user=$(echo "$session" | awk '{print $1}')
    tty=$(echo "$session" | awk '{print $2}')

    # Check if TTY exists
    if [ -e "/dev/$tty" ]; then
        # Send each line of your text file to the user's TTY
        while IFS= read -r line; do
            echo -e "$line" > "/dev/$tty"
        done < /home/pi/utils/"$text_file"
    else
        echo "TTY '/dev/$tty' not found for user $user."
    fi
done <<< "$(who)"
