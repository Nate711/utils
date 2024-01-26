#!/bin/bash



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
        done < /home/pi/utils/banner.txt
    else
        echo "TTY '/dev/$tty' not found for user $user."
    fi
done <<< "$(who)"

