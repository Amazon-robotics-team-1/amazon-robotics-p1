#!/bin/bash

# Close windows with title "Project Shell"

# Find window IDs with the specified title
window_ids=$(xdotool search --name "Project Shell")

# Iterate over each window ID and close it
for id in $window_ids; do
    xdotool windowclose "$id"
done