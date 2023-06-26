bash
Copy code
#!/bin/bash

# Define the skill name and perception directory
skill_name="insert"
perception_dir="./dump/perception/Stow_insert/23-May-2023-12:35:57.698997_corner"

# Define the last directory index
last_dir=300

archive_dir="$perception_dir/archive"
inspect_txt="./dump/perception/inspect/$skill_name/inspect.txt"

# Create an archive subdirectory
mkdir -p "$archive_dir"

# Archive the directories specified in the inspect.txt file
echo "Archiving..."
while IFS= read -r idx; do
    echo "$idx"
    mv "$perception_dir/$idx" "$archive_dir/$idx"
done < "$inspect_txt"

# Patch the directory structure by moving directories with higher indices to the specified indices
echo "Patching..."
while IFS= read -r idx; do
    while [ ! -d "$perception_dir/$last_dir" ]; do
        last_dir=$((last_dir - 1))
    done
    if [ "$last_dir" -gt "$idx" ]; then
        echo "$last_dir -> $idx"
        mv "$perception_dir/$last_dir" "$perception_dir/$idx"
    fi
done < "$inspect_txt"
