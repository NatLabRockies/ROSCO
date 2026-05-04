#!/bin/bash

# Change the directory to the script's location
cd "$(dirname "$0")"

# Create a temporary directory for cloning
temp_dir=$(mktemp -d)
echo "Created temporary directory: $temp_dir"

# Clone the repository
git clone -b develop https://github.com/IEAWindSystems/IEA-15-240-RWT.git "$temp_dir"

# Check if clone was successful
if [ $? -ne 0 ]; then
    echo "Failed to clone repository"
    rm -rf "$temp_dir"
    exit 1
fi

# Create IEA-15-240-RWT directory if it doesn't exist
mkdir -p IEA-15-240-RWT

# Copy OpenFAST directory contents
cp -r "$temp_dir/OpenFAST/"* IEA-15-240-RWT/

# Copy windIO inputs
cp -r "$temp_dir/WT_Ontology/"* ../example_inputs/windio/

# Clean up temporary directory
rm -rf "$temp_dir"

echo "Files copied successfully to IEA-15-240-RWT directory"

# Now do the same for the IEA-22 windIO files

# Create a temporary directory for cloning
temp_dir=$(mktemp -d)
echo "Created temporary directory: $temp_dir"

# Clone the repository
git clone -b main https://github.com/IEAWindSystems/IEA-22-280-RWT.git "$temp_dir"

# Check if clone was successful
if [ $? -ne 0 ]; then
    echo "Failed to clone repository"
    rm -rf "$temp_dir"
    exit 1
fi

# Copy windIO inputs
cp -r "$temp_dir/windIO/"* ../example_inputs/windio/

# Clean up temporary directory
rm -rf "$temp_dir"

echo "Files copied successfully from IEA-22-280-RWT repo"