#!/bin/bash

# Set variables for the source files and output binary
SOURCE_FILES=("test_unilib.c" "../unilib.c")
HEADER_FILES=("../unilib.h")
OUTPUT_BINARY="test_unilib_run"

# Check if all source files exist
for FILE in "${SOURCE_FILES[@]}"; do
  if [ ! -f "$FILE" ]; then
    echo "Error: Source file '$FILE' not found."
    exit 1
  fi
done

# Check if all header files exist
for HEADER in "${HEADER_FILES[@]}"; do
  if [ ! -f "$HEADER" ]; then
    echo "Error: Header file '$HEADER' not found."
    exit 1
  fi
done

# Compile the source files
echo "Compiling ${SOURCE_FILES[*]} with headers ${HEADER_FILES[*]}..."
gcc "${SOURCE_FILES[@]}" -o "$OUTPUT_BINARY"

# Check if the compilation was successful
if [ $? -ne 0 ]; then
  echo "Compilation failed."
  exit 1
fi

# Run the compiled binary
if [ -f "$OUTPUT_BINARY" ]; then
  echo "Executing $OUTPUT_BINARY..."
  ./"$OUTPUT_BINARY"
else
  echo "Error: Output binary '$OUTPUT_BINARY' not found."
  exit 1
fi
