#!/bin/bash

if [ -d "./ext" ]; then
    files_ext=$(find ./ext -type f -name "*.cpp.o")
else
    files_ext=""
fi
files_src=$(find ./src -type f -name "*.cpp.o")
files="$files_ext $files_src"

file_cnt=$(echo $files | wc -w)

echo "Cheking $file_cnt .cpp.o files for unqiue symbols"

files_with_uniqe=0
for file in $files; do
    syms=$(nm $file | grep "u " | sed 's/\|/ /' | awk '{print $3}')
    sym_cnt=$(echo $syms | wc -w)
    if [[ sym_cnt -gt 0 ]]; then
        echo $file
        for sym in $syms; do
            echo "  $sym"
        done
        files_with_unique=$(($files_with_unique+1))
    fi
done

if [[ files_with_unique -gt 0 ]]; then
    echo "Found $files_with_unique with unique symbols!"
    exit 1
else
    echo "Found no files with unique symbols."
    exit 0
fi

