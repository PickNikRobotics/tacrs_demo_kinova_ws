#!/bin/bash
SCRIPT_DIR=$( cd -- "$( dirname -- "${BASH_SOURCE[0]}" )" &> /dev/null && pwd )
filename=$SCRIPT_DIR/.env
printenv > $filename
echo "ASAN_OPTIONS=new_delete_type_mismatch=0:detect_leaks=0:abort_on_error=1" >> $filename