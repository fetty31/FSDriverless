#!/bin/bash

# If provided, takes the nth argument and assigns it to the variable
# Otherwise the default value is used:
# <VARIABLE NAME>=${<#ARGUMENT>:-<DEFAULT VALUE>}
SRC_DIR=${1:-../src}

patch -r- -u -d ${SRC_DIR} < CMRosIF.patch
