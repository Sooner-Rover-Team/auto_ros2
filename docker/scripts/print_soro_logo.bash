#!/usr/bin/bash
#
# this script just prints out the soro logo lol

NOCOLOR='\033[0m'
LIGHTRED='\033[1;31m'

# make it red
echo -e "${LIGHTRED}"

# print the logo ascii art.
#
# note: generated with a website. some fr art would be v cool
cat ./scripts/logo.txt

# reset color
echo -e "${NOCOLOR}"
