#!/bin/bash

# Syntax printing
function print_syntax() {
    echo
    echo "To generate the txt with the descrition of the possible grammar for the finite state machine"
    echo "specify MAX_NBR_STATES, MAX_NBR_CONNECTIONS, and the txt file name in which the description will be saved:"
    echo "$0 <MAX_NBR_STATES> <MAX_NBR_CONNECTIONS> <TXT_FILE>"
    echo
    exit 1
}

# Write description of state
function write_state() {
  INDEX=$1
  NB_TRANS=$2
  # 4 possible states
  echo "S$INDEX     \"--s$INDEX \"  c   (0,1,2,3) | as.numeric(NumStates)>$INDEX " >> ${TXT_FILE}
  # --rwm can appear in state 0
  echo "RWM$INDEX   \"--rwm$INDEX \"  i (1,100) | as.numeric(S$INDEX)==0" >> ${TXT_FILE}
  # --att can appear in state 2
  echo "ATT$INDEX   \"--att$INDEX \"  r (1,5) | as.numeric(S$INDEX)==2" >> ${TXT_FILE}
  # --rep can appear in state 3
  echo "REP$INDEX   \"--rep$INDEX \"  r (1,5) | as.numeric(S$INDEX)==3" >> ${TXT_FILE}
  if [ ${INDEX} == 0 ]; then
    echo "NumConnections$INDEX \"--n$INDEX \" i (1,$NB_TRANS) | as.numeric(NumStates)>1" >> ${TXT_FILE}
  else
    echo "NumConnections$INDEX \"--n$INDEX \" i (1,$NB_TRANS) | as.numeric(NumStates)>${INDEX}" >> ${TXT_FILE}
  fi
}

function write_connection() {
  STATE=$1
  CONNECTION=$2
  create_range STATE
  echo "N${STATE}x$CONNECTION  \"--n${STATE}x$CONNECTION \" i   (0,3) | as.numeric(NumConnections$STATE)>$CONNECTION " >> ${TXT_FILE}
  # 7 conditions in [0;6]
  echo "C${STATE}x$CONNECTION  \"--c${STATE}x$CONNECTION \" c   (0,1,2,3,4,5,6) | as.numeric(NumConnections$STATE)>$CONNECTION " >> ${TXT_FILE}
  # p as a probability in [0;1] exists in conditions 0,1,2,5 and 6
  echo "P${STATE}x$CONNECTION  \"--p${STATE}x$CONNECTION \" r   (0,1) | as.numeric(C${STATE}x$CONNECTION) %in% c(0,1,2,5,6) " >> ${TXT_FILE}
  # p as an integer in [0;10] and w as an real in [0;20] exists in conditions 3 and 4
  echo "B${STATE}x$CONNECTION  \"--p${STATE}x$CONNECTION \" i   (1,10) | as.numeric(C${STATE}x$CONNECTION)==3 " >> ${TXT_FILE}
  echo "W${STATE}x$CONNECTION  \"--w${STATE}x$CONNECTION \" r   (0,20) | as.numeric(C${STATE}x$CONNECTION)==3 " >> ${TXT_FILE}
  echo "BI${STATE}x$CONNECTION  \"--p${STATE}x$CONNECTION \" i   (1,10) | as.numeric(C${STATE}x$CONNECTION)==4 " >> ${TXT_FILE}
  echo "WI${STATE}x$CONNECTION  \"--w${STATE}x$CONNECTION \" r   (0,20) | as.numeric(C${STATE}x$CONNECTION)==4 " >> ${TXT_FILE}
  # l as a integer (color id) in [0;3] exists in condition 6
  echo "L${STATE}x$CONNECTION  \"--l${STATE}x$CONNECTION \" i   (0,3) | as.numeric(C${STATE}x$CONNECTION)==6 " >> ${TXT_FILE}
}

function create_range() {
  FORBIDDEN=$1
  RANGE="("
  for I in $(seq 0 $MAX_NBR_STATES)
  do
    if [ ${I} != ${STATE} ]; then
      RANGE="${RANGE}${I},"
    fi
  done
  RANGE=${RANGE::-1}
  RANGE="${RANGE})"
}


if [ $# -lt 3 ]; then
    print_syntax
fi

MAX_NBR_STATES=$(echo "$1-1" | bc)
MAX_NBR_CONNECTIONS=$(echo "$2-1" | bc)
TXT_FILE=$3

# Clear content of file
truncate -s 0 $TXT_FILE

# Write grammar in file
echo "NumStates   \"--nstates \"   i (1,$1)" >> $TXT_FILE
for STATE in $(seq 0 $MAX_NBR_STATES)
do
  write_state $STATE $2
  for CONNECTION in $(seq 0 $MAX_NBR_CONNECTIONS)
  do
    write_connection $STATE $CONNECTION
  done
done
