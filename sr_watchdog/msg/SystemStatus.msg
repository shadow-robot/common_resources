# This message holds the status of a watched-over system
#

# Possible status values
byte PENDING=-1
byte OK=0
byte ERROR=1

string system_name
byte status
int32 checks_cycle_completion
CheckStatus[] check_statuses
SystemLog[] logs