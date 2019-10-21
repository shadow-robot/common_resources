#!/bin/bash
timeout 5 echo "polyscopeversion" | nc -w2 $1 $2 | 
while read -r cmd; do
    case $cmd in
        3.3.4.310*) echo "UR version $cmd is supported by Shadow"; kill $$;;
        3.7.0.40195*) echo "UR version $cmd is supported by Shadow"; kill $$ ;;
        3.7.2.40245*) echo "UR version $cmd is supported by Shadow"; kill $$ ;;
        3.8.0.61336*) echo "UR version $cmd is supported by Shadow"; kill $$ ;;
        3.4.0.0*) echo "UR version $cmd is supported by Shadow"; kill $$ ;;
        3.5.0.0*) echo "UR version $cmd is supported by Shadow"; kill $$ ;;
        3.6.0.0*) echo "UR version $cmd is supported by Shadow"; kill $$ ;;        
        *) echo "UR version $cmd is NOT supported by Shadow. Use our software at your own risk"; kill $$
        ;;
    esac
done
echo "UR robot did not respond"
