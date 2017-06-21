FROM shadowrobot/dexterous-hand:recognizer

MAINTAINER "Shadow Robot's Software Team <software@shadowrobot.com>"

LABEL Description="An image generated during the testing of Shadow's release procedures." Version="1.0"

ENV remote_shell_script="https://raw.githubusercontent.com/shadow-robot/sr-build-tools/master/ansible/deploy.sh"
ENV PROJECTS_WS=/home/user/projects/shadow_robot
ENV toolset_repo=sr-build-tools
ENV toolset_branch=F_api_blockly_blackbox

COPY interface_setup.sh /usr/local/bin/interface_setup.sh

RUN set +x && \    

    echo "Running one-liner" && \
    apt-get update && \
    wget -O /tmp/oneliner "$( echo "$remote_shell_script" | sed 's/#/%23/g' )" && \
    chmod 755 /tmp/oneliner && \
    gosu $MY_USERNAME /tmp/oneliner -w $PROJECTS_WS/interface -r $toolset_repo -b $toolset_branch -i data/restAPI_blockly_arm_and_box.rosinstall -v "kinetic" && \
    
    
    echo "Removing cache" && \
    apt-get clean && \
    rm -rf /var/lib/apt/lists/* /tmp/* /var/tmp/*
    
ENTRYPOINT ["/usr/local/bin/entrypoint.sh"]

CMD ["/usr/local/bin/interface_setup.sh"]
