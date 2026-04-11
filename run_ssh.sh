#!/bin/bash
ssh morph@192.168.1.99 "echo hello; ls ~/ads1299-cpp/" > /tmp/ssh_result.txt 2>&1
echo "SSH_RC=$?"
cat /tmp/ssh_result.txt
