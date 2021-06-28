#!/bin/bash
docker run --network host \
  --rm \
  --privileged \
  --runtime=nvidia \
  --security-opt seccomp=unconfined \
  138467776890.dkr.ecr.us-east-1.amazonaws.com/subt/nusseds:latest
