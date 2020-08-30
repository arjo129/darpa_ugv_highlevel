#!/bin/bash
docker run --network host \
  --rm \
  --privileged \
  --runtime=nvidia \
  --security-opt seccomp=unconfined \
  arjo129/nusseds_subt_sol:latest