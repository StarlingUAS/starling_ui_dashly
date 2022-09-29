MAKEFILE_DIR:=$(shell dirname $(realpath $(firstword $(MAKEFILE_LIST))))
BAKE_SCRIPT:=$(MAKEFILE_DIR)/buildtools/docker-bake.hcl
BUILDX_HOST_PLATFORM:=$(shell docker buildx inspect default | sed -nE 's/^Platforms: ([^,]*),.*$$/\1/p')
BAKE:=docker buildx bake --builder default --load --set *.platform=$(BUILDX_HOST_PLATFORM) -f $(BAKE_SCRIPT)

NETWORK?=bridge
ENV?=
BUILD_ARGS?=
RUN_ARGS?=
PORTS?=-p 3002:3000


all: main

main:
	$(BAKE) starling-ui-dashly

# This mybuilder needs the following lines to be run:
# docker run --rm --privileged multiarch/qemu-user-static --reset -p yes
# docker buildx create --name mybuilder
# docker buildx use mybuilder
# docker buildx inspect --bootstrap
local-build-push:
	docker buildx bake --builder mybuilder -f $(BAKE_SCRIPT) mickeyli789/starling-ui-dashly:latest

run: main
	docker run -it --rm --net=$(NETWORK) $(ENV) $(PORTS) -e USE_SIMULATED_TIME=true $(RUN_ARGS) mickeyli789/starling-ui-dashly:latest

run_bash: main
	docker run -it --rm --net=$(NETWORK) $(ENV) $(PORTS) -e USE_SIMULATED_TIME=true $(RUN_ARGS) mickeyli789/starling-ui-dashly:latest bash

.PHONY: all main run run_bash