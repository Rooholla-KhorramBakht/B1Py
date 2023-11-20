all: help

help:
	@echo ""
	@echo "-- Help Menu"
	@echo ""
	@echo "   1. make build              - build all images"
	# @echo "   1. make pull             - pull all images"
	@echo "   1. make clean              - remove all images"
	@echo ""

# build:
# 	@docker build --tag ros:vins-fusion -f ./Dockerfile ..

jetson23:
	@docker build --tag jetson:23 --no-cache . -f jetson23.docker

jetson24:
	@docker build --tag jetson:24 --no-cache . -f jetson24.docker

jetson25:
	@docker build --tag jetson:25 --no-cache . -f jetson25.docker

jetson_debug:
	@docker build --tag jetson:debug --no-cache . -f jetson_debug.docker

clean:
	@docker rmi -f jetson:23	
	@docker rmi -f jetson:24	
	@docker rmi -f jetson:25	
	@docker rmi -f jetson:debug