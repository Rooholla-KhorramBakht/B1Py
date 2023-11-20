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
	@docker build --tag jetson23:latest . -f jetson23.docker

jetson24:
	@docker build --tag jetson23:latest . -f jetson24.docker

clean:
	@docker rmi -f jetson23:latest	
	@docker rmi -f jetson24:latest