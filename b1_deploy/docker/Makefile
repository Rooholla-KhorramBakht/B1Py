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

jetson-nx:
	@docker build --tag jetson-nx:latest . -f jetson-nx.docker

clean:
	@docker rmi -f jetson-nx:latest