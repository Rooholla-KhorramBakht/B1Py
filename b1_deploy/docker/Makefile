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
	@docker build --tag jetson:23 --no-cache . -f docker-hw-bridge-23.docker

jetson24:
	@docker build --tag jetson:24 --no-cache . -f docker-hw-bridge-24.docker

jetson25:
	@docker build --tag jetson:25 --no-cache . -f docker-hw-bridge-25.docker

jetson_debug:
	@docker build --tag jetson:debug --no-cache . -f jetson_debug.docker

docker23_install:
	@cp ../autostart/docker-hw-bridge-23.service /etc/systemd/system/
	@systemctl enable docker-hw-bridge-23.service
	@systemctl start docker-hw-bridge-23.service


docker23_uninstall:
	@systemctl disable docker-hw-bridge-23.service
	@systemctl stop docker-hw-bridge-23.service
	@rm /etc/systemd/system/docker-hw-bridge-23.service

docker24_install:
	@cp ../autostart/docker-hw-bridge-24.service /etc/systemd/system/
	@systemctl enable docker-hw-bridge-24.service
	@systemctl start docker-hw-bridge-24.service


docker24_uninstall:
	@systemctl disable docker-hw-bridge-24.service
	@systemctl stop docker-hw-bridge-24.service
	@rm /etc/systemd/system/docker-hw-bridge-24.service

docker25_install:
	@cp ../autostart/docker-hw-bridge-25.service /etc/systemd/system/
	@systemctl enable docker-hw-bridge-25.service
	@systemctl start docker-hw-bridge-25.service


docker25_uninstall:
	@systemctl disable docker-hw-bridge-25.service
	@systemctl stop docker-hw-bridge-25.service
	@rm /etc/systemd/system/docker-hw-bridge-25.service

clean:
	@docker rmi -f jetson:23	
	@docker rmi -f jetson:24	
	@docker rmi -f jetson:25	
	@docker rmi -f jetson:debug