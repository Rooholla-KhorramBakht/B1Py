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
base:
	@docker buildx build --platform linux/arm64,linux/amd64 -t robocaster/robobase:latest -f docker-base.docker --push .
jetson23:
	@docker build --tag jetson:23 --no-cache . -f docker-hw-bridge-23.docker

jetson24:
	@docker build --tag jetson:24 --no-cache . -f docker-hw-bridge-24.docker

jetson25:
	@docker build --tag jetson:25 --no-cache . -f docker-hw-bridge-25.docker

docker_highlevel:
	@docker build --tag jetson:220 --no-cache . -f docker-hw-bridge-highlevel.docker

jetson_orin:
	@docker build --tag jetson:orin --no-cache . -f docker-hw-bridge-orin.docker

openvins:
	@docker build --tag openvins:latest . -f docker-openvins.docker

docker23_install:
	@cp autostart/docker-hw-bridge-23.service /etc/systemd/system/
	@systemctl enable docker-hw-bridge-23.service
	@systemctl start docker-hw-bridge-23.service


docker23_uninstall:
	@systemctl disable docker-hw-bridge-23.service
	@systemctl stop docker-hw-bridge-23.service
	@rm /etc/systemd/system/docker-hw-bridge-23.service

docker24_install:
	@cp autostart/docker-hw-bridge-24.service /etc/systemd/system/
	@systemctl enable docker-hw-bridge-24.service
	@systemctl start docker-hw-bridge-24.service

docker24_uninstall:
	@systemctl disable docker-hw-bridge-24.service
	@systemctl stop docker-hw-bridge-24.service
	@rm /etc/systemd/system/docker-hw-bridge-24.service

docker25_install:
	@cp autostart/docker-hw-bridge-25.service /etc/systemd/system/
	@systemctl enable docker-hw-bridge-25.service
	@systemctl start docker-hw-bridge-25.service


docker25_uninstall:
	@systemctl disable docker-hw-bridge-25.service
	@systemctl stop docker-hw-bridge-25.service
	@rm /etc/systemd/system/docker-hw-bridge-25.service

docker_highlevel_install:
	@cp autostart/docker-hw-bridge-highlevel.service /etc/systemd/system/
	@systemctl enable docker-hw-bridge-highlevel.service
	@systemctl start docker-hw-bridge-highlevel.service

docker_highlevel_install:
	@systemctl disable docker-hw-bridge-highlevel.service
	@systemctl stop docker-hw-bridge-highlevel.service
	@rm /etc/systemd/system/docker-hw-bridge-highlevel.service

docker_orin_install:
	@cp autostart/docker-hw-bridge-orin.service /etc/systemd/system/
	@systemctl enable docker-hw-bridge-orin.service
	@systemctl start docker-hw-bridge-orin.service


docker_orin_uninstall:
	@systemctl disable docker-hw-bridge-orin.service
	@systemctl stop docker-hw-bridge-orin.service
	@rm /etc/systemd/system/docker-hw-bridge-orin.service

openvins_front_install:
	@cp autostart/docker-openvins-front.service /etc/systemd/system/
	# @systemctl enable docker-openvins-front.service
	# @systemctl start docker-openvins-front.service
	

openvins_side_left_install:
	@cp autostart/docker-openvins-side-left.service /etc/systemd/system/
	# @systemctl enable docker-openvins-side-left.service
	# @systemctl start docker-openvins-side-left.service

openvins_side_right_install:
	@cp autostart/docker-openvins-side-right.service /etc/systemd/system/
	# @systemctl enable docker-openvins-side-right.service
	# @systemctl start docker-openvins-side-right.service

openvins_front_uninstall:
	@systemctl disable docker-openvins-front.service
	@systemctl stop docker-openvins-front.service
	@rm /etc/systemd/system/docker-openvins-front.service

openvins_side_left_uninstall:
	@systemctl disable docker-openvins-side-left.service
	@systemctl stop docker-openvins-side-left.service
	@rm /etc/systemd/system/docker-openvins-side-left.service

openvins_side_right_uninstall:
	@systemctl disable docker-openvins-side-right.service
	@systemctl stop docker-openvins-side-right.service
	@rm /etc/systemd/system/docker-openvins-side-right.service

clean:
	@docker rmi -f jetson:23	
	@docker rmi -f jetson:24	
	@docker rmi -f jetson:25	
	@docker rmi -f jetson:debug