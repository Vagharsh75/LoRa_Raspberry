.DEFAULT_GOAL := install
A = $(shell pwd)
CC = g++
install:
		${CC} main.cpp src/LoRa_RasPi.cpp -o main -I $A -lwiringPi

crypto:
		${CC} main1.cpp src/LoRa_RasPi.cpp -o main -I $A -lwiringPi -lcrypto