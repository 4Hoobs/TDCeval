# TDCeval

This repository contains a design lab project focused on interfacing the ScioSense TDC-GPX2 Time-to-Digital Converter with a Raspberry Pi Pico using SPI.
The goal of the project is to explore practical time-measurement techniques, digital communication, and embedded system design by building a working controller for a high-resolution TDC device.

The project provides a simple USB-serial command-line interface (CLI) that allows the user to configure the GPX2, validate the configuration, and read measurement results in real time. 

Key Features:

-Interactive CLI for configuring STOP pins, HIT_ENA, REFCLK, HIRES, FIFO modes, XOSC, CMOS input, and more.

-Configuration validator that checks for conflicts
-SPI communication routines for writing and verifying the GPX2 configuration 

-Continuous measurement loop with real-time readout

-Simple runtime controls: pause, resume, REFCLK reset and system reboot

Documentation used:

[DATASHEET] https://www.sciosense.com/wp-content/uploads/2023/12/TDC-GPX2-Datasheet.pdf
[USER-GUIDE] https://www.sciosense.com/wp-content/uploads/2023/12/TDC-GPX2-Evaluation-Kit-User-Guide.pdf


Project realted to FIT in ALICE. Project for Design Laboratory credit. Team: Hubert Sierant, Lena Przybylska 



