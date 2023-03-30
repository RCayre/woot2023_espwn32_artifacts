# ESPwn32: Hacking with ESP32 System-on-Chips

[![DOI](https://zenodo.org/badge/621285507.svg)](https://zenodo.org/badge/latestdoi/621285507)

This repository contains the artifacts used by the paper "ESPwn32: Hacking with ESP32 System-on-Chips" submitted and accepted at WOOT'23.

## Artifacts overview

These artifacts includes the code, library and firmwares related to the paper **ESPwn32: Hacking with ESP32 System-on-Chips**. It is divided into three main artifact, separated in three different folders.

The first artifact, named **hackwatch**, includes a firmware providing WiFi and BLE testing tools for a Lilygo T-Watch 2020
v1/v2/v3 based on an ESP32. It uses the BLE hooking techniques described in our article to alter on-the-fly the LL PDUs and
perform the fingerprinting approach using LL VERSION IND packets.

The second artifact, named **ant\_network\_keys**, is the implementation of ANT key validation algorithm and the associated specification. To prevent any copyright-related issues, the reverse-engineering and the implementation were done by two different persons. We conducted a responsive disclosure and reported the ANT security related issues to Garmin Security Team. We agreed on an embargo until 30th March 2023. As the embargo period is now over, we release the code and specifications associated to this artifact as open-source software under MIT license.

The third artifact, named **cross\_protocols**, is the implementation of a minimalist cross-protocol library for ESP32, ESP32-S3
and ESP32-C3. It is implemented as an Espressif component, and a set of examples using this library are available, demonstrating
various attacks discussed in the paper. Note that some examples are specific to boards with specific capabilities: for example,
dot15d4 rx and dot15d4 tx examples are only supported on ESP32-S3 and ESP32-C3.

## Repository structure

* In **cross\_protocols**, you will find a minimal library and some examples showing cross-protocol attacks from ESP32, ESP32-C3 and ESP32-S3 devboards.
* In **ant\_network\_keys**, you will find the implementation of ANT key validation and preamble derivation and the associated specification.
* In **hackwatch**, you will find the firmware providing WiFi and BLE testing tools for a T-Watch 2020 v1/v2/v3 based on an ESP32, making use of our BLE hooking techniques.
