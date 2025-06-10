#!/bin/bash

nmcli radio wifi on

sleep 2
nmcli con up __wipicar
