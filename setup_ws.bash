#!/bin/bash

cd $HOME
mkdir -p muto/src
cd muto/src/
git clone https://github.com/eclipse-muto/agent.git
git clone https://github.com/ibrahimsel/messages.git
git clone https://github.com/ibrahimsel/core.git
git clone -b dev/compose https://github.com/ibrahimsel/composer.git