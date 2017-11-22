#!/bin/bash

# create alias to server in bashrc. remove it if exists

echo "INSTALLING SERVER..."
SERVER_LOCATION="$(pwd)/$(dirname "$0")/server/server.py"
REQUIREMENT_FILE="$(pwd)/$(dirname "$0")/requirement.txt"
python -m pip install -r $REQUIREMENT_FILE --user
if grep "alias vsimple" ~/.bashrc
then
    sed -i "/alias vsimple/d" ~/.bashrc
fi
echo "alias vsimple='python $SERVER_LOCATION'" >> ~/.bashrc

# install vsimplegrep
echo "INSTALLING CLIENT..."
CLIENT_FILE="$(pwd)/$(dirname "$0")/client/"
cd $CLIENT_FILE
python setup.py install --user

echo "successfully install vsimple."
echo "start a new terminal and try command 'vsimple' or 'vsimple map'"